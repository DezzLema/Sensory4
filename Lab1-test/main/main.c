#include <stdio.h>
#include <driver/gpio.h>
#include <driver/rmt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <string.h>
#include "esp_random.h"
#include <math.h>
#include "esp_timer.h"

// Пины для ESP32-H2
#define BUTTON_PIN GPIO_NUM_8      // Кнопка запуска (отдельная)
#define ENCODER_CLK GPIO_NUM_4     // Энкодер CLK
#define ENCODER_DT GPIO_NUM_5      // Энкодер DT  
#define ENCODER_SW GPIO_NUM_6      // Кнопка энкодера
#define LED_STRIP_PIN GPIO_NUM_7   // Управление светодиодным кольцом

// Параметры рулетки
#define NUM_LEDS 37
#define NUM_RED 18
#define NUM_BLACK 18
#define GREEN_POS 0

// RMT конфигурация для WS2812
#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define RMT_TX_GPIO LED_STRIP_PIN
#define RMT_CLK_DIV 2  // 40MHz / 2 = 20MHz
#define WS2812_T0H_NS 400
#define WS2812_T0L_NS 850
#define WS2812_T1H_NS 800
#define WS2812_T1L_NS 450
#define WS2812_RESET_US 50

// Состояния игры
typedef enum {
    STATE_IDLE,
    STATE_SPINNING,
    STATE_SLOWING,
    STATE_STOPPED,
    STATE_FINAL_STOP  // Новое состояние для полной остановки
} game_state_t;

// Цвета
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_color_t;

// Глобальные переменные
static const char *TAG = "ROULETTE";
static game_state_t game_state = STATE_IDLE;
static volatile int encoder_count = 0;
static int last_encoder_value = 0;
static bool button_pressed = false;
static bool encoder_button_pressed = false;
static uint64_t last_button_debounce_time = 0;
static const uint64_t debounce_delay = 50000;

// Параметры вращения
static float wheel_speed = 0.0f;
static float ball_speed = 0.0f;
static float wheel_position = 0.0f;
static float ball_position = 0.0f;
static float target_speed = 50.0f;
static const float acceleration = 0.1f;
static uint64_t slowing_start_time = 0;
static uint64_t stopped_start_time = 0;

// Буферы
static rgb_color_t led_buffer[NUM_LEDS];
static rgb_color_t display_buffer[NUM_LEDS];
static rmt_item32_t rmt_buffer[NUM_LEDS * 24 + 1];

// Прототипы
static void init_gpio(void);
static void init_rmt(void);
static void init_led_strip(void);
static void update_rotation(void);
static void render_frame(void);
static void send_led_data(void);
static int get_led_index(float position);
static void start_spin(void);
static void encoder_isr_handler(void* arg);
static void roulette_task(void* arg);

void app_main(void) {
    ESP_LOGI(TAG, "Starting Roulette Simulator on ESP32-H2");
    
    init_gpio();
    init_rmt();
    init_led_strip();
    
    xTaskCreate(roulette_task, "roulette_task", 4096, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "System ready. Rotate encoder to adjust speed, press BUTTON (GPIO8) to spin");
}

// Обработчик энкодера
static void IRAM_ATTR encoder_isr_handler(void* arg) {
    static int last_a = 1;
    
    int a = gpio_get_level(ENCODER_CLK);
    int b = gpio_get_level(ENCODER_DT);
    
    if (a != last_a) {
        if (b != a) {
            encoder_count++;
        } else {
            encoder_count--;
        }
        last_a = a;
    }
}

static void init_gpio(void) {
    // Кнопка запуска
    gpio_reset_pin(BUTTON_PIN);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_pullup_dis(BUTTON_PIN);
    gpio_pulldown_en(BUTTON_PIN);
    
    // Энкодер
    gpio_reset_pin(ENCODER_CLK);
    gpio_set_direction(ENCODER_CLK, GPIO_MODE_INPUT);
    gpio_pullup_en(ENCODER_CLK);
    
    gpio_reset_pin(ENCODER_DT);
    gpio_set_direction(ENCODER_DT, GPIO_MODE_INPUT);
    gpio_pullup_en(ENCODER_DT);
    
    // Кнопка энкодера
    gpio_reset_pin(ENCODER_SW);
    gpio_set_direction(ENCODER_SW, GPIO_MODE_INPUT);
    gpio_pullup_en(ENCODER_SW);
    
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENCODER_CLK, encoder_isr_handler, NULL);
    gpio_isr_handler_add(ENCODER_DT, encoder_isr_handler, NULL);
}

// Инициализация RMT
static void init_rmt(void) {
    rmt_config_t config = {
        .rmt_mode = RMT_MODE_TX,
        .channel = RMT_TX_CHANNEL,
        .gpio_num = RMT_TX_GPIO,
        .clk_div = RMT_CLK_DIV,
        .mem_block_num = 1,
        .tx_config = {
            .carrier_freq_hz = 0,
            .carrier_level = RMT_CARRIER_LEVEL_LOW,
            .idle_level = RMT_IDLE_LEVEL_LOW,
            .carrier_duty_percent = 0,
            .carrier_en = false,
            .loop_en = false,
            .idle_output_en = true,
        }
    };
    
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
    
    ESP_LOGI(TAG, "RMT initialized for WS2812 on GPIO%d", LED_STRIP_PIN);
}

static void init_led_strip(void) {
    // Зеленый
    led_buffer[GREEN_POS] = (rgb_color_t){0, 255, 0};
    
    // Распределение красных и черных
    int red_count = 0;
    for (int i = 1; i < NUM_LEDS; i++) {
        if (i % 2 == 1 && red_count < NUM_RED) {
            led_buffer[i] = (rgb_color_t){255, 0, 0};
            red_count++;
        } else {
            led_buffer[i] = (rgb_color_t){0, 0, 0};
        }
    }
}

// Отправка данных через RMT
static void send_led_data(void) {
    uint32_t rmt_index = 0;
    
    for (int i = 0; i < NUM_LEDS; i++) {
        uint32_t color = ((uint32_t)display_buffer[i].g << 16) |
                         ((uint32_t)display_buffer[i].r << 8) |
                          (uint32_t)display_buffer[i].b;
        
        for (int bit = 23; bit >= 0; bit--) {
            if (color & (1 << bit)) {
                rmt_buffer[rmt_index].level0 = 1;
                rmt_buffer[rmt_index].duration0 = (WS2812_T1H_NS * 20) / 1000;
                rmt_buffer[rmt_index].level1 = 0;
                rmt_buffer[rmt_index].duration1 = (WS2812_T1L_NS * 20) / 1000;
            } else {
                rmt_buffer[rmt_index].level0 = 1;
                rmt_buffer[rmt_index].duration0 = (WS2812_T0H_NS * 20) / 1000;
                rmt_buffer[rmt_index].level1 = 0;
                rmt_buffer[rmt_index].duration1 = (WS2812_T0L_NS * 20) / 1000;
            }
            rmt_index++;
        }
    }
    
    rmt_buffer[rmt_index].level0 = 0;
    rmt_buffer[rmt_index].duration0 = (WS2812_RESET_US * 1000) / 50;
    rmt_buffer[rmt_index].level1 = 0;
    rmt_buffer[rmt_index].duration1 = 0;
    
    rmt_write_items(RMT_TX_CHANNEL, rmt_buffer, rmt_index + 1, false);
    rmt_wait_tx_done(RMT_TX_CHANNEL, portMAX_DELAY);
}

static void roulette_task(void* arg) {
    const TickType_t delay_ms = 16 / portTICK_PERIOD_MS;
    
    while (1) {
        uint64_t now = esp_timer_get_time();
        
        // Кнопка запуска (GPIO8)
        if (gpio_get_level(BUTTON_PIN) == 0) {
            if (!button_pressed && (now - last_button_debounce_time) > debounce_delay) {
                button_pressed = true;
                last_button_debounce_time = now;
                if (game_state == STATE_IDLE || game_state == STATE_FINAL_STOP) {
                    start_spin();
                }
            }
        } else {
            button_pressed = false;
        }
        
        // Энкодер - только в режиме ожидания
        if (game_state == STATE_IDLE || game_state == STATE_FINAL_STOP) {
            int encoder_change = encoder_count - last_encoder_value;
            if (encoder_change != 0) {
                target_speed += encoder_change * 2.0f;
                target_speed = fmaxf(10.0f, fminf(target_speed, 200.0f));
                ESP_LOGI(TAG, "Speed: %.1f°/s", target_speed);
                last_encoder_value = encoder_count;
            }
        }
        
        update_rotation();
        render_frame();
        send_led_data();
        
        vTaskDelay(delay_ms);
    }
}

static void start_spin(void) {
    game_state = STATE_SPINNING;
    wheel_speed = target_speed;
    ball_speed = -target_speed * 1.2f;
    slowing_start_time = esp_timer_get_time();
    ESP_LOGI(TAG, "Spin started! Speed: %.1f°/s", target_speed);
}

static void update_rotation(void) {
    static uint64_t last_update_time = 0;
    uint64_t now = esp_timer_get_time();
    
    if (last_update_time == 0) {
        last_update_time = now;
        return;
    }
    
    float delta_time = (now - last_update_time) / 1000000.0f;
    
    switch (game_state) {
        case STATE_IDLE:
        case STATE_FINAL_STOP:
            // Ничего не делаем
            break;
            
        case STATE_SPINNING:
            if (now - slowing_start_time > 1500000) {
                game_state = STATE_SLOWING;
                ESP_LOGI(TAG, "Starting to slow down");
            }
            break;
            
        case STATE_SLOWING:
            if (wheel_speed > 5.0f) {
                wheel_speed -= acceleration * 0.5f;
                ball_speed += acceleration * 0.4f;
                
                if (fabsf(wheel_speed + ball_speed) < 2.0f) {
                    wheel_speed = 3.0f;
                    ball_speed = -3.0f;
                    game_state = STATE_STOPPED;
                    stopped_start_time = now;
                    
                    int stop_position = esp_random() % NUM_LEDS;
                    ball_position = stop_position * (360.0f / NUM_LEDS);
                    
                    ESP_LOGI(TAG, "Ball stopped at LED %d", stop_position);
                    if (stop_position == GREEN_POS) {
                        ESP_LOGI(TAG, "Result: GREEN!");
                    } else if (led_buffer[stop_position].r == 255) {
                        ESP_LOGI(TAG, "Result: RED!");
                    } else {
                        ESP_LOGI(TAG, "Result: BLACK!");
                    }
                }
            }
            break;
            
        case STATE_STOPPED:
            // Экспоненциальное замедление в течение 0.8 секунды
            if (now - stopped_start_time < 800000) {
                wheel_speed *= 0.9f;
                ball_speed *= 0.9f;
            }
            // Затем ПРИНУДИТЕЛЬНАЯ остановка
            else if (now - stopped_start_time < 1000000) {
                wheel_speed = 0.0f;
                ball_speed = 0.0f;
            }
            // И переходим в FINAL_STOP
            else {
                game_state = STATE_FINAL_STOP;
                ESP_LOGI(TAG, "Fully stopped. Ready for next spin in 1 second");
            }
            break;
    }
    
    // Обновление позиций только если скорости не нулевые
    if (wheel_speed != 0.0f) {
        wheel_position += wheel_speed * delta_time;
    }
    if (ball_speed != 0.0f) {
        ball_position += ball_speed * delta_time;
    }
    
    // Нормализация углов
    wheel_position = fmodf(wheel_position, 360.0f);
    if (wheel_position < 0) wheel_position += 360.0f;
    
    ball_position = fmodf(ball_position, 360.0f);
    if (ball_position < 0) ball_position += 360.0f;
    
    last_update_time = now;
}

static void render_frame(void) {
    int wheel_offset = get_led_index(wheel_position);
    
    // Отображаем колесо рулетки
    for (int i = 0; i < NUM_LEDS; i++) {
        int source_pos = (NUM_LEDS + i - wheel_offset) % NUM_LEDS;
        display_buffer[i] = led_buffer[source_pos];
    }
    
    // Отображаем шарик
    int ball_led = get_led_index(ball_position);
    
    // ВСЕГДА БЕЛЫЙ ШАРИК, независимо от состояния
    display_buffer[ball_led] = (rgb_color_t){255, 255, 255};
    
    // Если всё остановилось, можно сделать дополнительную индикацию
    if (game_state == STATE_FINAL_STOP) {
        // Можно сделать мигание или другую индикацию, но шарик остается белым
        // Например, можно сделать зеленый круг вокруг выигрышной позиции
        if (led_buffer[get_led_index(ball_position)].r == 255) {
            // Красный выигрыш - подсветить соседние светодиоды
            int prev_led = (ball_led - 1 + NUM_LEDS) % NUM_LEDS;
            int next_led = (ball_led + 1) % NUM_LEDS;
            display_buffer[prev_led] = (rgb_color_t){100, 0, 0};
            display_buffer[next_led] = (rgb_color_t){100, 0, 0};
        }
    }
}

static int get_led_index(float position) {
    return (int)(position / 360.0f * NUM_LEDS) % NUM_LEDS;
}