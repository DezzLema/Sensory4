#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

// Пины
#define BUTTON_PIN GPIO_NUM_5
#define LED_PIN GPIO_NUM_2

// Тег для логирования
static const char *TAG = "BUTTON_TEST";

void app_main(void) {
    // Инициализация
    ESP_LOGI(TAG, "Starting ESP32 Button Test");
    
    // Настройка светодиода
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);
    
    // Настройка кнопки БЕЗ внутренней подтяжки
    gpio_reset_pin(BUTTON_PIN);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    // Убираем внутреннюю подтяжку, так как есть внешний резистор
    gpio_pullup_dis(BUTTON_PIN);
    gpio_pulldown_dis(BUTTON_PIN);
    
    ESP_LOGI(TAG, "System initialized. Press button...");
    
    int press_count = 0;
    bool last_state = 1; // HIGH благодаря внешнему резистору
    
    while (1) {
        bool current_state = gpio_get_level(BUTTON_PIN);
        
        // Нажатие = переход от HIGH к LOW
        if (current_state == 0 && last_state == 1) {
            press_count++;
            
            // Переключаем светодиод
            static bool led_on = false;
            led_on = !led_on;
            gpio_set_level(LED_PIN, led_on ? 1 : 0);
            
            ESP_LOGI(TAG, "Button pressed! Count: %d, LED: %s", 
                    press_count, led_on ? "ON" : "OFF");
            
            // Простая антидребезг
            vTaskDelay(pdMS_TO_TICKS(200));
            
            // Ждем отпускания кнопки
            while (gpio_get_level(BUTTON_PIN) == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        
        last_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}