#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "button.h"
#include "esp_log.h"
#include "esp_err.h"
#include "led_strip.h"

#define BUTTON_PIN GPIO_NUM_39
#define LED_STRIP_LENGTH 1
#define LED_PIN GPIO_NUM_27
#define LED_STRIP_LED_NUMBERS 1
#define LED_MODEL LED_MODEL_SK6812
#define LED_PIXEL_FORMAT LED_PIXEL_FORMAT_GRBW
#define INT_BRIGHT 64
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

static const char *TAG = "LED_SK6812_BUTTON";
led_strip_handle_t led_strip;  // Declare a global variable

led_strip_handle_t configure_led(void) {
    // LED strip general initialization, according to your LED board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_PIN,   // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_NUMBERS,        // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT, // Pixel format of your LED strip
        .led_model = LED_MODEL,            // LED strip model
        .flags.invert_out = false,                // Whether to invert the output signal
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .rmt_channel = 0,
#else
        .clk_src = RMT_CLK_SRC_DEFAULT,        // Different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3
#endif
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}

void button_task(void *pvParameters) {
    ESP_LOGI(TAG, "Waiting for button press....");

    button_event_t ev;
    QueueHandle_t button_events = button_init(PIN_BIT(BUTTON_PIN));

    while (true) {
        if (xQueueReceive(button_events, &ev, 1000/portTICK_PERIOD_MS)) {
            if ((ev.pin == BUTTON_PIN) && (ev.event == BUTTON_DOWN)) {
                ESP_LOGI(TAG, "Button has being pressed");

                static int state = 0;
                switch (state) {
                    case 0:
                        ESP_LOGI(TAG, "Colour Red");
                        // Turn on the LED strip with a specific color (e.g., red)
                        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, INT_BRIGHT, 0, 0));
                        break;

                    case 1:
                        ESP_LOGI(TAG, "Colour Blue");
                        // Set another color (e.g., blue)
                        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 0, INT_BRIGHT));
                        break;

                    case 2:
                        ESP_LOGI(TAG, "Colour Green");
                        // Set another color (e.g., green)
                        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, INT_BRIGHT, 0));
                        break;

                    case 3:
                        ESP_LOGI(TAG, "Colour White");
                        // Set another color (e.g., white)
                        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, INT_BRIGHT, INT_BRIGHT, INT_BRIGHT));
                        break;

                    default:
                        state = -1;  // Reset state to start from off state
                }

                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
                state = (state + 1) % 4;  // Cycle through 5 states
            }
        }
    }
}

void app_main() {

    // Initialize LED strip
    led_strip = configure_led();
    ESP_LOGI(TAG, "LED configured!");

    // Set LED to Green as OK!
    ESP_ERROR_CHECK(led_strip_clear(led_strip));
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, INT_BRIGHT, 0)); // Green to start with to signal OK
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    ESP_LOGI(TAG, "Ready to proceed!");

    // Create task for button control
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
}
