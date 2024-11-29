#include <esp_log.h>
#include <esp_err.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <esp_event.h>

#include "application.h"
#include "system_info.h"

#define TAG "main"

/** 在 ESP-IDF 或其他需要 C/C++ 混合编程的项目中，常常会看到 extern "C" 的用法，尤其是在定义设备驱动程序、回调函数或其他需要与 C 语言兼容的接口时。
 * 使用 extern "C" 的好处是：
    保证了 C 和 C++ 代码的互操作性。
    避免了因名称改编导致的链接错误。
 * 当你在 C++ 代码中调用 C 语言编写的函数时，如果不使用 extern "C"，编译器会对 C 函数名进行名称改编，从而导致链接错误。
    extern "C" 的作用是告诉 C++ 编译器：
        不要对这些函数进行名称改编。
        使用 C 的方式进行链接。
*/
extern "C" void app_main(void)
{
    // Initialize the default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize NVS flash for WiFi configuration
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS flash to fix corruption");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Otherwise, launch the application
    Application::GetInstance().Start();

    // Dump CPU usage every 10 second
    while (true) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        // SystemInfo::PrintRealTimeStats(pdMS_TO_TICKS(1000));
        int free_sram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        int min_free_sram = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
        ESP_LOGI(TAG, "Free internal: %u minimal internal: %u", free_sram, min_free_sram);
    }
}
