#include <esp_log.h>
#include <esp_err.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <esp_event.h>

#include "application.h"
#include "system_info.h"

#define TAG "main"

/** 
 * 总的来说，这段代码展示了在ESP-IDF环境下初始化设备、处理潜在的NVS闪存问题、启动应用程序，并定期检查内部SRAM使用情况的典型流程。
 * 在 ESP-IDF 或其他需要 C/C++ 混合编程的项目中，常常会看到 extern "C" 的用法，尤其是在定义设备驱动程序、回调函数或其他需要与 C 语言兼容的接口时。
 * 当你在 C++ 代码中调用 C 语言编写的函数时，如果不使用 extern "C"，编译器会对 C 函数名进行名称改编，从而导致链接错误。
    extern "C" 的作用是告诉 C++ 编译器：
        不要对这些函数进行名称改编。
        使用 C 的方式进行链接。
*/
extern "C" void app_main(void)
{
    /**
     * 这行代码创建了一个默认的事件循环，这对于处理异步事件（如WiFi事件、TCP/IP事件等）是必需的。
     * ESP_ERROR_CHECK是一个宏，用于检查函数调用的返回值。如果返回值表示错误，它将停止程序执行。
     * */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /**
     * NVS（非易失性存储）用于存储键值对数据，通常用于保存WiFi配置等需要持久存储的数据。
     * */ 
    esp_err_t ret = nvs_flash_init();
    /**
     * 接下来的代码块检查NVS初始化是否因为某些原因失败（例如，没有空闲页或找到了新版本），并在需要时擦除NVS闪存以修复损坏：
    */
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // 如果NVS初始化失败，它会记录一个警告日志，擦除NVS闪存，并重新尝试初始化。
        ESP_LOGW(TAG, "Erasing NVS flash to fix corruption");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /**
     * 这行代码调用了一个名为Application的类的GetInstance静态方法获取其实例，然后调用Start方法启动应用程序。
     * 这个Application类很可能是用户定义的，用于封装应用程序的主要逻辑。
     * */
    Application::GetInstance().Start();

    /**
     * 这个无限循环使用vTaskDelay函数每隔10秒（10000毫秒）暂停一次。
     * 在每次循环中，它使用heap_caps_get_free_size和heap_caps_get_minimum_free_size函数
     * 获取内部SRAM的当前空闲大小和自启动以来的最小空闲大小，
     * 并使用ESP_LOGI宏打印这些信息。
     * MALLOC_CAP_INTERNAL参数指示这些函数应该返回内部SRAM的相关信息。
    */
    while (true) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        // SystemInfo::PrintRealTimeStats(pdMS_TO_TICKS(1000));
        int free_sram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        int min_free_sram = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
        ESP_LOGI(TAG, "Free internal: %u minimal internal: %u", free_sram, min_free_sram);
    }
}
