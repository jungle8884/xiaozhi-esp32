#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include <freertos/FreeRTOS.h>           // FreeRTOS 核心库
#include <freertos/event_groups.h>       // FreeRTOS 事件组管理
#include <freertos/task.h>               // FreeRTOS 任务管理
#include <opus.h>                        // Opus 音频编解码库
#include <mutex>                        // C++ 标准库中的互斥量
#include <list>                          // C++ 标准库中的双向链表
#include <condition_variable>            // C++ 标准库中的条件变量

#include "opus_encoder.h"                // 自定义音频编码器头文件
#include "opus_resampler.h"              // 自定义音频重采样器头文件

#include "protocol.h"                    // 自定义协议处理头文件
#include "display.h"                     // 自定义显示控制头文件
#include "board.h"                       // 自定义硬件板接口头文件
#include "ota.h"                         // 自定义 OTA（过空中升级）控制头文件
#include "background_task.h"             // 自定义后台任务处理头文件

#if CONFIG_IDF_TARGET_ESP32S3          // 针对 ESP32S3 特定的配置
#include "wake_word_detect.h"            // 唤醒词检测头文件
#include "audio_processor.h"             // 音频处理器头文件
#endif

// 事件标志宏定义，用于 FreeRTOS 事件组
#define SCHEDULE_EVENT (1 << 0)             // 1: 0001
#define AUDIO_INPUT_READY_EVENT (1 << 1)    // 2: 0010
#define AUDIO_OUTPUT_READY_EVENT (1 << 2)   // 4: 0100

// 聊天状态的枚举类型定义
enum ChatState {
    kChatStateUnknown,    // 未知状态
    kChatStateIdle,       // 空闲状态
    kChatStateConnecting,  // 连接中
    kChatStateListening,   // 正在听
    kChatStateSpeaking,    // 正在说
    kChatStateUpgrading    // 升级状态
};

#define OPUS_FRAME_DURATION_MS 60           // Opus 帧持续时间（毫秒）

class Application {
public:
    /** 单例模式
     * GetInstance() 方法返回 Application 类的唯一实例引用。
     * 通过 static 关键字确保类只有一个实例，且仅在首次调用时创建。
     * 该方法保证了线程安全性，且实例在整个程序运行期间持续存在。
     * 
     * Application&：返回 Application 类型的引用，避免了复制。
     */
    static Application& GetInstance() {
        static Application instance; // 确保只有一次实例被创建
        return instance;
    }

    // 删除拷贝构造函数和赋值运算符，防止复制实例，确保单例性
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    // 公共方法定义
    void Start();                                  // 启动应用程序
    ChatState GetChatState() const { return chat_state_; } // 获取当前聊天状态
    void Schedule(std::function<void()> callback); // 安排任务回调
    void SetChatState(ChatState state);           // 设置聊天状态
    void Alert(const std::string&& title, const std::string&& message); // 弹出警告
    void AbortSpeaking(AbortReason reason);       // 中止说话
    void ToggleChatState();                        // 切换聊天状态
    void StartListening();                         // 开始监听
    void StopListening();                          // 停止监听

private:
    // 私有构造函数和析构函数，确保外部无法直接创建或销毁 Application 的实例
    Application();
    ~Application();

#if CONFIG_IDF_TARGET_ESP32S3
    WakeWordDetect wake_word_detect_;            // 唤醒词检测实例
    AudioProcessor audio_processor_;              // 音频处理实例
#endif

    // 状态变量定义
    Ota ota_;                                     // OTA 实例
    std::mutex mutex_;                            // 互斥量，用于线程安全
    std::list<std::function<void()>> main_tasks_; // 任务队列
    Protocol* protocol_ = nullptr;                // 协议处理指针
    EventGroupHandle_t event_group_;              // 事件组句柄
    volatile ChatState chat_state_ = kChatStateUnknown; // 当前聊天状态
    bool keep_listening_ = false;                 // 控制是否持续监听
    bool aborted_ = false;                        // 控制是否中止

    // 音频处理相关成员
    BackgroundTask background_task_;              // 后台任务处理
    std::chrono::steady_clock::time_point last_output_time_; // 上次输出时间
    std::list<std::string> audio_decode_queue_;   // 音频解码队列

    // Opus 编解码器和重采样器
    OpusEncoder opus_encoder_;                    // Opus 编码器实例
    OpusDecoder* opus_decoder_ = nullptr;        // Opus 解码器指针

    // 解码样本率
    int opus_decode_sample_rate_ = -1;           // Opus 解码样本率
    OpusResampler input_resampler_;               // 输入重采样器
    OpusResampler reference_resampler_;           // 参考重采样器
    OpusResampler output_resampler_;              // 输出重采样器

    // 私有方法定义
    void MainLoop();                               // 主循环
    void InputAudio();                             // 输入音频处理
    void OutputAudio();                            // 输出音频处理
    void ResetDecoder();                           // 重置解码器
    void SetDecodeSampleRate(int sample_rate);    // 设置解码样本率
    void CheckNewVersion();                        // 检查新版本

    void PlayLocalFile(const char* data, size_t size); // 播放本地文件
};

#endif // _APPLICATION_H_
