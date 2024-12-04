#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <cJSON.h>
#include <string>
#include <functional>

struct BinaryProtocol3 {
    uint8_t type;          // 1 字节，表示数据类型
    uint8_t reserved;      // 1 字节，保留字段
    uint16_t payload_size; // 2 字节，表示负载数据的大小
    uint8_t payload[];     // 可变长度的负载数据
} __attribute__((packed)); // 使用__attribute__((packed))来确保结构体按字节对齐

enum AbortReason {
    kAbortReasonNone,
    kAbortReasonWakeWordDetected
};

enum ListeningMode {
    kListeningModeAutoStop,
    kListeningModeManualStop,
    kListeningModeAlwaysOn // 需要 AEC 支持
};

/**
 * 协议处理基类
 * 使用了抽象工厂模式，这是一个抽象基类，定义了音频通信协议的基本接口
 * 采用回调机制处理异步事件
*/
class Protocol {
public:
    // 虚析构函数，确保正确释放派生类资源
    virtual ~Protocol() = default;

    // 获取服务器采样率
    inline int server_sample_rate() const {
        return server_sample_rate_;
    }

    // 回调函数设置接口
    // 设置接收音频数据的回调函数
    void OnIncomingAudio(std::function<void(const std::string& data)> callback);
    // 设置接收 JSON 数据的回调函数
    void OnIncomingJson(std::function<void(const cJSON* root)> callback);
    // 设置音频通道打开时的回调函数
    void OnAudioChannelOpened(std::function<void()> callback);
    // 设置音频通道关闭时的回调函数
    void OnAudioChannelClosed(std::function<void()> callback);
    // 设置网络错误时的回调函数
    void OnNetworkError(std::function<void(const std::string& message)> callback);

    // 纯虚函数接口，需要派生类实现
    // 打开音频通道
    virtual bool OpenAudioChannel() = 0;
    // 关闭音频通道
    virtual void CloseAudioChannel() = 0;
    // 检查音频通道是否打开
    virtual bool IsAudioChannelOpened() const = 0;
    // 发送音频数据
    virtual void SendAudio(const std::string& data) = 0;
    // 发送唤醒词检测信号
    virtual void SendWakeWordDetected(const std::string& wake_word);
    // 发送开始监听信号，可指定监听模式
    virtual void SendStartListening(ListeningMode mode);
    // 发送停止监听信号
    virtual void SendStopListening();
    // 发送中止语音信号，可指定中止原因
    virtual void SendAbortSpeaking(AbortReason reason);

protected:
    // 回调函数成员变量
    std::function<void(const cJSON* root)> on_incoming_json_;      // JSON 数据接收回调
    std::function<void(const std::string& data)> on_incoming_audio_; // 音频数据接收回调
    std::function<void()> on_audio_channel_opened_;                // 音频通道打开回调
    std::function<void()> on_audio_channel_closed_;               // 音频通道关闭回调
    std::function<void(const std::string& message)> on_network_error_; // 网络错误回调

    // 基础属性
    int server_sample_rate_ = 16000;    // 服务器采样率，默认16kHz
    std::string session_id_;            // 会话ID

    // 发送文本消息的纯虚函数接口
    virtual void SendText(const std::string& text) = 0;
};

#endif // PROTOCOL_H

