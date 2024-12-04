#include "application.h"
#include "system_info.h"
#include "ml307_ssl_transport.h"
#include "audio_codec.h"
#include "mqtt_protocol.h"
#include "websocket_protocol.h"
#include "font_awesome_symbols.h"

#include <cstring>
#include <esp_log.h>
#include <cJSON.h>
#include <driver/gpio.h>
#include <arpa/inet.h>

#define TAG "Application"

extern const char p3_err_reg_start[] asm("_binary_err_reg_p3_start");
extern const char p3_err_reg_end[] asm("_binary_err_reg_p3_end");
extern const char p3_err_pin_start[] asm("_binary_err_pin_p3_start");
extern const char p3_err_pin_end[] asm("_binary_err_pin_p3_end");
extern const char p3_err_wificonfig_start[] asm("_binary_err_wificonfig_p3_start");
extern const char p3_err_wificonfig_end[] asm("_binary_err_wificonfig_p3_end");


/**
 * @brief Application类的构造函数
 *
 * 初始化Application对象，创建事件组，并配置OTA更新功能
 */
Application::Application() : background_task_(4096 * 8) {
    // 创建一个事件组
    event_group_ = xEventGroupCreate();

    // 配置OTA更新功能
    // 设置检查版本的URL
    ota_.SetCheckVersionUrl(CONFIG_OTA_VERSION_URL);
    // 设置请求头信息
    // "Device-Id" 为请求头字段名
    // SystemInfo::GetMacAddress().c_str() 为请求头的值，即设备的MAC地址
    ota_.SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
}

/**
 * @brief 析构函数，用于销毁 Application 对象
 *
 * 释放 protocol_、opus_decoder_ 以及 event_group_ 所占用的资源
 */
Application::~Application() {
    // 如果 protocol_ 不为空
    if (protocol_ != nullptr) {
        // 删除 protocol_
        delete protocol_;
    }

    // 如果 opus_decoder_ 不为空
    if (opus_decoder_ != nullptr) {
        // 销毁 opus_decoder_
        opus_decoder_destroy(opus_decoder_);
    }

    // 删除事件组
    vEventGroupDelete(event_group_);
}

/**
 * @brief 检查新版本
 *
 * 检查是否有可用的新固件版本，如果有，则尝试进行升级。
 *
 * @return 无返回值
 */
void Application::CheckNewVersion() {
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    // 检查是否有新的固件版本可用
    ota_.SetPostData(board.GetJson());

    while (true) {
        if (ota_.CheckVersion()) {
            if (ota_.HasNewVersion()) {
                // 等待聊天状态变为空闲
                do {
                    vTaskDelay(pdMS_TO_TICKS(3000));
                } while (GetChatState() != kChatStateIdle);

                // 设置聊天状态为升级中
                SetChatState(kChatStateUpgrading);
                
                // 设置显示图标和状态
                display->SetIcon(FONT_AWESOME_DOWNLOAD);
                display->SetStatus("新版本 " + ota_.GetFirmwareVersion());

                // 预先关闭音频输出，避免升级过程有音频操作
                board.GetAudioCodec()->EnableOutput(false);

                // 开始升级，并更新显示状态
                ota_.StartUpgrade([display](int progress, size_t speed) {
                    char buffer[64];
                    snprintf(buffer, sizeof(buffer), "%d%% %zuKB/s", progress, speed / 1024);
                    display->SetStatus(buffer);
                });

                // 如果升级成功，设备将重启，不会执行到这里
                // If upgrade success, the device will reboot and never reach here
                ESP_LOGI(TAG, "Firmware upgrade failed...");
                SetChatState(kChatStateIdle);
            } else {
                // 标记当前版本有效，并显示通知
                ota_.MarkCurrentVersionValid();
                display->ShowNotification("版本 " + ota_.GetCurrentVersion());
            }
            return;
        }

        // 60秒后再次检查
        // Check again in 60 seconds
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}

/**
 * @brief 显示警告信息
 *
 * 根据传入的标题和消息显示警告信息，并根据特定消息播放相应的音频文件。
 *
 * @param title 警告标题，类型为std::string的右值引用
 * @param message 警告消息，类型为std::string的右值引用
 */
void Application::Alert(const std::string&& title, const std::string&& message) {
    // 输出警告信息
    ESP_LOGW(TAG, "Alert: %s, %s", title.c_str(), message.c_str());

    // 获取显示设备实例
    auto display = Board::GetInstance().GetDisplay();

    // 显示通知信息
    display->ShowNotification(message);

    // 根据消息内容播放不同的音频文件
    if (message == "PIN is not ready") {
        // 播放PIN未就绪的音频文件
        PlayLocalFile(p3_err_pin_start, p3_err_pin_end - p3_err_pin_start);
    } else if (message == "Configuring WiFi") {
        // 播放配置WiFi的音频文件
        PlayLocalFile(p3_err_wificonfig_start, p3_err_wificonfig_end - p3_err_wificonfig_start);
    } else if (message == "Registration denied") {
        // 播放注册被拒绝的音频文件
        PlayLocalFile(p3_err_reg_start, p3_err_reg_end - p3_err_reg_start);
    }
}

/**
 * @brief 播放本地文件
 *
 * 从给定的数据中读取音频数据并将其解码后添加到解码队列中。
 *
 * @param data 音频数据的指针
 * @param size 音频数据的字节大小
 */
void Application::PlayLocalFile(const char* data, size_t size) {
    ESP_LOGI(TAG, "PlayLocalFile: %zu bytes", size);

    // 设置解码采样率为16000
    SetDecodeSampleRate(16000);

    // 遍历文件数据
    for (const char* p = data; p < data + size; ) {
        auto p3 = (BinaryProtocol3*)p;
        p += sizeof(BinaryProtocol3); //跳过4字节的头部，移动指针到负载数据的开始位置

        auto payload_size = ntohs(p3->payload_size);
        std::string opus;
        opus.resize(payload_size);
        memcpy(opus.data(), p3->payload, payload_size);
        p += payload_size; // 跳过负载数据，移动到下一个数据包的开始位置

        // 从这里开始加锁
        std::lock_guard<std::mutex> lock(mutex_);
        audio_decode_queue_.emplace_back(std::move(opus)); 
        // 这里结束作用域，自动解锁
    }
}

/**
 * @brief 切换聊天状态
 *
 * 根据当前聊天状态执行相应的操作。
 *
 * 如果当前状态为空闲，则尝试打开音频通道并开始监听；
 * 如果当前状态为正在说话，则中止说话；
 * 如果当前状态为正在监听，则关闭音频通道。
 */
void Application::ToggleChatState() {
    // 调度一个异步任务
    Schedule([this]() {
        // 如果当前聊天状态为空闲
        if (chat_state_ == kChatStateIdle) {
            // 设置聊天状态为连接中
            SetChatState(kChatStateConnecting);
            // 尝试打开音频通道
            if (!protocol_->OpenAudioChannel()) {
                // 如果打开音频通道失败
                ESP_LOGE(TAG, "Failed to open audio channel");
                // 设置聊天状态为空闲
                SetChatState(kChatStateIdle);
                // 退出当前任务
                return;
            }

            // 设置持续监听标志为真
            keep_listening_ = true;
            // 发送开始监听命令
            protocol_->SendStartListening(kListeningModeAutoStop);
            // 设置聊天状态为监听中
            SetChatState(kChatStateListening);
        } else if (chat_state_ == kChatStateSpeaking) { // 如果当前聊天状态为说话中
            // 中止说话
            AbortSpeaking(kAbortReasonNone);
        } else if (chat_state_ == kChatStateListening) { // 如果当前聊天状态为监听中
            // 关闭音频通道
            protocol_->CloseAudioChannel();
        }
    });
}

/**
 * @brief 开始监听音频输入
 *
 * 该函数用于启动音频监听功能。
 *
 * 如果当前聊天状态为空闲（kChatStateIdle），则检查音频通道是否已打开。
 * 如果音频通道未打开，则尝试打开音频通道。如果打开失败，则记录错误日志并返回。
 * 如果音频通道已打开或成功打开，则发送开始监听指令，并将聊天状态设置为监听（kChatStateListening）。
 *
 * 如果当前聊天状态为说话（kChatStateSpeaking），则中止当前说话，发送开始监听指令，
 * 并等待扬声器清空缓冲区（等待120毫秒），然后将聊天状态设置为监听（kChatStateListening）。
 */
void Application::StartListening() {
    // 安排一个任务
    Schedule([this]() {
        // 设置监听状态为 false
        keep_listening_ = false;
        if (chat_state_ == kChatStateIdle) { // 检查聊天状态是否为空闲
            // 检查音频通道是否未打开
            if (!protocol_->IsAudioChannelOpened()) {
                // 设置聊天状态为连接中
                SetChatState(kChatStateConnecting);
                // 尝试打开音频通道
                if (!protocol_->OpenAudioChannel()) {
                    // 如果打开失败，将聊天状态设置回空闲
                    SetChatState(kChatStateIdle);
                    // 记录错误日志
                    ESP_LOGE(TAG, "Failed to open audio channel");
                    // 返回，停止执行后续代码
                    return;
                }
            }
            // 发送开始监听指令
            protocol_->SendStartListening(kListeningModeManualStop);
            // 设置聊天状态为监听中
            SetChatState(kChatStateListening);
        } else if (chat_state_ == kChatStateSpeaking) { // 检查聊天状态是否为正在说话
            // 终止说话
            AbortSpeaking(kAbortReasonNone);
            // 发送开始监听指令
            protocol_->SendStartListening(kListeningModeManualStop);
            // 等待扬声器清空缓冲区（待修复）
            vTaskDelay(pdMS_TO_TICKS(120));
            // 设置聊天状态为监听中
            SetChatState(kChatStateListening);
        }
    });
}

/**
 * @brief 停止监听函数
 *
 * 该函数用于停止应用程序的监听功能。
 *
 * 如果当前聊天状态为监听状态，则发送停止监听指令，并将聊天状态设置为空闲状态。
 */
void Application::StopListening() {
    Schedule([this]() {
        if (chat_state_ == kChatStateListening) {
            protocol_->SendStopListening();
            SetChatState(kChatStateIdle);
        }
    });
}

/**
 * @brief 启动应用程序
 *
 * 初始化设备，启动主循环和网络服务，并初始化协议以与服务器进行通信。
 */
void Application::Start() {
    auto& board = Board::GetInstance();
    board.Initialize();

    auto builtin_led = board.GetBuiltinLed();
    builtin_led->SetBlue();
    builtin_led->StartContinuousBlink(100);

    /* Setup the display */
    auto display = board.GetDisplay();

    /* Setup the audio codec */
    auto codec = board.GetAudioCodec();
    opus_decode_sample_rate_ = codec->output_sample_rate();
    opus_decoder_ = opus_decoder_create(opus_decode_sample_rate_, 1, NULL);
    opus_encoder_.Configure(16000, 1, OPUS_FRAME_DURATION_MS);
    if (codec->input_sample_rate() != 16000) {
        input_resampler_.Configure(codec->input_sample_rate(), 16000);
        reference_resampler_.Configure(codec->input_sample_rate(), 16000);
    }
    codec->OnInputReady([this, codec]() {
        BaseType_t higher_priority_task_woken = pdFALSE;
        xEventGroupSetBitsFromISR(event_group_, AUDIO_INPUT_READY_EVENT, &higher_priority_task_woken);
        return higher_priority_task_woken == pdTRUE;
    });
    codec->OnOutputReady([this]() {
        BaseType_t higher_priority_task_woken = pdFALSE;
        xEventGroupSetBitsFromISR(event_group_, AUDIO_OUTPUT_READY_EVENT, &higher_priority_task_woken);
        return higher_priority_task_woken == pdTRUE;
    });
    codec->Start();

    /* Start the main loop */
    xTaskCreate([](void* arg) {
        Application* app = (Application*)arg;
        app->MainLoop();
        vTaskDelete(NULL);
    }, "main_loop", 4096 * 2, this, 2, nullptr);

    /* Wait for the network to be ready */
    board.StartNetwork();

    // Check for new firmware version or get the MQTT broker address
    xTaskCreate([](void* arg) {
        Application* app = (Application*)arg;
        app->CheckNewVersion();
        vTaskDelete(NULL);
    }, "check_new_version", 4096 * 2, this, 1, nullptr);

#if CONFIG_IDF_TARGET_ESP32S3
    audio_processor_.Initialize(codec->input_channels(), codec->input_reference());
    audio_processor_.OnOutput([this](std::vector<int16_t>&& data) {
        background_task_.Schedule([this, data = std::move(data)]() {
            opus_encoder_.Encode(data, [this](const uint8_t* opus, size_t opus_size) {
                Schedule([this, opus = std::string(reinterpret_cast<const char*>(opus), opus_size)]() {
                    protocol_->SendAudio(opus);
                });
            });
        });
    });

    wake_word_detect_.Initialize(codec->input_channels(), codec->input_reference());
    wake_word_detect_.OnVadStateChange([this](bool speaking) {
        Schedule([this, speaking]() {
            auto builtin_led = Board::GetInstance().GetBuiltinLed();
            if (chat_state_ == kChatStateListening) {
                if (speaking) {
                    builtin_led->SetRed(HIGH_BRIGHTNESS);
                } else {
                    builtin_led->SetRed(LOW_BRIGHTNESS);
                }
                builtin_led->TurnOn();
            }
        });
    });

    wake_word_detect_.OnWakeWordDetected([this](const std::string& wake_word) {
        Schedule([this, &wake_word]() {
            if (chat_state_ == kChatStateIdle) {
                SetChatState(kChatStateConnecting);
                wake_word_detect_.EncodeWakeWordData();

                if (!protocol_->OpenAudioChannel()) {
                    ESP_LOGE(TAG, "Failed to open audio channel");
                    SetChatState(kChatStateIdle);
                    wake_word_detect_.StartDetection();
                    return;
                }
                
                std::string opus;
                // Encode and send the wake word data to the server
                while (wake_word_detect_.GetWakeWordOpus(opus)) {
                    protocol_->SendAudio(opus);
                }
                // Set the chat state to wake word detected
                protocol_->SendWakeWordDetected(wake_word);
                ESP_LOGI(TAG, "Wake word detected: %s", wake_word.c_str());
                keep_listening_ = true;
                SetChatState(kChatStateListening);
            } else if (chat_state_ == kChatStateSpeaking) {
                AbortSpeaking(kAbortReasonWakeWordDetected);
            }

            // Resume detection
            wake_word_detect_.StartDetection();
        });
    });
    wake_word_detect_.StartDetection();
#endif

    // Initialize the protocol
    display->SetStatus("初始化协议");
#ifdef CONFIG_CONNECTION_TYPE_WEBSOCKET
    protocol_ = new WebsocketProtocol();
#else
    protocol_ = new MqttProtocol();
#endif
    protocol_->OnNetworkError([this](const std::string& message) {
        Alert("Error", std::move(message));
    });
    protocol_->OnIncomingAudio([this](const std::string& data) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (chat_state_ == kChatStateSpeaking) {
            audio_decode_queue_.emplace_back(std::move(data));
        }
    });
    protocol_->OnAudioChannelOpened([this, codec, &board]() {
        if (protocol_->server_sample_rate() != codec->output_sample_rate()) {
            ESP_LOGW(TAG, "服务器的音频采样率 %d 与设备输出的采样率 %d 不一致，重采样后可能会失真",
                protocol_->server_sample_rate(), codec->output_sample_rate());
        }
        SetDecodeSampleRate(protocol_->server_sample_rate());
        board.SetPowerSaveMode(false);
    });
    protocol_->OnAudioChannelClosed([this, &board]() {
        Schedule([this]() {
            SetChatState(kChatStateIdle);
        });
        board.SetPowerSaveMode(true);
    });
    protocol_->OnIncomingJson([this, display](const cJSON* root) {
        // Parse JSON data
        auto type = cJSON_GetObjectItem(root, "type");
        if (strcmp(type->valuestring, "tts") == 0) {
            auto state = cJSON_GetObjectItem(root, "state");
            if (strcmp(state->valuestring, "start") == 0) {
                Schedule([this]() {
                    aborted_ = false;
                    if (chat_state_ == kChatStateIdle || chat_state_ == kChatStateListening) {
                        SetChatState(kChatStateSpeaking);
                    }
                });
            } else if (strcmp(state->valuestring, "stop") == 0) {
                Schedule([this]() {
                    if (chat_state_ == kChatStateSpeaking) {
                        background_task_.WaitForCompletion();
                        if (keep_listening_) {
                            protocol_->SendStartListening(kListeningModeAutoStop);
                            SetChatState(kChatStateListening);
                        } else {
                            SetChatState(kChatStateIdle);
                        }
                    }
                });
            } else if (strcmp(state->valuestring, "sentence_start") == 0) {
                auto text = cJSON_GetObjectItem(root, "text");
                if (text != NULL) {
                    ESP_LOGI(TAG, "<< %s", text->valuestring);
                    display->SetChatMessage("assistant", text->valuestring);
                }
            }
        } else if (strcmp(type->valuestring, "stt") == 0) {
            auto text = cJSON_GetObjectItem(root, "text");
            if (text != NULL) {
                ESP_LOGI(TAG, ">> %s", text->valuestring);
                display->SetChatMessage("user", text->valuestring);
            }
        } else if (strcmp(type->valuestring, "llm") == 0) {
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (emotion != NULL) {
                display->SetEmotion(emotion->valuestring);
            }
        }
    });

    // Blink the LED to indicate the device is running
    display->SetStatus("待命");
    builtin_led->SetGreen();
    builtin_led->BlinkOnce();

    SetChatState(kChatStateIdle);
}

/**
 * @brief 将回调函数添加到任务队列中并触发调度事件
 *
 * 将传入的回调函数添加到主任务队列中，并触发调度事件以通知调度器有新的任务需要处理。
 *
 * @param callback 待调度的回调函数
 */
void Application::Schedule(std::function<void()> callback) {
    // 加锁
    mutex_.lock();
    // 将回调函数添加到主任务队列中
    main_tasks_.push_back(callback);
    // 解锁
    mutex_.unlock();
    // 设置事件组中的SCHEDULE_EVENT位来通知主循环
    xEventGroupSetBits(event_group_, SCHEDULE_EVENT);
}

/**
 * @brief 主循环函数
 *
 * 该函数是应用程序的主循环，不断等待事件并处理相应的任务。
 *
 * 主循环会持续运行，等待特定的事件发生。如果检测到音频输入或输出事件，则分别调用InputAudio()和OutputAudio()函数处理音频数据。
 * 如果检测到SCHEDULE_EVENT事件，则锁定互斥锁，并将主任务队列中的所有任务移动到临时列表中，然后遍历临时列表并执行每个任务。
 *
 * @return 无返回值
 */
void Application::MainLoop() {
    while (true) {  // 无限循环，持续处理事件
        // 等待以下任意事件发生:
        // - SCHEDULE_EVENT: 有新的任务需要调度
        // - AUDIO_INPUT_READY_EVENT: 音频输入缓冲区准备就绪
        // - AUDIO_OUTPUT_READY_EVENT: 音频输出缓冲区准备就绪
        // pdTRUE: 表示在返回前清除触发的事件位
        // pdFALSE: 表示等待任意一个事件，而不是所有事件
        // portMAX_DELAY: 永久等待，直到有事件发生
        auto bits = xEventGroupWaitBits(event_group_,
            SCHEDULE_EVENT | AUDIO_INPUT_READY_EVENT | AUDIO_OUTPUT_READY_EVENT,
            pdTRUE, pdFALSE, portMAX_DELAY);

        // 检查是否有音频输入事件
        if (bits & AUDIO_INPUT_READY_EVENT) {
            InputAudio();  // 处理音频输入数据
        }

        // 检查是否有音频输出事件
        if (bits & AUDIO_OUTPUT_READY_EVENT) {
            OutputAudio();  // 处理音频输出数据
        }

        // 检查是否有调度事件
        if (bits & SCHEDULE_EVENT) {
            mutex_.lock();  // 加锁保护任务队列
            // 将主任务队列移动到临时列表中
            // 使用 move 避免复制，提高效率
            std::list<std::function<void()>> tasks = std::move(main_tasks_);
            mutex_.unlock();  // 立即解锁，减少锁的持有时间

            // 执行所有待处理的任务
            for (auto& task : tasks) {
                task();
            }
        }
    }
}

/**
 * @brief 重置解码器
 *
 * 该函数用于重置解码器状态，清空音频解码队列，并重新启用音频输出。
 *
 * 具体的步骤如下：
 * 1. 使用互斥锁锁定资源，确保线程安全。
 * 2. 通过 opus_decoder_ctl 函数重置 Opus 解码器的状态。
 * 3. 清空音频解码队列。
 * 4. 更新最后输出时间。
 * 5. 启用音频输出。
 */
void Application::ResetDecoder() {
    // 加锁保护共享资源
    std::lock_guard<std::mutex> lock(mutex_);
    // 重置 Opus 解码器状态
    opus_decoder_ctl(opus_decoder_, OPUS_RESET_STATE);
    // 清空音频解码队列
    audio_decode_queue_.clear();
    // 更新最后输出时间
    last_output_time_ = std::chrono::steady_clock::now();
    // 启用音频输出
    Board::GetInstance().GetAudioCodec()->EnableOutput(true);
}

/**
 * @brief 输出音频数据
 *
 * 该函数负责从音频解码队列中获取音频数据并输出。
 *
 * 首先，获取当前时间，然后获取音频编解码器实例。
 * 如果音频解码队列为空，并且当前时间与上一次输出时间之间的间隔超过最大静默时间（默认为10秒），则禁用音频输出。
 * 如果当前聊天状态为监听状态，则清空音频解码队列并返回。
 * 否则，从音频解码队列中取出音频数据，并解锁互斥锁。
 *
 * 使用后台任务来解码并输出音频数据。如果解码失败，则记录错误日志并返回。
 * 如果解码后的采样率与编解码器输出的采样率不同，则进行重采样。
 * 最后，调用编解码器的输出数据函数来输出音频数据。
 */
void Application::OutputAudio() {
    // 获取当前时间
    auto now = std::chrono::steady_clock::now();

    // 获取音频编解码器实例
    auto codec = Board::GetInstance().GetAudioCodec();

    // 设置最大静音时间
    const int max_silence_seconds = 10;

    // 加锁
    std::unique_lock<std::mutex> lock(mutex_);

    // 如果音频解码队列为空
    if (audio_decode_queue_.empty()) {
        // 如果没有音频数据持续一段时间，则禁用输出
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_output_time_).count();
        if (duration > max_silence_seconds) {
            codec->EnableOutput(false);
        }
        return;
    }

    // 如果处于聊天监听状态，清空音频解码队列并返回
    if (chat_state_ == kChatStateListening) {
        audio_decode_queue_.clear();
        return;
    }

    // 更新最后输出时间
    last_output_time_ = now;

    // 获取并移除队列中的音频数据
    auto opus = std::move(audio_decode_queue_.front());
    audio_decode_queue_.pop_front();

    // 解锁
    lock.unlock();

    // 调度后台任务
    background_task_.Schedule([this, codec, opus = std::move(opus)]() {
        // 如果任务已中止，则直接返回
        if (aborted_) {
            return;
        }

        // 计算帧大小
        int frame_size = opus_decode_sample_rate_ * OPUS_FRAME_DURATION_MS / 1000;
        std::vector<int16_t> pcm(frame_size);

        // 解码音频数据
        int ret = opus_decode(opus_decoder_, (const unsigned char*)opus.data(), opus.size(), pcm.data(), frame_size, 0);
        if (ret < 0) {
            // 解码失败，记录日志并返回
            ESP_LOGE(TAG, "Failed to decode audio, error code: %d", ret);
            return;
        }

        // 如果采样率不同，则进行重采样
        if (opus_decode_sample_rate_ != codec->output_sample_rate()) {
            int target_size = output_resampler_.GetOutputSamples(frame_size);
            std::vector<int16_t> resampled(target_size);
            output_resampler_.Process(pcm.data(), frame_size, resampled.data());
            pcm = std::move(resampled);
        }
        
        // 输出音频数据
        codec->OutputData(pcm);
    });
}

/**
 * @brief 输入音频数据
 *
 * 该函数从音频编解码器获取音频数据，并根据需要对其进行重采样和处理，然后将处理后的数据传递给音频处理器或唤醒词检测器。
 */
void Application::InputAudio() {
    // 获取音频编解码器实例
    auto codec = Board::GetInstance().GetAudioCodec();
    std::vector<int16_t> data;
    // 从编解码器获取输入数据，如果获取失败则返回
    if (!codec->InputData(data)) {
        return;
    }

    // 如果输入采样率不是16kHz，需要进行重采样处理
    if (codec->input_sample_rate() != 16000) {
        // 处理双通道(立体声)输入
        if (codec->input_channels() == 2) { // 分离麦克风通道和参考通道
            auto mic_channel = std::vector<int16_t>(data.size() / 2);
            auto reference_channel = std::vector<int16_t>(data.size() / 2);
            // 交错的音频数据分离到两个通道
            for (size_t i = 0, j = 0; i < mic_channel.size(); ++i, j += 2) {
                mic_channel[i] = data[j];         // 麦克风数据在偶数位
                reference_channel[i] = data[j + 1]; // 参考数据在奇数位
            }

            // 为重采样后的数据创建缓冲区
            auto resampled_mic = std::vector<int16_t>(input_resampler_.GetOutputSamples(mic_channel.size()));
            auto resampled_reference = std::vector<int16_t>(reference_resampler_.GetOutputSamples(reference_channel.size()));
            
            // 对两个通道分别进行重采样
            input_resampler_.Process(mic_channel.data(), mic_channel.size(), resampled_mic.data());
            reference_resampler_.Process(reference_channel.data(), reference_channel.size(), resampled_reference.data());

            // 重新组合重采样后的数据
            data.resize(resampled_mic.size() + resampled_reference.size());
            for (size_t i = 0, j = 0; i < resampled_mic.size(); ++i, j += 2) {
                data[j] = resampled_mic[i];       // 重采样后的麦克风数据
                data[j + 1] = resampled_reference[i]; // 重采样后的参考数据
            }
        } else { // 处理单通道(单声道)输入
            // 创建重采样后的数据缓冲区
            auto resampled = std::vector<int16_t>(input_resampler_.GetOutputSamples(data.size()));
            // 进行重采样
            input_resampler_.Process(data.data(), data.size(), resampled.data());
            // 移动重采样后的数据到原始数据向量
            data = std::move(resampled);
        }
    }
    
    // ESP32S3 特定的处理逻辑
#if CONFIG_IDF_TARGET_ESP32S3
    // 如果音频处理器正在运行，输入数据进行处理
    if (audio_processor_.IsRunning()) {
        audio_processor_.Input(data);
    }
    // 如果唤醒词检测正在运行，输入数据进行检测
    if (wake_word_detect_.IsDetectionRunning()) {
        wake_word_detect_.Feed(data);
    }
#else
    // 非ESP32S3设备的处理逻辑
    // 如果当前状态是监听状态，则进行音频编码和发送
    if (chat_state_ == kChatStateListening) {
        // 在后台任务中进行音频编码
        background_task_.Schedule([this, data = std::move(data)]() {
            // 使用Opus编码器编码音频数据
            opus_encoder_.Encode(data, [this](const uint8_t* opus, size_t opus_size) {
                // 将编码后的数据调度到主线程发送
                Schedule([this, opus = std::string(reinterpret_cast<const char*>(opus), opus_size)]() {
                    protocol_->SendAudio(opus);
                });
            });
        });
    }
#endif
}

/**
 * @brief 终止当前语音播放
 *
 * 调用此函数将终止当前正在进行的语音播放。
 *
 * @param reason 终止原因，表示为什么需要终止语音播放
 */
void Application::AbortSpeaking(AbortReason reason) {
    ESP_LOGI(TAG, "Abort speaking");
    aborted_ = true;
    protocol_->SendAbortSpeaking(reason);
}

/**
 * @brief 设置聊天状态
 *
 * 根据传入的状态更新聊天状态，并相应地更新设备显示和LED状态。
 *
 * @param state 聊天状态，可以是以下值之一：
 *        - kChatStateUnknown: 未知状态
 *        - kChatStateIdle: 空闲状态
 *        - kChatStateConnecting: 连接中
 *        - kChatStateListening: 聆听中
 *        - kChatStateSpeaking: 说话中
 *        - kChatStateUpgrading: 升级中
 *        - kChatStateInvalid: 无效状态
 */
void Application::SetChatState(ChatState state) {
    const char* state_str[] = {
        "unknown",
        "idle",
        "connecting",
        "listening",
        "speaking",
        "upgrading",
        "invalid_state"
    };
    if (chat_state_ == state) {
        // No need to update the state
        return;
    }

    chat_state_ = state;
    ESP_LOGI(TAG, "STATE: %s", state_str[chat_state_]);
    // The state is changed, wait for all background tasks to finish
    background_task_.WaitForCompletion();

    auto display = Board::GetInstance().GetDisplay();
    auto builtin_led = Board::GetInstance().GetBuiltinLed();
    switch (state) {
        case kChatStateUnknown:
        case kChatStateIdle:
            builtin_led->TurnOff();
            display->SetStatus("待命");
            display->SetEmotion("neutral");
#ifdef CONFIG_IDF_TARGET_ESP32S3
            audio_processor_.Stop();
#endif
            break;
        case kChatStateConnecting:
            builtin_led->SetBlue();
            builtin_led->TurnOn();
            display->SetStatus("连接中...");
            break;
        case kChatStateListening:
            builtin_led->SetRed();
            builtin_led->TurnOn();
            display->SetStatus("聆听中...");
            display->SetEmotion("neutral");
            ResetDecoder();
            opus_encoder_.ResetState();
#if CONFIG_IDF_TARGET_ESP32S3
            audio_processor_.Start();
#endif
            break;
        case kChatStateSpeaking:
            builtin_led->SetGreen();
            builtin_led->TurnOn();
            display->SetStatus("说话中...");
            ResetDecoder();
#if CONFIG_IDF_TARGET_ESP32S3
            audio_processor_.Stop();
#endif
            break;
        case kChatStateUpgrading:
            builtin_led->SetGreen();
            builtin_led->StartContinuousBlink(100);
            break;
        default:
            ESP_LOGE(TAG, "Invalid chat state: %d", chat_state_);
            return;
    }
}

/**
 * @brief 创建 Opus 解码器并配置重采样器
 *
 * 该函数首先使用指定的采样率创建一个 Opus 解码器实例，
 * 然后获取当前音频编解码器的实例，并检查 Opus 解码器的采样率是否与音频编解码器的输出采样率一致。
 * 如果不一致，则使用重采样器将音频从 Opus 解码器的采样率重采样到音频编解码器的输出采样率。
 *
 * @param opus_decode_sample_rate_ Opus 解码器的采样率
 * @param TAG 日志标签
 */
void Application::SetDecodeSampleRate(int sample_rate) {
    if (opus_decode_sample_rate_ == sample_rate) {
        return;
    }

    opus_decoder_destroy(opus_decoder_);
    opus_decode_sample_rate_ = sample_rate;
    opus_decoder_ = opus_decoder_create(opus_decode_sample_rate_, 1, NULL);

    auto codec = Board::GetInstance().GetAudioCodec();
    if (opus_decode_sample_rate_ != codec->output_sample_rate()) {
        ESP_LOGI(TAG, "Resampling audio from %d to %d", opus_decode_sample_rate_, codec->output_sample_rate());
        output_resampler_.Configure(opus_decode_sample_rate_, codec->output_sample_rate());
    }
}
