# 项目长期记忆 — xiaozhi-esp32 自建服务端联调

## 环境常量
- 本机 WLAN IP：`10.1.10.130`（SSID `MM Intelligence_5G 4`，同 `/24` 网段；设备连的是 `启明`）
- 开发板：ESP32-S3 面包板 WiFi 版 → `CONFIG_BOARD_TYPE_BREAD_COMPACT_WIFI`
- ESP-IDF：`C:\Espressif\frameworks\esp-idf-v5.4.4`，v5.4.4，串口 **COM31**
- 服务端：`D:\CODE\TEST\ESP32\xiaozhi-esp32-server`，conda 环境 `xiaozhi-server`
  （`D:\ProgramData\anaconda_envs\envs\xiaozhi-server\python.exe`），启动 `python -u app.py`
  → :8000 WebSocket `/xiaozhi/v1/`，:8003 HTTP OTA `/xiaozhi/ota/`
- 服务端最终配置优先级：智控台(DB) > `data/.config.yaml` > `config.yaml`
- 智控台（2026-10-13 跑通）：`manager-api`(SpringBoot 3.4.3 + **JDK21 `D:\Java\jdk21`**) :8002 前缀 `/xiaozhi`，
  IDEA 跑 `AdminApplication`；`manager-web`(Vue2) `npm run serve` :8001；MySQL 5.7 root/root @3306，
  库 `xiaozhi_esp32_server`（Liquibase 首启自动建 18 表，**无需手工导 SQL**）；Redis :6379
- 设备 MAC `a0:85:e3:e6:53:a4`，client-id `4a9e1bf4-a9be-4ee2-b9a7-cb1662c8cbc6`

## 配套文档（改动后同步维护）
- `xiaozhi-esp32-server/小智AI-学习进度与下一步计划.md` — 学习进度与分阶段计划
- `xiaozhi-esp32-server/小智AI-配置总览与IP变更手册.md` — **换 WiFi / IP 变更 SOP**（第 5 章是清单）

## 三条硬规则（踩过坑）
1. **同一 build 目录严禁并发编译**。`idf.py build` 报 `xxx.obj.d: Permission denied` 时，
   99% 不是权限问题，而是有两个 ninja 在抢写。先 `Get-Process ninja,ccache` 排查残留进程，
   确认只有一个后才重编。终端关掉 ≠ ninja 退出。
2. **在 Git Bash 里不能用 `export.sh`**，必须用 PowerShell 的 `export.ps1`。
3. **FunASR 模型必须走 ModelScope**（HuggingFace 在本机不可达）：
   `modelscope.snapshot_download('iic/SenseVoiceSmall', local_dir='models/SenseVoiceSmall')`。

## 已打的关键补丁（升级依赖前先确认是否已合入上游）
- `core/providers/tts/edge.py`：导入时替换 `edge_tts.communicate.WSS_URL` 为 v2 端点
  `wss://api.msedgeservices.com/tts/cognitiveservices/websocket/v1?Ocp-Apim-Subscription-Key=...`
  （只改 `constants.WSS_URL` 无效，`communicate.py` 引用的是模块级全局）
- `core/http_server.py:17`：`websocket` 配置若为**非空且不含"你"字**则原样下发 → `data/.config.yaml`
  里必须**注释掉**写死的 websocket/vision_explain，才能走自动探测
- `core/api/ota_handler.py`：① `handle_post` 的 except 已加 `logger.error`；
  ② `_get_websocket_url` 已加**空值兜底**（空串必须走 `get_local_ip()`，否则下发空 URL）
- `config/config_loader.py`：`get_config_from_api()` 的 `server` 段已补透传 `websocket` + `timezone_offset`。
  **根因**：智控台模式**不合并 `config.yaml`**，原实现只透传 5 个 key → `websocket` 丢失 →
  OTA 下发 `"websocket":{"url":""}` → 设备报 `Invalid URI format`
- `core/http_server.py` 的判定串同步统一为 `"你的"`（原为 `"你"`，与 ota_handler 不一致）

## IP 变更速记（详见配置总览手册第 5 章）
1. **只有设备端 `main/Kconfig.projbuild:7` + `sdkconfig:574` 的 OTA_URL 需要改 + 重编译烧录**；
2. 服务端侧全靠 `get_local_ip()` 自动探测 → `data/.config.yaml` 的 websocket/vision_explain **保持注释**即可；
3. 换 WiFi 还需长按 BOOT 重新配网（NVS 里存的是旧 SSID）；
4. 最优解：路由器给电脑 WLAN 网卡做 **DHCP 静态绑定**，IP 永不变，固件永不用重编。

## 设备绑定（激活码）机制
- 6 位验证码由 **manager-api 的 OTA 接口**（`POST http://<IP>:8002/xiaozhi/ota/`，
  在 `DeviceServiceImpl.buildActivation()`）生成，存 Redis `ota:activation:data:<mac>` +
  `ota:activation:code:<code>`；智控台输码 → `POST /device/bind/{agentId}/{code}` → 写 `ai_device`
- **设备 OTA 指向 8003(Python) 时 `buildActivation()` 不会执行** → 拿不到码，
  设备会播报"没有找到该设备的版本信息，请正确配置 OTA地址，然后重新编译固件"
  （来自 `core/handle/receiveAudioHandle.py:133`，因 `conn.bind_code` 为空）
- **补救**：代设备打一次 8002 的 OTA（`Device-Id` 必须与 body 里 `mac_address` 完全一致，
  且 body 必带 `application`），响应 `activation.code` 即验证码

## 设备端改动
- `main/Kconfig.projbuild` 的 `OTA_URL` default + `sdkconfig` 的 `CONFIG_OTA_URL`
  → `http://10.1.10.130:8003/xiaozhi/ota/`
- 验证手段：`strings build/xiaozhi.bin | grep -E "10\.1\.10\.130|api\.tenclass\.net"`
  （必须只剩新地址，无 `api.tenclass.net`）

## 配网日志速查
| 日志 | 含义 |
|---|---|
| `WifiConfigurationAp: Connected to WiFi xxx` | 配网时校验密码成功（`wifi_configuration_ap.cc:664`） |
| `wifi:state: run -> init` / `<ba-del>` | 紧随其后的 `esp_wifi_disconnect()`（:665），正常 |
| 之后应有 `POST /reboot` → `esp_restart()` | 网页点"完成"触发；不点则一直停在配网模式 |

## 已知遗留风险
- Edge TTS v2 端点 **429 限流**（回复越长越容易触发）→ 阶段 E 换国内 TTS
- `ws://` 明文连接：小智 1.6.x 官方 WS 也走非加密，本地调试可接受
