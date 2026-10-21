# 项目长期记忆 — xiaozhi-esp32 自建服务端联调

> 2026-10-21 重建（原 `.workbuddy` 目录被 `git clean` 清掉过，内容按关键事实重写）

## 环境常量
- 开发板：ESP32-S3 面包板 WiFi 版 → `CONFIG_BOARD_TYPE_BREAD_COMPACT_WIFI`，串口 **COM31**
- 设备 MAC `a0:85:e3:e6:53:a4`，client-id `4a9e1bf4-a9be-4ee2-b9a7-cb1662c8cbc6`
- ESP-IDF：`C:\Espressif\frameworks\esp-idf-v5.4.4`（v5.4.4）
- 服务端：`D:\CODE\TEST\ESP32\xiaozhi-esp32-server`，conda 环境 `xiaozhi-server`
  （`D:\ProgramData\anaconda_envs\envs\xiaozhi-server\python.exe`），在 `main/xiaozhi-server` 下 `python -u app.py`
- 端口：8000 WS / 8003 HTTP OTA+视觉 / 8002 manager-api（前缀 `/xiaozhi`）/ 8001 manager-web / 3306 MySQL 5.7(root/root) / 6379 Redis
- 智控台：JDK21 `D:\Java\jdk21`，IDEA 跑 `AdminApplication`；MySQL 库 `xiaozhi_esp32_server`（Liquibase 自动建表）
- 网络会变：`10.1.10.130`（`MM Intelligence_5G 4`）↔ `192.168.31.204`（`Xiaomi_45DF`）

## 三条硬规则
1. **同一 build 目录严禁并发编译**。`xxx.obj.d: Permission denied` 的真因是两个 ninja 抢写，
   先 `Get-Process ninja,ccache` 清理。终端关掉 ≠ ninja 退出。
2. **Git Bash 不能用 `export.sh`**，必须 PowerShell 的 `export.ps1`。
3. **FunASR 模型必须走 ModelScope**，HuggingFace 本机不可达。

## 本地已打的关键补丁（官方 Docker 镜像里没有！）
- `core/providers/tts/edge.py`：替换 `edge_tts.communicate.WSS_URL` 为 v2 端点（修 TTS 403）
- `config/config_loader.py`：`get_config_from_api()` 的 `server` 段补透传 `websocket` + `timezone_offset`
- `core/api/ota_handler.py`：`_get_websocket_url` 空值兜底 + 异常日志
- `core/http_server.py`：同上，判定串统一为 `"你的"`

## OTA / 设备绑定的架构要点（最大坑）
- **两套 OTA**：`8003`(Python) 只下发 ws 地址；`8002`(manager-api) 下发 ws 地址（读 `server.websocket` 参数）
  **并生成 6 位激活码**（`buildActivation()` → Redis `ota:activation:data:<mac>` / `ota:activation:code:<code>`）
- 设备 OTA 指向 8003 时**没有激活码** → 设备播报"没有找到该设备的版本信息"（`receiveAudioHandle.py:133`）
  → 补救：代设备打 `POST 8002/xiaozhi/ota/`（`Device-Id` 与 body 的 `mac_address` 必须一致，body 必带 `application`）
  → **全模块 Docker 部署应让设备 OTA 直接走 8002**
- 智控台模式 `ai_model_config` 全是占位符 `"api_key":"你的api_key"` → LLM 报
  `'ascii' codec can't encode characters in position 7-8`（HTTP 头不能放中文）→ 必须在【模型配置】填真实 key

## 固件侧
- `main/Kconfig.projbuild` 的 `OTA_URL` + `sdkconfig` 的 `CONFIG_OTA_URL` 必须同时改，改完要重编译
- NVS 里 `ota_url` 优先级**高于**编译期值（`ota.cc:43-50`），有值时改 sdkconfig 不生效
- OTA 兼任"服务发现"：不能跳过，否则没有 ws 地址 → fallback 到空 MQTT
- `application.cc:102-118`：失败重试 10 次（10/20/40…秒退避），耗尽后仍继续往下走但连不上

## 配套文档
- `xiaozhi-esp32-server/小智AI-学习进度与下一步计划.md`
- `xiaozhi-esp32-server/小智AI-配置总览与IP变更手册.md` — 换 WiFi/IP 的 SOP
- `xiaozhi-esp32-server/xiaozhi-server-服务器部署SOP.md` — 服务器 Docker 部署（有已知待修正项，见日志）
