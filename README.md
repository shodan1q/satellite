# "喂！星 · 卫星工坊" -- 你的第一台真实联动机械臂

> ASSEMBLE. POINT. MOVE. -- 在手机里搭一颗卫星，举起手，让真的机械臂跟着你的胳膊动。

---

## 我们的题目

"喂！星 · 卫星工坊"是 **HarmonyOS 手机原生 app**，把"教学型 AR 卫星拼装"和"实物 7 自由度机械臂遥控"装在了同一台手机里。

用户在手机里用三种自然交互方式实时控制一台**真实在桌面上转动的机械臂**：

- **手臂跟随**：摄像头识别用户右臂的肩 / 肘 / 腕，机械臂末端按本体偏移跟着动
- **倾斜遥控**：手机怎么倾斜，机械臂末端就怎么旋转
- **双摇杆**：左摇杆控制 roll/pitch，右摇杆控制 yaw，作为精细调节

同时用户在 AR 工坊里能从零开始拼装属于自己的卫星 -- 平台舱、太阳翼、天线、燃料舱、有效载荷一件一件叠上去，配合卫星科普、轨道演示和虚拟同伴形成完整的太空科普 + 实操闭环。

### 系统架构

```
"喂！星 · 卫星工坊"
  |
  |-- 鸿蒙手机端（satellite app）
  |     |
  |     |-- AR 工坊 ........... three.js 渲染卫星 + 摄像头透视
  |     |                       12 类组件实时拼装、爆炸图、ISS 场景
  |     |
  |     |-- 手势模式 ........... MediaPipe Hands 识别拇指/食指距离
  |     |                       手部位置驱动卫星爆炸度
  |     |
  |     |-- 手臂模式 ⭐ ........ MediaPipe Pose 识别右臂关节
  |     |                       发送世界坐标到机械臂
  |     |
  |     |-- 姿态模式 ⭐ ........ DeviceMotion 陀螺仪
  |     |                       手机倾角直接驱动末端 6-DOF 姿态
  |     |
  |     |-- 双摇杆 ............. 触控精细调节
  |     |
  |     +-- 急停按钮 ........... 顶部红色 -- 立即冻结机械臂
  |
  +-- 桌面机械臂（woan a1_r 7-DOF）
        |
        +-- teach_monitor.py ... WebSocket 服务 + 三层安全闸
              + 看门狗 1.0s
              + 跳变检测 20cm/帧
              + 急停 latch
```

## 为什么我们做这件事

太空科普做得太浅，机器人遥控做得太严肃 -- 我们想试试把它们融在一起：

让一个对编程一窍不通的初中生，也能**举起手控制一台真的机械臂**。

让科普不再是看视频，而是**自己组装出一颗能动的卫星，然后亲手驱动一台机器与之联动**。

48 小时，从一个想法到一台机器人能听懂手机指令的端到端 demo。

## 我们实现了哪些功能

### 鸿蒙手机端

**AR 工坊**：摄像头透视 + 程序化卫星几何 + Gaussian Splat 国际空间站背景。

- 12 类卫星组件（平台舱、太阳翼、高增益天线、有效载荷、推进器、燃料舱…）实时拼装
- 一键爆炸图，方便看清每个组件的位置和功能
- 球形金箔本体 + 8 经线 + 4 RCS 喷嘴的精细几何
- ISS Gaussian Splat 场景作为太空背景（120MB，Git LFS 托管）

**手臂模式**：MediaPipe Pose（lite/full）实时识别右臂三个关键点。

- 右肩 / 右肘 / 右腕的世界坐标（米制）→ 机械臂 TCP 偏移
- 上半身脱离画面时画面 HUD 提示，但本地仍持续重连
- 通过 ArkTS↔WebView 的 JS 桥转发命令到原生 WebSocket

**姿态模式**：DeviceMotion 陀螺仪 → 末端绝对姿态映射。

- 启动时捕获手机当前倾角作为零位
- 之后倾斜 / 翻转手机直接驱动末端 roll / pitch / yaw
- 30Hz 平滑流，0.5° 死区

**手势模式**：MediaPipe Hands 控制 AR 卫星的爆炸度和拖动。

**多端同步**：DeviceManagerKit 跨手机 / 平板 / 手表同步当前火箭设计与控制状态。

### 桌面机械臂端

**teach_monitor.py**：跑在机器人主机上的 Python WebSocket 服务（端口 8765）。

- 接收手机端 `phone_frame` / `phone_orient` / `phone_rot` / `estop` 命令
- 单线程执行器消费"最新一帧目标位姿"，避免 movel 队列堆叠
- **三层安全闸**：1s 看门狗 / 20cm 跳变检测 / 急停 latch
- 实时彩色终端 -- 状态栏显示 WS 客户端数 / 帧计数 / TCP 位置 / 关节角
- 终端按 `i` 自动 movej 到 ready 位姿（`[0, 0.4, 0, -0.8, 0, 0.4, 0]`），脱离零位奇点

### 视觉感知工具（实验性，Mac 端）

`tools/vision/` 用 Insta360 X4 实时追踪机械臂位姿，作为下一阶段闭环视觉控制的基础：

- `aruco_generate.py` -- 生成 ArUco 二维码贴片
- `arm_pose_tracker.py` -- 标记 6-DOF 末端位姿
- `arm_roi_tracker.py` -- 框选追踪（无需打印标记）
- `arm_motion_detector.py` -- MOG2 运动事件检测

## 技术栈

| 层 | 选型 |
|---|---|
| 手机系统 | HarmonyOS NEXT · ArkTS · ArkUI · ArkWeb |
| AR 渲染 | three.js（程序化几何 + Gaussian Splat） |
| 视觉识别 | MediaPipe Hands / Pose（lite & full）-- 全部 client-side 推理 |
| 网络 | 原生 `@ohos.net.webSocket` + `@ohos.net.connection` 强制 WiFi 路由 |
| 多端同步 | DeviceManagerKit |
| 机器人 | Python 3.12 + websockets + scipy + WoanArm SDK |
| 视觉感知 | OpenCV (CSRT / KCF / ArUco / MOG2) |

## 主要技术挑战 & 解法

| 挑战 | 解法 |
|---|---|
| **ArkWeb 屏蔽 cleartext ws://** | 把 WebSocket 从 WebView 内移到 ArkTS 原生层，通过 JS Proxy 桥接 |
| **HarmonyOS 默认走蜂窝路由** | `@ohos.net.connection.setAppNet` 强制绑定 WiFi NetHandle |
| **WSL2 + iPhone 热点环境** | `network_security_config` 白名单 + Windows Firewall 入站规则文档化 |
| **机械臂在零位奇点** | 启动时按 `i` 或 `phone_ready` WS 命令，自动 movej 到非奇点位姿 |
| **MediaPipe lite 模型 visibility 偏低** | 放宽阈值 + 落到 2D 像素坐标 fallback |

## 跑起来

### 手机 app

```
DevEco Studio 打开 satellite/ → 部署到 HarmonyOS NEXT 设备
```

首次启动授权摄像头 / 网络 / 加速度计 / 蓝牙 / DataSync 即可。

### 机械臂后端（如有 woan a1_r）

```bash
~/.local/bin/micromamba run -n woanarm python tools/robot/teach_monitor.py
```

启动后**按 `i` 把机械臂送到 ready 位姿**，然后在手机里打开 **手臂** 或 **姿态** 模式即可联动。

### 修改机器人 IP（运行时）

App 里**长按 topBar 的 ROBOT 小圆点** → 弹出对话框输入新 URL → 立即重连，URL 持久化保存。

## 已知限制

- iPhone 个人热点会拦客户端间 TCP（建议用 HarmonyOS 手机做热点解决）
- WSL2 + Windows 主机时 Windows Firewall 默认拦 8765（一行 PowerShell 放行）
- ArkWeb 不支持 cleartext WebSocket，所以机械臂 WS 走原生 `@ohos.net.webSocket`

## 未来规划

| 阶段 | 内容 |
|------|------|
| 第一阶段（当前） | 手机端三模式 + 机械臂 WS 联动 + AR 工坊 |
| 第二阶段 | 视觉闭环（Insta360 X4 实时位姿反馈，IK 失败自动恢复） |
| 第三阶段 | 自然语言遥控（小艺/AI对话 + 任务分解 → 多步动作序列） |
| 第四阶段 | 多机协同（一手机控制多臂，或一机器人接收多人姿态融合） |

## License

MIT

## 致谢

- **AttraX Spring Hackathon** 主办方
- **HarmonyOS** 生态与原生 NetStack
- **MediaPipe** by Google
- **three.js** community
- **OpenWonderLabs / WoanArm** SDK
- **影石 Insta360** 摄像设备
- Lucide icons

---

**参赛赛事**：AttraX Spring Hackathon
**Tag**：`#AttraX_Spring_Hackathon`
