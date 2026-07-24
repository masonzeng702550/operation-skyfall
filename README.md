# Operation Skyfall

無人機通訊鏈路的中間人攻擊靶場。

一架四旋翼正在執行已核准的測繪任務，地面站看著它。你不在飛機上，也不在地面站裡 ——
**你在它們中間。你不能命令它做任何事，只能修改它聽到的話，以及它說出去的話。**

全部在 Docker 內執行，**不涉及任何實體無人機或射頻發射**。

---

## 為什麼是中間人

「送一道指令讓無人機降落」證明的只是協定沒有身分驗證，選手學到的是 pymavlink 的 API 用法。
真正的問題是另一件事：

> 當通訊鏈路的完整性被破壞，地面操作者失去的不只是控制權，
> 更是**知道自己失去控制權**的能力。

所以這裡每一題的 flag 條件，都要求造成「真實狀態」與「操作者認知」的分歧。
**只騙一邊不算解題。**

### 這不是假想的威脅

| 事件 | 發生了什麼 |
|---|---|
| **RQ-170（伊朗，2011）** | 切斷通訊鏈路後竄改 GPS，讓無人機**以為自己正降落回基地**，實際降落在伊朗 |
| **DJI DroneID / AeroScope（烏克蘭）** | 未加密廣播飛手座標，被用來定位並攻擊操作者，**已造成實際傷亡** |

第一個是本靶場 Q3 的原型：飛機忠實執行任務，只是它相信的世界是假的。
第二個是 Q1 的原型：竊聽是免費的，而後果是致命的。

---

## 架構

```
   docker network: dronenet
   ┌────────────────────────────────────────────────────────────────┐
   │                                                                │
   │  [ gcs ]  ◀── udp ──▶  [ mitm ]  ◀── udp ──▶  [ sitl ]          │
   │  合法操作者              ▲ tcp:14580           ArduCopter SITL   │
   │  跑 AUTO 航線             │                          │           │
   │  每 60 秒重傳任務          │  你在這裡                 │           │
   │  回報「它相信的狀態」        │                          │           │
   │      │                                                │        │
   │      │ belief                              ground truth│        │
   │      ▼                                                 ▼        │
   │  ┌──────────────────────────────────────────────────────────┐  │
   │  │  watcher — 比對「認知」與「真值」，分歧才發 flag              │  │
   │  └──────────────────────────────────────────────────────────┘  │
   │      │                                                         │
   │      ▼                                                         │
   │  [ viz ] :8080  雙視圖戰情台  OPERATOR VIEW ‖ GROUND TRUTH       │
   └────────────────────────────────────────────────────────────────┘
```

**安全不變式**：flag 只存在於 `watcher` 的環境變數中，而 `watcher` 從 `sitl`
**直接旁路**取得真值，不經過任何選手可竄改的路徑。
選手就算在 `sitl`、`mitm`、`gcs`、`viz` 任何一個容器內取得 shell 也拿不到 flag。

---

## 啟動

```bash
cp example.env .env && $EDITOR .env

# 中間人題目（q2 / q3）
docker compose --profile mitm up -d --build

# 暖身題（q0）— 不啟動地面站與中繼
docker compose up -d --build
```

首次 build 會編譯 ArduPilot（10–20 分鐘）。之後啟動只要幾秒。

> 模擬器的 `udpin` 輸出只記得**一個**回送位址，所以中繼與 q0 的直連選手
> 不能共用它。`mitm` 與 `gcs` 因此掛在 `mitm` profile 下。

**容器啟動後需等 60–90 秒**（EKF 初始化 + GPS 3D fix），地面站才會開始飛行。

```bash
curl -s http://127.0.0.1:8080/api/state | jq '.truth.connected, .belief.link_ok'
```

戰情台：<http://127.0.0.1:8080>

---

## 中間人接口

`mitm` 服務在 TCP `14580` 開了一個 tap。沒有人接上時它是透明的，任務正常飛行。
**一旦你接上，鏈路就是你的** —— 每個 datagram 都交給你，而且**只有你送回來的才會被轉發**。

這個不對稱是刻意的。接上之後不動作，等於切斷了鏈路，操作者會立刻發現。
要維持隱形，你必須把不打算竄改的東西原封不動轉發回去，並維持鏈路預期的節奏。

### Wire format

```
+--------+------------------+-------------------+
| dir:1  | length:2 (big-e) | MAVLink datagram  |
+--------+------------------+-------------------+

dir 0x00 = GCS → UAV   （上行：指令、任務上傳、RTCM 修正）
dir 0x01 = UAV → GCS   （下行：遙測、ack、狀態訊息）
```

長度為 0 的 frame 代表丟棄。你可以在任何時候注入 frame，不必對應到任何收到的東西。

最小可行的攻擊者是一個純轉發器：

```python
from tap import Tap, UPLINK, DOWNLINK

tap = Tap("127.0.0.1", 14580)
for direction, data in tap.frames():
    tap.send(direction, data)      # 什麼都不改 —— 任務照常飛
```

`solution/tap.py` 提供了 frame 收發與 MAVLink 重新封裝（含 CRC）的輔助。

---

## 題目

環境變數 `CHALLENGE` 決定判分邏輯。

### `q0` — Hello, Drone（暖身）

不需要中間人位置。直接連 `udp:14550`，切 GUIDED、arm、起飛、飛到指定座標、降落。
全程零憑證。**配分應明顯低於其他題** —— 它的存在是為了降低入門門檻。

判分：曾經 armed、最高爬升 ≥ 10 m、落點距目標 ≤ 20 m、落地高度 ≤ 2 m。

### `q2` — Middleman（竄改）

地面站每 60 秒重新上傳一次任務。攔截它，改一個數字。

`param1` 以 32-bit float 傳輸，在飛控端對某些指令會被直接轉成 `uint16` 而**不做範圍檢查**。
給它一個裝不下的值，轉換就會出錯 —— 而飛控沒有處理這個錯誤，行程當場死亡，**在空中**。

判分：機體在 5 m 以上且 armed 時，真值遙測連續中斷 ≥ 5 秒。

```bash
python3 solution/q2_middleman.py --host <host> --port 14580
```

> **時機**：地面站在起飛**前**也會上傳一次任務。太早接上並下毒會讓飛控死在地面，
> 飛機永遠不會起飛，判分條件（空中失聯）也就無法成立，整個 stack 需要重置。
> 先看戰情台確認飛機已在 `AUTO` 巡航，再接上 tap 等下一次重傳。

### `q3` — Ghost in the Machine（偽造）

同時做兩件方向相反的事。

**對飛機**：接管它。壓掉操作者的模式切換讓它無法被拉回 AUTO，切 GUIDED，導向 rogue LZ。

**對操作者**：說謊。把每一則位置回報改寫成測繪框上的合理巡航點，
把 heartbeat 的模式改回 AUTO，並維持完全相同的節奏 ——
地面站判斷鏈路健康的唯一依據就是 heartbeat 有沒有到，
而 **MAVLink v2 的 sequence number 從未被拿去跟任何東西比對**，
所以我們偽造的任何東西都不會引起抱怨。

判分：真實位置在 rogue LZ 半徑內、操作者認知位置仍在授權框內、
操作者仍認為鏈路正常，**三者同時成立且持續 ≥ 10 秒**。

```bash
python3 solution/q3_ghost.py --host <host> --port 14580
```

---

## 驗證

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -r solution/requirements.txt
cd solution && python3 q3_ghost.py --host 127.0.0.1 --port 14580
```

解題後 flag 會出現在戰情台右側，並印在 `docker compose logs watcher`。

> `mitm` 只接受一個 attacker。要換解題程式時先斷開前一個。

---

## 檔案

| 路徑 | 用途 |
|---|---|
| `sitl/` | ArduCopter SITL，pin 至 `Copter-4.6.3` |
| `mitm/relay.py` | 中間人位置：UDP↔UDP 轉送 + TCP tap |
| `gcs/gcs.py` | 合法操作者：飛航線、重傳任務、回報 belief |
| `watcher/flag_watcher.py` | 判分（唯一持有 flag）；真值旁路 + belief 端點 |
| `viz/` | 雙視圖戰情台，零外部相依、可離線 |
| `solution/` | 官方解與 tap 客戶端 |

---

## 部署

每隊一份獨立 stack，隊伍之間網路不可互達：

```bash
for n in $(seq -w 1 20); do
  TAP_PORT=$((14500+10#$n)) \
  VIZ_PORT=$((8000+10#$n)) \
  MAVLINK_PORT=$((14600+10#$n)) \
  FLAG="$(gen_flag team$n)" \
  SOLVE_TOKEN="$(openssl rand -hex 16)" \
  docker compose -p team$n up -d
done
```

- 重置：`docker compose -p teamNN restart sitl gcs watcher`
- 資源：每組 idle ~0.5 core / ~2.0 GB RAM，飛行中 ~1.5 core。
  12 核 / 24 GB 主機建議上限 8 組。
- **UDP 注意**：`q0` 的 MAVLink 埠走 UDP，多數 HTTP-only 的反向代理不轉發。
  中間人題目的 tap 走 TCP，代理友善。

### 版本鎖定

`q2` 依賴未修補的 ArduPilot 樹。**映像必須 pin 到固定 tag，絕不可用 `latest`。**

---

## 已驗證行為

在 arm64 主機上以 `Copter-4.6.3` 實測：

| 項目 | 結果 |
|---|---|
| 中繼透明轉發 | 2129 frames tapped / 2129 injected，零丟包 |
| `q2` — 竄改在途任務 | `param1 0.0 → 1e10` 注入後，飛控在 **17.7 m 空中**行程終止；`arducopter` 從行程表消失，中繼開始 `Connection refused` 空轉。判分於靜默 5.2 s 後發旗 |
| `q3` — 雙向欺騙 | 飛機被開到 rogue LZ（距目標 < 1 m），操作者顯示維持 `AUTO` 與 `link_ok`，分歧 136 m 持續 10.1 s → 發旗 |
| `q0` — 直接指令注入 | 落點距目標 0.6 m，peak 30.0 m → 發旗 |

`q2` 的崩潰機制與架構有關（arm64 的 float→int 是飽和轉換，不會自己 trap），
但 `NAV_WAYPOINT` 搭配 `param1 = 1e10` 在此環境確實致命。
換 branch 或換架構後請重新確認。

## 已知限制

| 限制 | 說明 |
|---|---|
| 冷啟動需 60–90 秒 | EKF 就緒前地面站不會起飛 |
| `mitm` 僅接受單一 attacker | 換解題程式前需先斷線 |
| MAVProxy 不做 output→output 轉發 | flag 無法經遙測鏈路回送，改用內網 HTTP 推送給戰情台 |
| 航跡僅存於 `viz` 記憶體 | 重啟後消失，不影響判分 |
| `viz` 的 `/api/solved` 需正確 token | 即使偽造也只影響自己的畫面，flag 以 watcher log 為準 |

---

## 法規

**本環境為純軟體模擬。**

在台灣，對真實無人機進行 RF 干擾、GPS 欺騙、Wi-Fi deauthentication
違反《遙控無人機管理規則》及《電信管理法》。
本靶場所示技術僅適用於**獲得授權的測試環境**。
