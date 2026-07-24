# Operation Skyfall

無人機通訊鏈路的中間人攻擊靶場。

一架四旋翼正在執行已核准的測繪任務,地面站看著它。你不在飛機上,也不在地面站裡 ——
**你在它們中間。你不能命令它做任何事,只能修改它聽到的話,以及它說出去的話。**

全部在 Docker 內執行,**不涉及任何實體無人機或射頻發射**。

---

## 目標

> 讓地面站的螢幕與飛機的實際位置分開,並維持住。

判分程式同時看兩份資料:地面站**相信**的位置(只由鏈路上到達的資料構成),
以及模擬器**實際**的位置(走選手碰不到的旁路)。四項同時成立且**持續 10 秒**才發 flag:

| 條件 | 為什麼 |
|---|---|
| 機體 armed 且高於 5 m | 停在地上偽造遙測不算 |
| 真值與認知相距 ≥ 50 m | 遠超 GPS 雜訊,也不是畫面延遲能解釋的 |
| 地面站仍認為鏈路正常 | 你被發現就不算 |
| 以上維持 10 秒 | 閃一下是雜訊,不是控制 |

**這兩半缺一不可。** 把飛機開走但操作者看得到 —— 不算。偽造遙測但飛機沒動 —— 也不算。
這正是中間人與單純指令注入的分野。

### 這不是假想的威脅

| 事件 | 發生了什麼 |
|---|---|
| **RQ-170(伊朗,2011)** | 切斷通訊鏈路後竄改 GPS,讓無人機**以為自己正降落回基地**,實際降落在伊朗 |
| **DJI DroneID / AeroScope(烏克蘭)** | 未加密廣播飛手座標,被用來定位並攻擊操作者,**已造成實際傷亡** |

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
   │  │  watcher — 比對「認知」與「真值」,分歧才發 flag              │  │
   │  └──────────────────────────────────────────────────────────┘  │
   │      │                                                         │
   │      ▼                                                         │
   │  [ viz ] :8080  雙視圖戰情台  OPERATOR VIEW ‖ GROUND TRUTH       │
   └────────────────────────────────────────────────────────────────┘
```

**安全不變式**:

- flag 只存在於 `watcher` 的環境變數中,而 `watcher` 從 `sitl` **直接旁路**取真值,
  不經過任何選手可竄改的路徑。選手在 `sitl` / `mitm` / `gcs` / `viz` 任何一個容器內
  取得 shell 都拿不到 flag。
- **模擬器的 MAVLink 埠不對外開放。** 直接跟飛機講話會繞過整題的重點,
  所以通往它的唯一路徑經過中繼 —— 也就是經過你。

---

## 啟動

```bash
cp example.env .env && $EDITOR .env
docker compose up -d --build
```

首次 build 會編譯 ArduPilot(10–20 分鐘)。之後啟動只要幾秒。

**容器啟動後需等 60–90 秒**(EKF 初始化 + GPS 3D fix),地面站才會開始飛行。

```bash
curl -s http://127.0.0.1:8080/api/state | jq '.truth.connected, .belief.link_ok'
```

戰情台:<http://127.0.0.1:8080> —— 底部的 DIVERGENCE 條就是你的計分板。

---

## 中間人接口

`mitm` 服務在 TCP `14580` 開了一個 tap。沒有人接上時它是透明的,任務正常飛行。
**一旦你接上,鏈路就是你的** —— 每個 datagram 都交給你,而且**只有你送回來的才會被轉發**。

這個不對稱是刻意的。接上之後不動作,等於切斷了鏈路,操作者會立刻發現。
要維持隱形,你必須把不打算竄改的東西原封不動轉發回去,並維持鏈路預期的節奏。

### Wire format

```
+--------+------------------+-------------------+
| dir:1  | length:2 (big-e) | MAVLink datagram  |
+--------+------------------+-------------------+

dir 0x00 = GCS → UAV   (上行:指令、任務上傳、RTCM 修正)
dir 0x01 = UAV → GCS   (下行:遙測、ack、狀態訊息)
```

長度為 0 的 frame 代表丟棄。你可以在任何時候注入 frame,不必對應到任何收到的東西。

最小可行的攻擊者是一個純轉發器 —— 先確認自己隱形,再開始動手腳:

```python
from tap import Tap, UPLINK, DOWNLINK

tap = Tap("127.0.0.1", 14580)
for direction, data in tap.frames():
    tap.send(direction, data)      # 什麼都不改 —— 任務照常飛
```

`solution/tap.py` 提供 frame 收發與 MAVLink 重新封裝(含 CRC)的輔助:

```python
msgs = tap.decode(direction, data)        # bytes → MAVLink 訊息
raw  = tap.encode(direction, msgs)        # 訊息 → bytes(重算 CRC、保留原 header)
raw  = tap.build(direction, msg, seq=n)   # 自己造的訊息 → bytes
```

不需要改的封包請轉發**原始 bytes**,不要重新編碼 —— 看不懂的東西不要動。

---

## 驗證(官方解)

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -r solution/requirements.txt
cd solution && python3 solve.py --host 127.0.0.1 --port 14580
```

解題後 flag 會在戰情台上噴出,並印在 `docker compose logs watcher`。

> `mitm` 只接受一個 attacker。要換解題程式時先斷開前一個。

---

## 這個位置還能做什麼

拿到中間人位置之後,竄改任務上傳不只能改航點。ArduPilot 的
`mavlink_int_to_mission_cmd()` 對多數指令是直接 float→int 賦值、**沒有邊界檢查**
(只有 `NAV_LOITER_TURNS` 與 Plane 的 `NAV_WAYPOINT` 有 `MIN()`)。
在地面站每分鐘一次的例行重傳中改掉一個 `param1`:

```python
for m in tap.decode(UPLINK, data):
    if m.get_type() == "MISSION_ITEM_INT":
        m.param1 = 1e10
```

實測(`Copter-4.6.3`,arm64)飛控行程會在空中終止 —— `arducopter` 從行程表消失,
中繼開始 `Connection refused` 空轉。這不是本題的判分條件,但它說明了同一個位置
能造成什麼。崩潰機制與架構有關(arm64 的 float→int 是飽和轉換),換 branch
或換架構請重新確認。

---

## 檔案

| 路徑 | 用途 |
|---|---|
| `sitl/` | ArduCopter SITL,pin 至 `Copter-4.6.3` |
| `mitm/relay.py` | 中間人位置:UDP↔UDP 轉送 + TCP tap |
| `gcs/gcs.py` | 合法操作者:飛航線、重傳任務、回報 belief |
| `watcher/flag_watcher.py` | 判分(唯一持有 flag);真值旁路 + belief 端點 |
| `viz/` | 雙視圖戰情台,零外部相依、可離線 |
| `solution/` | 官方解與 tap 客戶端 |

---

## 部署

每隊一份獨立 stack,隊伍之間網路不可互達:

```bash
for n in $(seq -w 1 20); do
  TAP_PORT=$((14500+10#$n)) \
  VIZ_PORT=$((8000+10#$n)) \
  FLAG_MODE=dynamic \
  FLAG_SECRET="$THJCC_SECRET" \
  TEAM_ID="team$n" \
  SOLVE_TOKEN="$(openssl rand -hex 16)" \
  docker compose -p team$n up -d
done
```

- 重置:`docker compose -p teamNN restart sitl gcs watcher`
- 資源:每組 idle ~0.5 core / ~2.0 GB RAM,飛行中 ~1.5 core。
  12 核 / 24 GB 主機建議上限 8 組。
- **對外只有 TCP**(tap 與戰情台),代理友善,不需要轉發 UDP。

---

## 已知限制

| 限制 | 說明 |
|---|---|
| 冷啟動需 60–90 秒 | EKF 就緒前地面站不會起飛 |
| `mitm` 僅接受單一 attacker | 換解題程式前需先斷線 |
| MAVProxy 不做 output→output 轉發 | flag 無法經遙測鏈路回送,改用內網 HTTP 推送給戰情台 |
| 航跡僅存於 `viz` 記憶體 | 重啟後消失,不影響判分 |
| `viz` 的 `/api/solved` 需正確 token | 即使偽造也只影響自己的畫面,flag 以 watcher log 為準 |

---

## 法規

**本環境為純軟體模擬。**

在台灣,對真實無人機進行 RF 干擾、GPS 欺騙、Wi-Fi deauthentication
違反《遙控無人機管理規則》及《電信管理法》。
本靶場所示技術僅適用於**獲得授權的測試環境**。
