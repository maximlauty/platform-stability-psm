# Linux Interface Guide — Platform Stability Monitor

Teensy 4.1 at **192.168.168.71:80** (static). Auth token: `psm-change-me-v1` (change before deployment via CR-3).

---

## Reading stable / unstable

### One-shot with curl + jq

```bash
curl -s http://192.168.168.71/api/status | jq '{stable: .stable, why: .why, fault: .fault}'
```

Response fields you care about:

| Field    | Type    | Meaning |
|----------|---------|---------|
| `stable` | bool    | `true` = SAFE_OUT_PIN asserted, platform is stable |
| `why`    | string  | Reason if not stable: `OK`, `FAULT`, `FILL`, `GYRO_SPK`, `ACCEL_SPK`, `SPREAD`, `DRIFT`, `HOLD`, `HW_FSM`, `SHADOW`, `CRC` |
| `fault`  | string  | Hex fault mask, e.g. `"0x0000"` means no fault |
| `fsm_s`  | string  | FSM state: `INIT`, `FILL`, `STBL`, `MOVE`, `FALT` |

### Poll stable/unstable in a shell loop

```bash
while true; do
  STATUS=$(curl -s http://192.168.168.71/api/status)
  STABLE=$(echo "$STATUS" | jq -r '.stable')
  WHY=$(echo "$STATUS" | jq -r '.why')
  echo "$(date +%T)  stable=$STABLE  why=$WHY"
  sleep 1
done
```

### Python snippet

```python
import requests, time

PSM = "http://192.168.168.71"

def is_stable():
    r = requests.get(f"{PSM}/api/status", timeout=2)
    d = r.json()
    return d["stable"], d["why"]

while True:
    stable, why = is_stable()
    print(f"stable={stable}  why={why}")
    time.sleep(1)
```

---

## Flashlight on / off

The endpoint is `/light/toggle` — it flips the current state. To set a specific state, check `light` in the status first.

### Toggle (flip current state)

```bash
curl -s -X POST http://192.168.168.71/light/toggle \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -d "auth=psm-change-me-v1"
```

Returns `HTTP 200` with empty body on success, `HTTP 403` if auth is wrong.

### Turn light ON

```bash
# Read current state, then toggle only if currently off
STATE=$(curl -s http://192.168.168.71/api/status | jq -r '.light')
if [ "$STATE" = "0" ]; then
  curl -s -X POST http://192.168.168.71/light/toggle \
    -H "Content-Type: application/x-www-form-urlencoded" \
    -d "auth=psm-change-me-v1"
  echo "Light turned ON"
else
  echo "Light already ON"
fi
```

### Turn light OFF

```bash
STATE=$(curl -s http://192.168.168.71/api/status | jq -r '.light')
if [ "$STATE" = "1" ]; then
  curl -s -X POST http://192.168.168.71/light/toggle \
    -H "Content-Type: application/x-www-form-urlencoded" \
    -d "auth=psm-change-me-v1"
  echo "Light turned OFF"
else
  echo "Light already OFF"
fi
```

### Python snippet — set light to explicit state

```python
import requests

PSM   = "http://192.168.168.71"
TOKEN = "psm-change-me-v1"

def light_set(on: bool):
    r = requests.get(f"{PSM}/api/status", timeout=2)
    current = bool(r.json()["light"])
    if current != on:
        requests.post(
            f"{PSM}/light/toggle",
            data={"auth": TOKEN},
            headers={"Content-Type": "application/x-www-form-urlencoded"},
            timeout=2,
        )

light_set(True)   # turn on
light_set(False)  # turn off
```

---

## Serial monitor (USB CDC, 115200 baud)

Connect the Teensy USB cable. The device usually appears as `/dev/ttyACM0`.

```bash
# Find the port
ls /dev/ttyACM* /dev/ttyUSB*

# Monitor with screen (Ctrl-A K to quit)
screen /dev/ttyACM0 115200

# Or with minicom (Ctrl-A X to quit)
minicom -D /dev/ttyACM0 -b 115200

# Or raw with stty + cat
stty -F /dev/ttyACM0 115200 cs8 -cstopb -parenb raw
cat /dev/ttyACM0
```

### Key serial output lines

**Heartbeat (1 Hz)**
```
[HBEAT] loops=412  fault=0x0000  staleA=0  staleB=0  spi=0ms  ms=12345
```

**Status line (every 5 s)**
```
[STATUS] stable=TRUE   why=OK         fsm=STBL  hw=5
         fault=none  inhib=0  fill=100%
         staleA=0  staleB=0
         gyro_spk=0/20  accel_spk=0/5
         omega=0.12dps  pk=0.45dps
         RA=0.31 PA=-0.12  RB=0.30 PB=-0.13  dR=0.01 dP=0.01
         spread=0.05deg  miss=0  stk=312  cp=0  spi=0ms
```

Extract just `stable=` in real time:
```bash
stty -F /dev/ttyACM0 115200 raw cs8 -cstopb -parenb
grep --line-buffered "STATUS" /dev/ttyACM0 | grep -oP 'stable=\w+'
```

---

## Full status JSON reference

```
GET http://192.168.168.71/api/status
```

```json
{
  "stable":    true,
  "inhibit":   false,
  "fault":     "0x0000",
  "why":       "OK",
  "fsm_s":     "STBL",
  "hw_cnt":    5,
  "fill":      100,
  "gyro_spk":  0,
  "gyro_max":  20,
  "accel_spk": 0,
  "accel_max": 5,
  "omega":     0.12,
  "roll":      0.31,
  "pitch":    -0.12,
  "roll_b":    0.30,
  "pitch_b":  -0.13,
  "diverge_r": 0.01,
  "diverge_p": 0.01,
  "stale_a":   0,
  "stale_b":   0,
  "spread":    0.05,
  "light":     0
}
```

`light` is `1` when the Olight Odin GL PWM output is active, `0` when off.

---

## Network setup (Linux side)

The Teensy expects a host on the `192.168.168.0/24` subnet. Assign a static address to the interface connected to it:

```bash
# Replace eth0 with your actual interface (ip link to find it)
sudo ip addr add 192.168.168.100/24 dev eth0
sudo ip link set eth0 up

# Verify connectivity
ping 192.168.168.71
```

To make it permanent, use `/etc/network/interfaces` or a NetworkManager static profile.
