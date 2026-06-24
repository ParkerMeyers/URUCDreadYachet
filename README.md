# URUCDreadYachet — ROV Control Stack

Topside UI and control launcher for an ROV with thruster stabilization, arm control, dual camera feeds, and SSH-managed onboard programs on a Raspberry Pi.

## Architecture

```
Topside (laptop)                         Onboard (Raspberry Pi)
─────────────────                        ──────────────────────
main_control_ui.py  ──SSH launch──►      onboard/stabilization.py   (UDP 5005)
topside/thrust_sender.py ──UDP 5005──►   onboard/new_ar.py          (UDP 5006)
topside/arm_sender.py    ──UDP 5006──►
```

| Component | Role |
|-----------|------|
| `main_control_ui.py` | Launch screen, live control dashboard, SSH to Pi, MOSFET/mode/missions |
| `topside/thrust_sender.py` | Joystick → thruster commands + telemetry receive |
| `topside/arm_sender.py` | Serial arm controller → UDP to Pi |
| `onboard/stabilization.py` | Thruster mixer, depth/yaw hold, pitch/roll stabilization |
| `onboard/new_ar.py` | Arm servos, MOSFET power, J6 stabilization |
| `topside/camera_feed.py` | H.264 RTP camera decode for UI (ports 5600 / 5601) |

## Quick start

### 1. Topside laptop

```bash
python install.py --topside
```

Then launch the UI (uses `.venv` automatically):

```bash
run_ui.bat          # Windows
./run_ui.sh         # Linux / macOS
```

Or:

```bash
.venv\Scripts\activate    # Windows
python main_control_ui.py
```

### 2. Raspberry Pi (onboard)

Copy this repo to the Pi (default path: `/home/uruc/URUCDreadYachet`), then:

```bash
python3 install.py --onboard
```

This creates `venv/` and installs hardware dependencies. The UI launches onboard scripts with:

```
{ROV_ROOT}/venv/bin/python3
```

### 3. Run a mission

1. Open the UI on the laptop.
2. Enter **SSH host**, **username**, and **password** for the Pi.
3. Click **Start onboard program** (wait for success).
4. Click **Start topside programs** (joystick + arm serial must be connected).
5. Click **Next → Live Control**.
6. Switch mode from **Disarmed** to **Stabilization** or **Drive/Armed** when ready.

**Recommended order:** onboard first, then topside.

## UI features

**Launch screen**

- Start onboard programs over SSH (`stabilization.py`, `new_ar.py`)
- Start topside programs (`arm_sender.py`, `thrust_sender.py`)
- Stop all (safe PWM shutdown + MOSFET low on Pi)

**Control screen**

- Dual camera views (RTP/UDP H.264 on ports 5600 and 5601)
- Heading compass + pitch/roll
- Live telemetry from the ROV
- MOSFET toggle (servo power on Pi)
- Mode: **Stabilization** / **Drive/Armed** / **Disarmed**
- Mission actions: Colmap, Crabs (SSH to Pi)
- System status bar (process state, battery, depth)

## Configuration

Environment variables (optional — defaults are in the UI SSH fields):

| Variable | Description | Default |
|----------|-------------|---------|
| `ROV_HOST` | Pi IP / hostname | `192.168.2.249` |
| `ROV_USER` | SSH username | `uruc` |
| `ROV_PASSWORD` | SSH password | (set in UI) |
| `ROV_ROOT` | Repo path on Pi | `/home/uruc/URUCDreadYachet` |
| `ROV_VENV` | Onboard venv folder | `venv` |
| `ROV_ARM_SERIAL` | Arm serial port | `COM3` (Win) / `/dev/ttyACM0` (Linux) |
| `ROV_CAMERA_1_URL` | Camera 1 source | `rov-udp:5600` |
| `ROV_CAMERA_2_URL` | Camera 2 source | `rov-udp:5601` |
| `ROV_COLMAP_CMD` | Remote Colmap command | (see `main_control_ui.py`) |
| `ROV_CRABS_CMD` | Remote Crabs command | (see `main_control_ui.py`) |

CLI overrides:

```bash
python main_control_ui.py --onboard-host 192.168.2.249 --onboard-user uruc --onboard-root /home/uruc/URUCDreadYachet --onboard-venv venv
```

## Cameras

The UI embeds the same feeds as `topside/ROV_Cameras.sh`:

- Camera 1: UDP port **5600**
- Camera 2: UDP port **5601**
- Codec: H.264 RTP

You also need **GStreamer** on the laptop (`gst-launch-1.0` on PATH). On Windows, install from [gstreamer.freedesktop.org](https://gstreamer.freedesktop.org/download/).

Standalone camera windows (legacy):

```bash
bash topside/ROV_Cameras.sh
```

## Dependencies

| File | Where to install |
|------|------------------|
| `requirements.txt` | Topside laptop (`.venv`) |
| `requirements-onboard.txt` | Pi onboard (`venv/`) |

**Paramiko** is required on the **laptop only** for password SSH from the UI. It is included in `requirements.txt`. If you see a Paramiko error, run the UI via `run_ui.bat` or activate `.venv` — do not use a different Python interpreter.

```bash
python install.py --topside --recreate-venv   # fresh topside venv
python3 install.py --onboard --recreate-venv  # fresh Pi venv
python install.py --system-deps               # print apt/GStreamer hints
```

## Project layout

```
URUCDreadYachet/
├── main_control_ui.py      # Main UI + SSH launcher
├── install.py              # Setup script (topside / onboard)
├── run_ui.bat / run_ui.sh  # Launch UI with correct venv
├── requirements.txt        # Topside Python deps
├── requirements-onboard.txt
├── logs/                   # Topside process logs + UI mode file
├── topside/
│   ├── thrust_sender.py
│   ├── arm_sender.py
│   ├── camera_feed.py
│   └── ROV_Cameras.sh
└── onboard/
    ├── stabilization.py
    └── new_ar.py
```

## Stop / safety

**Stop all** in the UI:

1. Sets mode to **Disarmed** (stops thrust commands)
2. Stops topside senders
3. SSH to Pi: SIGTERM → clear PCA9685 PWM → release servos → MOSFET low → kill stale processes

Onboard launch always runs a safe-stop first to avoid duplicate processes fighting over I2C/PWM.

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Paramiko not found | Use `run_ui.bat` or `.venv\Scripts\python.exe` |
| Topside windows flash and close | Run `python install.py --topside`; check `logs/thrust_sender.log` |
| UI stuck on “launching over SSH” | Confirm Pi reachable; password in UI; Paramiko in `.venv` |
| No camera in UI | Install GStreamer; confirm ROV streams on UDP 5600/5601 |
| Arm serial errors | Set `ROV_ARM_SERIAL` (e.g. `COM4`) |
| Motors jitter on restart | Use **Stop all**, then start onboard before topside |
| Wrong Python on Pi | Ensure `venv/` exists: `python3 install.py --onboard` |

## Manual run (without UI)

**Topside:**

```bash
python topside/thrust_sender.py <PI_IP>
python topside/arm_sender.py <PI_IP>
```

**Onboard (on Pi, inside venv):**

```bash
source venv/bin/activate
python onboard/stabilization.py
python onboard/new_ar.py
```

## License

University ROV project — internal use.
