# Az/El Satellite Tracker

An ESP32-S3-CAM based satellite antenna tracker with automatic tracking, GPredict compatibility, and web-based control interface.

![Satellite Tracker](docs/images/tracker-banner.png)

## Features

- **Automatic Satellite Tracking**: On-device SGP4/SDP4 orbit propagation for autonomous tracking
- **GPredict Compatible**: Full rotctld protocol support on TCP port 4533
- **Web Interface**: Modern responsive UI for control, monitoring, and configuration
- **TLE Management**: Automatic TLE updates from Celestrak when connected to WiFi
- **GPS Integration**: Automatic position and time from GPS module
- **Sensorless Homing**: TMC2209 StallGuard for reliable homing without limit switches
- **Zenith Pass Handling**: Automatic flip management for overhead satellite passes

## Hardware Requirements

### Core Components

| Component | Specification | Notes |
|-----------|--------------|-------|
| MCU | ESP32-S3-CAM | With OV2640 camera |
| Az Motor | NEMA 17 5.18:1 geared stepper | 0.35° resolution |
| El Motor | NEMA 17 5.18:1 geared stepper | 0.35° resolution |
| Drivers | TMC2209 × 2 | UART mode, StallGuard |
| GPS | Any NMEA GPS module | 9600 baud default |
| Power | 12V 3A+ | For motors + ESP32 |

### Pin Connections

```
ESP32-S3-CAM Pin Mapping
========================

TMC2209 Azimuth:
  STEP  → GPIO 38
  DIR   → GPIO 39
  DIAG  → GPIO 40

TMC2209 Elevation:
  STEP  → GPIO 41
  DIR   → GPIO 42
  DIAG  → GPIO 2

Shared TMC:
  EN    → GPIO 1  (active LOW)
  UART TX → GPIO 43
  UART RX → GPIO 44

GPS:
  TX → GPIO 14 (GPS TX to ESP RX)
  RX → GPIO 21 (ESP TX to GPS RX)

TMC2209 UART Addressing:
  Azimuth:   MS1=LOW,  MS2=LOW  (address 0b00)
  Elevation: MS1=HIGH, MS2=LOW  (address 0b01)
```

### Wiring Diagram

```
                    ┌─────────────────┐
                    │   ESP32-S3-CAM  │
                    │                 │
    ┌───────────────┤ GPIO38    GPIO1 ├───────────┐
    │               │ GPIO39   GPIO43 ├─────┬─────┼───────┐
    │               │ GPIO40   GPIO44 ├───┬─┼─────┼───────┤
    │               │                 │   │ │     │       │
    │               │ GPIO41   GPIO14 ├───┼─┼─────┼───┐   │
    │               │ GPIO42   GPIO21 ├───┼─┼─────┼───┤   │
    │               │ GPIO2           │   │ │     │   │   │
    │               └─────────────────┘   │ │     │   │   │
    │                                     │ │     │   │   │
    │    ┌──────────────┐                 │ │     │   │   │
    │    │  TMC2209 AZ  │                 │ │     │   │   │
    ├────┤ STEP     EN  ├─────────────────┼─┼─────┘   │   │
    ├────┤ DIR      TX  ├─────────────────┘ │         │   │
    ├────┤ DIAG     RX  ├───────────────────┘         │   │
    │    │ MS1=GND      │                             │   │
    │    │ MS2=GND      │                             │   │
    │    └──────────────┘                             │   │
    │                                                 │   │
    │    ┌──────────────┐                             │   │
    │    │  TMC2209 EL  │                             │   │
    ├────┤ STEP     EN  ├─────────────────────────────┘   │
    ├────┤ DIR      TX  ├─────────────────────────────────┤
    ├────┤ DIAG     RX  ├─────────────────────────────────┘
    │    │ MS1=3.3V     │
    │    │ MS2=GND      │
    │    └──────────────┘
    │
    │    ┌──────────────┐
    │    │     GPS      │
    │    │ TX      VCC  ├─── 3.3V
    │    │ RX      GND  ├─── GND
    │    └──────────────┘
```

## Software Setup

### Prerequisites

- [PlatformIO](https://platformio.org/) (VS Code extension recommended)
- Python 3.x (for PlatformIO)

### Building

1. Clone the repository:
```bash
git clone https://github.com/yourusername/satellite-tracker.git
cd satellite-tracker
```

2. Build the firmware:
```bash
pio run
```

3. Upload filesystem (web UI):
```bash
pio run -t uploadfs
```

4. Upload firmware:
```bash
pio run -t upload
```

### First Boot

1. The tracker will create a WiFi access point:
   - SSID: `SatTracker-AP`
   - Password: `satellite123`

2. Connect to the AP and open `http://192.168.4.1`

3. Configure your home WiFi network

4. After connecting, access the tracker at:
   - `http://satellite-tracker.local` (mDNS)
   - Or check your router for the assigned IP

## Usage

### Web Interface

The web interface provides:

- **Dashboard**: Real-time position display, tracking status, GPS info
- **Control**: Manual positioning, stop, park, home functions
- **Tracking**: Satellite selection, TLE management, pass predictions
- **Settings**: WiFi configuration, calibration, system info

### GPredict Integration

1. In GPredict, go to Edit → Preferences → Interfaces → Rotators

2. Add new rotator:
   - Name: `SatTracker`
   - Host: `satellite-tracker.local` (or IP address)
   - Port: `4533`
   - Az Type: `0° → 180° → 360°`
   - El Type: `0° → 90°`

3. In the rotator control window, select your rotator and engage

### REST API

Full REST API available at `/api/`:

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/status` | GET | Full system status |
| `/api/position` | GET | Current az/el position |
| `/api/position` | POST | Set target position |
| `/api/stop` | POST | Stop all motion |
| `/api/park` | POST | Move to park position |
| `/api/home` | POST | Run homing sequence |
| `/api/track` | POST | Start satellite tracking |
| `/api/track` | DELETE | Stop tracking |
| `/api/tle` | GET | List stored TLEs |
| `/api/tle` | POST | Add/update TLE |
| `/api/tle/fetch` | POST | Fetch TLE from Celestrak |
| `/api/gps` | GET | GPS status and position |
| `/api/wifi/status` | GET | WiFi connection status |
| `/api/wifi/connect` | POST | Connect to WiFi network |

### WebSocket

Real-time updates via WebSocket at `ws://[host]/ws`

Messages are JSON formatted:
```json
{
  "type": "status",
  "az": 180.5,
  "el": 45.2,
  "moving": true,
  "mode": 2,
  "sat": "ISS (ZARYA)",
  "visible": true
}
```

### Manual Alignment

1. Point antenna at known reference (landmark, compass bearing)
2. In web UI, enter current actual az/el
3. Click "Set as Reference"

## Specifications

| Parameter | Value |
|-----------|-------|
| Azimuth Range | -180° to 540° |
| Elevation Range | 0° to 90° |
| Resolution (full step) | 0.35° |
| Resolution (1/32 microstep) | 0.011° |
| Max Slew Speed | 30°/s |
| Tracking Speed | 5°/s |
| Position Accuracy | ±0.1° |

## Troubleshooting

### Motors not moving
- Check TMC2209 UART connection (TX/RX)
- Verify motor power supply (12V, sufficient current)
- Check enable pin is LOW
- Monitor serial output for driver communication errors

### No GPS fix
- Ensure clear sky view
- Check GPS TX/RX connections (crossed)
- Wait up to 2 minutes for cold start fix
- Verify GPS module baud rate matches config (9600)

### Can't connect to WiFi
- Connect to AP mode and reconfigure
- Check SSID/password
- Verify router allows new connections
- Check signal strength

### GPredict not connecting
- Verify IP address and port 4533
- Check firewall settings
- Ensure tracker is on same network
- Try IP address instead of mDNS hostname

### Tracking inaccurate
- Update TLEs (they decay over time)
- Verify GPS has fix and correct time
- Run calibration procedure
- Check for mechanical backlash

## Project Structure

```
satellite-tracker/
├── src/
│   ├── main.cpp              # Application entry point
│   ├── config.h              # Configuration constants
│   ├── stepper_control.*     # TMC2209 motor control
│   ├── gps.*                 # GPS NMEA parsing
│   ├── tracking_engine.*     # SGP4 orbit propagation
│   ├── tle_manager.*         # TLE fetch and storage
│   ├── rotctld_server.*      # Hamlib protocol server
│   ├── wifi_manager.*        # WiFi connectivity
│   ├── web_server.*          # HTTP server and API
│   ├── camera_align.*        # Camera-based alignment
│   └── nvs_storage.*         # Persistent storage
├── data/
│   ├── index.html            # Web UI
│   └── style.css             # Styles
├── docs/
│   └── ...                   # Documentation
├── platformio.ini            # Build configuration
└── README.md                 # This file
```

## Contributing

Contributions welcome! Please read our contributing guidelines before submitting PRs.

## License

MIT License - see LICENSE file for details.

## Acknowledgments

- [TinyGPS++](https://github.com/mikalhart/TinyGPSPlus) - GPS parsing
- [TMCStepper](https://github.com/teemuatlut/TMCStepper) - TMC2209 control
- [Sgp4-Library](https://github.com/Hopperpop/Sgp4-Library) - Orbit propagation
- [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer) - Async web server
- [Celestrak](https://celestrak.org/) - TLE data source

## Author

Stephen Packer

Built for amateur radio satellite operations.

73 de KD2NDR
