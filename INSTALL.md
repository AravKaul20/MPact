# Installation Guide - MPact

## Quick Start

### Prerequisites

1. **Arduino IDE 2.0+** - [Download here](https://www.arduino.cc/en/software)
2. **ESP32 Arduino Core** - Version 2.0.0 or higher
3. **Git** - For cloning the repository

### Step 1: Install ESP32 Support

1. Open Arduino IDE
2. Go to **File > Preferences**
3. Add this URL to "Additional Boards Manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Go to **Tools > Board > Boards Manager**
5. Search for "ESP32" and install **esp32 by Espressif Systems**

### Step 2: Install Required Libraries

Open **Tools > Manage Libraries** and install:

- `Arduino_GFX_Library` by moononournation
- `Arduino_DriveBus_Library` by moononournation  
- `SensorQMI8658` by lewisxhe
- `SimpleTimer` by Marcello Romani
- `ESP RainMaker` by Espressif Systems

### Step 3: Clone and Setup

1. Clone the repository:
   ```bash
   git clone https://github.com/AravKaul20/MPact.git
   cd MPact
   ```

2. Open `MPact.ino` in Arduino IDE

3. Configure board settings:
   - **Board**: "ESP32S3 Dev Module"
   - **USB CDC On Boot**: "Enabled"
   - **CPU Frequency**: "240MHz (WiFi)"
   - **Flash Size**: "4MB (32Mb)"
   - **Partition Scheme**: "Default 4MB with spiffs"
   - **Upload Speed**: "921600"

### Step 4: Upload Code

1. Connect your ESP32-S3 via USB
2. Select the correct port in **Tools > Port**
3. Click **Upload** button
4. Wait for compilation and upload to complete

## Hardware Setup

### Connections

Hardware connections and pin configurations will be provided separately. 
Ensure your hardware matches the pin configuration in `pin_config.h`.

### Power Supply

- Standard ESP32-S3 power requirements
- External power source recommended for stable operation

## First Boot

1. **Power On**: Device should show "MPact Pro" splash screen
2. **Touch Test**: Tap "START TRAINING" button
3. **Sensor Check**: Move device to verify IMU is working
4. **Serial Monitor**: Open at 115200 baud for debug output

## WiFi Setup (Optional)

For cloud features:

1. Device will create WiFi hotspot "MPact_Device"
2. Connect with password "123456"
3. Configure your WiFi credentials
4. Device will connect to your network

## Troubleshooting

### Common Issues

| Problem | Solution |
|---------|----------|
| Upload fails | Check USB cable, try different port |
| Display blank | Verify display connections and power |
| Touch not working | Check touch controller setup |
| IMU not detected | Verify sensor connections |
| WiFi won't connect | Check credentials, signal strength |

### Debug Output

Enable serial monitor (115200 baud) to see:
- Sensor initialization status
- Touch events
- Punch detection events
- WiFi connection status

### Factory Reset

Hold GPIO 0 button for:
- **3-10 seconds**: WiFi reset
- **10+ seconds**: Full factory reset

## Advanced Configuration

### Sensitivity Tuning

Edit these values in `MPact.ino`:

```cpp
float jerk_threshold = 35.0;      // Lower = more sensitive
float impact_threshold = 3.2;     // Punch impact detection
float min_punch_velocity = 1.5;   // Minimum valid punch speed
```

### Display Customization

Modify drawing functions:
- `drawHomeScreen()` - Startup screen
- `drawMetricsScreen()` - Training interface  
- `drawSummaryScreen()` - Session results

### Cloud Integration

Configure RainMaker parameters:
- Device name: Change `service_name`
- Provisioning: Modify `pop` (password)
- Parameters: Add custom metrics

## Development Setup

### VS Code (Recommended)

1. Install **Arduino** extension
2. Install **C/C++** extension
3. Open project folder
4. Configure `arduino.json`:
   ```json
   {
     "board": "esp32:esp32:esp32s3",
     "configuration": "PSRAM=disabled,PartitionScheme=default,CPUFreq=240,FlashMode=qio,FlashSize=4M,UploadSpeed=921600,DebugLevel=none",
     "sketch": "MPact.ino"
   }
   ```

### PlatformIO (Alternative)

1. Install PlatformIO extension
2. Create `platformio.ini`:
   ```ini
   [env:esp32s3]
   platform = espressif32
   board = esp32-s3-devkitc-1
   framework = arduino
   monitor_speed = 115200
   ```

## Support

- **Issues**: [GitHub Issues](https://github.com/AravKaul20/MPact/issues)
- **Discussions**: [GitHub Discussions](https://github.com/AravKaul20/MPact/discussions)
- **Documentation**: Hardware details will be provided separately

---

*Happy punching! 🥊* 