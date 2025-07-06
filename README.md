# MPact - Advanced Punch Tracking System

A professional-grade punch tracking system built for ESP32-S3 with advanced physics calculations and real-time metrics analysis.

## 🥊 Features

- **Real-time Punch Detection**: Advanced multi-stage filtering and physics-based punch detection
- **Comprehensive Metrics**: Speed, force, power, efficiency, and impact analysis
- **Smart Filtering**: Kalman filtering, moving averages, and adaptive thresholds
- **Touch Interface**: Intuitive LCD touchscreen controls
- **Cloud Integration**: ESP RainMaker support for remote monitoring
- **Professional Physics**: Accurate force, power, and kinetic energy calculations

## 🔧 Hardware Requirements

- ESP32-S3 development board with compatible sensors
- 6-axis IMU sensor for motion detection
- LCD display with touch capability
- Power management circuit

## 🚀 Getting Started

### Prerequisites

1. **Arduino IDE** (version 2.0 or higher)
2. **ESP32 Arduino Core** (version 2.0.0 or higher)

### Required Libraries

Install the following libraries through Arduino IDE Library Manager:

```
- Arduino_GFX_Library
- Arduino_DriveBus_Library
- SensorQMI8658
- SimpleTimer
- ESP RainMaker
```

*Note: Pin configurations and hardware setup details will be provided separately.*

### Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/AravKaul20/MPact.git
   ```

2. Open `MPact.ino` in Arduino IDE

3. Install required libraries

4. Configure your board settings:
   - Board: "ESP32S3 Dev Module"
   - Port: Select your ESP32-S3 port
   - Upload Speed: 921600

5. Upload the code to your ESP32-S3

## 📊 Metrics Calculated

### Physics-Based Measurements

- **Impact Velocity**: Peak velocity during punch (m/s)
- **Impact Force**: Force generated using F = ma (Newtons)
- **Kinetic Energy**: Energy at impact point (Joules)
- **Power Output**: Instantaneous power during impact (Watts)
- **Efficiency**: Velocity retention through impact (%)

### Advanced Features

- **Adaptive Thresholds**: System learns from user behavior
- **Multi-stage Filtering**: Kalman + Moving Average + High-pass filtering
- **Gravity Compensation**: Orientation-based gravity removal
- **State Machine**: 6-state punch detection system

## 🎯 How It Works

1. **Initialization**: System calibrates sensors and establishes baselines
2. **Training Mode**: Touch "START TRAINING" to begin session
3. **Punch Detection**: Advanced algorithms detect and analyze punches
4. **Real-time Display**: Metrics update instantly on screen
5. **Session Summary**: Complete workout analysis when finished

## 🔬 Technical Details

### Sensor Fusion
- High-precision accelerometer and gyroscope data
- Advanced complementary filtering for orientation estimation

### Signal Processing
- **Kalman Filtering**: Reduces sensor noise
- **Moving Average**: Additional smoothing (8-sample window)
- **High-pass Filtering**: Removes low-frequency drift
- **Adaptive Thresholds**: Learns optimal detection parameters

### Physics Calculations
```cpp
// Force calculation using two methods
float deceleration_force = FIST_MASS * acceleration * GRAVITY;
float impulse_force = FIST_MASS * velocity / CONTACT_TIME;
float impact_force = (deceleration_force + impulse_force) / 2.0;

// Energy and power calculations
float kinetic_energy = 0.5 * FIST_MASS * velocity²;
float power = kinetic_energy / CONTACT_TIME;
```

## 📱 Cloud Integration

The system supports ESP RainMaker for remote monitoring:

1. **WiFi Provisioning**: Connect via SoftAP
2. **Real-time Data**: Metrics sent to cloud
3. **Historical Analysis**: Time-series data storage
4. **Remote Monitoring**: Access data from anywhere

## 🛠️ Customization

### Adjusting Sensitivity

Modify these parameters in the code:
```cpp
float jerk_threshold = 35.0;      // Punch detection sensitivity
float impact_threshold = 3.2;     // Impact detection threshold
float min_punch_velocity = 1.5;   // Minimum velocity for valid punch
```

### Display Customization

The LCD interface can be customized by modifying the drawing functions:
- `drawMetricsScreen()` - Training interface
- `drawSummaryScreen()` - Session summary
- `updateMetrics()` - Real-time updates

## 🔧 Troubleshooting

### Common Issues

1. **Sensor Not Detected**: Check sensor connections and power
2. **Touch Not Responding**: Verify touch controller setup
3. **Inaccurate Readings**: Perform calibration with device stationary
4. **WiFi Issues**: Check credentials and signal strength

### Debug Mode

Enable debug output by monitoring serial console at 115200 baud.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 👨‍💻 Author

**Arav Kaul**
- GitHub: [@AravKaul20](https://github.com/AravKaul20)

## 🙏 Acknowledgments

- ESP32 Arduino Core team
- Arduino_GFX_Library contributors
- Open source sensor library developers

---

*Built with ❤️ for the boxing and fitness community*
