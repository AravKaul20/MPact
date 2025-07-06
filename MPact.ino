/*
 * MPact - Advanced Punch Tracking System
 * 
 * A professional-grade punch tracking system with advanced physics calculations
 * and real-time metrics analysis for boxing and fitness training.
 * 
 * Author: Arav Kaul
 * GitHub: https://github.com/AravKaul20/MPact
 * 
 * Hardware: ESP32-S3 with QMI8658 IMU, ST7789 LCD, CST816T Touch
 */

#include <Wire.h>
#include <Arduino.h>
#include "pin_config.h"
#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include "HWCDC.h"
#include "SensorQMI8658.hpp"
#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <nvs_flash.h>
#include <SimpleTimer.h>

// ======================= GLOBAL VARIABLES =======================

// Timers and system control
SimpleTimer Timer;
unsigned long lastTouchTime = 0;
unsigned long trainingStartTime = 0;
const unsigned long TOUCH_DEBOUNCE = 500;
const unsigned long LONG_PRESS_DURATION = 2000;

// Power management
unsigned long pressStartTime = 0;
bool buttonPressed = false;

// Training state
bool training_mode = false;
bool wifi_connected = false;

// Display and touch setup
Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 0, true, LCD_WIDTH, LCD_HEIGHT, 0, 20, 0, 0);

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
  std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

void Arduino_IIC_Touch_Interrupt(void);
std::unique_ptr<Arduino_IIC> CST816T(new Arduino_CST816x(IIC_Bus, CST816T_DEVICE_ADDRESS,
                                                       TP_RST, TP_INT, Arduino_IIC_Touch_Interrupt));

// IMU sensor
SensorQMI8658 qmi;
IMUdata acc, gyr;

// RainMaker configuration
const char *service_name = "MPact_Device";
const char *pop = "123456";
static Device *punch_device = NULL;

// ======================= ADVANCED FILTERING SYSTEM =======================

struct KalmanFilter {
    float Q, R, P, K, X;
    
    KalmanFilter(float q = 0.003, float r = 0.08) : Q(q), R(r), P(1.0), K(0.0), X(0.0) {}
    
    float update(float measurement) {
        P = P + Q;
        K = P / (P + R);
        X = X + K * (measurement - X);
        P = (1 - K) * P;
        return X;
    }
};

struct MovingAverage {
    static const int WINDOW_SIZE = 8;
    float buffer[WINDOW_SIZE];
    int index;
    float sum;
    bool filled;
    
    MovingAverage() : index(0), sum(0.0), filled(false) {
        for(int i = 0; i < WINDOW_SIZE; i++) buffer[i] = 0.0;
    }
    
    float update(float value) {
        sum -= buffer[index];
        buffer[index] = value;
        sum += value;
        index = (index + 1) % WINDOW_SIZE;
        if(!filled && index == 0) filled = true;
        return sum / (filled ? WINDOW_SIZE : (index == 0 ? WINDOW_SIZE : index));
    }
};

struct HighPassFilter {
    float alpha = 0.95;
    float prev_output = 0;
    float prev_input = 0;
    
    float update(float input) {
        float output = alpha * (prev_output + input - prev_input);
        prev_output = output;
        prev_input = input;
        return output;
    }
};

// Filter instances
KalmanFilter kalman_x, kalman_y, kalman_z;
MovingAverage ma_x, ma_y, ma_z;
HighPassFilter hp_x, hp_y, hp_z;

// ======================= PUNCH PHYSICS & DETECTION =======================

struct PunchMetrics {
    int count = 0;
    float current_velocity = 0.0;
    float max_velocity = 0.0;
    float current_force = 0.0;
    float max_force = 0.0;
    float avg_force = 0.0;
    float total_force = 0.0;
    float power = 0.0;
    float max_power = 0.0;
    float energy = 0.0;
    float punch_efficiency = 0.0;
    float impact_time = 0.0;
} metrics;

enum PunchState {
    IDLE, WIND_UP, ACCELERATION, IMPACT, DECELERATION, RECOVERY
};

struct PunchDetector {
    PunchState state = IDLE;
    unsigned long state_start_time = 0;
    float velocity_x = 0.0, velocity_y = 0.0, velocity_z = 0.0;
    float prev_acc_x = 0.0, prev_acc_y = 0.0, prev_acc_z = 0.0;
    float jerk_threshold = 35.0;
    float impact_threshold = 3.2;
    float min_punch_velocity = 1.5;
    unsigned long last_punch_time = 0;
    float max_velocity_in_punch = 0.0;
    unsigned long punch_start_time = 0;
    
    static const int JERK_BUFFER_SIZE = 5;
    float jerk_buffer[JERK_BUFFER_SIZE];
    int jerk_index = 0;
    
    PunchDetector() {
        for(int i = 0; i < JERK_BUFFER_SIZE; i++) jerk_buffer[i] = 0.0;
    }
};

PunchDetector punch_detector;

// Physics constants
const float FIST_MASS = 0.65;
const float CONTACT_TIME = 0.015;
const float GRAVITY = 9.81;
const float SAMPLING_RATE = 1000.0;
const float DT = 1.0 / SAMPLING_RATE;

// ======================= HELPER FUNCTIONS =======================

void Arduino_IIC_Touch_Interrupt(void) {
    CST816T->IIC_Interrupt_Flag = true;
}

void initNVS() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

float calculateJerk(float current_acc, float prev_acc) {
    float jerk = abs(current_acc - prev_acc) / DT;
    if (jerk > 500.0) jerk = 0; // Cap at reasonable maximum
    return jerk;
}

// ======================= PUNCH DETECTION ALGORITHM =======================

bool detectPunch(float ax, float ay, float az, float gx, float gy, float gz) {
    unsigned long current_time = millis();
    
    // Calculate acceleration magnitude and jerk
    float acc_magnitude = sqrt(ax*ax + ay*ay + az*az);
    float jerk_x = calculateJerk(ax, punch_detector.prev_acc_x);
    float jerk_y = calculateJerk(ay, punch_detector.prev_acc_y);
    float jerk_z = calculateJerk(az, punch_detector.prev_acc_z);
    float total_jerk = sqrt(jerk_x*jerk_x + jerk_y*jerk_y + jerk_z*jerk_z);
    
    // Update jerk buffer for smoothing
    punch_detector.jerk_buffer[punch_detector.jerk_index] = total_jerk;
    punch_detector.jerk_index = (punch_detector.jerk_index + 1) % PunchDetector::JERK_BUFFER_SIZE;
    
    float avg_jerk = 0;
    for(int i = 0; i < PunchDetector::JERK_BUFFER_SIZE; i++) {
        avg_jerk += punch_detector.jerk_buffer[i];
    }
    avg_jerk /= PunchDetector::JERK_BUFFER_SIZE;
    
    // Velocity integration with drift correction
    punch_detector.velocity_x += ax * DT;
    punch_detector.velocity_y += ay * DT;
    punch_detector.velocity_z += az * DT;
    
    float acc_total = sqrt(ax*ax + ay*ay + az*az);
    float damping_factor = (acc_total < 0.5) ? 0.985 : 0.998;
    
    punch_detector.velocity_x *= damping_factor;
    punch_detector.velocity_y *= damping_factor;
    punch_detector.velocity_z *= damping_factor;
    
    // Zero out small velocities
    if (abs(punch_detector.velocity_x) < 0.1) punch_detector.velocity_x = 0;
    if (abs(punch_detector.velocity_y) < 0.1) punch_detector.velocity_y = 0;
    if (abs(punch_detector.velocity_z) < 0.1) punch_detector.velocity_z = 0;
    
    float velocity_magnitude = sqrt(pow(punch_detector.velocity_x, 2) + 
                                   pow(punch_detector.velocity_y, 2) + 
                                   pow(punch_detector.velocity_z, 2));
    
    // State machine for punch detection
    switch(punch_detector.state) {
        case IDLE:
            if(avg_jerk > punch_detector.jerk_threshold && velocity_magnitude > 0.5) {
                punch_detector.state = WIND_UP;
                punch_detector.state_start_time = current_time;
                punch_detector.max_velocity_in_punch = 0;
            }
            break;
            
        case WIND_UP:
            if(velocity_magnitude > punch_detector.max_velocity_in_punch) {
                punch_detector.max_velocity_in_punch = velocity_magnitude;
            }
            
            if(velocity_magnitude > punch_detector.min_punch_velocity) {
                punch_detector.state = ACCELERATION;
                punch_detector.punch_start_time = current_time;
            } else if((current_time - punch_detector.state_start_time) > 500) {
                punch_detector.state = IDLE;
            }
            break;
            
        case ACCELERATION:
            if(velocity_magnitude > punch_detector.max_velocity_in_punch) {
                punch_detector.max_velocity_in_punch = velocity_magnitude;
            }
            
            if(acc_magnitude > punch_detector.impact_threshold || 
               (velocity_magnitude < punch_detector.max_velocity_in_punch * 0.3)) {
                punch_detector.state = IMPACT;
            } else if((current_time - punch_detector.state_start_time) > 1000) {
                punch_detector.state = IDLE;
            }
            break;
            
        case IMPACT:
            punch_detector.state = DECELERATION;
            
            // Calculate punch metrics
            float impact_velocity = punch_detector.max_velocity_in_punch;
            float deceleration_force = FIST_MASS * acc_magnitude * GRAVITY;
            float impulse_force = FIST_MASS * impact_velocity / CONTACT_TIME;
            float impact_force = (deceleration_force + impulse_force) / 2.0;
            float kinetic_energy = 0.5 * FIST_MASS * pow(impact_velocity, 2);
            float power = kinetic_energy / CONTACT_TIME;
            float velocity_retention = (velocity_magnitude / punch_detector.max_velocity_in_punch) * 100.0;
            
            // Update metrics
            metrics.count++;
            metrics.current_velocity = impact_velocity;
            metrics.current_force = impact_force;
            metrics.power = power;
            metrics.energy += kinetic_energy;
            metrics.impact_time = current_time - punch_detector.punch_start_time;
            
            if(impact_velocity > metrics.max_velocity) metrics.max_velocity = impact_velocity;
            if(impact_force > metrics.max_force) metrics.max_force = impact_force;
            if(power > metrics.max_power) metrics.max_power = power;
            
            metrics.total_force += impact_force;
            metrics.avg_force = metrics.total_force / metrics.count;
            metrics.punch_efficiency = velocity_retention;
            
            punch_detector.last_punch_time = current_time;
            
            Serial.printf("Punch detected! Velocity: %.2f m/s, Force: %.2f N, Power: %.2f W\n",
                         impact_velocity, impact_force, power);
            
            return true;
            
        case DECELERATION:
            if(velocity_magnitude < 0.5 && acc_magnitude < 1.5) {
                punch_detector.state = RECOVERY;
            } else if((current_time - punch_detector.state_start_time) > 500) {
                punch_detector.state = IDLE;
            }
            break;
            
        case RECOVERY:
            if((current_time - punch_detector.state_start_time) > 200) {
                punch_detector.state = IDLE;
                punch_detector.velocity_x = punch_detector.velocity_y = punch_detector.velocity_z = 0;
            }
            break;
    }
    
    // Store previous acceleration
    punch_detector.prev_acc_x = ax;
    punch_detector.prev_acc_y = ay;
    punch_detector.prev_acc_z = az;
    
    return false;
}

// ======================= DISPLAY FUNCTIONS =======================

void drawHomeScreen() {
    gfx->fillScreen(BLACK);
    
    // Title
    gfx->setTextSize(2);
    gfx->setTextColor(YELLOW);
    const char* titleText = "MPact Pro";
    int textWidth = strlen(titleText) * 12;
    int textX = (LCD_WIDTH - textWidth) / 2;
    gfx->setCursor(textX, 50);
    gfx->println(titleText);
    
    // Subtitle
    gfx->setTextSize(1);
    gfx->setTextColor(CYAN);
    const char* subtitleText = "Advanced Punch Tracking System";
    textWidth = strlen(subtitleText) * 6;
    textX = (LCD_WIDTH - textWidth) / 2;
    gfx->setCursor(textX, 70);
    gfx->println(subtitleText);
    
    // Start button
    int buttonWidth = 200;
    int buttonHeight = 80;
    int buttonX = (LCD_WIDTH - buttonWidth) / 2;
    int buttonY = (LCD_HEIGHT - buttonHeight) / 2;
    
    gfx->fillRoundRect(buttonX, buttonY, buttonWidth, buttonHeight, 10, BLUE);
    
    gfx->setTextColor(WHITE);
    gfx->setTextSize(2);
    const char* buttonText = "START TRAINING";
    textWidth = strlen(buttonText) * 12;
    textX = buttonX + (buttonWidth - textWidth) / 2;
    int textY = buttonY + (buttonHeight - 16) / 2;
    gfx->setCursor(textX, textY);
    gfx->println(buttonText);
    
    // Author credit
    gfx->setTextSize(2);
    gfx->setTextColor(WHITE);
    const char* creditText = "By Arav Kaul";
    textWidth = strlen(creditText) * 12;
    textX = (LCD_WIDTH - textWidth) / 2;
    gfx->setCursor(textX, buttonY + buttonHeight + 40);
    gfx->println(creditText);
}

void drawMetricsScreen() {
    gfx->fillScreen(BLACK);
    
    // Title
    gfx->setTextSize(2);
    gfx->setTextColor(YELLOW);
    gfx->setCursor(50, 20);
    gfx->println("PUNCH METRICS");
    
    // Metrics labels
    gfx->setTextSize(2);
    gfx->setTextColor(WHITE);
    
    gfx->setCursor(20, 60);
    gfx->print("Count:");
    gfx->setCursor(20, 85);
    gfx->print("Speed:");
    gfx->setCursor(20, 110);
    gfx->print("Force:");
    gfx->setCursor(20, 135);
    gfx->print("Power:");
    gfx->setCursor(20, 160);
    gfx->print("Efficiency:");
    
    // Stop button
    int buttonWidth = 160;
    int buttonHeight = 40;
    int buttonX = (LCD_WIDTH - buttonWidth) / 2;
    int buttonY = LCD_HEIGHT - buttonHeight - 20;
    
    gfx->fillRoundRect(buttonX, buttonY, buttonWidth, buttonHeight, 8, RED);
    gfx->setTextSize(2);
    gfx->setTextColor(WHITE);
    const char* buttonText = "STOP";
    int textWidth = strlen(buttonText) * 12;
    int textX = buttonX + (buttonWidth - textWidth) / 2;
    int textY = buttonY + (buttonHeight - 16) / 2;
    gfx->setCursor(textX, textY);
    gfx->print(buttonText);
    
    updateMetrics();
}

void updateMetrics() {
    // Clear previous values
    gfx->fillRect(120, 60, 180, 125, BLACK);
    
    gfx->setTextSize(2);
    gfx->setTextColor(CYAN);
    
    char buffer[32];
    
    sprintf(buffer, "%6d", metrics.count);
    gfx->setCursor(120, 60);
    gfx->print(buffer);
    
    sprintf(buffer, "%5.1f", metrics.current_velocity);
    gfx->setCursor(120, 85);
    gfx->print(buffer);
    gfx->print(" m/s");
    
    sprintf(buffer, "%5.0f N", metrics.current_force);
    gfx->setCursor(120, 110);
    gfx->print(buffer);
    
    sprintf(buffer, "%5.0f W", metrics.power);
    gfx->setCursor(120, 135);
    gfx->print(buffer);
    
    sprintf(buffer, "%4.0f%%", metrics.punch_efficiency);
    gfx->setCursor(120, 160);
    gfx->print(buffer);
}

void drawSummaryScreen(unsigned long duration) {
    gfx->fillScreen(BLACK);
    
    // Title
    gfx->setTextSize(2);
    gfx->setTextColor(YELLOW);
    gfx->setCursor(40, 30);
    gfx->println("GREAT WORKOUT!");
    
    // Stars
    gfx->setTextSize(3);
    gfx->setTextColor(YELLOW);
    gfx->setCursor(80, 50);
    gfx->print("*****");
    
    // Summary stats
    gfx->setTextSize(2);
    gfx->setTextColor(WHITE);
    
    int minutes = duration / 60000;
    int seconds = (duration % 60000) / 1000;
    
    gfx->setCursor(20, 100);
    gfx->printf("Time: %02d:%02d", minutes, seconds);
    
    gfx->setCursor(20, 125);
    gfx->printf("Punches: %d", metrics.count);
    
    gfx->setCursor(20, 150);
    gfx->printf("Max Force: %.0f N", metrics.max_force);
    
    gfx->setCursor(20, 175);
    gfx->printf("Max Power: %.0f W", metrics.max_power);
    
    // Continue button
    int buttonWidth = 160;
    int buttonHeight = 40;
    int buttonX = (LCD_WIDTH - buttonWidth) / 2;
    int buttonY = LCD_HEIGHT - buttonHeight - 20;
    
    gfx->fillRoundRect(buttonX, buttonY, buttonWidth, buttonHeight, 8, BLUE);
    
    gfx->setTextSize(2);
    gfx->setTextColor(WHITE);
    const char* buttonText = "CONTINUE";
    int textWidth = strlen(buttonText) * 12;
    int textX = buttonX + (buttonWidth - textWidth) / 2;
    int textY = buttonY + (buttonHeight - 16) / 2;
    gfx->setCursor(textX, textY);
    gfx->print(buttonText);
}

// ======================= RAINMAKER FUNCTIONS =======================

void setupRainMaker() {
    pinMode(RESET_BUTTON_PIN, INPUT);
    
    Node my_node;
    my_node = RMaker.initNode("MPact_Pro");

    punch_device = new Device("PunchTracker", "custom.device.punchtracker", NULL);
    if (!punch_device) {
        Serial.println("Failed to create punch device!");
        return;
    }

    punch_device->addNameParam();
    
    // Add parameters with time series support
    Param count_param("PunchCount", "custom.param.count", value(0), 
                     PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    count_param.addUIType(ESP_RMAKER_UI_TEXT);
    punch_device->addParam(count_param);
    
    Param velocity_param("Speed", "custom.param.speed", value(0.0f), 
                        PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    velocity_param.addUIType(ESP_RMAKER_UI_TEXT);
    punch_device->addParam(velocity_param);
    
    Param force_param("Force", "custom.param.force", value(0.0f), 
                     PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    force_param.addUIType(ESP_RMAKER_UI_TEXT);
    punch_device->addParam(force_param);
    
    Param power_param("Power", "custom.param.power", value(0.0f), 
                     PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    power_param.addUIType(ESP_RMAKER_UI_TEXT);
    punch_device->addParam(power_param);

    my_node.addDevice(*punch_device);

    Serial.printf("\nStarting ESP-RainMaker\n");
    RMaker.start();
    RMaker.enableTZService();
    RMaker.enableSchedule();

    WiFi.onEvent(sysProvEvent);
    WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, 
                          WIFI_PROV_SECURITY_1, pop, service_name);
}

void sysProvEvent(arduino_event_t *sys_event) {
    switch (sys_event->event_id) {
        case ARDUINO_EVENT_PROV_START:
            Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", 
                        service_name, pop);
            break;

        case ARDUINO_EVENT_PROV_CRED_RECV: 
            Serial.println("\nReceived Wi-Fi credentials");
            WiFi.setTxPower(WIFI_POWER_8_5dBm);
            break;
            
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            Serial.printf("\nConnected to Wi-Fi!\n");
            wifi_connected = true;
            break;
            
        case ARDUINO_EVENT_PROV_INIT:
            wifi_prov_mgr_disable_auto_stop(10000);
            break;
            
        case ARDUINO_EVENT_PROV_CRED_SUCCESS:
            wifi_prov_mgr_stop_provisioning();
            break;
    }
}

void updateRainMakerMetrics() {
    if (punch_device) {
        punch_device->updateAndReportParam("PunchCount", metrics.count);
        punch_device->updateAndReportParam("Speed", metrics.current_velocity);
        punch_device->updateAndReportParam("Force", metrics.current_force);
        punch_device->updateAndReportParam("Power", metrics.power);
    }
}

// ======================= MAIN SETUP & LOOP =======================

void setup() {
    Serial.begin(115200);
    
    // Power management setup
    pinMode(SYS_EN_PIN, OUTPUT);
    digitalWrite(SYS_EN_PIN, HIGH);
    pinMode(SYS_OUT_PIN, INPUT_PULLUP);
    Serial.println("MPact system initializing...");

    // Initialize I2C
    Wire.begin(IIC_SDA, IIC_SCL);

    // Initialize IMU
    Serial.println("Initializing IMU sensor...");
    if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
        Serial.println("Failed to find QMI8658 chip");
        while (1) delay(1000);
    }
    Serial.println("QMI8658 Found!");

    // Configure IMU with optimal settings
    qmi.configAccelerometer(
        SensorQMI8658::ACC_RANGE_8G,
        SensorQMI8658::ACC_ODR_1000Hz,
        SensorQMI8658::LPF_MODE_3,
        true
    );

    qmi.configGyroscope(
        SensorQMI8658::GYR_RANGE_512DPS,
        SensorQMI8658::GYR_ODR_1000Hz,
        SensorQMI8658::LPF_MODE_3,
        true
    );

    qmi.enableGyroscope();
    qmi.enableAccelerometer();

    // Initialize touch controller
    while (CST816T->begin() == false) {
        Serial.println("Touch initialization failed!");
        delay(1000);
    }
    Serial.println("Touch controller initialized");

    CST816T->IIC_Write_Device_State(CST816T->Arduino_IIC_Touch::Device::TOUCH_DEVICE_INTERRUPT_MODE,
                                  CST816T->Arduino_IIC_Touch::Device_Mode::TOUCH_DEVICE_INTERRUPT_PERIODIC);

    // Initialize display
    gfx->begin();
    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, HIGH);
    
    drawHomeScreen();

    // Initialize RainMaker
    setupRainMaker();
    
    Serial.println("MPact system ready!");
}

void loop() {
    // Handle touch input
    if (CST816T->IIC_Interrupt_Flag) {
        int32_t touchX = CST816T->IIC_Read_Device_Value(CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
        int32_t touchY = CST816T->IIC_Read_Device_Value(CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);
        
        unsigned long currentTime = millis();

        if (touchX > 0 && touchY > 0 && (currentTime - lastTouchTime) > TOUCH_DEBOUNCE) {
            if (!training_mode && 
                touchX >= (LCD_WIDTH - 200)/2 && touchX <= (LCD_WIDTH + 200)/2 && 
                touchY >= (LCD_HEIGHT - 80)/2 && touchY <= (LCD_HEIGHT + 80)/2) {
                
                Serial.println("Training mode started!");
                lastTouchTime = currentTime;
                training_mode = true;
                trainingStartTime = currentTime;
                drawMetricsScreen();
                
                // Reset metrics
                memset(&metrics, 0, sizeof(metrics));
                punch_detector.state = IDLE;
                punch_detector.velocity_x = punch_detector.velocity_y = punch_detector.velocity_z = 0;
            }
            else if (training_mode && 
                     touchX >= (LCD_WIDTH - 160)/2 && touchX <= (LCD_WIDTH + 160)/2 && 
                     touchY >= (LCD_HEIGHT - 60) && touchY <= LCD_HEIGHT - 20) {
                
                Serial.println("Training mode stopped!");
                lastTouchTime = currentTime;
                
                unsigned long trainingDuration = currentTime - trainingStartTime;
                drawSummaryScreen(trainingDuration);
                training_mode = false;
            }
            else if (!training_mode && 
                     touchX >= (LCD_WIDTH - 160)/2 && touchX <= (LCD_WIDTH + 160)/2 && 
                     touchY >= LCD_HEIGHT - 60 && touchY <= LCD_HEIGHT - 20) {
                lastTouchTime = currentTime;
                drawHomeScreen();
            }
        }
        
        CST816T->IIC_Interrupt_Flag = false;
    }
    
    // Process sensor data during training
    if (training_mode && qmi.getDataReady()) {
        if (qmi.getAccelerometer(acc.x, acc.y, acc.z) && qmi.getGyroscope(gyr.x, gyr.y, gyr.z)) {
            // Multi-stage filtering
            float filtered_x = hp_x.update(ma_x.update(kalman_x.update(acc.x)));
            float filtered_y = hp_y.update(ma_y.update(kalman_y.update(acc.y)));
            float filtered_z = hp_z.update(ma_z.update(kalman_z.update(acc.z)));
            
            // Punch detection
            if (detectPunch(filtered_x, filtered_y, filtered_z, gyr.x, gyr.y, gyr.z)) {
                updateMetrics();
                updateRainMakerMetrics();
            }
        }
    }
    
    // Handle reset button
    if (digitalRead(RESET_BUTTON_PIN) == LOW) {
        Serial.printf("Reset Button Pressed!\n");
        delay(100);
        int startTime = millis();
        while (digitalRead(RESET_BUTTON_PIN) == LOW) delay(50);
        int endTime = millis();

        if ((endTime - startTime) > 10000) {
            Serial.printf("Factory reset initiated\n");
            wifi_connected = false;
            RMakerFactoryReset(2);
        } else if ((endTime - startTime) > 3000) {
            Serial.printf("Wi-Fi reset initiated\n");
            wifi_connected = false;
            RMakerWiFiReset(2);
        }
    }

    // Handle power button
    int buttonState = digitalRead(SYS_OUT_PIN);
    
    if (buttonState == LOW && !buttonPressed) {
        buttonPressed = true;
        pressStartTime = millis();
    }
    
    if (buttonPressed && buttonState == HIGH) {
        unsigned long pressDuration = millis() - pressStartTime;
        buttonPressed = false;
        
        if (pressDuration >= LONG_PRESS_DURATION) {
            Serial.println("Long press detected. Shutting down...");
            digitalWrite(SYS_EN_PIN, LOW);
        } else {
            Serial.println("Short press detected.");
        }
    }

    delay(1);
} 