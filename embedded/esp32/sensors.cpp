#include "sensors.h"
#include <Wire.h>
#include <math.h>

//Include drivers
#include "sensors/bmp280.h"
#include "sensors/mpu6050.h"
#include "sensors/gps.h"
#include "sensors/rtc_drivers.h"


static BMP280_Driver bmp280;
static MPU6050_Driver mpu6050;
static GPS_Driver gps_neo6m;  
static RTC_Driver rtc_ds3231;

//Constructor

SensorManager::SensorManager()
    : groundPressure_hPa(1013.25),
      groundAltitude_MSL(0.0),
      calibrated(false),
      bmp280_initialized(false),
      mpu6050_initialized(false),
      gps_initialized(false),
      rtc_initialized(false) {
}

//Initialize all sensors
bool SensorManager::begin() {
    Serial.println("[SENSORS] Initializing sensor subsystem...");
    
    //Initialize I2C
    Wire.begin();
    Wire.setClock(400000);  
    
    //Initialize battery monitoring pin
    pinMode(BATTERY_PIN, INPUT);
    


    //BMP280 INITIALIZATION
    Serial.print("[SENSORS] BMP280: ");
    if (bmp280.begin(0x76)) {
        bmp280_initialized = true;
    } else if (bmp280.begin(0x77)) {
        bmp280_initialized = true;
    } else {
        Serial.println("FAILED - CRITICAL!");
        return false;  //Cannot continue without BMP280
    }
    



    //MPU6050 INITIALIZATION
    Serial.print("[SENSORS] MPU6050: ");
    if (mpu6050.begin(0x68)) {
        mpu6050_initialized = true;
    } else {
        Serial.println("FAILED - Non-critical, continuing");
        mpu6050_initialized = false;
    }
    

    //GPS INITIALIZATION
    Serial.print("[SENSORS] GPS NEO-6M: ");
    if (gps_neo6m.begin(9600)) {
        gps_initialized = true;
    } else {
        Serial.println("FAILED - Non-critical, continuing");
        gps_initialized = false;
    }

    
    //RTC INITIALIZATION
    Serial.print("[SENSORS] RTC DS3231: ");
    if (rtc_ds3231.begin()) {
        rtc_initialized = true;
    } else {
        Serial.println("FAILED - Non-critical, using millis()");
        rtc_initialized = false;
    }
    
    Serial.println("[SENSORS] Initialization complete");
    Serial.println("");
    


    //Return true if at least BMP280 initialized (critical sensor, altitude depends on it)
    return bmp280_initialized;
}

//Ground calibration 
bool SensorManager::calibrateGround(uint8_t num_samples) {
    if (!bmp280_initialized) {
        Serial.println("[SENSORS] ERROR: Cannot calibrate altitude - BMP280 not initialized");
        return false;
    }
    
    Serial.println("[SENSORS] ═══════════════════════════════════════");
    Serial.println("[SENSORS] GROUND CALIBRATION STARTING");
    Serial.println("[SENSORS] Keep CubeSat STATIONARY on launch pad!");
    Serial.println("[SENSORS] ═══════════════════════════════════════");
    
    float pressureSum = 0.0;
    float altitudeSum = 0.0;
    uint8_t validSamples = 0;
    
    for (uint8_t i = 0; i < num_samples; i++) {
        //Read from BMP280 using your driver
        float pressure = bmp280.readPressure() / 100.0;  //Pa to hPa
        float altitude = bmp280.readAltitude(1013.25);   //standard sea level
        
        if (pressure > 0 && altitude > -500) {  //Sanity check
            pressureSum += pressure;
            altitudeSum += altitude;
            validSamples++;
        }
        
        Serial.print(".");
        delay(100);  //100ms between samples
    }
    
    Serial.println(" CALIBRATION DONE!");
    
    if (validSamples < num_samples / 2) {
        Serial.println("[SENSORS] ERROR: Too many failed readings during calibration");
        return false;
    }
    
    groundPressure_hPa = pressureSum / validSamples;
    groundAltitude_MSL = altitudeSum / validSamples;
    calibrated = true;
    
    Serial.println("");
    Serial.println("[SENSORS] ===== CALIBRATION RESULTS ====");
    Serial.print("[SENSORS] Ground Pressure: ");
    Serial.print(groundPressure_hPa, 2);
    Serial.println(" hPa");
    Serial.print("[SENSORS] Ground Altitude MSL: ");
    Serial.print(groundAltitude_MSL, 2);
    Serial.println(" m");
    Serial.print("[SENSORS] Valid Samples: ");
    Serial.print(validSamples);
    Serial.print("/");
    Serial.println(num_samples);
    Serial.println("[SENSORS] ===================================");
    Serial.println("");
    
    return true;
}

//Read all sensors
 
bool SensorManager::readAll(SensorData& data) {
    //Clear error flags
    data.error_flags = 0;
    
    //Read timestamp first (consistent for all sensors)
    if (!readRTC(data)) {
        data.timestamp_ms = millis();  // Fallback to millis()
    }
    
    //Read barometric sensor (CRITICAL)
    if (!readBMP280(data)) {
        data.error_flags |= ERROR_BMP280_FAIL;
        data.pressure_hPa = 0.0;
        data.temperature_C = 0.0;
        data.altitude_MSL = 0.0;
        data.altitude_AGL = 0.0;
        data.bmp_valid = false;
    }
    
    //Read IMU (important sensor but not critical)
    if (!readMPU6050(data)) {
        data.error_flags |= ERROR_MPU6050_FAIL;
        data.pitch_deg = 0.0;
        data.roll_deg = 0.0;
        data.accel_x_g = 0.0;
        data.accel_y_g = 0.0;
        data.accel_z_g = 0.0;
        data.imu_valid = false;
    }
    
    //Read GPS (important but not critical)
    if (!readGPS(data)) {
        data.error_flags |= ERROR_GPS_NO_FIX;
        data.gps_fix = false;
    }
    
    //Read battery voltage
    data.battery_voltage = readBatteryVoltage();
    if (data.battery_voltage < LOW_BATTERY_THRESHOLD) {
        data.error_flags |= ERROR_LOW_BATTERY;
    }
    
    //Return true if at least BMP280 read successfully
    return data.bmp_valid;
}

//Read BMP280 sensor
bool SensorManager::readBMP280(SensorData& data) {
    if (!bmp280_initialized) {
        return false;
    }
    
    //Read from your BMP280 driver
    data.pressure_hPa = bmp280.readPressure() / 100.0;  //Pa to hPa
    data.temperature_C = bmp280.readTemperature();
    data.altitude_MSL = bmp280.readAltitude(1013.25);
    
    //Calculate AGL (Above Ground Level) this is what fsm needs
    if (calibrated) {
        data.altitude_AGL = data.altitude_MSL - groundAltitude_MSL;
    } else {
        data.altitude_AGL = 0.0;
    }
    
    data.bmp_valid = true;
    return true;
}

//Read MPU6050 sensor
 
bool SensorManager::readMPU6050(SensorData& data) {
    if (!mpu6050_initialized) {
        return false;
    }
    
    //Read accelerometer and gyroscope using mpu6050.h
    float ax, ay, az, gx, gy, gz;
    if (!mpu6050.read(ax, ay, az, gx, gy, gz)) {
        return false;
    }
    
    //Convert to g (m/s2 / 9.81)
    data.accel_x_g = ax / 9.81;
    data.accel_y_g = ay / 9.81;
    data.accel_z_g = az / 9.81;
    
    //Calculate orientation from accelerometer
    calculateOrientation(data.accel_x_g, data.accel_y_g, data.accel_z_g,
                        data.pitch_deg, data.roll_deg);
    
    data.imu_valid = true;
    return true;
}

//read GPS sensor
bool SensorManager::readGPS(SensorData& data) {
    if (!gps_initialized) {
        return false;
    }
    
    //Update GPS parser (feeds it serial data)
    gps_neo6m.update();
    
    //Check if we have a valid fix
    data.gps_fix = gps_neo6m.hasFix();
    
    if (data.gps_fix) {
        data.latitude = gps_neo6m.getLatitude();
        data.longitude = gps_neo6m.getLongitude();
        data.gps_altitude_m = gps_neo6m.getAltitude();
        data.gps_speed_mps = gps_neo6m.getSpeed();
        data.satellites = gps_neo6m.getSatellites();
    } else {
        //No fix - set to zero
        data.latitude = 0.0;
        data.longitude = 0.0;
        data.gps_altitude_m = 0.0;
        data.gps_speed_mps = 0.0;
        data.satellites = 0;
    }
    
    return data.gps_fix;
}

//Read RTC for timestamp
bool SensorManager::readRTC(SensorData& data) {
    if (!rtc_initialized) {
        return false;
    }
    
    //Get Unix timestamp in milliseconds from your RTC driver
    data.timestamp_ms = rtc_ds3231.getUnixTime() * 1000UL;
    data.gps_time = 0;  //Will be populated by GPS when available
    
    return true;
}

//Read battery voltage
float SensorManager::readBatteryVoltage() {
    //Read ADC (0-4095 for ESP32)
    int adcValue = analogRead(BATTERY_PIN);
    
    //Convert to voltage (ESP32: 3.3V reference, 12-bit ADC)
    float voltage = (adcValue / 4095.0) * 3.3;
    
    // Account for voltage divider
    voltage *= BATTERY_DIVIDER_RATIO;
    
    return voltage;
}

/**
 * Calculate pitch and roll from accelerometer
 */
void SensorManager::calculateOrientation(float ax, float ay, float az,
                                        float& pitch, float& roll) {
    // Calculate pitch (rotation around Y-axis)
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
    
    // Calculate roll (rotation around X-axis)
    roll = atan2(ay, az) * 180.0 / M_PI;
}

/**
 * Get ground pressure
 */
float SensorManager::getGroundPressure() const {
    return groundPressure_hPa;
}

/**
 * Get ground altitude
 */
float SensorManager::getGroundAltitude() const {
    return groundAltitude_MSL;
}

/**
 * Check if calibrated
 */
bool SensorManager::isCalibrated() const {
    return calibrated;
}

/**
 * Print sensor status
 */
void SensorManager::printStatus() const {
    Serial.println("");
    Serial.println("[SENSORS] ═══ SENSOR STATUS ═══");
    Serial.print("[SENSORS] BMP280:  ");
    Serial.println(bmp280_initialized ? "ONLINE" : "OFFLINE");
    Serial.print("[SENSORS] MPU6050: ");
    Serial.println(mpu6050_initialized ? "ONLINE" : "OFFLINE");
    Serial.print("[SENSORS] GPS:     ");
    Serial.println(gps_initialized ? "ONLINE" : "OFFLINE");
    Serial.print("[SENSORS] RTC:     ");
    Serial.println(rtc_initialized ? "ONLINE" : "OFFLINE");
    Serial.print("[SENSORS] Ground Calibration: ");
    Serial.println(calibrated ? "YES" : "NO");
    Serial.println("[SENSORS] ═══════════════════");
    Serial.println("");
}