#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "BluetoothSerial.h"
#include "MotorControl.h"
#include <math.h>
#include "UltrasonicSensors.h"

// Pin Definitions for GPS
#define GPS_RX_PIN 16   // ESP32 RX Pin
#define GPS_TX_PIN 17   // ESP32 TX Pin

// Pin Definitions for Compass (HMC5883L I2C)
#define SDA_PIN 21      // ESP32 SDA Pin
#define SCL_PIN 22      // ESP32 SCL Pin

#define TRIG_PIN_LEFT 12
#define ECHO_PIN_LEFT 13
#define TRIG_PIN_MID 14
#define ECHO_PIN_MID 15
#define TRIG_PIN_RIGHT 18
#define ECHO_PIN_RIGHT 19

UltrasonicSensors ultrasonicSensors;


#define RELAY_PIN 27  // ESP32 GPIO pin for relay control
bool relayState = false;  // Track relay state

// Mode Control
bool manualMode = false;  // false = autonomous GPS mode, true = manual control mode
const int MANUAL_SPEED = 150;  // Speed for manual control (0-255)


// GPS and Compass instances
TinyGPSPlus gps;                               // GPS library instance
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);  // Compass instance
HardwareSerial GPS_Serial(1);                  // Use HardwareSerial 1 for GPS

// Bluetooth Setup
BluetoothSerial SerialBT;
String receivedData = "";                      // Buffer for incoming Bluetooth data

// Navigation Variables
float currentLat = 0.0, currentLong = 0.0;     // Current GPS coordinates
int currentHeading = 0, headingError = 0;      // Compass heading and error
int targetHeading;                             // Target heading
int distanceToTarget = 0;                      // Distance to the target in meters

// State Management
bool stopFlag = false;                         // Stops all actions when true
bool targetReachedFlag = false;                // Indicates if the target has been reached
bool allTargetsReached = false;                // Indicates if all target coordinates have been reached

// Thresholds and Tolerances
#define HEADING_TOLERANCE 8                    // Degrees within which heading is considered correct
#define WAYPOINT_DIST_TOLERANCE 1.5            // Distance in meters to consider waypoint reached

// PID Controller Variables
float Kp = 1.0, Ki = 0, Kd = 0.83;              // PID constants (tune these for your system)
float previousError = 0;                       // Previous error for derivative term
float integral = 0;                            // Integral term for PID
float error_output = 0;                        // PID controller output

// Magnetic Declination for Compass
float magneticDeclination = -3.19; // Use this to correct for any bias errors from true north

// Compass Calibration Values 
float magX_min = -25.36;
float magY_min = -39.00;
float magZ_min = -1.00;
float magX_max = 50.45;
float magY_max = 34.55;
float magZ_max = 1.00;
bool useCalibratedMag = true;  // Use calibration values if set to true


bool isCalibrating = false;
unsigned long calibrationStartTime = 0;
const unsigned long CALIBRATION_DURATION = 20000; // 20 seconds in milliseconds
float magX_min_temp = 32767;
float magX_max_temp = -32768;
float magY_min_temp = 32767;
float magY_max_temp = -32768;

// Multiple Target Locations
struct Location {
    float latitude;
    float longitude;
};

Location targetLocations[6]; // Array to hold up to 6 target locations
int currentTargetIndex = 0;  // Index of the current target location

// Previous heading value for comparison
float previousHeading = -1;  // Initialize with an invalid heading to ensure the first print

void setup() {
    Serial.begin(115200);                                     // For debugging
    GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); // GPS initialization

    // Initialize Bluetooth
    SerialBT.begin("IoT Based Lawn Mower");
    Serial.println("Bluetooth ready. Connect and send commands.");

    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);  // Start with relay off

    // Initialize Compass
    if (!compass.begin()) {
        Serial.println("Error: Compass not found!");
        while (1);
    }
    Serial.println("Compass initialized.");

    setupMotors();                                            // Initialize motor pins
    Serial.println("System ready. Waiting for coordinates...");
  
    // Initialize ultrasonic sensors
    ultrasonicSensors.begin(TRIG_PIN_LEFT, ECHO_PIN_LEFT,
                           TRIG_PIN_MID, ECHO_PIN_MID,
                           TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
    ultrasonicSensors.setObstacleThreshold(30); // 30cm threshold
}


void loop() {
    handleBluetooth();

    if (isCalibrating) {
        calibrateCompass();
        return;  // Skip other processing while calibrating
    }

    if (manualMode) {
        // In manual mode, all control is handled by parseBluetoothData()
        return;
    }

    // Existing autonomous navigation code
    if (stopFlag) {
        stopMotors();
        return;
    }

    updateGPS();
    updateCompass();
    navigateToTarget();
}

// GPS Data Reading
void updateGPS() {
    while (GPS_Serial.available() > 0) {
        if (gps.encode(GPS_Serial.read())) {
            if (gps.location.isValid()) {
                currentLat = gps.location.lat();
               currentLong = gps.location.lng();
                
                //Serial.printf("Current Location: Lat = %.6f, Long = %.6f\n", currentLat, currentLong);
            }
        }
    }
}

// Compass Data Reading
void updateCompass() {
    sensors_event_t event;
    compass.getEvent(&event);
  
    float mag_x, mag_y;
    if (useCalibratedMag) {
        mag_x = mapf(event.magnetic.x, magX_min, magX_max, -100, 100);
        mag_y = mapf(event.magnetic.y, magY_min, magY_max, -100, 100);
    } else {
        mag_x = event.magnetic.x;
        mag_y = event.magnetic.y;
    }
    currentHeading = ((atan2(mag_y, mag_x) * 180) / PI) + magneticDeclination;

    if (currentHeading < 0) {
        currentHeading += 360;
    }
    if (currentHeading > 360) {
        currentHeading -= 360;
    }

    // Print only when the heading changes significantly
    if (abs(currentHeading - previousHeading) > 1.0) { // 1 degree threshold
        Serial.printf("Heading: %.2f°\n", currentHeading);
        previousHeading = currentHeading;
    }
}

void calibrateCompass() {
    sensors_event_t event;
    compass.getEvent(&event);
    
    // Update min/max values for X and Y
    if (event.magnetic.x < magX_min_temp) magX_min_temp = event.magnetic.x;
    if (event.magnetic.x > magX_max_temp) magX_max_temp = event.magnetic.x;
    if (event.magnetic.y < magY_min_temp) magY_min_temp = event.magnetic.y;
    if (event.magnetic.y > magY_max_temp) magY_max_temp = event.magnetic.y;
    
    // Calculate time remaining
    unsigned long timeElapsed = millis() - calibrationStartTime;
    unsigned long timeRemaining = (CALIBRATION_DURATION - timeElapsed) / 1000;
    
    // Send feedback every second
    if (timeElapsed % 1000 < 100) {  // Send approx every second
        SerialBT.printf("Calibrating... %lu seconds remaining\n", timeRemaining);
        Serial.printf("Current X: %.2f, Y: %.2f\n", event.magnetic.x, event.magnetic.y);
    }
    
    // Check if calibration is complete
    if (timeElapsed >= CALIBRATION_DURATION) {
        // Save the calibrated values
        magX_min = magX_min_temp;
        magX_max = magX_max_temp;
        magY_min = magY_min_temp;
        magY_max = magY_max_temp;
        
        // Print results
        String calibrationResults = "Calibration complete!\n";
        calibrationResults += "X min: " + String(magX_min) + ", X max: " + String(magX_max) + "\n";
        calibrationResults += "Y min: " + String(magY_min) + ", Y max: " + String(magY_max);
        
        Serial.println(calibrationResults);
        SerialBT.println(calibrationResults);
        
        // Reset calibration state
        isCalibrating = false;
        stopMotors();  // Ensure motors are stopped
        useCalibratedMag = true;  // Enable use of calibrated values
    }
}

// Bluetooth Data Handling
void handleBluetooth() {
    while (SerialBT.available()) {
        char incomingChar = SerialBT.read();
        if (incomingChar == '\n') {
            parseBluetoothData();
            receivedData = "";  // Clear buffer
        } else {
            receivedData += incomingChar;
        }
    }
}

// Parse Bluetooth data into coordinates or commands
void parseBluetoothData() {

    if (receivedData == "cc") {
        // Start calibration
        isCalibrating = true;
        calibrationStartTime = millis();
        
        // Reset temporary min/max values
        magX_min_temp = 32767;
        magX_max_temp = -32768;
        magY_min_temp = 32767;
        magY_max_temp = -32768;
        
        // Start rotating the robot
        sharpRight();  // Start rotating right for calibration
        
        Serial.println("Starting compass calibration...");
        SerialBT.println("Starting compass calibration. Please wait 20 seconds...");
        return;
    }
    
    // If we're calibrating, ignore other commands except stop
    if (isCalibrating && receivedData != "s") {
        SerialBT.println("Calibration in progress, please wait...");
        return;
    }
    // Common commands that work in both modes
    if (receivedData == "c") {  // Relay control - works in both modes
        digitalWrite(RELAY_PIN, LOW);
        relayState = true;
        Serial.println("Relay turned ON");
        SerialBT.println("Relay turned ON");
        return;  // Exit after handling relay command
    } else if (receivedData == "x") {  // Relay control - works in both modes
        digitalWrite(RELAY_PIN, HIGH);
        relayState = false;
        Serial.println("Relay turned OFF");
        SerialBT.println("Relay turned OFF");
        return;  // Exit after handling relay command
    }
    
    // Mode switching and other commands
    if (receivedData == "l") {
        sendLocationViaBluetooth();
    } else if (receivedData == "s") {
        stopFlag = true;
        stopMotors();
        digitalWrite(RELAY_PIN, HIGH);
        relayState = false;
        Serial.println("STOP command received. Motors stopped.");
        SerialBT.println("STOP command received. Motors stopped.");
    } else if (receivedData == "g") {
        stopFlag = false;
        targetReachedFlag = false;
        allTargetsReached = false;
        manualMode = false;  // Exit manual mode when starting GPS navigation
        Serial.println("GO command received. Resuming autonomous operation.");
        SerialBT.println("GO command received. Resuming autonomous operation.");
    } else if (receivedData == "m") {  // Enter manual mode
        manualMode = true;
        stopFlag = true;  // Stop any ongoing autonomous movement
        Serial.println("Entering manual control mode");
        SerialBT.println("Entering manual control mode");
    } else if (receivedData == "a") {  // Enter autonomous mode
        manualMode = false;
        Serial.println("Entering autonomous mode");
        SerialBT.println("Entering autonomous mode");
    } else if (manualMode) {  // Handle manual control commands
        switch (receivedData[0]) {
            case '8':  // Forward
                moveForward();
                Serial.println("Manual: Moving Forward");
                break;
            case '2':  // Backward
                moveBackward();
                Serial.println("Manual: Moving Backward");
                break;
            case '4':  // Left
                turnLeft();
                Serial.println("Manual: Turning Left");
                break;
            case '6':  // Right
                turnRight();
                Serial.println("Manual: Turning Right");
                break;
            case '5':  // Stop
                stopMotors();
                Serial.println("Manual: Stopped");
                break;
        }
    } else {
        parseTargetLocations(receivedData);
    }
}

// Parse multiple target coordinates in format [[lat,long], [lat,long], ...]
// Parse multiple target coordinates in format [[lat,long], [lat,long], ...]
void parseTargetLocations(String data) {
    int index = 0;
    int startPos = 0;
    
    // Clear any previous coordinates
    memset(targetLocations, 0, sizeof(targetLocations));
    
    // Find the outermost brackets
    int firstBracket = data.indexOf('[');
    int lastBracket = data.lastIndexOf(']');
    
    if (firstBracket == -1 || lastBracket == -1) {
        Serial.println("Invalid format: Missing outer brackets");
        SerialBT.println("Error: Use format [[lat,long], [lat,long], ...]");
        return;
    }
    
    // Extract the content between outer brackets
    String coordsArray = data.substring(firstBracket + 1, lastBracket);
    startPos = 0;
    
    while (startPos < coordsArray.length() && index < 6) {
        // Find the next coordinate pair
        int coordStart = coordsArray.indexOf('[', startPos);
        if (coordStart == -1) break;
        
        int coordEnd = coordsArray.indexOf(']', coordStart);
        if (coordEnd == -1) break;
        
        // Extract the coordinate pair string
        String coordPair = coordsArray.substring(coordStart + 1, coordEnd);
        
        // Find the comma separator
        int commaPos = coordPair.indexOf(',');
        if (commaPos != -1) {
            String latStr = coordPair.substring(0, commaPos);
            String lonStr = coordPair.substring(commaPos + 1);
            
            // Convert to float and store
            float lat = latStr.toFloat();
            float lon = lonStr.toFloat();
            
            // Store coordinates if they are valid
            if (isValidCoordinate(lat, lon)) {
                targetLocations[index].latitude = lat;
                targetLocations[index].longitude = lon;
                index++;
                
                // Debug output only to Serial, not to SerialBT
                Serial.printf("Added target %d: [%.6f, %.6f]\n", 
                            index, lat, lon);
            }
        }
        
        // Move to next coordinate pair
        startPos = coordEnd + 1;
    }
    
    if (index > 0) {
        currentTargetIndex = 0;
        stopFlag = false;
        targetReachedFlag = false;
        allTargetsReached = false;
        
        // Simple target count message to SerialBT
        SerialBT.printf("New targets set: %d\n", index);
        
        // Detailed debug info only to Serial monitor
        Serial.printf("New targets set: %d locations\n", index);
        Serial.print("Parsed coordinates: [");
        for (int i = 0; i < index; i++) {
            Serial.printf("[%.6f, %.6f]%s", 
                        targetLocations[i].latitude, 
                        targetLocations[i].longitude,
                        (i < index - 1) ? ", " : "");
        }
        Serial.println("]");
    } else {
        Serial.println("Invalid coordinate format!");
        SerialBT.println("Error");
    }
}

// Helper function to validate coordinates
bool isValidCoordinate(float lat, float lon) {
    // Basic coordinate validation
    if (lat < -90 || lat > 90) return false;  // Invalid latitude
    if (lon < -180 || lon > 180) return false; // Invalid longitude
    if (isnan(lat) || isnan(lon)) return false; // NaN check
    
    return true;
}


void sendLocationViaBluetooth() {
    if (gps.location.isValid()) {
        int satellites = gps.satellites.value();
        float hdop = gps.hdop.hdop();

        // Format current location in the new array format
        SerialBT.printf("Current Location: [[%.6f, %.6f]]\n", 
                       currentLat, currentLong);
        SerialBT.printf("Satellites: %d, HDOP: %.2f\n", 
                       satellites, hdop);

        Serial.printf("Satellites: %d, HDOP: %.2f\n", 
                     satellites, hdop);
    } else {
        SerialBT.println("GPS location unavailable.");
    }
}

// Navigation Logic
void navigateToTarget() {
    if (currentTargetIndex >= 6 || targetLocations[currentTargetIndex].latitude == 0.0 || targetLocations[currentTargetIndex].longitude == 0.0) {
        if (!allTargetsReached) {
            Serial.println("No target coordinates set or all targets reached.");
            allTargetsReached = true;  // Set flag to indicate message has been printed
        }
        return;
    }

    float targetLat = targetLocations[currentTargetIndex].latitude;
    float targetLong = targetLocations[currentTargetIndex].longitude;

    distanceToTarget = calculateDistance(targetLat, targetLong);
    targetHeading = calculateHeading(targetLat, targetLong);

    Serial.printf("Distance to Target: %d meters\n", distanceToTarget);
    Serial.printf("Target Heading: %d°, Current Heading: %d°\n", targetHeading, currentHeading);

    // If robot reaches target, stop motors and send message
    if (distanceToTarget <= WAYPOINT_DIST_TOLERANCE) {
        if (!targetReachedFlag) {
            Serial.println("Target reached!");
            stopMotors();  // Ensure motors are stopped
            targetReachedFlag = true;  // Mark as reached
            SerialBT.println("Mower has arrived at the target location.");  // Send notification

            // Add a 5-second stop at the destination before proceeding to the next target
            delay(5000);  // 5000 milliseconds = 5 seconds
            Serial.println("5-second stop complete. Proceeding to the next target.");
            currentTargetIndex++;  // Move to the next target
        }
        return;
    }

    pidController();
    move_robot();
}

// PID Controller for heading adjustment
void pidController() {
    headingError = targetHeading - currentHeading;
    if (headingError < -180) headingError += 360;
    if (headingError > 180) headingError -= 360;

    // Integral term calculation
    integral += headingError;

    // Derivative term calculation
    float derivative = headingError - previousError;

    // PID calculation
    error_output = Kp * headingError + Ki * integral + Kd * derivative;
    previousError = headingError;

    // Debugging statements
    Serial.printf("Heading Error: %d\n", headingError);
    Serial.printf("PID Output: %f\n", error_output);
}

// Calculate Distance to Target using Haversine Formula
int calculateDistance(float targetLat, float targetLong) {
    const float R = 6371000;  // Earth's radius in meters
    float dLat = radians(targetLat - currentLat);
    float dLon = radians(targetLong - currentLong);
    float lat1 = radians(currentLat);
    float lat2 = radians(targetLat);

    float a = sin(dLat / 2) * sin(dLat / 2) +
              cos(lat1) * cos(lat2) *
              sin(dLon / 2) * sin(dLon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;  // Distance in meters
}

// Calculate Target Heading
int calculateHeading(float targetLat, float targetLong) {
    float dLon = radians(targetLong - currentLong);
    float y = sin(dLon) * cos(radians(targetLat));
    float x = cos(radians(currentLat)) * sin(radians(targetLat)) -
              sin(radians(currentLat)) * cos(radians(targetLat)) * cos(dLon);
    float bearing = atan2(y, x) * 180.0 / PI;
    return (bearing >= 0) ? bearing : (360.0 + bearing);
}

// Move Robot based on PID controller output
void move_robot() {

    ultrasonicSensors.update();

    if (!ultrasonicSensors.isPathClear()) {
        // Obstacle detected, handle avoidance
        handleObstacleAvoidance();
        return;
    }

    if (abs(error_output) <= HEADING_TOLERANCE) {
        Serial.println("Moving forward...");
        moveForward();
    } else if (error_output > 0 && error_output < 60) {
        Serial.println("Adjusting right...");
        turnRight();
    } else if (error_output < 0 && error_output > -60) {
        Serial.println("Adjusting left...");
        turnLeft();
    } else if (error_output >= 60) {
        Serial.println("Sharp right turn...");
        sharpRight();
    } else if (error_output <= -60) {
        Serial.println("Sharp left turn...");
        sharpLeft();
    }
}

void handleObstacleAvoidance() {
    int leftDist = ultrasonicSensors.getLeftDistance();
    int midDist = ultrasonicSensors.getMidDistance();
    int rightDist = ultrasonicSensors.getRightDistance();
    
    // Stop immediately
    stopMotors();
    
    // Determine best direction to avoid obstacle
    if (leftDist > rightDist && leftDist > 30) {
        // More space on the left
        sharpLeft();
        delay(500);  // Turn for 500ms
    } else if (rightDist > leftDist && rightDist > 30) {
        // More space on the right
        sharpRight();
        delay(500);  // Turn for 500ms
    } else {
        // No clear path, back up and turn
        moveBackward();
        delay(1000);  // Back up for 1 second
        sharpRight();
        delay(1000);  // Turn for 1 second
    }
    
    stopMotors();
}

// Map function for floating-point values
double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
