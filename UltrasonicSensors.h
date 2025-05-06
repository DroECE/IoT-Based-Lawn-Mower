#ifndef ULTRASONIC_SENSORS_H
#define ULTRASONIC_SENSORS_H

class UltrasonicSensors {
public:
    UltrasonicSensors();
    
    // Initialize all three sensors
    void begin(int trigLeft, int echoLeft,
               int trigMid, int echoMid,
               int trigRight, int echoRight);
               
    // Non-blocking distance measurements
    void update();
    
    // Get the last measured distances
    int getLeftDistance();
    int getMidDistance();
    int getRightDistance();
    
    // Check if path is clear
    bool isPathClear();
    
    // Configure thresholds
    void setObstacleThreshold(int cm);
    
private:
    // Pin configurations
    int _trigPinLeft, _echoPinLeft;
    int _trigPinMid, _echoPinMid;
    int _trigPinRight, _echoPinRight;
    
    // Last measured distances
    volatile int _leftDistance;
    volatile int _midDistance;
    volatile int _rightDistance;
    
    // Timing variables for non-blocking measurements
    unsigned long _lastUpdateTime;
    static const unsigned long UPDATE_INTERVAL = 100; // Update every 100ms
    
    // Threshold for obstacle detection (in cm)
    int _obstacleThreshold;
    
    // Helper function to measure distance for a single sensor
    int measureDistance(int trigPin, int echoPin);
};

#endif