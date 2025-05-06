#include "UltrasonicSensors.h"
#include "Arduino.h"

UltrasonicSensors::UltrasonicSensors() {
    _leftDistance = 0;
    _midDistance = 0;
    _rightDistance = 0;
    _lastUpdateTime = 0;
    _obstacleThreshold = 30; // Default 30cm threshold
}

void UltrasonicSensors::begin(int trigLeft, int echoLeft,
                             int trigMid, int echoMid,
                             int trigRight, int echoRight) {
    // Store pin configurations
    _trigPinLeft = trigLeft;
    _echoPinLeft = echoLeft;
    _trigPinMid = trigMid;
    _echoPinMid = echoMid;
    _trigPinRight = trigRight;
    _echoPinRight = echoRight;
    
    // Configure pins
    pinMode(_trigPinLeft, OUTPUT);
    pinMode(_echoPinLeft, INPUT);
    pinMode(_trigPinMid, OUTPUT);
    pinMode(_echoPinMid, INPUT);
    pinMode(_trigPinRight, OUTPUT);
    pinMode(_echoPinRight, INPUT);
}

void UltrasonicSensors::update() {
    unsigned long currentTime = millis();
    
    // Only update if enough time has passed
    if (currentTime - _lastUpdateTime >= UPDATE_INTERVAL) {
        _leftDistance = measureDistance(_trigPinLeft, _echoPinLeft);
        _midDistance = measureDistance(_trigPinMid, _echoPinMid);
        _rightDistance = measureDistance(_trigPinRight, _echoPinRight);
        
        _lastUpdateTime = currentTime;
    }
}

int UltrasonicSensors::measureDistance(int trigPin, int echoPin) {
    // Send trigger pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // Measure the response
    long duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms
    
    // Convert to distance in cm
    return (duration == 0) ? 400 : duration * 0.034 / 2;
}

int UltrasonicSensors::getLeftDistance() {
    return _leftDistance;
}

int UltrasonicSensors::getMidDistance() {
    return _midDistance;
}

int UltrasonicSensors::getRightDistance() {
    return _rightDistance;
}

bool UltrasonicSensors::isPathClear() {
    return _leftDistance > _obstacleThreshold &&
           _midDistance > _obstacleThreshold &&
           _rightDistance > _obstacleThreshold;
}

void UltrasonicSensors::setObstacleThreshold(int cm) {
    _obstacleThreshold = cm;
}