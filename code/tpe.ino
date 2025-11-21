// Third Eye Project (TEP)
// Assistive device for visually impaired individuals

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

// MPU-9265 I2C address
#define MPU_ADDR 0x68

// OLED display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pin definitions
const int trigPin = 9;
const int echoPin = 10;
const int buzzerPin = 8;
const int servoPin = 11;

// Component objects
Servo eyeServo;

// IMU variables
int16_t accelerometerX, accelerometerY, accelerometerZ;
int16_t gyroX, gyroY, gyroZ;
float pitch = 0;

// Distance measurement variables
long duration;
int distance;

// Servo control variables
int servoPos = 90;  // Center position
int targetServoPos = 90;

// Buzzer control variables
unsigned long previousBuzzerTime = 0;
int buzzerInterval = 1000;  // Start with 1 second between beeps
int buzzerFrequency = 1000; // Start with 1kHz frequency

// Eye bitmap for OLED display
const unsigned char eyeBitmap [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Initialize I2C communication
  Wire.begin();
  
  // Initialize MPU-9265
  setupMPU();
  
  // Initialize OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Initialize buzzer pin
  pinMode(buzzerPin, OUTPUT);
  
  // Initialize servo
  eyeServo.attach(servoPin);
  eyeServo.write(servoPos);
  
  // Display startup screen
  display.clearDisplay();
  display.drawBitmap(0, 0, eyeBitmap, 128, 64, WHITE);
  display.display();
  delay(2000);
}

void loop() {
  // Read IMU data
  readMPU();
  
  // Calculate pitch from accelerometer data
  calculatePitch();
  
  // Adjust servo position based on pitch
  adjustServo();
  
  // Measure distance with ultrasonic sensor
  measureDistance();
  
  // Control buzzer based on distance
  controlBuzzer();
  
  // Update OLED display
  updateDisplay();
  
  // Small delay for stability
  delay(50);
}

void setupMPU() {
  // Wake up the MPU-9265
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Set to zero (wakes up the MPU-9265)
  Wire.endTransmission(true);
}

void readMPU() {
  // Start communication with MPU-9265
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);  // Read 14 registers
  
  // Read accelerometer data
  accelerometerX = Wire.read() << 8 | Wire.read();
  accelerometerY = Wire.read() << 8 | Wire.read();
  accelerometerZ = Wire.read() << 8 | Wire.read();
  
  // Read temperature data (not used)
  Wire.read() << 8 | Wire.read();
  
  // Read gyroscope data
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

void calculatePitch() {
  // Calculate pitch from accelerometer data
  pitch = atan2(-accelerometerX, sqrt(accelerometerY * accelerometerY + accelerometerZ * accelerometerZ)) * 180 / PI;
}

void adjustServo() {
  // Map pitch angle to servo position (compensate for head tilt)
  // When head tilts down (positive pitch), servo should move up (decrease position)
  // When head tilts up (negative pitch), servo should move down (increase position)
  targetServoPos = 90 - map(pitch, -90, 90, -45, 45);
  
  // Constrain servo position to safe limits
  targetServoPos = constrain(targetServoPos, 0, 180);
  
  // Smoothly move servo to target position
  if (servoPos < targetServoPos) {
    servoPos++;
  } else if (servoPos > targetServoPos) {
    servoPos--;
  }
  
  eyeServo.write(servoPos);
}

void measureDistance() {
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Set the trigPin HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance in centimeters
  distance = duration * 0.034 / 2;
}

void controlBuzzer() {
  unsigned long currentTime = millis();
  
  // Only activate buzzer if object is within 200 cm
  if (distance > 0 && distance < 200) {
    // Map distance to buzzer parameters
    // Closer objects = higher frequency and shorter intervals
    buzzerFrequency = map(distance, 2, 200, 3000, 500);
    buzzerInterval = map(distance, 2, 200, 100, 1000);
    
    // Constrain values to reasonable limits
    buzzerFrequency = constrain(buzzerFrequency, 500, 3000);
    buzzerInterval = constrain(buzzerInterval, 100, 1000);
    
    // Activate buzzer in intervals
    if (currentTime - previousBuzzerTime >= buzzerInterval) {
      tone(buzzerPin, buzzerFrequency, 100); // Beep for 100ms
      previousBuzzerTime = currentTime;
    }
  } else {
    // No object detected or out of range, turn off buzzer
    noTone(buzzerPin);
  }
}

void updateDisplay() {
  display.clearDisplay();
  
  // Draw eye bitmap
  display.drawBitmap(0, 0, eyeBitmap, 128, 64, WHITE);
  
  // Display distance information
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(10, 10);
  display.print("Distance: ");
  display.print(distance);
  display.print(" cm");
  
  // Display pitch information
  display.setCursor(10, 20);
  display.print("Pitch: ");
  display.print(pitch);
  display.print(" deg");
  
  display.display();
}