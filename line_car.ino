
#include <AccelStepper.h>

// ========== MOTOR PINS ==========
#define LEFT_TOP_STEP 36
#define LEFT_TOP_DIR 34
#define LEFT_TOP_EN 30

#define LEFT_BOT_STEP 26
#define LEFT_BOT_DIR 28
#define LEFT_BOT_EN 24

#define RIGHT_TOP_STEP 60
#define RIGHT_TOP_DIR 61
#define RIGHT_TOP_EN 56

#define RIGHT_BOT_STEP 54
#define RIGHT_BOT_DIR 55
#define RIGHT_BOT_EN 38

// ========== SENSOR PINS ==========
#define LINE_S1 35  // PHẢI ngoài
#define LINE_S2 31
#define LINE_S3 27  // Giữa
#define LINE_S4 23
#define LINE_S5 16  // TRÁI ngoài

#define TRIG_PIN 42
#define ECHO_PIN 40

// ========== THÔNG SỐ BO GÓC  ==========
#define MAX_SPEED 4500.0          // Tốc độ MAX của bánh nhanh
#define BASE_SPEED 2800.0         // Tốc độ đi thẳng

#define STOP_SPEED 80.0           // Bánh chậm GẦN DỪNG 
#define FULL_SPEED 4200.0         // Bánh nhanh TOÀN TỐC 

#define MIN_DISTANCE 20

// ========== PID ==========
float Kp = 5.0;      
float Ki = 0.0;
float Kd = 2.5;      

int weights[5] = {2, 1, 0, -1, -2};

float error = 0;
float lastError = 0;

// ========== MOTOR OBJECTS ==========
AccelStepper leftTop(AccelStepper::DRIVER, LEFT_TOP_STEP, LEFT_TOP_DIR);
AccelStepper leftBot(AccelStepper::DRIVER, LEFT_BOT_STEP, LEFT_BOT_DIR);
AccelStepper rightTop(AccelStepper::DRIVER, RIGHT_TOP_STEP, RIGHT_TOP_DIR);
AccelStepper rightBot(AccelStepper::DRIVER, RIGHT_BOT_STEP, RIGHT_BOT_DIR);

unsigned long lastDebug = 0;
bool isRunning = true;

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("║  XE DÒ LINE ║");
  
  // Enable
  pinMode(LEFT_TOP_EN, OUTPUT);
  pinMode(LEFT_BOT_EN, OUTPUT);
  pinMode(RIGHT_TOP_EN, OUTPUT);
  pinMode(RIGHT_BOT_EN, OUTPUT);
  
  digitalWrite(LEFT_TOP_EN, LOW);
  digitalWrite(LEFT_BOT_EN, LOW);
  digitalWrite(RIGHT_TOP_EN, LOW);
  digitalWrite(RIGHT_BOT_EN, LOW);
  
  // Setup motors
  leftTop.setMaxSpeed(MAX_SPEED);
  leftBot.setMaxSpeed(MAX_SPEED);
  rightTop.setMaxSpeed(MAX_SPEED);
  rightBot.setMaxSpeed(MAX_SPEED);
  
  leftTop.setSpeed(0);
  leftBot.setSpeed(0);
  rightTop.setSpeed(0);
  rightBot.setSpeed(0);
  
  // Sensors
  pinMode(LINE_S1, INPUT);
  pinMode(LINE_S2, INPUT);
  pinMode(LINE_S3, INPUT);
  pinMode(LINE_S4, INPUT);
  pinMode(LINE_S5, INPUT);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  Serial.println("  CẤU HÌNH:");
  Serial.println("   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.print("   Kp = "); Serial.print(Kp); Serial.println(" ← PHẢN ỨNG NHANH");
  Serial.print("   Kd = "); Serial.println(Kd);
  Serial.println();
  Serial.print("   Đi thẳng: "); Serial.print((int)BASE_SPEED); Serial.println(" | "); Serial.print((int)BASE_SPEED);
  Serial.println();
  
  delay(3000);
  Serial.println(" BẮT ĐẦU!\n");
}

void loop() {
  // ========== KIỂM TRA VẬT CẢN ==========
  static unsigned long lastDistCheck = 0;
  
  if (millis() - lastDistCheck > 100) {
    lastDistCheck = millis();
    
    long distance = getDistance();
    
    if (distance > 0 && distance < MIN_DISTANCE) {
      if (isRunning) {
        isRunning = false;
        Serial.println("\n DỪNG!\n");
      }
      
      leftTop.setSpeed(0);
      leftBot.setSpeed(0);
      rightTop.setSpeed(0);
      rightBot.setSpeed(0);
      
      leftTop.runSpeed();
      leftBot.runSpeed();
      rightTop.runSpeed();
      rightBot.runSpeed();
      return;
    }
    
    if (!isRunning && distance >= MIN_DISTANCE + 5) {
      isRunning = true;
      Serial.println("✓ CHẠY\n");
    }
  }
  
  if (!isRunning) return;
  
  // ========== ĐỌC CẢM BIẾN ==========
  int sensors[5];
  sensors[0] = digitalRead(LINE_S1);
  sensors[1] = digitalRead(LINE_S2);
  sensors[2] = digitalRead(LINE_S3);
  sensors[3] = digitalRead(LINE_S4);
  sensors[4] = digitalRead(LINE_S5);
  
  // ========== TÍNH ERROR ==========
  int sum = 0;
  int count = 0;
  
  for (int i = 0; i < 5; i++) {
    if (sensors[i] == 0) {
      sum += weights[i];
      count++;
    }
  }
  
  if (count > 0) {
    error = (float)sum / count;
  } else {
    // Mất line (giữ hướng cũ, tăng mạnh)
    error = lastError * 2.0;
    if (abs(error) < 2.0) {
      error = lastError > 0 ? 2.5 : -2.5;
    }
  }
  
  // ========== PID  ==========
  float derivative = error - lastError;
  float turnValue = (Kp * error) + (Kd * derivative);
  
  // Cho phép turnValue lớn
  turnValue = constrain(turnValue, -10.0, 10.0);
  
  lastError = error;
  
  // ========== BO GÓC ==========
  float leftSpeed, rightSpeed;
  
  // Tính intensity (0.0 = thẳng, 1.0 = rẽ tối đa)
  float intensity = abs(turnValue) / 10.0;  // 0.0 → 1.0
  intensity = constrain(intensity, 0.0, 1.0);
  
  if (abs(turnValue) < 0.3) {
    // ========== ĐI THẲNG - 2 BÁNH BẰNG NHAU ==========
    leftSpeed = BASE_SPEED;
    rightSpeed = BASE_SPEED;
    
  } else if (turnValue < 0) {
    // ========== RẼ TRÁI ==========
    // Công thức: Lerp từ BASE → STOP/FULL
    
    // Bánh TRÁI (chậm): giảm từ BASE xuống STOP
    leftSpeed = BASE_SPEED - (BASE_SPEED - STOP_SPEED) * intensity;
    
    // Bánh PHẢI (nhanh): tăng từ BASE lên FULL
    rightSpeed = BASE_SPEED + (FULL_SPEED - BASE_SPEED) * intensity;
    
  } else {
    // ========== RẼ PHẢI ==========
    
    // Bánh TRÁI (nhanh): tăng từ BASE lên FULL
    leftSpeed = BASE_SPEED + (FULL_SPEED - BASE_SPEED) * intensity;
    
    // Bánh PHẢI (chậm): giảm từ BASE xuống STOP
    rightSpeed = BASE_SPEED - (BASE_SPEED - STOP_SPEED) * intensity;
  }
  
  // Giới hạn an toàn
  leftSpeed = constrain(leftSpeed, STOP_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, STOP_SPEED, MAX_SPEED);
  
  // ========== SET TỐC ĐỘ ==========
  leftTop.setSpeed(leftSpeed);
  leftBot.setSpeed(leftSpeed);
  rightTop.setSpeed(rightSpeed);
  rightBot.setSpeed(rightSpeed);
  
  // ========== CHẠY ==========
  leftTop.runSpeed();
  leftBot.runSpeed();
  rightTop.runSpeed();
  rightBot.runSpeed();
  
  // ========== DEBUG ==========
  if (millis() - lastDebug > 100) {  // Debug nhanh hơn
    lastDebug = millis();
    
    Serial.print("S:[");
    for (int i = 0; i < 5; i++) {
      Serial.print(sensors[i] == 0 ? "■" : "□");
      if (i < 4) Serial.print(" ");
    }
    Serial.print("] ");
    
    // Hiển thị hướng
    if (count == 0) {
      Serial.print("MISS");
    } else {
      if (abs(error) < 0.3) Serial.print("⬆️⬆️");
      else if (error < -1.5) Serial.print("⬅️⬅️⬅️");
      else if (error < -0.5) Serial.print("⬅️⬅️");
      else if (error < 0) Serial.print("⬅️");
      else if (error > 1.5) Serial.print("➡️➡️➡️");
      else if (error > 0.5) Serial.print("➡️➡️");
      else Serial.print("➡️");
    }
    
    Serial.print(" | E:");
    Serial.print(error, 2);
    Serial.print(" T:");
    Serial.print(turnValue, 2);
    Serial.print(" I:");
    Serial.print(intensity, 2);
    
    Serial.print(" | L:");
    Serial.print((int)leftSpeed);
    Serial.print(" R:");
    Serial.print((int)rightSpeed);
    
    // Chênh lệch
    int diff = abs((int)leftSpeed - (int)rightSpeed);
    Serial.print(" | Δ:");
    Serial.print(diff);
    
    // Tỷ lệ
    float ratio = max(leftSpeed, rightSpeed) / min(leftSpeed, rightSpeed);
    Serial.print(" (1:");
    Serial.print(ratio, 1);
    Serial.print(")");
    
    // Hiển thị % so với max
    int leftPercent = (leftSpeed / FULL_SPEED) * 100;
    int rightPercent = (rightSpeed / FULL_SPEED) * 100;
    Serial.print(" [");
    Serial.print(leftPercent);
    Serial.print("%|");
    Serial.print(rightPercent);
    Serial.print("%]");
    
    Serial.println();
  }
}

long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999;
  
  return duration * 0.034 / 2;
}

