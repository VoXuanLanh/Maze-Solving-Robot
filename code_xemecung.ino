// ====================== CẤU HÌNH CHÂN ĐIỀU KHIỂN ==========================
#define ENA 10   
#define IN1 9
#define IN2 8
#define ENB 6   
#define IN3 11    
#define IN4 12

// ====================== ENCODER ==========================
#define ENCL_A 2   // encoder trái - kênh A
#define ENCL_B 4   // encoder trái - kênh B
#define ENCR_A 3   // encoder phải - kênh A
#define ENCR_B 5   // encoder phải - kênh B

// ====================== CẢM BIẾN IR CHO MAZE ==========================
#define irL 24  
#define irF 22  
#define irR 26   

// ====================== BIẾN TOÀN CỤC ==========================
volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;

int baseSpeed = 78; 
int L, F, R;  
int g=0;
int a=0;
int t=0;
int t1=0;
int t2=0;
int t3=0;
int t4=0;
int t5=0;
int c=0;


// Biến đo tốc độ
unsigned long lastSpeedTime = 0;
long lastEncoderLeft = 0;
long lastEncoderRight = 0;
float leftSpeed = 0;
float rightSpeed = 0;

// ====================== HÀM ĐỌC ENCODER ==========================
void readEncoderLeft() {
  if (digitalRead(ENCL_B) == HIGH) encoderCountLeft++;
  else encoderCountLeft--;
}

void readEncoderRight() {
  if (digitalRead(ENCR_B) == HIGH) encoderCountRight++;
  else encoderCountRight--;
}

// ====================== HÀM ĐIỀU KHIỂN ĐỘNG CƠ ==========================
void motorLeft(int speed) {
  if (speed > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, speed);
  } else if (speed < 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, -speed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void motorRight(int speed) {
  if (speed > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, speed);
  } else if (speed < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, -speed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void stopMotor() {
  motorLeft(0);
  motorRight(0);
}

// ====================== HÀM ĐỌC CẢM BIẾN IR ==========================
void readIRSensors() {
  L = digitalRead(irL); 
  F = digitalRead(irF);
  R = digitalRead(irR);
}

// ====================== HÀM ĐIỀU KHIỂN TỐC ĐỘ ==========================
void speedControl() {
  motorLeft(baseSpeed+1);
  motorRight(baseSpeed);
}

// ====================== ENCODER UTILS ==========================
void resetEncoders() {
  noInterrupts();
  encoderCountLeft = 0;
  encoderCountRight = 0;
  interrupts();
}

// ====================== HÀM QUAY 90° CÓ KIỂM TRA KẸT ==========================
#define TURN_TICKS_90 109
#define STUCK_TIMEOUT 500   // 500ms không thay đổi encoder → coi như kẹt

void turnRightEncoder(long targetTicks) {
  resetEncoders();
  motorLeft(baseSpeed + 20);
  motorRight(-baseSpeed - 20);

  unsigned long lastChangeTime = millis();
  long lastLeft = 0, lastRight = 0;

  while (true) {
    noInterrupts();

    
    long left = abs(encoderCountLeft);
    long right = abs(encoderCountRight);
    interrupts();

    long avgTicks = (left + right) / 2;

    // Nếu đạt số xung mong muốn → hoàn tất quay
    if (avgTicks >= targetTicks) break;

    // Kiểm tra kẹt: encoder không đổi trong 500ms
    if (left == lastLeft && right == lastRight) {
      if (millis() - lastChangeTime > STUCK_TIMEOUT) {
        stopMotor();
        delay(100);
        motorLeft(-baseSpeed - 50);
        motorRight(-baseSpeed );
        delay(400);   // hoặc gọi hàm khác để xử lý
        return;
      }
    } else {
      lastLeft = left;
      lastRight = right;
      lastChangeTime = millis();  // reset bộ đếm thời gian
    }
  }
  stopMotor();
  delay(100);
}

void turnLeftEncoder(long targetTicks) {
  resetEncoders();
  motorLeft(-baseSpeed - 20);
  motorRight(baseSpeed + 20);

  unsigned long lastChangeTime = millis();
  long lastLeft = 0, lastRight = 0;

  while (true) {
    noInterrupts();
    long left = abs(encoderCountLeft);
    long right = abs(encoderCountRight);
    interrupts();

    long avgTicks = (left + right) / 2;

    if (avgTicks >= targetTicks) break;

    if (left == lastLeft && right == lastRight) {
      if (millis() - lastChangeTime > STUCK_TIMEOUT) {
        stopMotor();
        motorLeft(-baseSpeed );
        motorRight(-baseSpeed-50 ); 
        delay(400);  // hoặc hành động khác
        return;
      }
    } else {
      lastLeft = left;
      lastRight = right;
      lastChangeTime = millis();
    }
  }
  stopMotor();
  delay(100);
}


// ====================== CÁC HÀM HÀNH ĐỘNG ==========================
void rephai() {
  stopMotor();
  delay(100);
  turnRightEncoder(TURN_TICKS_90);  // quay phải 90°
}

void retrai() {
  stopMotor();
  delay(100);
  turnLeftEncoder(TURN_TICKS_90);   // quay trái 90°
}

void dilui() {

  motorLeft(-baseSpeed);
  motorRight(-baseSpeed+1);
  delay(70);

}

// ====================== LOGIC MAZE ==========================
void cuong_che() {
  readIRSensors();

  if (F == 1 && L == 1 && R == 0) {
    t1++;
    t2=0;
    c=1;
    t=0;
    t3=0;
    if (t1 >= 80) {
      detachInterrupt(digitalPinToInterrupt(ENCL_A));
      detachInterrupt(digitalPinToInterrupt(ENCR_A));

      stopMotor();
      delay(100);
      motorLeft(-baseSpeed-30);
      motorRight(-baseSpeed);
      delay(400);

      stopMotor();
      delay(100);
      resetEncoders();

      attachInterrupt(digitalPinToInterrupt(ENCL_A), readEncoderLeft, RISING);
      attachInterrupt(digitalPinToInterrupt(ENCR_A), readEncoderRight, RISING);
// Reset biến đếm t
      t1 = 0;
    }
}

  if (F == 1 && L == 0 && R == 1) {
    t2++;
    t1=0;
    c=0;
    t3=0;
    t=0;

    if (t2 >= 80) {
      detachInterrupt(digitalPinToInterrupt(ENCL_A));
      detachInterrupt(digitalPinToInterrupt(ENCR_A));

      stopMotor();
      delay(100);
      motorLeft(-baseSpeed);
      motorRight(-baseSpeed - 30);
      delay(400);

      stopMotor();
      delay(100);

      resetEncoders();

      attachInterrupt(digitalPinToInterrupt(ENCL_A), readEncoderLeft, RISING);
      attachInterrupt(digitalPinToInterrupt(ENCR_A), readEncoderRight, RISING);

      // Reset biến đếm t
      t2 = 0;
  }
}

  if (F == 1 && L == 0 && R == 0) { 
    t1=0;
    t2=0;
    t=0;
    t3++;
    t4++;

    if(t3>=500){
      if (c==1){
        motorLeft(-baseSpeed);
        motorRight(-baseSpeed-30);
        delay(400);
        t3=0;
        if(t4 >= 540){
        motorLeft(baseSpeed+40);
        motorRight(baseSpeed);
        delay(400);
        t4 = 0;
        c=0;

          
        }
      }
      if (c==0){
        motorLeft(-baseSpeed-30);
        motorRight(-baseSpeed);
        delay(400);
        t3=0;
        if(t4 >= 540){
        motorRight(baseSpeed+40);
        motorLeft(baseSpeed);
        delay(400);
        t4 = 0;
        c=1;
        }
      }
    }

    readIRSensors();
    speedControl(); 
    return; 
  }
  if (F == 0 && L == 0 && R == 1) { 
    t=0;
    t1=0;
    t2=0;
    t3=0;
    rephai(); return; }
  if (F == 0 && L == 1 && R == 0) { 
    t=0;
    t1=0;
    t2=0;
    t3=0;
    retrai(); 
    return; }
  if (F == 0 && L == 1 && R == 1) { 
    t1=0;
    t2=0;
    t3=0;
    stopMotor(); 
    delay(100);
    rephai(); 
    return; }
  if (F == 0 && L == 0 && R == 0) { 
    t=0;
    t1=0;
    t2=0;
    t3=0;      
    stopMotor();
    delay(100);
    g=1;
    a=1;
    unsigned long startime = millis();
    while(a==1){
      readIRSensors();
      dilui(); 
      if (millis()-startime >=2000){
        stopMotor();
        delay(100);
        motorLeft(0);
        motorRight(baseSpeed);
        delay(400);
        a=0;
      }
      
      if (R==1){
        a=0;
        stopMotor();
        delay(150);
        rephai();
        speedControl();
        delay(560);
        readIRSensors();
        if (R==1){
          stopMotor();
          delay(100);
          rephai();
          g=1;
          a=0;
          }
        if (g==0 && L==1){
          stopMotor();
          delay(100);
          retrai();
          a=0;      
        } 
      }
      if (L==1){
        a=0;
        stopMotor();
        delay(150);
        retrai();
        speedControl();
        delay(560);
        readIRSensors();
        if (R==1){
          g=1;
          stopMotor();
          delay(100);
          rephai();
          a=0;
          }
        if (L==1 && g==0){
          stopMotor();
          delay(100);
          retrai();
          a=0;          
        } 
      }      
    }
    return; }
      readIRSensors();
      if (F == 1 && L == 1 && R == 1){
        t1=0;
        t2=0;
        t3=0;
        t++;
        t5++;
        if (t>300){
          stopMotor();
          delay(100);
          if (c==1){
            motorLeft(-baseSpeed-30);
            motorRight(-baseSpeed);
            delay(350);
            if (t5>=350){
              motorLeft(baseSpeed);
              motorRight(baseSpeed+35);
              delay(350);
              t5=0;
            }
            t=0;
            c=0;
          }
          if (c==0){
            motorLeft(-baseSpeed);
            motorRight(-baseSpeed-30);
            delay(350);
            if (t5>=350){
              motorLeft(baseSpeed+35);
              motorRight(baseSpeed);
              delay(350);
              t5=0;
            }
            t=0;
            c=1;
          }
        }
      }
    speedControl();
}

// ====================== HÀM CẬP NHẬT TỐC ĐỘ ==========================
void updateSpeed() {
  unsigned long currentTime = millis();
  if (currentTime - lastSpeedTime >= 500) {
    noInterrupts();
    long currentLeft = encoderCountLeft;
    long currentRight = encoderCountRight;
    interrupts();

    leftSpeed = (float)(currentLeft - lastEncoderLeft) * 1000.0 / (currentTime - lastSpeedTime);
    rightSpeed = (float)(currentRight - lastEncoderRight) * 1000.0 / (currentTime - lastSpeedTime);

    lastEncoderLeft = currentLeft;
    lastEncoderRight = currentRight;
    lastSpeedTime = currentTime;

    Serial.print("IR_L: "); Serial.print(L);
    Serial.print(" | IR_F: "); Serial.print(F);
    Serial.print(" | IR_R: "); Serial.print(R);
    Serial.print(" || LeftSpeed: "); Serial.print(leftSpeed);
    Serial.print(" | RightSpeed: "); Serial.println(rightSpeed);
  }
}
// ====================== SETUP ==========================
void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(irL, INPUT); pinMode(irF, INPUT); pinMode(irR, INPUT);

  pinMode(ENCL_A, INPUT_PULLUP);
  pinMode(ENCL_B, INPUT_PULLUP);
  pinMode(ENCR_A, INPUT_PULLUP);
  pinMode(ENCR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCL_A), readEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCR_A), readEncoderRight, RISING);
  delay(500);
}

// ====================== LOOP ==========================
void loop() {
  cuong_che();
  updateSpeed();
  delay(10);
}