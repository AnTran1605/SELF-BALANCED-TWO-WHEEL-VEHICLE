#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <U8g2lib.h>
#include <PID_v1.h>

// Khởi tạo màn hình OLED với U8g2 
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Khởi tạo cảm biến MPU6050 và các thông số điều khiển
Adafruit_MPU6050 mpu;

// Khởi tạo các chân kết nối L298 với ESP32
const int motorEnablePin1 = 18;
const int motorInputPin1 = 19;
const int motorInputPin2 = 5;
const int motorEnablePin2 = 4;
const int motorInputPin3 = 16;
const int motorInputPin4 = 17;

// Chân kết nối MPU6050 với ESP32
const int MPU_SCL = 22;
const int MPU_SDA = 21;

// Offset cho MPU6050
const float accOffsetX = 1.2827;
const float accOffsetY = -0.3331;
const float accOffsetZ = 9.8546;
const float gyroOffsetX = -0.0312;
const float gyroOffsetY = 0.0104;
const float gyroOffsetZ = -0.0300;

double angle;
double OGN_setpoint = -4;
double setpoint = OGN_setpoint;
double motorSpeed;
double E;
bool mpu_found = false;

// Khai báo các tham số PID
double Kp = 1.9, Ki = 0, Kd = 0 ;
double input, output, previousInput = 0;

// Tạo đối tượng PID
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Hàm khởi tạo
void setup() {
  Serial.begin(115200);
  Wire.begin(MPU_SDA, MPU_SCL);
  u8g2.begin();

  Serial.println("Bắt đầu chương trình và khởi tạo I2C...");
  if (!mpu.begin(0x68)) {
    Serial.println("Không tìm thấy cảm biến MPU6050!");
    while (1);
  } else {
    Serial.println("MPU6050 đã được kết nối thành công!");
    mpu_found = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  pinMode(motorEnablePin1, OUTPUT);
  pinMode(motorInputPin1, OUTPUT);
  pinMode(motorInputPin2, OUTPUT);
  pinMode(motorEnablePin2, OUTPUT);
  pinMode(motorInputPin3, OUTPUT);
  pinMode(motorInputPin4, OUTPUT);

  analogWrite(motorEnablePin1, 0);
  analogWrite(motorEnablePin2, 0);

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  myPID.SetOutputLimits(-200, 200);
}


void readAngle() {
  if (mpu_found) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Bù offset cho gia tốc kế
    float accX = a.acceleration.x - accOffsetX;
    float accY = a.acceleration.y - accOffsetY;
    float accZ = a.acceleration.z - accOffsetZ;

    // Bù offset cho con quay hồi chuyển
    float gyroX = g.gyro.x - gyroOffsetX;
    float gyroY = g.gyro.y - gyroOffsetY;
    float gyroZ = g.gyro.z - gyroOffsetZ;

    // Tính toán góc nghiêng từ dữ liệu đã hiệu chỉnh
    float accAngle = atan2(accX, accZ) * 180 / PI;
    float gyroRate = gyroY * 180 / PI;

    static float lastAngle;
    angle = 0 * (lastAngle + gyroRate * 0.01) + 1  * accAngle; // Complementary filter
    lastAngle = angle;

    // In thông tin sau khi hiệu chỉnh ra Serial Monitor
    Serial.print("Gia tốc sau hiệu chỉnh: ");
    Serial.print("X: "); Serial.print(accX); Serial.print(" m/s^2, ");
    Serial.print("Y: "); Serial.print(accY); Serial.print(" m/s^2, ");
    Serial.print("Z: "); Serial.print(accZ); Serial.println(" m/s^2");

    Serial.print("Con quay hồi chuyển sau hiệu chỉnh: ");
    Serial.print("X: "); Serial.print(gyroX); Serial.print(" rad/s, ");
    Serial.print("Y: "); Serial.print(gyroY); Serial.print(" rad/s, ");
    Serial.print("Z: "); Serial.print(gyroZ); Serial.println(" rad/s");

    Serial.print("Angle: "); Serial.println(angle);
  } else {
    Serial.println("Cảm biến MPU6050 chưa sẵn sàng.");
  }

  // Cập nhật đầu vào PID (góc hiện tại)
  input = angle;
  E =  setpoint - angle ;
  
  // In thông tin PID
  Serial.print("Input: "); Serial.print(input);
  Serial.print(" Setpoint: "); Serial.print(setpoint);
}


// Hàm hiển thị dữ liệu
void displayMPU6050Data(const sensors_event_t &a) {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(15, 10);
  u8g2.print("Gia toc X: ");
  u8g2.print(a.acceleration.x); u8g2.print(" m/s^2");

  u8g2.setCursor(15, 20);
  u8g2.print("Output: "); u8g2.print(output);

  u8g2.setCursor(15, 30);
  u8g2.print("Error: "); u8g2.print(E);

  u8g2.setCursor(15, 40);
  u8g2.print("Angle: "); u8g2.print(angle); u8g2.print(" deg");

  u8g2.sendBuffer();
}

void computePID() {
  myPID.Compute();
}

void motorControl() {
  Serial.print("Motor Speed: "); Serial.println(output);
  if (output > 0) {
    analogWrite(motorEnablePin1, output);
    digitalWrite(motorInputPin1, HIGH);
    digitalWrite(motorInputPin2, LOW);

    analogWrite(motorEnablePin2, output);
    digitalWrite(motorInputPin3, HIGH);
    digitalWrite(motorInputPin4, LOW);
  } else {
    analogWrite(motorEnablePin1, -output);
    digitalWrite(motorInputPin1, LOW);
    digitalWrite(motorInputPin2, HIGH);

    analogWrite(motorEnablePin2, -output);
    digitalWrite(motorInputPin3, LOW);
    digitalWrite(motorInputPin4, HIGH);
  }
}

void loop() {
 
  sensors_event_t a, g, temp; // Khai báo các biến sự kiện
  mpu.getEvent(&a, &g, &temp); // Đọc dữ liệu cảm biến và lưu vào a, g, temp

  readAngle();
  computePID();
  motorControl();
 displayMPU6050Data(a);
  
}
