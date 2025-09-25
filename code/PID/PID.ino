#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <U8g2lib.h>
#include <L298N.h>
#include <PID_v1.h>  // Thêm thư viện PID_v1.h

// Khởi tạo màn hình OLED với U8g2 
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); // Sử dụng I2C phần cứng

// Khởi tạo cảm biến MPU6050 và các thông số điều khiển
Adafruit_MPU6050 mpu;

// Khởi tạo các chân kết nối L298 với ESP32
const int motorEnablePin1 = 18; // PWM điều khiển tốc độ động cơ 1
const int motorInputPin1 = 19;  // Điều khiển chiều quay động cơ 1
const int motorInputPin2 = 5;   // Điều khiển chiều quay động cơ 1
const int motorEnablePin2 = 4;  // PWM điều khiển tốc độ động cơ 2
const int motorInputPin3 = 16;  // Điều khiển chiều quay động cơ 2
const int motorInputPin4 = 17;  // Điều khiển chiều quay động cơ 2

// Chân kết nối MPU6050 với ESP32
const int MPU_SCL = 22; // Chân SCL cho I2C
const int MPU_SDA = 21; // Chân SDA cho I2C


double angle;
double OGN_setpoint = 3.6;  // Góc hiện tại và góc mong muốn
double setpoint = OGN_setpoint;  // Góc hiện tại và góc mong muốn
double motorSpeed;  // Tốc độ động cơ (được điều khiển bởi PID)
double E;
bool mpu_found = false;

// Khai báo các tham số PID
double Kp =6.0, Ki =0.1 , Kd = 0.8;
double input, output, previousInput = 0;

// Tạo đối tượng PID
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Hàm khởi tạo
void setup() {
  Serial.begin(115200);
  Wire.begin(MPU_SDA, MPU_SCL);  // Khởi tạo I2C với các chân SDA và SCL
  u8g2.begin();  // Khởi tạo màn hình OLED

  Serial.println("Bắt đầu chương trình và khởi tạo I2C...");
  if (!mpu.begin(0x68)) {  // Địa chỉ mặc định của MPU6050
    Serial.println("Không tìm thấy cảm biến MPU6050!");
    while (1);
  } else {
    Serial.println("MPU6050 đã được kết nối thành công!");
    mpu_found = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // Khởi tạo chân điều khiển động cơ
  pinMode(motorEnablePin1, OUTPUT);
  pinMode(motorInputPin1, OUTPUT);
  pinMode(motorInputPin2, OUTPUT);
  pinMode(motorEnablePin2, OUTPUT);
  pinMode(motorInputPin3, OUTPUT);
  pinMode(motorInputPin4, OUTPUT);

  analogWrite(motorEnablePin1, 0);
  analogWrite(motorEnablePin2, 0);

  // Khởi tạo PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  myPID.SetOutputLimits(-255, 255);  // Giới hạn tốc độ động cơ trong khoảng -255 đến 255
}

// Hàm đọc dữ liệu góc nghiêng từ MPU6050 và hiển thị lên Serial Monitor
void readAngle() {
  if (mpu_found) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Tính toán góc nghiêng từ dữ liệu gia tốc và con quay hồi chuyển
    float accAngle = atan2(a.acceleration.x, a.acceleration.z) * 180 / PI;
    float gyroRate = g.gyro.y * 180 / PI;

    
    static float lastAngle;
    angle = 0.2 * (lastAngle + gyroRate * 0.01) + 0.8 * accAngle;
    lastAngle = angle;

     // In gia tốc kế, con quay hồi chuyển và nhiệt độ ra Serial Monitor
    Serial.print("Gia tốc kế: ");
    Serial.print("X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2, ");
    Serial.print("Y: "); Serial.print(a.acceleration.y); Serial.print(" m/s^2, ");
    Serial.print("Z: "); Serial.print(a.acceleration.z); Serial.println(" m/s^2");

    Serial.print("Con quay hồi chuyển: ");
    Serial.print("X: "); Serial.print(g.gyro.x); Serial.print(" rad/s, ");
    Serial.print("Y: "); Serial.print(g.gyro.y); Serial.print(" rad/s, ");
    Serial.print("Z: "); Serial.print(g.gyro.z); Serial.println(" rad/s");
    
    Serial.print("Angle: "); Serial.print(angle);
   
    
  } else {
    Serial.println("Cảm biến MPU6050 chưa sẵn sàng.");
  }

  // Cập nhật đầu vào PID (góc hiện tại)
  input = angle;
  E= angle-setpoint;
   // In ra giá trị của input, output, và setpoint để kiểm tra
  Serial.print("Input: "); Serial.print(input); 
  Serial.print(" Setpoint: "); Serial.print(setpoint);
  
}

// Hàm tính toán PID
void computePID() {
  // Tính toán PID
  myPID.Compute();
}

// Hàm hiển thị dữ liệu từ MPU6050 lên màn hình OLED
void displayMPU6050Data(const sensors_event_t& a) {
  u8g2.clearBuffer();

  // Hiển thị gia tốc kế
  u8g2.setFont(u8g2_font_ncenB08_tr);  // Chọn font hiển thị
  u8g2.setCursor(15, 10);
  u8g2.print("Gia toc ke X:");
  u8g2.print(a.acceleration.x); u8g2.print(" m/s^2");
  u8g2.setCursor(15, 20);
  u8g2.print("Y: "); u8g2.print(a.acceleration.y); u8g2.print(" m/s^2");
  u8g2.setCursor(15, 30);
  u8g2.print("Output: "); u8g2.print(output); 
  u8g2.setCursor(15, 40);
  u8g2.print("Error: ");
  u8g2.print(E);
  u8g2.setCursor(15, 50);
  u8g2.print("Angle: ");
  u8g2.print(angle);
  u8g2.print(" deg");

  u8g2.sendBuffer();  // Gửi dữ liệu hiển thị lên màn hình
}

// Hàm điều khiển động cơ
void motorControl() {
  // In ra giá trị PID output
  Serial.print("Motor Speed: "); Serial.println(output);
  Serial.println(output);
  
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

// Vòng lặp chính
void loop() {
  sensors_event_t a, g, temp;
  
  // Đọc dữ liệu từ MPU6050
  mpu.getEvent(&a, &g, &temp);
  
  
  
  // Đọc góc nghiêng và hiển thị lên Serial Monitor
  readAngle();
  
  // Tính toán PID
  computePID();
  
  // Điều khiển động cơ theo kết quả PID

  
  // Hiển thị dữ liệu lên OLED
  displayMPU6050Data(a);

}
