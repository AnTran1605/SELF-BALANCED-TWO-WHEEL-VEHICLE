#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <U8g2lib.h>

Adafruit_MPU6050 mpu; // Khởi tạo đối tượng MPU6050

// Khởi tạo màn hình OLED sử dụng U8g2lib
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);

float offsetAccX = 0.31;
float offsetAccY = -0.38;
float accX;
float accY;

// Hàm đọc dữ liệu từ cảm biến MPU6050
void readMPU6050(sensors_event_t &a, sensors_event_t &g, sensors_event_t &temp) {
  mpu.getEvent(&a, &g, &temp); // Đọc gia tốc (a), tốc độ quay (g), và nhiệt độ (temp)
  accX = a.acceleration.x - offsetAccX;
  accY = a.acceleration.y - offsetAccY;
}

// Hàm hiển thị dữ liệu lên màn hình OLED
void displayDataOnOLED(sensors_event_t &a, sensors_event_t &g) {
  u8g2.clearBuffer(); // Xóa bộ đệm màn hình

  // Hiển thị gia tốc
  u8g2.setCursor(0, 10);
  u8g2.print(F("Acc X: "));
  u8g2.print(accX, 1);
  u8g2.print(F(" m/s^2"));

  u8g2.setCursor(0, 20);
  u8g2.print(F("Acc Y: "));
  u8g2.print(accY, 1);
  u8g2.print(F(" m/s^2"));

  u8g2.setCursor(0, 30);
  u8g2.print(F("Acc Z: "));
  u8g2.print(a.acceleration.z, 1);
  u8g2.print(F(" m/s^2"));

  // Hiển thị tốc độ quay
  u8g2.setCursor(0, 40);
  u8g2.print(F("Gyro X: "));
  u8g2.print(g.gyro.x, 1);
  u8g2.print(F(" rad/s"));

  u8g2.setCursor(0, 50);
  u8g2.print(F("Gyro Y: "));
  u8g2.print(g.gyro.y, 1);
  u8g2.print(F(" rad/s"));

  u8g2.setCursor(0, 60);
  u8g2.print(F("Gyro Z: "));
  u8g2.print(g.gyro.z, 1);
  u8g2.print(F(" rad/s"));

  u8g2.sendBuffer(); // Cập nhật màn hình
}

// Hàm hiển thị dữ liệu lên Serial Monitor
void displayDataOnSerial(sensors_event_t &a, sensors_event_t &g) {
  Serial.print("Gia tốc sau hiệu chỉnh: ");
  Serial.print("X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2, ");
  Serial.print("Y: "); Serial.print(a.acceleration.y); Serial.print(" m/s^2, ");
  Serial.print("Z: "); Serial.print(a.acceleration.z); Serial.println(" m/s^2");

  Serial.print("Con quay hồi chuyển sau hiệu chỉnh: ");
  Serial.print("X: "); Serial.print(g.gyro.x); Serial.print(" rad/s, ");
  Serial.print("Y: "); Serial.print(g.gyro.y); Serial.print(" rad/s, ");
  Serial.print("Z: "); Serial.print(g.gyro.z); Serial.println(" rad/s");
}

void setup() {
  Serial.begin(115200); // Khởi tạo giao tiếp Serial

  // Khởi tạo màn hình OLED
  u8g2.begin();

  // Kiểm tra cảm biến MPU6050
  if (!mpu.begin()) {
    Serial.println(F("MPU6050 không kết nối"));
    while (1);
  }
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.sendBuffer();  
}

void loop() {
  sensors_event_t a, g, temp;
  
  // Gọi các hàm để thực hiện các nhiệm vụ cụ thể
  readMPU6050(a, g, temp);
  displayDataOnOLED(a, g);
  displayDataOnSerial(a, g);

  delay(10); // Đợi 10ms
}
