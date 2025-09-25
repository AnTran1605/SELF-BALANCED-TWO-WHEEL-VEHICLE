#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <U8g2lib.h>

Adafruit_MPU6050 mpu;  // Khởi tạo đối tượng MPU6050

// Khởi tạo màn hình OLED sử dụng U8g2lib
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);

void setup() {
  Serial.begin(115200);  // Khởi tạo giao tiếp Serial

  // Khởi tạo màn hình OLED
  u8g2.begin();

  // Kiểm tra cảm biến MPU6050
  if (!mpu.begin()) {
    Serial.println(F("MPU6050 không kết nối"));
    while (1);
  }

  // In thông tin cảm biến lên màn hình OLED
  u8g2.clearBuffer();  // Xóa bộ đệm màn hình
  u8g2.setFont(u8g2_font_ncenB08_tr);  // Chọn font chữ
  u8g2.drawStr(0, 10, "MPU6050 kết nối!");  // In thông điệp lên màn hình
  u8g2.sendBuffer();  // Hiển thị lên màn hình
  delay(2000);  // Đợi 2 giây
}

void loop() {
  // Đọc dữ liệu từ MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);  // Đọc gia tốc (a), tốc độ quay (g), và nhiệt độ (temp)

  // Hiển thị dữ liệu gyroscope (tốc độ quay) trên màn hình OLED
  u8g2.clearBuffer();  // Xóa bộ đệm màn hình

  u8g2.setCursor(0, 10);  // Đặt con trỏ tại vị trí
  u8g2.print(F("Acc X: "));
  u8g2.print(a.acceleration.x, 1);  // Hiển thị gia tốc theo trục X
  u8g2.print(F(" m/s^2"));

  u8g2.setCursor(0, 20);
  u8g2.print(F("Acc Y: "));
  u8g2.print(a.acceleration.y, 1);  // Hiển thị gia tốc theo trục Y
  u8g2.print(F(" m/s^2"));

  u8g2.setCursor(0, 30);
  u8g2.print(F("Acc Z: "));
  u8g2.print(a.acceleration.z, 1);  // Hiển thị gia tốc theo trục Z
  u8g2.print(F(" m/s^2"));
  
  u8g2.setCursor(0, 40);  // Đặt con trỏ tại vị trí
  u8g2.print(F("Gyro X: "));
  u8g2.print(g.gyro.x, 1);  // Hiển thị tốc độ quay theo trục X
  u8g2.print(F(" rad/s"));

  u8g2.setCursor(0, 50);
  u8g2.print(F("Gyro Y: "));
  u8g2.print(g.gyro.y, 1);  // Hiển thị tốc độ quay theo trục Y
  u8g2.print(F(" rad/s"));

  u8g2.setCursor(0, 60);
  u8g2.print(F("Gyro Z: "));
  u8g2.print(g.gyro.z, 1);  // Hiển thị tốc độ quay theo trục Z
  u8g2.print(F(" rad/s"));

  u8g2.sendBuffer();  // Cập nhật màn hình


  // In thông tin sau khi hiệu chỉnh ra Serial Monitor
    Serial.print("Gia tốc sau hiệu chỉnh: ");
    Serial.print("X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2, ");
    Serial.print("Y: "); Serial.print(a.acceleration.y); Serial.print(" m/s^2, ");
    Serial.print("Z: "); Serial.print(a.acceleration.z); Serial.println(" m/s^2");

    Serial.print("Con quay hồi chuyển sau hiệu chỉnh: ");
    Serial.print("X: "); Serial.print(g.gyro.x); Serial.print(" rad/s, ");
    Serial.print("Y: "); Serial.print(g.gyro.y); Serial.print(" rad/s, ");
    Serial.print("Z: "); Serial.print(g.gyro.z); Serial.println(" rad/s");
    
  delay(10);         // Đợi nửa giây
}
