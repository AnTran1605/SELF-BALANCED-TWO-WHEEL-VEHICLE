#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <U8g2lib.h>
#include <SimpleKalmanFilter.h> // Thư viện Kalman
#include <PID_v1.h>  // Thêm thư viện PID_v1.h
#include <ESP32Encoder.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Adafruit_MPU6050 mpu; // Khởi tạo đối tượng MPU6050

//****************************************************************************************************************************************************************

// Khởi tạo màn hình OLED sử dụng U8g2lib
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);

//****************************************************************************************************************************************************************

//OFFSET của Acc X và Acc Y
float offsetAccX = 0.31;
float offsetAccY = -0.38;
float accX, accY;

//****************************************************************************************************************************************************************

// Biến để tính toán thời gian trôi qua
unsigned long lastTime = 0;  // Thời gian của lần đọc trước
float dt = 0.0;  // Thời gian trôi qua (delta time)

//****************************************************************************************************************************************************************
//setpoint
double OGN_setpoint = 2;  // Góc hiện tại và góc mong muốn
double setpoint = OGN_setpoint;  // Góc hiện tại và góc mong muốn
double Error;

//****************************************************************************************************************************************************************

// Khai báo các tham số PID
double Kp =60, Ki =30 , Kd =20; // 255-17
double input, output;
// Tạo đối tượng PID
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

//****************************************************************************************************************************************************************

// Biến tính toán
SimpleKalmanFilter kalmanFilter(1, 0.5, 0.047); // Khởi tạo Kalman Filter (measurement noise, process noise, dt)
float angleAcc;     // Góc nghiêng tính từ gia tốc kế
float angleGyro = 0; // Góc nghiêng tính từ con quay hồi chuyển
float angleFiltered; // Góc nghiêng sau khi lọc Kalman

//****************************************************************************************************************************************************************

// Khai báo các chân điều khiển động cơ
const int ENA = 18;  // PWM điều khiển tốc độ động cơ 1 
const int IN1 = 19;  // Điều khiển chiều quay động cơ 1 
const int IN2 = 5;   // Điều khiển chiều quay động cơ 1 
const int ENB = 4;   // PWM điều khiển tốc độ động cơ 2 
const int IN3 = 16;  // Điều khiển chiều quay động cơ 2 
const int IN4 = 17;  // Điều khiển chiều quay động cơ 2

//****************************************************************************************************************************************************************

ESP32Encoder encoder; // Khởi tạo đối tượng encoder
unsigned long lastTimeEn = 0; // Thời gian lần đọc trước
int32_t lastCount = 0; // Giá trị encoder lần trước
float speedRPM = 0; // Tốc độ động cơ (RPM)
float speedAngular = 0; // Tốc độ góc (rad/s)
float speedLinear = 0; // Tốc độ tịnh tiến (m/s)
const int pulsesPerRevolution = 330; // Số xung mỗi vòng quay
const float wheelDiameter = 0.065; // Đường kính bánh xe (m)
const float wheelRadius = wheelDiameter / 2; // Bán kính bánh xe (m)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() 
    {
        Serial.begin(115200); // Khởi tạo giao tiếp Serial
        
        // Khởi tạo màn hình OLED
        u8g2.begin();
        
        // Kiểm tra cảm biến MPU6050
        if (!mpu.begin()) 
          {
              Serial.println(F("MPU6050 không kết nối"));
              while (1); // Dừng chương trình nếu không kết nối được với MPU6050
          }
        
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.sendBuffer();  
        
        lastTime = millis(); // Khởi tạo thời gian ban đầu
    
        // Khởi tạo chân điều khiển động cơ
        pinMode(ENA, OUTPUT);
        pinMode(IN1, OUTPUT);
        pinMode(IN2, OUTPUT);
        pinMode(ENB, OUTPUT);
        pinMode(IN3, OUTPUT);
        pinMode(IN4, OUTPUT);
        analogWrite(ENA, 0);
        analogWrite(ENB, 0);
    
        // Khởi tạo PID

        myPID.SetMode(AUTOMATIC);
       
        myPID.SetOutputLimits(-200, 200);  // Giới hạn tốc độ động cơ trong khoảng -255 đến 255
    
        //ENCODER
        // Bật điện trở kéo lên
        ESP32Encoder::useInternalWeakPullResistors = puType::up;
    
        // Kết nối encoder với các chân A và B
        encoder.attachHalfQuad(26, 27);
        
        // Xóa giá trị cũ và khởi tạo bộ đếm
        encoder.clearCount();
    
        Serial.println("Encoder initialized.");
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void readMPU6050(sensors_event_t &a, sensors_event_t &g, sensors_event_t &temp) 
    {
        mpu.getEvent(&a, &g, &temp);  // Đọc dữ liệu cảm biến
        
        accX = a.acceleration.x - offsetAccX;
       
        
        // Tính góc từ gia tốc kế và con quay hồi chuyển
        angleAcc = atan2(accX, a.acceleration.z) * 180.0 / PI;  // Góc từ gia tốc kế
    
    
    
        
        // Lọc góc bằng Kalman Filter
        angleFiltered = kalmanFilter.updateEstimate(angleAcc);  // Sử dụng phương thức updateEstimate() của Kalman Filter
    
        // Cập nhật đầu vào PID (góc hiện tại)
        input = angleFiltered;
        Error = angleFiltered - setpoint;
        
        // In ra giá trị của input, output, và setpoint để kiểm tra
        Serial.print("\nInput: "); Serial.print(input); 
        Serial.print("\nSetpoint: "); Serial.print(setpoint);
        Serial.print("\nError: "); Serial.print(Error);
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void displayDataOnOLED() 
    {
        u8g2.clearBuffer(); // Xóa bộ đệm màn hình
        
        // Hiển thị góc nghiêng sau khi lọc
    
        
        // Hiển thị góc từ các nguồn
        u8g2.setCursor(10, 10);
        u8g2.print(F("Acc Ang: "));
        u8g2.print(angleAcc, 1);
        u8g2.print(F(" deg"));
        
        u8g2.setCursor(10, 20);
        u8g2.print(F("Filt Ang: "));
        u8g2.print(angleFiltered, 1);
        u8g2.print(F(" deg"));
    
        u8g2.setCursor(10, 30);
        u8g2.print(F("Error: "));
        u8g2.print(Error, 1);
        u8g2.print(F(" deg"));
    
        u8g2.setCursor(10, 40);
        u8g2.print(F("PID Out: "));
        u8g2.print(output, 1);
        u8g2.print(F(" PWM"));
    
        
        u8g2.setCursor(10, 50);
        u8g2.print(F("Kp: "));
        u8g2.print(Kp, 1);
    
        u8g2.setCursor(70, 50);
        u8g2.print(F("Ki: "));
        u8g2.print(Ki, 1);
    
        u8g2.setCursor(10, 60);
        u8g2.print(F("Kd: "));
        u8g2.print(Kd, 1);
        
        u8g2.sendBuffer(); // Cập nhật màn hình
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void displayDataOnSerial() 
    {
        Serial.print("Filtered Angle: ");
        Serial.print(angleFiltered);
        Serial.print(" deg\n");
        
        Serial.print("Acc Angle: ");
        Serial.print(angleAcc);
        Serial.print(" deg\n\n");
        
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Hàm tính toán PID và điều khiển động cơ
void computePID() 
    {

        myPID.SetSampleTime(dt);
        // Compute the PID output
        myPID.Compute();
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Hàm điều khiển động cơ
void control() 
    {
      // In ra giá trị PID output
      Serial.print("PID Output: "); Serial.print(output);
      
      if ( Error < -2) 
          {
            analogWrite(ENA, output);
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
        
            analogWrite(ENB, output);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
          } 
      else if (Error > 2)
          {
            analogWrite(ENA, -output);
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
        
            analogWrite(ENB, -output);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
          }
      else 
          {
            // Stop motors when balanced
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, LOW);
          }     
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void calculateSpeed() 
    {
        unsigned long currentTime = millis(); // Lấy thời gian hiện tại
        int32_t currentCount = encoder.getCount(); // Đọc giá trị encoder hiện tại
    
        // Tính số xung đã quay từ lần trước
        int32_t deltaCount = currentCount - lastCount;
        unsigned long deltaTime = currentTime - lastTimeEn; // Tính thời gian đã trôi qua (ms)
    
        // Đảm bảo deltaTime > 0 để tránh chia cho 0
        if (deltaTime > 0) 
          {
              // Tính tốc độ động cơ (RPM)
              speedRPM = (float)deltaCount / pulsesPerRevolution * (60000.0 / deltaTime);
      
              // Tính tốc độ góc (rad/s)
              speedAngular = (speedRPM * 2 * PI) / 60;
      
              // Tính tốc độ tịnh tiến (m/s)
              speedLinear = speedAngular * wheelRadius;
          }
    
        // In tốc độ ra Serial Monitor
        Serial.print("\nTốc độ (RPM): ");
        Serial.print(speedRPM);
        Serial.print(" RPM\n");
    
        Serial.print("Tốc độ góc: ");
        Serial.print(speedAngular);
        Serial.print(" rad/s\n");
    
        Serial.print("Tốc độ tịnh tiến: ");
        Serial.print(speedLinear);
        Serial.println(" m/s");
    
        // Cập nhật giá trị cho lần đo tiếp theo
        lastCount = currentCount;
        lastTimeEn = currentTime;
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() 
    {
        unsigned long currentMillis = millis();
        dt = (currentMillis - lastTime); // Tính toán thời gian trôi qua (s)
        
        sensors_event_t a, g, temp;
        
        // Gọi các hàm để thực hiện các nhiệm vụ cụ thể
        readMPU6050(a, g, temp);      // Đọc dữ liệu cảm biến và tính toán góc
        Serial.print("X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2 ");
        Serial.print("\ndt: "); Serial.print(dt); Serial.print(" ms \n");
    
        computePID();  // Tính toán PID và điều khiển động cơ
        control();
        calculateSpeed(); // Tính toán và in tốc độ
        displayDataOnOLED();          // Hiển thị dữ liệu lên OLED
        displayDataOnSerial();        // Hiển thị dữ liệu lên Serial
        
        lastTime = currentMillis; // Cập nhật thời gian đọc mới nhất
       
}
