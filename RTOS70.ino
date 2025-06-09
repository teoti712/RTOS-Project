#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <MAX30100_PulseOximeter.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <RTClib.h>
#include <AdafruitIO.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define dht_pin 4
#define dht_type DHT22
#define touch 25
#define buzzer 33
#define ena 19
#define btnhour 17
#define btnmin 16

#define IO_USERNAME  "rashaumon0810"
#define IO_KEY       "aio_LyRa07tJBPrEdYcbny9BnnAcwdfR"

#define WIFI_SSID "AnhVu" 
#define WIFI_PASS "anhvu1999"  

#define REPORTING_PERIOD_MS 1000 

SemaphoreHandle_t Sem_Handle; // semaphore của heart
SemaphoreHandle_t Sem_Handlewarning;
SemaphoreHandle_t Sem_Handlealarm;
TimerHandle_t softtimer1s;

#include <AdafruitIO_WiFi.h>
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS); 
AdafruitIO_Feed *HEARTRATE = io.feed("HEARTRATE");
AdafruitIO_Feed *SPO2 = io.feed("SPO2");

RTC_DS1307 rtc;
DHT dht(dht_pin, dht_type); 
LiquidCrystal_I2C lcd(0x27, 16, 2);
PulseOximeter pox;

uint32_t lastReportTime = 0;
int heart, spo2;
int year, month, day, hour, mini, sec;
float humid, temp;
int touch_value = 0;
int min_alarm, hour_alarm;
bool check;

void update(TimerHandle_t softtimer1s){
  static unsigned long lastSendTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= 15000) {
  HEARTRATE->save(heart);
  SPO2->save(spo2);
  lastSendTime = currentTime;
  }
}
void taskcloud(){
  softtimer1s = xTimerCreate("timer1s", 1000 / portTICK_PERIOD_MS, pdTRUE, 0, update); 
  xTimerStart(softtimer1s, portMAX_DELAY);
}

void screen_alarm(){

    lcd.setCursor(0, 0);
    lcd.print("phut bao thuc:");
    lcd.print(min_alarm);
    lcd.setCursor(0, 1);
    lcd.print("gio bao thuc: ");
    lcd.print(hour_alarm);

}

void screentime(){
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print(day);
    lcd.print('/');
    lcd.print(month);
    lcd.print('/');
    lcd.print(year);
    lcd.setCursor(3, 1);
    lcd.print(hour);
    lcd.print(':');
    lcd.print(mini);
    //Serial.print(mini);
    lcd.print(':');
    lcd.print(sec);
  
}

void screenheart(){

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("heart = ");
    lcd.print(heart);
    lcd.setCursor(0, 1);
    lcd.print("SpO2 = ");
    lcd.print(spo2);
 
}

void screenheat(){
  lcd.setCursor(0,0);
  lcd.print("temp = ");
  lcd.print(temp);
  lcd.setCursor(0,1);
  lcd.print("humid = ");
  lcd.print(humid);
}

//task báo thức
void taskCheckAlarm(void *parameters) {
  while(1) {
    if(xSemaphoreTake(Sem_Handlealarm, portMAX_DELAY)){
    if (hour == hour_alarm && mini == min_alarm ) {
      if(check == 1){
      tone(buzzer, 50, 256);
      Serial.println("báo thức");
      }
      else if(check == 0){
       noTone(buzzer);
      }
    }
    if ((hour != hour_alarm || mini != min_alarm) && check == 0) {
      noTone(buzzer);
    }
    xSemaphoreGive(Sem_Handlealarm);
  }
  vTaskDelay(10 / portTICK_PERIOD_MS); // Kiểm tra mỗi giây
}}

// task thiết lập báo thức
void taskSetAlarm(void *parameters) {
    pinMode(btnmin, INPUT_PULLUP);
    pinMode(btnhour, INPUT_PULLUP);
  while(1) {
    if(digitalRead(btnhour) == LOW) { // Nút để tăng giờ
      xSemaphoreTake(Sem_Handlealarm, portMAX_DELAY);
      hour_alarm = (hour_alarm + 1) % 24; // Tăng giờ
      xSemaphoreGive(Sem_Handlealarm);
      vTaskDelay(200 / portTICK_PERIOD_MS); // Delay để tránh bounce nút
    }

    if(digitalRead(btnmin) == LOW) { // Nút để tăng phút
      xSemaphoreTake(Sem_Handlealarm, portMAX_DELAY);
      min_alarm = (min_alarm + 1) % 60; // Tăng phút
      Serial.print(min_alarm);
      xSemaphoreGive(Sem_Handlealarm);
      vTaskDelay(200 / portTICK_PERIOD_MS); // Delay để tránh bounce nút
    }

    vTaskDelay(50 / portTICK_PERIOD_MS); // Delay trước khi kiểm tra lại nút
  }
}

//task cảnh báo
void warning(void *parameters){
  while(1){

    if(xSemaphoreTake(Sem_Handlewarning, portMAX_DELAY) == pdTRUE){
      tone(buzzer, 50, 256);
      //Serial.println(uxSemaphoreGetCount(Sem_Handlewarning));
      vTaskDelay(10 / portTICK_PERIOD_MS); // Nghỉ 1 giây giữa các cảnh báo
    }
  }
}

//task ngưỡng cảnh báo
void taskwarningthreshold(void *parameters){
  while(1){
    pox.update();
    if (xSemaphoreTake(Sem_Handle, portMAX_DELAY) == 1) {
    if(heart < 70 || heart > 140 || spo2 < 80){
    xSemaphoreGive(Sem_Handlewarning);
    //Serial.println(pox.getHeartRate());
    //Serial.println(uxSemaphoreGetCount(Sem_Handlewarning));
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}}
}
//task đo nhiệt độ
void taskheat(void *parameters){
  dht.begin();
  while(1){
    humid = dht.readHumidity();
    //Serial.println(humid);
    temp = dht.readTemperature();
    //Serial.println(temp);
    vTaskDelay(10 / portTICK_PERIOD_MS); // Nghỉ 1 giây để đồng bộ với cảm biến
  }
}

//task đếm thời gian
void tasktime(void *parameters){
  while(1){
    
    DateTime now = rtc.now();
    year = now.year();
    month = now.month();
    day = now.day();
    hour = now.hour();
    mini = now.minute();
    //Serial.println(mini);
    sec = now.second();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
} 

//task đo nhịp tim
void taskheart(void *parameters){

  while(1){
    pox.update();
    if (millis() - lastReportTime > REPORTING_PERIOD_MS) {
    heart = pox.getHeartRate();
    //Serial.print("nhip tim");
    //Serial.println(heart);
    spo2 = pox.getSpO2();
    //Serial.print("oxi");
    //Serial.print(spo2);
    lastReportTime = millis();
    xSemaphoreGive(Sem_Handle);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Thêm khoảng nghỉ
  
}}

//task màn hình
void taskscreen(void *parameters) {
    pinMode(touch, INPUT);
    pinMode(ena, INPUT_PULLUP);
    touch_value = 0;

    while (1) {
      if(digitalRead(ena) == 0){
        check = !check;
        Serial.print("check ");
        Serial.print(check);
        vTaskDelay(200 / portTICK_PERIOD_MS);
      }

        int touchState = digitalRead(touch);
        if (touchState == HIGH) {
            vTaskDelay(200 / portTICK_PERIOD_MS); // Tránh đọc liên tục khi nhấn
            touch_value = (touch_value + 1) % 4; // Xoay vòng giữa 0, 1, 2, 3
        }

        switch (touch_value) {
            case 0:
                screentime();
                break;
            case 1:
                screenheart();
                break;
            case 2:
                screenheat();
                break;
            case 3:
              lcd.clear();
              if(check == 0){
                lcd.setCursor(3, 0);
                lcd.print("KHONG CO");
                lcd.setCursor(3, 1);
                lcd.print("BAO THUC!");
              }
              else if(check == 1){
                screen_alarm();
              }
        }
        vTaskDelay(50 / portTICK_PERIOD_MS); // Nghỉ giữa các lần cập nhật
    }
}

void setup(){
  Serial.begin(115200);
  Wire.begin();
  min_alarm = 21;
  hour_alarm = 10;
  check = 0;
  lcd.init();
  lcd.backlight();
  Serial.println("Khởi động cảm biến MAX30100...");

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  rtc.adjust(DateTime(2024, 12, 4, 10, 20, 0));

  if (!pox.begin()) {
    Serial.println("Không tìm thấy cảm biến MAX30100. Vui lòng kiểm tra kết nối!");
    while (true);
  }

  pinMode(buzzer, OUTPUT);
  Sem_Handlewarning = xSemaphoreCreateBinary();
  Sem_Handle = xSemaphoreCreateBinary();
  Sem_Handlealarm = xSemaphoreCreateMutex();

  xTaskCreate(taskheart, "taskheart", 4096, NULL, 3, NULL);
  xTaskCreate(taskscreen, "taskscreen", 4096, NULL, 2, NULL);
  xTaskCreate(tasktime, "tasktime", 4096, NULL, 3, NULL);
  xTaskCreate(taskheat, "taskheat", 4096, NULL, 2, NULL);

  xTaskCreate(warning, "warning", 4096, NULL, 3, NULL);
  xTaskCreate(taskwarningthreshold, "taskwarningthreshold", 4096, NULL, 2, NULL);
  xTaskCreate(taskSetAlarm, "taskSetAlarm", 4096, NULL, 3, NULL);
  xTaskCreate(taskCheckAlarm, "taskCheckAlarm", 4096, NULL, 2, NULL);
  taskcloud();
  io.connect(); // Kết nối tới Adafruit IO
}

void loop(){
    io.run();
}