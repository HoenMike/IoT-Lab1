#include <Wire.h>
#include <SPI.h>

#include "RTClib.h"
RTC_DS1307 RTC;
void setup () {
  Wire.begin(4, 5); // Chan 4,5 I2C cua ESP8266
  RTC.begin();      // Khoi dong RTC
  delay(500);
  // Dong bo thoi gian voi may tinh
  RTC.adjust(DateTime(__DATE__, __TIME__)); 
  delay(1000);
  Serial.begin(115200); // Khoi dong serial port de lay du lieu

}
void loop() {
  DateTime now = RTC.now(); // Thoi gian = thoi gian RTC hien tai
  // In thời gian
  Serial.print(now.year(), DEC); // Năm
  Serial.print('/');
  Serial.print(now.month(), DEC); // Tháng
  Serial.print('/');
  Serial.print(now.day(), DEC); // Ngày
  Serial.print(' ');
  Serial.print(now.hour(), DEC); // Giờ
  Serial.print(':');
  Serial.print(now.minute(), DEC); // Phút
  Serial.print(':');
  Serial.print(now.second(), DEC); // Giây
  Serial.println();
  delay(1000); // Delay
}
