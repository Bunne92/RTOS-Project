//#include <wirish/wirish.h>
//#include "libraries/FreeRTOS/MapleFreeRTOS.h"
#include <MapleFreeRTOS900.h>
#include "Arduino.h"
#include "Wire.h"
#include "uRTCLib.h"
uRTCLib rtc;

SemaphoreHandle_t xSemaphore=NULL;

unsigned int pos;
bool PIR;
int RealTimeMin=0;
struct LarmSchedule {
  int StartTime;
  int EndTime;
} WeekSchedule[2] = {
  {480, 1020},
  {0, 1440}
};
 struct LarmSchedule *Workday = (struct LarmSchedule *)&WeekSchedule[0];
 struct LarmSchedule *Weekend = (struct LarmSchedule *)&WeekSchedule[1];
static void BuzzerTask(void *pvParameters) {
  for(;;){
        if(PIR){
              digitalWrite(PB13, HIGH);
              Serial.println("PIIIIIIIP");
        }
        else{
          digitalWrite(PB13, LOW);
          Serial.println("FAIL");
        }
        vTaskDelay(1000);
  }    
}     
static void SensorTask(void *pvParameters) {
        for(;;){
              if(xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE){
                    if(rtc.dayOfWeek()<=6 && rtc.dayOfWeek() > 1){
                    
                          if((Workday->StartTime)<RealTimeMin && (Workday->EndTime)>RealTimeMin){
                                //larm av
                                Serial.println("Larm Av ja");
                                Serial.println(digitalRead(PB14));
                                Serial.println(digitalRead(PB15));
                                Serial.println(digitalRead(PB3));
                                Serial.println(digitalRead(PB4));
                                Serial.println(digitalRead(PB5));
                                Serial.println(digitalRead(PB8));
                                //Serial.println(digitalRead(PB9));
                                Serial.println(PIR);Serial.println(PIR);
                          }
                          else{
                                //larm på
                                Serial.println("Larm På");
                                PIR=digitalRead(PB12);
                                Serial.println(PIR);
                          }
                    }
                    if(rtc.dayOfWeek()==7 || rtc.dayOfWeek() == 1){
                          if((Weekend->StartTime)<RealTimeMin && (Weekend->EndTime)>RealTimeMin){
                                //larm av
                                Serial.println("Larm Av");
                                Serial.println(PIR);
                          }
                          else{
                                //larm på
                                Serial.println("Larm På");
                                PIR=digitalRead(PB12);
                                Serial.println(PIR);
                          }          
                    }
              xSemaphoreGive( xSemaphore );
              }
              vTaskDelay(1000);
        }
}
static void ClockTask(void *pvParameters) {      
    for(;;){
        rtc.refresh();
        Serial.println(RealTimeMin);
        if(xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE){
        RealTimeMin=(rtc.hour()*60)+rtc.minute();
        xSemaphoreGive( xSemaphore );
        }
        vTaskDelay(1000);
    }  
}

void setup() {
    pinMode(PB14, INPUT);
    pinMode(PB15, INPUT);
    pinMode(PB3, INPUT);
    pinMode(PB4, INPUT);
    pinMode(PB5, INPUT);
    pinMode(PB8, INPUT);
    //pinMode(PB9, INPUT);
    
    pinMode(PB12, INPUT);
    pinMode(PB13, OUTPUT);
    Serial.begin(115200);

      for(pos = 0; pos < 1000; pos++) {
    rtc.eeprom_write(pos, (unsigned char) pos % 256);
     }
      pos = 0;

// Only used once, then disabled
//rtc.set(0, 45, 11, 3, 25, 10, 17);
//  RTCLib::set(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)

  #ifdef _VARIANT_ARDUINO_STM32_
  Serial.println("Board: STM32");
  #else
  Serial.println("Board: Other");
  #endif
  rtc.set_rtc_address(0x68);
  rtc.set_ee_address(0x57);
  xSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive( xSemaphore );
  xTaskCreate(BuzzerTask,
                "Buzzer",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
   xTaskCreate(SensorTask,
                "PIR",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
   xTaskCreate(ClockTask,
                "RTC",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);
    vTaskStartScheduler();
}

void loop() {
    // Insert background code here
}

      
  /*Serial.print("RTC DateTime: ");
  Serial.print(rtc.year());
  Serial.print('/');
  Serial.print(rtc.month());
  Serial.print('/');
  Serial.print(rtc.day());
  Serial.print(' ');
  Serial.print(rtc.hour());
  Serial.print(':');
  Serial.print(rtc.minute());
  Serial.print(':');
  Serial.print(rtc.second());
  Serial.print(" DOW: ");
  Serial.print(rtc.dayOfWeek());
   Serial.println();

 */
