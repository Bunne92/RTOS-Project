#include <MapleFreeRTOS900.h>
#include "Arduino.h"
#include "Wire.h"
#include "uRTCLib.h"

#define pin1 PB13
#define pin2 PB8
#define pin3 PB5
#define pin4 PB4
#define pin5 PB3
#define pin6 PB15
#define pin7 PB14

uRTCLib rtc;
SemaphoreHandle_t xSemaphore=NULL;

bool PIR;
bool ManualONWeekend = false;
bool ManualONWorkNight = false;
bool ManualONWorkday = false;

int RealTimeMin=0;
char Password[4]={'1','2','3','4'};
char KeyInput[4]={'0','0','0','0'};
int KeyStep=0;

struct LarmSchedule {
  int StartTime;
  int EndTime;
} WeekSchedule[2] = {
  {480, 1020},
  {0, 1440}
};
 struct LarmSchedule *Workday = (struct LarmSchedule *)&WeekSchedule[0];
 struct LarmSchedule *Weekend = (struct LarmSchedule *)&WeekSchedule[1];

//This task read inputs and put it in KeyInput[], when 4 inputs is registered then a check will trigger and compare 
//KeyInput[4] with Password[4], if a match then ManualONWeekend or ManualONWorkNight or ManualONWorkday will change 
//status from false to true or the opposite. Depending on the time and day of the week a specific "ManualOn" will change .
static void KeypadTask(void *pvParameters) {
      for(;;){
            digitalWrite(pin7, LOW);
            digitalWrite(pin6, LOW);
            digitalWrite(pin4, LOW);
            digitalWrite(pin2, HIGH);
            if(digitalRead(pin3) == HIGH){KeyInput[KeyStep]='1';KeyStep++;digitalWrite(PB10, HIGH);vTaskDelay(500);}
            if(digitalRead(pin1) == HIGH){KeyInput[KeyStep]='2';KeyStep++;digitalWrite(PB10, HIGH);vTaskDelay(500);}
            if(digitalRead(pin5) == HIGH){KeyInput[KeyStep]='3';KeyStep++;digitalWrite(PB10, HIGH);vTaskDelay(500);}
            digitalWrite(pin2, LOW);
            digitalWrite(pin7, HIGH);
            if(digitalRead(pin3) == HIGH){KeyInput[KeyStep]='4';KeyStep++;digitalWrite(PB10, HIGH);vTaskDelay(500);}
            if(digitalRead(pin1) == HIGH){KeyInput[KeyStep]='5';KeyStep++;digitalWrite(PB10, HIGH);vTaskDelay(500);}
            if(digitalRead(pin5) == HIGH){KeyInput[KeyStep]='6';KeyStep++;digitalWrite(PB10, HIGH);vTaskDelay(500);}
            digitalWrite(pin7, LOW);
            digitalWrite(pin6, HIGH);
            if(digitalRead(pin3) == HIGH){KeyInput[KeyStep]='7';KeyStep++;digitalWrite(PB10, HIGH);vTaskDelay(500);}
            if(digitalRead(pin1) == HIGH){KeyInput[KeyStep]='8';KeyStep++;digitalWrite(PB10, HIGH);vTaskDelay(500);}
            if(digitalRead(pin5) == HIGH){KeyInput[KeyStep]='9';KeyStep++;digitalWrite(PB10, HIGH);vTaskDelay(500);}
            digitalWrite(pin6, LOW);
            digitalWrite(pin4, HIGH);
            if(digitalRead(pin3) == HIGH){KeyInput[KeyStep]='*';KeyStep++;digitalWrite(PB10, HIGH);vTaskDelay(500);}
            if(digitalRead(pin1) == HIGH){KeyInput[KeyStep]='0';KeyStep++;digitalWrite(PB10, HIGH);vTaskDelay(500);}
            if(digitalRead(pin5) == HIGH){KeyInput[KeyStep]='#';KeyStep++;digitalWrite(PB10, HIGH);vTaskDelay(500);}
            digitalWrite(pin4, LOW);
            digitalWrite(PB10, LOW);
            vTaskDelay(100);
            if(KeyStep>=4){
              digitalWrite(PC13, HIGH);
              vTaskDelay(200);
              digitalWrite(PC13, LOW);  
                  KeyStep=0;
                  for(int ix=0;ix<4;ix++){
                        if(KeyInput[ix]==Password[ix]){
                              if (ix==3 && ManualONWorkday==false && (Workday->StartTime)<RealTimeMin && (Workday->EndTime)>RealTimeMin){
                                    ManualONWorkday = true;
                              }
                              else if(ix==3 && ManualONWorkday==true && (Workday->StartTime)<RealTimeMin && (Workday->EndTime)>RealTimeMin){
                                    ManualONWorkday = false;
                              }
                              if (ix==3 && ManualONWorkNight==false &&(Workday->EndTime)<=RealTimeMin){
                                    ManualONWorkNight = true;
                              }
                              else if(ix==3 && ManualONWorkNight==true && (Workday->EndTime)<=RealTimeMin){
                                    ManualONWorkNight = false;
                              }
                              
                              if (ix==3 && ManualONWeekend==false && (Weekend->StartTime)<RealTimeMin && (Weekend->EndTime)>RealTimeMin){
                                    ManualONWeekend = true;
                              }
                              else if(ix==3 && ManualONWeekend==true && (Weekend->StartTime)<RealTimeMin && (Weekend->EndTime)>RealTimeMin){
                                    ManualONWeekend = false;
                              }
                        }
                        else{
                              break;
                        }
                  }
            }
      }
}
//Buzzer will go On if PIR have a value of 1
static void BuzzerTask(void *pvParameters) {
      for(;;){
            if(PIR){
                  digitalWrite(PB11, HIGH); //if PIR=1 buzzer on
            }
            vTaskDelay(1000);
      }    
}     
//If xSemaphoreTake then Depending on Day of Week Compares real time (in min) with a predetermend schedule and ManualOn to see if larm is on or off.
//if on then read PIR else if off shut down buzzer and set PIR to 0 then xSemaphoreTakeGive
static void SensorTask(void *pvParameters) {
      for(;;){
            if(xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE){
                  if(rtc.dayOfWeek()<=6 && rtc.dayOfWeek() > 1){
                        if((Workday->StartTime)<RealTimeMin && (Workday->EndTime)>RealTimeMin && ManualONWorkday==false){
                              //Larm Off
                              digitalWrite(PB11, LOW); 
                              PIR=0;
                              Serial.println(PIR);
                        }
                        else if ((Workday->EndTime)<=RealTimeMin && ManualONWorkNight==true){
                              //Larm Off
                              digitalWrite(PB11, LOW);
                              PIR=0;
                              Serial.println(PIR);
                          }
                        else {
                              //Larm On
                              PIR=digitalRead(PB12);
                              Serial.println(PIR);
                        }
                  }
                  if(rtc.dayOfWeek()==7 || rtc.dayOfWeek() == 1){
                        if((Weekend->StartTime)<RealTimeMin && (Weekend->EndTime)>RealTimeMin && ManualONWeekend==false){
                              //Larm Off
                              digitalWrite(PB11, LOW);
                              PIR=0;
                              Serial.println(PIR);
                        }
                        else if ((Workday->EndTime)<RealTimeMin && ManualONWeekend==true){
                              //Larm Off 
                              digitalWrite(PB11, LOW);
                              PIR=0;
                              Serial.println(PIR);
                          }
                        else{
                              //Larm On
                              PIR=digitalRead(PB12);
                              Serial.println(PIR);
                        }          
                  }
            xSemaphoreGive( xSemaphore );
            }
            vTaskDelay(1000);
      }
}
//if xSemaphoreTake then read new time in minutes then give xSemaphore
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
      pinMode(pin7, OUTPUT);    // pin7 in Keypad
      pinMode(pin6, OUTPUT);    // pin6 in Keypad
      pinMode(pin5, INPUT);     // pin5 in Keypad
      pinMode(pin4, OUTPUT);    // pin4 in Keypad
      pinMode(pin3, INPUT);     // pin3 in Keypad
      pinMode(pin2, OUTPUT);    // pin2 in Keypad
      pinMode(pin1, INPUT);     // pin1 in Keypad
      pinMode(PB12, INPUT);     // PIR
      pinMode(PB11, OUTPUT);    // Buzzer
      pinMode(PC13, OUTPUT);     // LED
      pinMode(PB10, OUTPUT);     // LED
      Serial.begin(115200);

      //Set peramitors for RTC
      for(KeyStep = 0; KeyStep < 1000; KeyStep++) {
          rtc.eeprom_write(KeyStep, (unsigned char) KeyStep % 256);
      }
      KeyStep = 0;
      // Only used once, then disable. Set the time and date for RTC
      //rtc.set(0, 32, 12, 2, 30, 10, 17);
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
      xTaskCreate(KeypadTask,
                  "Keypad",
                  configMINIMAL_STACK_SIZE,
                  NULL,
                  tskIDLE_PRIORITY + 2,
                  NULL);
      vTaskStartScheduler();
}
void loop() {
    // Insert background code here
}
