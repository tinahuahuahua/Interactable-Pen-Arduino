#include <Wire.h>
#include "DFRobot_TCS34725.h"
#include <ArduinoBLE.h>
#include <Adafruit_GFX.h>  //核心圖形庫
#include <Adafruit_ST7735.h>  //Hardware-specific library for ST7735
#include <Fonts/FreeMonoBold9pt7b.h>  //字型FreeMonoBold9pt7b
#include <Fonts/FreeSansBold9pt7b.h>  //字型FreeSansBold9pt7b
#include <Fonts/FreeSerif9pt7b.h>  //字型FreeSerif9pt7b
#include <SPI.h> 

#define redpin 4
#define greenpin 5
#define bluepin 6
#define redpin2 9
#define greenpin2 10
#define bluepin2 11
#define redpin_mix 2
#define greenpin_mix 3
#define bluepin_mix 12
#define vibr 21

//兩個按鈕的接角
#define button_get 7
#define button_delete 8

#define TFT_CS  2 // TFT CS PIN腳
#define TFT_DC   3 // TFT DC(A0、RS) PIN腳
#define TFT_RST  9 // TFT RES(Reset) PIN腳

int counter = 1;
int just_record = 0;
int just_delete = 0;
byte record[8];
byte record_mix[3];

//boolean ledState = LOW;
int debounceDelay = 200; // debounce delay (ms)
unsigned long lastMillis; // record last millis

// for a common anode LED, connect the common pin to +5V
// for common cathode, connect the common to ground

// set to false if using a common cathode LED
#define commonAnode false

// our RGB -> eye-recognized gamma color
byte gammatable[256];

DFRobot_TCS34725 tcs = DFRobot_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

BLEService penService("19B10000-E8F2-537E-4F6C-D104768A1214"); 
// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central //後面的111111是定義傳多長
BLECharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite, "111111");

void setup() {
  Serial.begin(115200);
  Serial.println("Color View Test!");
  
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    // while (1); // halt!
  }
  
  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  tft.setRotation(1);  //螢幕轉向
  tft.fillScreen(ST77XX_BLACK);  //設定螢幕背景為黑色
  
  tft.drawRect(53, 3, 50, 120, 0xFFFF); //畫一個方框 x,y,w,h,顏色值
  

  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    // while (1);
  }
    // set advertised local name and service UUID:
    BLE.setLocalName("Pen");
    BLE.setAdvertisedService(penService);

    // add the characteristic to the service
    penService.addCharacteristic(switchCharacteristic);

    // add service
    BLE.addService(penService);
    // set the initial value for the characeristic: 這裡可以從arduino船
    // switchCharacteristic.writeValue("4287f5");
    // start advertising
    BLE.advertise();
    Serial.println("BLE LED Peripheral");
  // use these three pins to drive an LED
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);

  pinMode(redpin2, OUTPUT);
  pinMode(greenpin2, OUTPUT);
  pinMode(bluepin2, OUTPUT);

  pinMode(redpin_mix, OUTPUT);
  pinMode(greenpin_mix, OUTPUT);
  pinMode(bluepin_mix, OUTPUT);

  pinMode(button_get,INPUT_PULLUP);
  pinMode(button_delete,INPUT_PULLUP);

  pinMode(vibr,INPUT);

  // thanks PhilB for this gamma table!
  // it helps convert RGB colors to what humans see //把RGB轉成人類可難到的亮光(LED上)
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;
    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
    // Serial.println(gammatable[i]);
  }

}

void loop() {
  
  BLEDevice central = BLE.central();
  // if a central is connected to peripheral:
  //已經連到藍芽偵測的動作
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    
    // while the central is still connected to peripheral:
    while (central.connected()) {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (switchCharacteristic.written()) {
        if (switchCharacteristic.value()) {   // any value other than 0
          Serial.println("LED on");     
        } else {                              // a 0 value
          Serial.println(F("LED off"));
        }
      }
      
      long measurement = TP_init(); 
      if (measurement > 20000){
        Serial.println("shake");
        mix_color();
        
        delay(100);
      }

      if (digitalRead(button_get) == LOW){ //當擷取按鈕按下
      
          if (debounced() && digitalRead(button_get) == LOW)
          {//避免按下去就跑好多次，或是按住不放的情形
          while(digitalRead(button_get) == LOW);
         
            //偵測顏色
            uint16_t clear, red, green, blue;
            tcs.getRGBC(&red, &green, &blue, &clear);
            if(clear<20 && red<15 && green<15 && blue<15){
              red = 0;
              green = 0;
              blue = 0;
            }
            //轉成HEX
            uint32_t sum = clear;
            float r, g, b;
            r = red; r /= sum;
            g = green; g /= sum;
            b = blue; b /= sum;
            r *= 256; g *= 256; b *= 256;

            recording(r,g,b,clear); //呼叫擷取的function
            Serial.println("-----------");
            Serial.println(clear);
            Serial.println(red);
            Serial.println(green);
            Serial.println(blue);
            // Serial.println(freeMemory(), DEC);
           
            Serial.println("-----------");

            Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
            Serial.println();
            for (int i = 0; i < 8; i++) {
                Serial.print(record[i]);
                Serial.print("\t"); 
            }
          }
      }
      if (digitalRead(button_delete) == LOW){//當按下刪除鍵
         if (debounced() && digitalRead(button_delete) == LOW){
           while(digitalRead(button_delete) == LOW);
           delete_record();
         }
      }
      
    }
    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }

  //還沒連到藍芽偵測的動作
  long measurement = TP_init(); 
      if (measurement > 20000){
        Serial.println("shake");
        mix_color();
        delay(100);
      }

      if (digitalRead(button_get) == LOW){ //當擷取按鈕按下
      
          if (debounced() && digitalRead(button_get) == LOW)
          {//避免按下去就跑好多次，或是按住不放的情形
          while(digitalRead(button_get) == LOW);
         
            //偵測顏色
            uint16_t clear, red, green, blue;
            tcs.getRGBC(&red, &green, &blue, &clear);
            if(clear<20 && red<15 && green<15 && blue<15){
              red = 0;
              green = 0;
              blue = 0;
            }
            //轉成HEX
            uint32_t sum = clear;
            float r, g, b;
            r = red; r /= sum;
            g = green; g /= sum;
            b = blue; b /= sum;
            r *= 256; g *= 256; b *= 256;

            recording(r,g,b,clear); //呼叫擷取的function
            Serial.println("-----------");
            Serial.println(clear);
            Serial.println(red);
            Serial.println(green);
            Serial.println(blue);
            // Serial.println(freeMemory(), DEC);
           
            Serial.println("-----------");

            Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
            Serial.println();
            for (int i = 0; i < 8; i++) {
                Serial.print(record[i]);
                Serial.print("\t"); 
            }
          }
      }
      if (digitalRead(button_delete) == LOW){//當按下刪除鍵
         if (debounced() && digitalRead(button_delete) == LOW){
           while(digitalRead(button_delete) == LOW);
           delete_record();
         }
      }

    }
      

long TP_init(){
  long measurement=pulseIn (vibr, HIGH);  // 等待 D0 輸入高電壓，並回傳值
  return measurement;
}

void mix_color(){
    record_mix[0] = (record[0] + record[4])/2;
    record_mix[1] = (record[1] + record[5])/2;
    record_mix[2] = (record[2] + record[6])/2;
    // Serial.println("mix_color:"); 
    // for (int i = 0; i < 3; i++) {
    //   Serial.print("\t"); 
    //   Serial.print(record_mix[i]);
    // }
    analogWrite(redpin_mix, gammatable[(int)record_mix[0]]);
    analogWrite(greenpin_mix, gammatable[(int)record_mix[1]]);
    analogWrite(bluepin_mix, gammatable[(int)record_mix[2]]);
  
}

void recording(byte r,byte g,byte b,byte c){

  if(counter == 1){
              record[0]=(int)r;
              record[1]=(int)g;
              record[2]=(int)b;
              record[3]=(int)c;
              Serial.println("firstdetect");
              tft.drawRect(3, 3, 50, 120, 0xFFFF); //畫一個方框 x,y,w,h,顏色值
              tft.fillRect(6, 6, 44, 114, getColor(r,g,b)); //填滿方形 x,y,w,h,顏色值
              counter ++;
              analogWrite(redpin, gammatable[(int)r]);
              analogWrite(greenpin, gammatable[(int)g]);
              analogWrite(bluepin, gammatable[(int)b]);
              just_record = 1;
              
            }
            
  else if(counter == 2){
              record[4]=(int)r;
              record[5]=(int)g;
              record[6]=(int)b;
              record[7]=(int)c;
              Serial.println("seconddetect");
              tft.drawRect(103, 3, 50, 120, 0xFFFF); //畫一個方框 x,y,w,h,顏色值
              tft.fillRect(106, 6, 44, 114, getColor(r,g,b));
              counter ++;
              analogWrite(redpin2, gammatable[(int)r]);
              analogWrite(greenpin2, gammatable[(int)g]);
              analogWrite(bluepin2, gammatable[(int)b]);
              just_record = 2;
            }
  else if(counter >= 3){
              record[0]=(int)r;
              record[1]=(int)g;
              record[2]=(int)b;
              record[3]=(int)c;
              Serial.println("firstdetect");
              tft.drawRect(3, 3, 50, 120, 0xFFFF); //畫一個方框 x,y,w,h,顏色值
              tft.fillRect(6, 6, 44, 114, getColor(r,g,b));
              counter = 2;
              just_record = 1;
              analogWrite(redpin, gammatable[(int)r]);
              analogWrite(greenpin, gammatable[(int)g]);
              analogWrite(bluepin, gammatable[(int)b]);
              

  }
}

void delete_record(){
  if(just_record == 2){
             if(record[3]==0 && record[7]==0){
                //donothing
                Serial.print("donothing");
                Serial.println();
              }
              else{
                record[4]=0;
                record[5]=0;
                record[6]=0;
                record[7]=0;
                counter = 2;
                analogWrite(redpin2, gammatable[0]);
                analogWrite(greenpin2, gammatable[0]);
                analogWrite(bluepin2, gammatable[0]);
                Serial.println();
                for (int i = 0; i < 8; i++) {
                    Serial.print(record[i]);
              
            }
                just_record = 1;
                just_delete = 2;
              }

            }
             else if(just_record == 1){
              record[0]=0;
              record[1]=0;
              record[2]=0;
              record[3]=0;
              counter = 1;
              analogWrite(redpin, gammatable[0]);
              analogWrite(greenpin, gammatable[0]);
              analogWrite(bluepin, gammatable[0]);
              Serial.println();
                for (int i = 0; i < 8; i++) {
                    Serial.print(record[i]);
              
            }
              just_record = 2;
              just_delete = 1;         
    }  

}

uint16_t getColor(uint8_t red, uint8_t green, uint8_t blue){
  red   >>= 3;
  green >>= 2;
  blue  >>= 3;
  return (red << 11) | (green << 5) | blue;
}

boolean debounced() { //檢查是否按住不放以及避免按下一次就跑很多次的function
  unsigned long currentMillis = millis(); // get current elapsed time
  if ((currentMillis - lastMillis) > debounceDelay) {
    lastMillis = currentMillis; // update lastMillis with currentMillis
    return true; // debounced
  }
  else {return false;} // not debounced
}




