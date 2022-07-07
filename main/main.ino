/*!
 * @file colorview.ino
 * @brief DFRobot's Color Sensor
 * @n [Get the module here]
 * @n This example read current R,G,B,C value by the IIC bus
 * @n [Connection and Diagram](http://wiki.dfrobot.com.cn/index.php?title=(SKU:SEN0212)Color_Sensor-TCS34725_%E9%A2%9C%E8%89%B2%E4%BC%A0%E6%84%9F%E5%99%A8)
 *
 * @copyright   [DFRobot](https://www.dfrobot.com), 2016
 * @copyright   GNU Lesser General Public License
 *
 * @author [carl](carl.xu@dfrobot.com)
 * @version  V1.0
 * @date  2016-07-12
 */
#include <Wire.h>
#include "DFRobot_TCS34725.h"
#include "Arduino_LSM6DS3.h"

// Pick analog outputs, for the UNO these three work well
// use ~560  ohm resistor between Red & Blue, ~1K for green (its brighter)
//兩顆LED燈的RGB接角，接在D端
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

int counter = 1;
int just_record = 0;
int just_delete = 0;
byte record[6]; 
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

void setup() {
  Serial.begin(115200);
  Serial.println("Color View Test!");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");
  
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
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
    Serial.println(gammatable[i]);
  }
}

void loop() {
  
    long measurement = TP_init(); 
        // Serial.println(measurement);
        if (measurement > 20000){
          Serial.println("shake");
          mix_color();
          delay(100);
        }
        else{
          
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
            
          
           recording(r,g,b); //呼叫擷取的function
           Serial.println("-----------");
           Serial.println(clear);
           Serial.println(red);
           Serial.println(green);
           Serial.println(blue);
           Serial.println("-----------");

           Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
           Serial.println();
           for (int i = 0; i < 6; i++) {
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
      

//  tcs.lock();  // turn off LED
//  Serial.print("C:\t"); Serial.print(clear);
//  Serial.print("\tR:\t"); Serial.print(red);
//  Serial.print("\tG:\t"); Serial.print(green);
//  Serial.print("\tB:\t"); Serial.print(blue);
//  Serial.println("\t");
//  Serial.print((int)r ); Serial.print(" "); Serial.print((int)g);Serial.print(" ");  Serial.println((int)b );
long TP_init(){
  // delay(10);
  long measurement=pulseIn (vibr, HIGH);  // 等待 D0 輸入高電壓，並回傳值
  return measurement;
}

void mix_color(){
    record_mix[0] = (record[0] + record[3])/2;
    record_mix[1] = (record[1] + record[4])/2;
    record_mix[2] = (record[2] + record[5])/2;
    Serial.println("mix_color:"); 
    for (int i = 0; i < 3; i++) {
      Serial.print("\t"); 
      Serial.print(record_mix[i]);
    }
    analogWrite(redpin_mix, gammatable[(int)record_mix[0]]);
    analogWrite(greenpin_mix, gammatable[(int)record_mix[1]]);
    analogWrite(bluepin_mix, gammatable[(int)record_mix[2]]);
  
}

void recording(byte r,byte g,byte b){

  if(counter == 1){
              record[0]=(int)r;
              record[1]=(int)g;
              record[2]=(int)b;
              Serial.println("firstdetect");
              counter ++;
              analogWrite(redpin, gammatable[(int)r]);
              analogWrite(greenpin, gammatable[(int)g]);
              analogWrite(bluepin, gammatable[(int)b]);
              just_record = 1;
              
            }
            
  else if(counter == 2){
              record[3]=(int)r;
              record[4]=(int)g;
              record[5]=(int)b;
              Serial.println("seconddetect");
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
              Serial.println("firstdetect");
              counter = 2;
              just_record = 1;
              analogWrite(redpin, gammatable[(int)r]);
              analogWrite(greenpin, gammatable[(int)g]);
              analogWrite(bluepin, gammatable[(int)b]);
              

  }
}

void delete_record(){
  if(just_record == 2){
             if(record[0]==0 && record[1]==0 && record[2]==0 && record[3]==0 && record[4]==0 && record[5]==0){
                //donothing
                Serial.print("donothing");
                Serial.println();
                for (int i = 0; i < 6; i++) {
                    Serial.println(record[i]);
            }
              }
              else{
                record[3]=0;
                record[4]=0;
                record[5]=0;
                counter = 2;
                analogWrite(redpin2, gammatable[0]);
                analogWrite(greenpin2, gammatable[0]);
                analogWrite(bluepin2, gammatable[0]);
                Serial.println();
                for (int i = 0; i < 6; i++) {
                    Serial.println(record[i]);
              
            }
                just_record = 1;
                just_delete = 2;
              }

            }
             else if(just_record == 1){
              record[0]=0;
              record[1]=0;
              record[2]=0;
              counter = 1;
              analogWrite(redpin, gammatable[0]);
              analogWrite(greenpin, gammatable[0]);
              analogWrite(bluepin, gammatable[0]);
              Serial.println();
                for (int i = 0; i < 6; i++) {
                    Serial.println(record[i]);
              
            }
              just_record = 2;
              just_delete = 1;         
    }  

}

boolean debounced() { //檢查是否按住不放以及避免按下一次就跑很多次的function
  unsigned long currentMillis = millis(); // get current elapsed time
  if ((currentMillis - lastMillis) > debounceDelay) {
    lastMillis = currentMillis; // update lastMillis with currentMillis
    return true; // debounced
  }
  else {return false;} // not debounced
}

