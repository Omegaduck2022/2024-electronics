#include <Arduino.h>
#define Motorpin 17
#define pushbutton 5  


#include <Wire.h>
#include "ScioSense_ENS160.h"  // 引入ENS160函式庫

//ScioSense_ENS160      ens160(ENS160_I2CADDR_0);
ScioSense_ENS160      ens160(ENS160_I2CADDR_1);  // 使用ENS160的I2C地址1，為0x53

#include "DFRobot_AHT20.h"  // 引入AHT20函式庫

DFRobot_AHT20 aht20;  // 建立AHT20物件


#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };



int lastState= LOW;
int currentState;


void setup(){
Serial.begin(9600);
pinMode(Motorpin,OUTPUT);
pinMode(pushbutton,INPUT_PULLUP);


  uint8_t status;
  while((status = aht20.begin()) != 0){  // 初始化AHT20感測器，若失敗則持續嘗試
    Serial.print("AHT20感測器初始化失敗。錯誤狀態 : ");
    Serial.println(status);
    delay(1000);
  }

  Serial.println("----------------------------");
  Serial.println("ENS160 - 數位空氣品質感測器");
  Serial.println();
  Serial.println("標準模式下的感測器讀取");
  Serial.println();
  Serial.println("----------------------------");
  delay(1000);

  Serial.print("ENS160...");
  ens160.begin();  // 初始化ENS160感測器
  Serial.println(ens160.available() ? "完成." : "失敗!");

  if (ens160.available()) {
    // 打印ENS160版本信息
    Serial.print("\t版本: "); Serial.print(ens160.getMajorRev());
    Serial.print("."); Serial.print(ens160.getMinorRev());
    Serial.print("."); Serial.println(ens160.getBuild());
  
    Serial.print("\t標準模式 ");
    Serial.println(ens160.setMode(ENS160_OPMODE_STD) ? "完成." : "失敗!");
  }

// SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Draw a single pixel in white

}

void loop(){
    currentState = digitalRead(pushbutton);
    if(lastState ==HIGH &&currentState==LOW){
        Serial.println("LOW->HIGH");
        digitalWrite(Motorpin, HIGH);
         delay(15000);

    }
    else if(lastState ==LOW &&currentState==HIGH){
        Serial.println("HIGH->LOW");
          digitalWrite(Motorpin, LOW); 

    }
    else{
        digitalWrite(Motorpin, LOW); 
    }
    lastState = currentState;







    if (ens160.available()) {  // 若ENS160可用，則進行測量並輸出結果
    ens160.measure(true);
    ens160.measureRaw(true);
  
    Serial.print("空氣品質指數(AQI): "); Serial.println(ens160.getAQI());
    Serial.print("總揮發性有機化合物(TVOC): "); Serial.print(ens160.getTVOC()); Serial.println("ppb");
    Serial.print("二氧化碳等效濃度(eCO2): "); Serial.print(ens160.geteCO2()); Serial.println("ppm");
    Serial.print("電阻值HP0: "); Serial.print(ens160.getHP0()); Serial.println("Ohm");
    Serial.print("電阻值HP1: "); Serial.print(ens160.getHP1()); Serial.println("Ohm");
    Serial.print("電阻值HP2: "); Serial.print(ens160.getHP2()); Serial.println("Ohm");
    Serial.print("電阻值HP3: "); Serial.print(ens160.getHP3()); Serial.println("Ohm");
  }

  if(aht20.startMeasurementReady(/* crcEn = */true)){  // 若AHT20可進行測量，則輸出溫濕度數據
    Serial.print("溫度(-40~85°C): ");
    Serial.print(aht20.getTemperature_C());  // 獲取攝氏溫度
    Serial.print("°C, ");
    Serial.print(aht20.getTemperature_F());  // 獲取華氏溫度
    Serial.println("°F");
    Serial.print("濕度(0~100): ");
    Serial.print(aht20.getHumidity_RH());  // 獲取相對濕度
    Serial.println(" %RH");
  }

  Serial.println("----------------------------");


  display.clearDisplay();

  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print(F("Temp.: "));
  display.println(aht20.getTemperature_C());
 display.print(F("AQI: "));
  display.println(ens160.getAQI());
display.print(F("eCO2: "));
  display.println(ens160.geteCO2());

  display.display();      // Show initial text
  delay(100);

}
