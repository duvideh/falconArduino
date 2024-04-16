// Arduino UNO in Falcon FG connected to Adafruit TFT screen and MCP2515 CAN shield
// displays engine oil temp, trans oil temp etc. 
//  
// 

#include <Arduino.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SdFat.h>                // SD card & FAT filesystem library
#include <Adafruit_SPIFlash.h>    // SPI / QSPI flash library
#include <Adafruit_ImageReader.h> // Image-reading functions
#include <SPI.h>

 // TFT (use tft.### functions i.e. tft.println() )
 #define SD_CS          13 // SD card select pin
 #define TFT_DC         8 // TFT display/command pin
 #define TFT_RST        9 // Or set to -1 and connect to Arduino RESET pin
 #define TFT_CS        10 // TFT select pin 
 
 Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST); 

 //SD Card stuff 
  SdFat SD; // SD card filesystem
  Adafruit_ImageReader reader(SD); // Image-reader object, pass in SD filesys 

 Adafruit_Image       img;        // An image loaded into RAM
 int32_t              width  = 0, // BMP image dimensions
                      height = 0;
 
 //variables to hold cursor coordinates
 int x = 0;
 int y = 0;
 uint16_t black = 0x0000;
 uint16_t white = 0xFFFF;
 uint16_t red = 0xF800;
 uint16_t light_red = 0xE7E0;
 uint16_t blue = 0x001F;
 uint16_t yellow = 0xFFE0;
 uint16_t green = 0x07E0;
 uint16_t cyan = 0x07FF;
 uint16_t magenta = 0xF81F;
 uint16_t orange = 0xfd00;
 uint16_t greenDark = 0x05ab;
 uint16_t blueDark = 0x3997;

 //battery voltage
 #define batAnalogIn A0
 float batSensor = 0;
 float batVoltage = 0;
 float batLast = 0;
 float batAvg = 0;
 //***calibrate here*** 
 float aref = 1.063; //change this to the actual Aref voltage of ---YOUR--- Arduino 
 float r1 = 98.5; //insert first resistor value
 float r2 = 6.73; //insert second resistor value

 //headlight dimming
 #define headlightSignal 31
 #define Lite 12
 int headlights = 0;

//millis to set delay between cycles of program
 unsigned long millis10 = 0;
 unsigned long millis200 = 0;

static const unsigned char PROGMEM oil_lamp[] = {
  // 'Oil Temp', 47x32px
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 
  0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x38, 0x00, 0x38, 0x00, 0x00, 0x00, 
  0x3e, 0x00, 0x3f, 0x80, 0x00, 0x00, 0x3f, 0xc0, 0x3f, 0x80, 0x00, 0x00, 0x3f, 0xf0, 0x38, 0x00, 
  0x00, 0xf8, 0x7f, 0xf0, 0x38, 0x00, 0x0f, 0xfc, 0x79, 0xf0, 0x38, 0x00, 0x3f, 0xfe, 0x78, 0x3c, 
  0x3f, 0x81, 0xff, 0x9c, 0xfc, 0x3e, 0x3f, 0x87, 0xff, 0x00, 0xff, 0xbe, 0x38, 0x1f, 0xfe, 0x00, 
  0x3f, 0xfc, 0x38, 0x3f, 0x9e, 0x08, 0x0f, 0xf8, 0x38, 0x7c, 0x1c, 0x1c, 0x03, 0xf8, 0x7c, 0x70, 
  0x38, 0x1c, 0x00, 0x78, 0xfe, 0x00, 0x70, 0x1c, 0x00, 0x38, 0xfe, 0x00, 0xe0, 0x1c, 0x00, 0x38, 
  0x7c, 0x01, 0xe0, 0x3e, 0x00, 0x38, 0x38, 0x03, 0xc0, 0x3e, 0x00, 0x38, 0x00, 0x07, 0x80, 0x3e, 
  0x00, 0x38, 0x00, 0x07, 0x00, 0x3e, 0x00, 0x3f, 0xff, 0xff, 0x00, 0x1c, 0x00, 0x3f, 0xff, 0xfe, 
  0x00, 0x00, 0x00, 0x3f, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
static const unsigned char battSmall [] PROGMEM = {
  0x30, 0x00, 0x00, 0x30, 0x00, 0x00, 0xfc, 0x03, 0xf0, 0xfc, 0x03, 0xf0, 0x30, 0x00, 0x00, 0x30, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0xc0, 0x30, 0x00, 0xc0, 0xff, 0xff, 
  0xf0, 0xff, 0xff, 0xf0, 0xc0, 0x00, 0x30, 0xc0, 0x00, 0x30, 0xc0, 0x00, 0x30, 0xc0, 0x00, 0x30, 
  0xc0, 0x00, 0x30, 0xc0, 0x00, 0x30, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xf0
  };
// 'Coolant small', 26x30px
static const unsigned char coolant [] PROGMEM = {
  0x00, 0x60, 0x00, 0x00, 0x60, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x60, 0x00, 0x00, 
  0x7e, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x60, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x60, 
  0x00, 0x78, 0x61, 0xe0, 0xfc, 0xf3, 0xf0, 0xce, 0xf7, 0x30, 0x00, 0xf0, 0x00, 0x7c, 0x63, 0xe0, 
  0xfe, 0x07, 0xf0, 0xc7, 0x0e, 0x30, 0x03, 0xfc, 0x00, 0x01, 0xf8, 0x00
};

//  __      ______ _____ _____
//  \ \    / / __ \_   _|  __ \ "/"
//   \ \  / / |  | || | | |  | |
//    \ \/ /| |  | || | | |  | |
//     \  / | |__| || |_| |__| |
//      \/   \____/_____|_____/

//============

void batVolts (void)
{
  batSensor = analogRead(batAnalogIn);
  batVoltage = (batSensor * ((r1+r2)/r2) ) * (aref/1024);
  //ADC reads 0-5v in 0-1023
  //Therefore, equation is 5/1023 for measured voltage
  //Multiply that (0.0048828125) by 3.672619048 for battery voltage
  batAvg = (batVoltage + batLast) / 2;
  batLast = batAvg;
}

void setup() {
  analogReference(INTERNAL); // use ianalogReference(INTERNAL)nternal voltage reference
                                //***CAUTION*** do not connect >1v to any analogRead pin!!!

  //headlights I/O
  pinMode(Lite,OUTPUT);
  digitalWrite(Lite,LOW);
  pinMode(headlightSignal,INPUT);

  // TFT
  tft.initR(INITR_144GREENTAB); // Init ST7735R chip, green tab
  //SD Card stuff
  ImageReturnCode stat; // Status from image-reading functions
  // The Adafruit_ImageReader constructor call (above, before setup())
  // accepts an uninitialized SdFat or FatVolume object. This MUST
  // BE INITIALIZED before using any of the image reader functions!
  Serial.print(F("Initializing filesystem..."));
  if(!SD.begin(SD_CS, SD_SCK_MHZ(10))) { // Breakouts require 10 MHz limit due to longer wires
    Serial.println(F("SD begin() failed"));
    for(;;); // Fatal error, do not continue
  }
  Serial.println(F("OK!")); 
  Serial.print(F("Loading 86x128.bmp to screen..."));
  stat = reader.drawBMP("/86x128.bmp", tft, 0, 0);
  reader.printStatus(stat);   // How'd we do?     
    
  analogWrite(Lite,255); //turn on backlight
  tft.fillScreen(black);

    //bitmaps, bars, static numbers
   tft.drawBitmap(0, 0, oil_lamp, 47, 27, orange);
   tft.drawBitmap(0, 68, battSmall, 20, 20, orange);
   tft.drawBitmap(0, 47, coolant, 20, 20, orange);
   tft.drawChar(102,7,0x09,red,black,2);
   tft.setTextSize(2);
   tft.setTextColor(red);   
   tft.setCursor(112,13);
   tft.print("C");
   //oil temp gauge
   tft.fillRect(0,34,2,2,orange);
   tft.fillRect(25,34,2,2,orange);
   tft.fillRect(55,34,2,2,orange);
   tft.fillRect(110,34,2,2,orange);
   tft.fillRect(126,34,2,2,orange);
   tft.fillRect(83,34,2,2,orange);
   tft.fillRect(0,36,128,1,orange);
   tft.setTextColor(orange);      
   tft.setTextSize(1);
   tft.setCursor(21,39);
   tft.print(40);
   tft.setCursor(51,39);
   tft.print(85);
   tft.setCursor(75,39);
   tft.print(130);
   tft.setCursor(102,39);
   tft.print(170);

  millis10 = millis();
  millis200 = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}