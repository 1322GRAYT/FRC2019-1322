#include <Adafruit_NeoPixel.h>
#include <Wire.h>

#define PIN_L 6
#define PIN_R 5
#define LEDNUM 24

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)

// NOTE: On Our LEDS, the Color Code has the Green and Blue swapped. EX: strip.Color(red, blue, green)

/* The I2C color codes are as follows:
'r' = Set to red, 'g' = Set to green, 'b' = Set to blue, 'u' = Rainbow, 'c' = Rainbow cycle,
'h' = Chase, 'o' = Off
*/
#define MIC_PIN A0

Adafruit_NeoPixel stripL = Adafruit_NeoPixel(LEDNUM, PIN_L, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripR = Adafruit_NeoPixel(LEDNUM, PIN_R, NEO_GRB + NEO_KHZ800);


int vol = 0;
float total = 0;
int fadeCol = 0;
int val[25];
int volLast = 0;
int fadeAmt = 0;
int counter = 0;
int c;
char currentColor = 'y';
char IData = 'p';
int VData = 2;

uint32_t colorRed = stripR.Color(255, 0, 0);
uint32_t colorGreen = stripR.Color(0, 0, 255);
uint32_t colorBlue = stripR.Color(0, 255, 0);
uint32_t colorYellow = stripR.Color(255, 0, 200);
uint32_t colorOff = stripR.Color(0, 0, 0);

void setup() {
  Serial.begin(9600);
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
  stripL.begin();
  stripR.begin();
  stripL.setBrightness(100); //adjust brightness here
  stripR.setBrightness(100); //adjust brightness here
  stripL.show(); // Initialize all pixels to 'off'
  stripR.show(); // Initialize all pixels to 'off'
  colorWipe(colorYellow,2);
  delay(50);
  colorWipe(stripL.Color(0,0,0),0);
  delay(50);
  colorWipe(colorYellow,2);
  delay(50);
  colorWipe(stripL.Color(0,0,0),0);
  breathe('y');
  //visualize2();
}

void loop() {
  switch(IData){
    case 'r': // i2c code (decimal format of string): 114
      colorWipe(colorRed, 2); // Red
      break;
    case 'b': // i2c code (decimal format of string): 98
      colorWipe(colorBlue, 2); // Blue
      break;
    case 'g': // i2c code (decimal format of string): 103
      colorWipe(colorGreen, 2); // Green
      break;
    case 'y': // i2c code (decimal format of string): 
      colorWipe(colorYellow, 2); // Yellow
      break;
    case 'u': // i2c code (decimal format of string): 117
      rainbow(20); // Rainbow
      break;
    case 'c': // i2c code (decimal format of string): 99
      rainbowCycle(20);// Rainbow Cycle
      break;
    case 'h': // i2c code (decimal format of string): 104
      if(currentColor == 'r'){
        chase(colorRed); //Chase red
      }
      else if(currentColor == 'g'){
        chase(colorGreen); //Chase green
      }
      else if(currentColor == 'b'){
        chase(colorBlue); //Chase blue
      }
      else if(currentColor == 'y'){
        chase(colorYellow); //Chase yellow
      }
      break;
    case 't': // i2c code (decimal format of string): 116
      if(currentColor == 'r'){
        checkerboard(colorRed); //checkerboard red
      }
      else if(currentColor == 'g'){
        checkerboard(colorGreen); //checkerboard green
      }
      else if(currentColor == 'b'){
        checkerboard(colorBlue); //checkerboard blue
      }
      else if(currentColor == 'y'){
        checkerboard(colorYellow); //checkerboard yellow
      }
      break;
    case 'o': // i2c code (decimal format of string): 111
      colorWipe(colorOff, 0); //Off
      break;
    case 'p': // i2c code (decimal format of string): 112
      if(currentColor == 'r'){
        breathe('r'); //breath red
      }
      else if(currentColor == 'g'){
        breathe('g'); //breath green
      }
      else if(currentColor == 'b'){
        breathe('b'); //breath blue
      }
      else if(currentColor == 'y'){
        breathe('y'); //breath yellow
      }
      break;
    case 's': // i2c code (decimal format of string): 115
      visualize2();
      break;
}
}

void receiveEvent(int howMany){
  while(Wire.available()){
    c = Wire.read();
    Serial.println(c);
    IData = c;
  }
  if(c == 'r'){
    currentColor = 'r';
  }
  else if(c=='g'){
    currentColor = 'g';
  }
  else if(c=='b'){
    currentColor = 'b';
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<stripL.numPixels(); i++) {
      stripL.setPixelColor(i, c);
      stripR.setPixelColor(i, c);
      stripL.show();
      stripR.show();
      delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<stripL.numPixels(); i++) {
      stripL.setPixelColor(i, Wheel((i+j) & 255));
      stripR.setPixelColor(i, Wheel((i+j) & 255));
    }
    stripL.show();
    stripR.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< stripL.numPixels(); i++) {
      stripL.setPixelColor(i, Wheel(((i * 256 / stripL.numPixels()) + j) & 255));
      stripR.setPixelColor(i, Wheel(((i * 256 / stripL.numPixels()) + j) & 255));
    }
    stripL.show();
    stripR.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return stripL.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return stripL.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return stripL.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
 
static void chase(uint32_t c) {
  for(uint16_t i=0; i<stripL.numPixels()+4; i++) {
      stripL.setPixelColor(i  , c); // Draw new pixel
      stripL.setPixelColor(i-4, 0); // Erase pixel a few steps back
      stripL.show();
      stripR.setPixelColor(i  , c); // Draw new pixel
      stripR.setPixelColor(i-4, 0); // Erase pixel a few steps back
      stripR.show();
      delay(25);
  }
}

void checkerboard(uint32_t c){
  for(uint16_t i=0; i<stripL.numPixels() + 2; i++){
    stripL.setPixelColor(i,c);
    stripL.setPixelColor(i - 1, (0,0,0));
    stripL.setPixelColor(i + 1, (0,0,0));
    stripL.setPixelColor(i + 2,c);
    stripL.setPixelColor(i - 10, (0,0,0));
    stripL.setPixelColor(i + 10, (0,0,0));
    stripL.setPixelColor(i + 12,c);
    stripL.setPixelColor(i - 18, (0,0,0));
    stripL.setPixelColor(i + 18, (0,0,0));
    stripL.setPixelColor(i + 20,c);
    stripL.setPixelColor(i - 26, (0,0,0));
    stripL.setPixelColor(i + 26, (0,0,0));
    stripL.setPixelColor(i + 28,c);
    stripL.show();
    
    stripR.setPixelColor(i,c);
    stripR.setPixelColor(i - 1, (0,0,0));
    stripR.setPixelColor(i + 1, (0,0,0));
    stripR.setPixelColor(i + 2,c);
    stripR.setPixelColor(i - 10, (0,0,0));
    stripR.setPixelColor(i + 10, (0,0,0));
    stripR.setPixelColor(i + 12,c);
    stripR.setPixelColor(i - 18, (0,0,0));
    stripR.setPixelColor(i + 18, (0,0,0));
    stripR.setPixelColor(i + 20,c);
    stripR.setPixelColor(i - 26, (0,0,0));
    stripR.setPixelColor(i + 26, (0,0,0));
    stripR.setPixelColor(i + 28,c);
    stripR.show();
    delay(75);
  }
}

void breathe(char c){
  if(c == 'r'){
    for(int x=0; x<255;x+=15){
      colorWipe(stripL.Color(x,0,0),0);
      delay(2);
    }
    for(int x=255; x>0;x-=15){
      colorWipe(stripL.Color(x,0,0),0);
      delay(2);
    }
  }
  else if(c=='g'){
    for(int x=0; x<255;x+=15){
      colorWipe(stripL.Color(0,0,x),0);
      delay(2);
    }
    for(int x=255; x>0;x-=15){
      colorWipe(stripL.Color(0,0,x),0);
      delay(2);
    }
  }
  else if(c=='b'){
    for(int x=0; x<255;x+=15){
      colorWipe(stripL.Color(0,x,0),0);
      delay(2);
    }
    for(int x=255; x>0;x-=15){
      colorWipe(stripL.Color(0,x,0),0);
      delay(2);
    }
  }
  else if(c=='y'){
    for(int x=0; x<255;x+=15){
      int y = (x > 51) ? x - 50 : 0;
      colorWipe(stripL.Color(y,0,x),0);
      delay(2);
    }
    for(int x=255; x>0;x-=15){
      int y = (x > 51) ? x - 50 : 0;
      colorWipe(stripL.Color(y,0,x),0);
      delay(2);
    }
  }//TODO: Add Yellow Breath
}

void visualize(uint16_t c){
  VData = analogRead(0);
  delay(10);
  int y = map(VData,300,500,1,20);
  if(y<21){
    for(int r=0; r<20;r++){
      stripL.setPixelColor(r,stripL.Color(0,0,0));
      stripR.setPixelColor(r,stripR.Color(0,0,0));
    }
    if(y>8){
      for(int i=0;i<8;i++){
        stripL.setPixelColor(i,c);
        stripL.show();
        stripR.setPixelColor(i,c);
        stripR.show();
      }
      if(y>12){
        for(int x=8;x<y;x++){
        stripL.setPixelColor(x,stripL.Color(255,255,0));
        stripL.show();
        stripR.setPixelColor(x,stripR.Color(255,255,0));
        stripR.show();
      }
        if(y>=20){
          for(int p=13;p<y;p++){
        stripL.setPixelColor(p,stripL.Color(255,0,0));
        stripL.show();
        stripR.setPixelColor(p,stripR.Color(255,0,0));
        stripR.show();
      }
        }
      }
    }
    else if(y<=8){
      for(int z=0;z<y;z++){
        stripL.setPixelColor(z,c);
        stripL.show();
        stripR.setPixelColor(z,c);
        stripR.show();
      }
    }
  }
  stripL.show();
  stripR.show();
}

uint32_t Wheel2(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
   return stripL.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
   return stripL.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
   WheelPos -= 170;
   return stripL.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

void rainbowCycle2(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< stripL.numPixels(); i++) {
      stripL.setPixelColor(i, Wheel2(((i * 256 / stripL.numPixels()) + j) & 255));
      stripR.setPixelColor(i, Wheel2(((i * 256 / stripR.numPixels()) + j) & 255));
    }
    stripL.show();
    stripR.show();
    delay(wait);
     vol = analogRead(MIC_PIN);
     if (vol> 10) {

      return;
     }
  }
}

void visualize2(){
   fadeCol = 0;
  total = 0;

  for (int i = 0; i < 80; i++){
      counter = 0;
       do{
      vol = analogRead(MIC_PIN);

      counter = counter + 1;
      if (counter > 500){
         rainbowCycle2(10);

      }
    }while (vol == 0);

    total = total + vol;

  }

  vol = total / 100;

  Serial.print("BEFORE: ");
  Serial.println(vol);
  vol = map(vol,270,330,0,20);
  Serial.print("AFTER: ");
  Serial.println(vol);

  if (volLast > vol) {
    vol = volLast - 4;
  }

  volLast = vol;
  fadeAmt = 10;


  for (int i = 0; i<150;i++){
// Serial.print("AFTER: ");
//   Serial.println(vol);
    if (i < vol){
         stripL.setPixelColor((i+150), stripL.Color(0,255,0));
         stripL.setPixelColor((150-i), stripL.Color(0,255,0));
         stripR.setPixelColor((i+150), stripR.Color(0,255,0));
         stripR.setPixelColor((150-i), stripR.Color(0,255,0));
    }
    else if (i < (vol + 38)) {
         stripL.setPixelColor((i+150), stripL.Color(255,0,0));
         stripL.setPixelColor((150-i), stripL.Color(255,0,0));
         stripR.setPixelColor((i+150), stripR.Color(255,0,0));
         stripR.setPixelColor((150-i), stripR.Color(255,0,0));
    }
    else
    {
         stripL.setPixelColor((i+150), stripL.Color(0,0,255));
         stripL.setPixelColor((150-i), stripL.Color(0,0,255));
         stripR.setPixelColor((i+150), stripR.Color(0,0,255));
         stripR.setPixelColor((150-i), stripR.Color(0,0,255));
    }
  }
  stripL.show();
  stripR.show();

}
