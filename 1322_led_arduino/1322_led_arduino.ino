#include <Adafruit_NeoPixel.h>
#include <Wire.h>

#define PIN 6
#define LEDNUM 150

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)

/* The I2C color codes are as follows:
'r' = Set to red, 'g' = Set to green, 'b' = Set to blue, 'u' = Rainbow, 'c' = Rainbow cycle,
'h' = Chase, 'o' = Off
*/
#define MIC_PIN A0

Adafruit_NeoPixel strip = Adafruit_NeoPixel(LEDNUM, PIN, NEO_GRB + NEO_KHZ800);

int vol = 0;
float total = 0;
int fadeCol = 0;
int val[25];
int volLast = 0;
int fadeAmt = 0;
int counter = 0;
int c;
char currentColor = 'n';
char IData = 'o';
int VData = 2;

void setup() {
  Serial.begin(9600);
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
  strip.begin();
  strip.setBrightness(30); //adjust brightness here
  strip.show(); // Initialize all pixels to 'off'
  colorWipe(strip.Color(255,0,0),0);
  delay(50);
  colorWipe(strip.Color(0,0,0),0);
  delay(50);
  colorWipe(strip.Color(255,0,0),0);
  delay(50);
  colorWipe(strip.Color(0,0,0),0);
}

void loop() {
  switch(IData){
    case 'r': //Set color to red
      colorWipe(strip.Color(255, 0, 0), 2); // Red
      break;
    case 'g':
      colorWipe(strip.Color(0, 255, 0), 2); // Green
      break;
    case 'b':
      colorWipe(strip.Color(0, 0, 255), 2); // Blue
      break;
    case 'u':
      rainbow(20);
      break;
    case 'c':
      rainbowCycle(20);
      break;
    case 'h':
      if(currentColor == 'r'){
        chase(strip.Color(255,0,0)); //Chase red
      }
      else if(currentColor == 'g'){
        chase(strip.Color(0,255,0)); //Chase green
      }
      else if(currentColor == 'b'){
        chase(strip.Color(0,0,255)); //Chase blue
      }
      break;
    case 't':
      if(currentColor == 'r'){
        checkerboard(strip.Color(255,0,0)); //checkerboard red
      }
      else if(currentColor == 'g'){
        checkerboard(strip.Color(0,255,0)); //checkerboard green
      }
      else if(currentColor == 'b'){
        checkerboard(strip.Color(0,0,255)); //checkerboard blue
      }
      break;
    case 'o':
      colorWipe(strip.Color(0,0,0), 0); //Off
      break;
    case 'p':
      if(currentColor == 'r'){
        breathe('r'); //checkerboard red
      }
      else if(currentColor == 'g'){
        breathe('g'); //checkerboard green
      }
      else if(currentColor == 'b'){
        breathe('b'); //checkerboard blue
      }
      break;
    case 's':
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
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
 
static void chase(uint32_t c) {
  for(uint16_t i=0; i<strip.numPixels()+4; i++) {
      strip.setPixelColor(i  , c); // Draw new pixel
      strip.setPixelColor(i-4, 0); // Erase pixel a few steps back
      strip.show();
      delay(25);
  }
}

void checkerboard(uint32_t c){
  for(uint16_t i=0; i<strip.numPixels() + 2; i++){
    strip.setPixelColor(i,c);
    strip.setPixelColor(i - 1, (0,0,0));
    strip.setPixelColor(i + 1, (0,0,0));
    strip.setPixelColor(i + 2,c);
    strip.setPixelColor(i - 10, (0,0,0));
    strip.setPixelColor(i + 10, (0,0,0));
    strip.setPixelColor(i + 12,c);
    strip.setPixelColor(i - 18, (0,0,0));
    strip.setPixelColor(i + 18, (0,0,0));
    strip.setPixelColor(i + 20,c);
    strip.setPixelColor(i - 26, (0,0,0));
    strip.setPixelColor(i + 26, (0,0,0));
    strip.setPixelColor(i + 28,c);
    strip.show();
    delay(75);
  }
}

void breathe(char c){
  if(c == 'r'){
    for(int x=0; x<255;x+=15){
      colorWipe(strip.Color(x,0,0),0);
      delay(2);
    }
    for(int x=255; x>0;x-=15){
      colorWipe(strip.Color(x,0,0),0);
      delay(2);
    }
  }
  else if(c=='g'){
    for(int x=0; x<255;x+=15){
      colorWipe(strip.Color(0,x,0),0);
      delay(2);
    }
    for(int x=255; x>0;x-=15){
      colorWipe(strip.Color(0,x,0),0);
      delay(2);
    }
  }
  else if(c=='b'){
    for(int x=0; x<255;x+=15){
      colorWipe(strip.Color(0,0,x),0);
      delay(2);
    }
    for(int x=255; x>0;x-=15){
      colorWipe(strip.Color(0,0,x),0);
      delay(2);
    }
  }
}

void visualize(uint16_t c){
  VData = analogRead(0);
  delay(10);
  int y = map(VData,300,500,1,20);
  if(y<21){
    for(int r=0; r<20;r++){
      strip.setPixelColor(r,strip.Color(0,0,0));
    }
    if(y>8){
      for(int i=0;i<8;i++){
        strip.setPixelColor(i,c);
        strip.show();
      }
      if(y>12){
        for(int x=8;x<y;x++){
        strip.setPixelColor(x,strip.Color(255,255,0));
        strip.show();
      }
        if(y>=20){
          for(int p=13;p<y;p++){
        strip.setPixelColor(p,strip.Color(255,0,0));
        strip.show();
      }
        }
      }
    }
    else if(y<=8){
      for(int z=0;z<y;z++){
        strip.setPixelColor(z,c);
        strip.show();
      }
    }
  }
  strip.show();
}

uint32_t Wheel2(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

void rainbowCycle2(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel2(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
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
         strip.setPixelColor((i+150), strip.Color(0,255,0));
         strip.setPixelColor((150-i), strip.Color(0,255,0));
    }
    else if (i < (vol + 38)) {
         strip.setPixelColor((i+150), strip.Color(255,0,0));
         strip.setPixelColor((150-i), strip.Color(255,0,0));
    }
    else
    {
         strip.setPixelColor((i+150), strip.Color(0,0,255));
         strip.setPixelColor((150-i), strip.Color(0,0,255));
    }
  }
  strip.show();

}
