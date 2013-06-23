#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Wire.h>
#include <math.h>
// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin


// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

TinyGPS gps; // initialize our gps object here
#define GPSRXPIN 10
#define GPSTXPIN 11 //define the gps rx and tx pins
SoftwareSerial GPSserial(GPSTXPIN, GPSRXPIN);



static bool feedgps();
static void gpsdump(TinyGPS &gps);
static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);


float maxSpeed = 0;
float lastFlat = 0;
float lastFlon = 0;
long totalDistance = 0;

boolean start = 1;
int i = 0;

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET); //initialize the lcd screen as an object
void setup (void){
  Serial.begin(9600); // set serial to 9600 baud so we can read it for lcd screen
  tft.reset();
  uint16_t identifier = tft.readID(); //ok, so this variable is an unsigned integer with maximum value of 16 that determines which screen we're using and loads the driver for it
  tft.begin(identifier); // this is where we initialize the driver
 tft.fillScreen(BLACK);
 GPSserial.begin(9600); // initialize GPS serial 
}

void loop(){
  bool newdata = false;
  unsigned long start = millis();
  
  // Every second we print an update
  while (millis() - start < 1000)
  {
    if (feedgps())
      newdata = true;   
  }
  
  if (newdata)
  {
    Serial.println("Acquired Data");
    Serial.println("-------------");
    gpsdump(gps);
    Serial.println("-------------");
    Serial.println();
  }
  else {
    Serial.println("No data");
  } 
}


static void gpsdump(TinyGPS &gps)
{
  float flat, flon;
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;

  gps.f_get_position(&flat, &flon, &age);

  //print_date(gps);

  //gps.stats(&chars, &sentences, &failed);
  //print_int(chars, 0xFFFFFFFF, 6);
  //print_int(sentences, 0xFFFFFFFF, 10);
  //print_int(failed, 0xFFFFFFFF, 9);
  //Serial.println();
  
  if (gps.f_speed_kmph() > 3.9)
  {
    if (start == 1)
    {
      start = 0;
      lastFlat = flat;
      lastFlon = flon;
    }
    else
    {
      //totalDistance = gps.distance_between(flat, flon, LONDON_LAT, LONDON_LON);
      totalDistance = totalDistance + calc_dist(flat, flon, lastFlat, lastFlon);
      lastFlat = flat;
      lastFlon = flon;
    }
  }
  
  
  tft.setTextSize(1);
  tft.setTextColor(WHITE);// initialize text variables for tft
  tft.setCursor(0,0);
  
  float fDist = totalDistance; // in meters
 float fspeed = gps.f_speed_kmph();
  //float fSpeed = gps.f_speed_kmph();
  printLCDFloat (fDist, 2);  
  tft.print(" Metres ");
  printLCDFloat (fspeed, 2);
  tft.print(" km/h");
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  feedgps();
}

static void print_float(float val, float invalid, int len, int prec)
{
  char sz[32];
  if (val == invalid)
  {
    strcpy(sz, "*******");
    sz[len] = 0;
        if (len > 0) 
          sz[len-1] = ' ';
    for (int i=7; i<len; ++i)
        sz[i] = ' ';
    Serial.print(sz);
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(" ");
  }
  feedgps();
}

void printLCDFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0)
  {
     tft.print("-");
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  char sTemp[10];
  ltoa(int_part, sTemp, 10);
  tft.print(sTemp);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    tft.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    ltoa(toPrint, sTemp, 10);
   tft.print(sTemp);
    remainder -= toPrint; 
  } 
}

static void print_str(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  feedgps();
}

static bool feedgps()
{
  while (GPSserial.available())
  {
    if (gps.encode(GPSserial.read()))
      return true;
  }
  return false;
}


unsigned long calc_dist(float flat1, float flon1, float flat2, float flon2) // all of this below is to calculate the distance between two points when we only have the latitude and longitude degrees
{
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  diflat=radians(flat2-flat1);
  flat1=radians(flat1);
  flat2=radians(flat2);
  diflon=radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2= cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

  dist_calc*=6371000.0; //Converting to meters
  return dist_calc;
}
