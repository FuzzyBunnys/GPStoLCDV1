#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <SoftwareSerial.h>
#include <TinyGPS.h>
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
SoftwareSerial GPSserial (GPSRXPIN, GPSTXPIN);



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

void loop(void){
  bool newdata = false;
  unsigned long start = millis();
  
  // Every second we print an update
  while (millis() - start < 1000)
  {
    if (feedgps())
      newdata = true;
  }
  
  gpsdump(gps); 
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
 
 
  //float fSpeed = gps.f_speed_kmph();
  tft.print(fDist);
  tft.print(" Metres ");
  if (gps.f_speed_mps()<0){
    tft.print ("0.00");
  }else{
    tft.print(gps.f_speed_mps());
  }
  tft.print(" M/s");
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
