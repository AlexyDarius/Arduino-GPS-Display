/*
Arduino code for GPS & Display(UTM, LatLon, Data) from DariusDev
@author AlexyRoman
*/

/* Import */
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <math.h>

/* Pins */
#define rxPin 8 //GPS rx pin
#define txPin 9 //GPS tx pin
#define buttonPin 4 //Page button
SoftwareSerial mygps(rxPin, txPin);

/* Screen settings */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

/* GPS initialization */
TinyGPSPlus gps;

/* Maths for UTM conversion */
const float FOURTHPI = PI / 4.0f;
const float deg2rad = PI / 180.0f;
const float rad2deg = 180.0f / PI;
const float equrad = 6377563.0f;
const float squecc = 0.00667054f;

/* UTM Coordinates structure */
struct UTMCoordinates {
    double easting;
    double northing;
    String zoneNumber;
};

/* Flags to indicate whether to show pages */
bool showFirstPage = true;
bool showSecondPage = false;
bool showThirdPage = false;
int currentPage = 1;  // Initialize currentPage to 1

/* Blinking point on screen to show functioning screen */
bool onBlink = true;

/* Setup */
void setup() {
  Serial.begin(9600);
  mygps.begin(9600);

  pinMode(buttonPin, INPUT_PULLUP);  // Set the button pin as input with internal pull-up resistor

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    for (;;)
      ;  // Don't proceed, loop forever
  }

  /* Display initialization */
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(25, 25);
  display.setTextSize(2);
  display.print(F("HELLO !"));
  display.display();

  delay(3000); //Initialization delay, useful for GPS search
}

/* Function to display any type of data */
void displayData(const __FlashStringHelper* label, double data, int precision, const __FlashStringHelper* unit, int xLabel, int yLabel, int xValue, bool smallDisplay) {
  display.setCursor(xLabel, yLabel); //x and y pos on screen for label
  display.setTextSize(2);
  if (smallDisplay) { // for small informations (bottom of the screen)
    display.setTextSize(1);
  }
  display.print(label);
  if (!smallDisplay) {
    display.setCursor(xValue , yLabel + 5); //value pos on screen
  }
  display.setTextSize(1);
  display.print(data, precision); //data and numbers after the comma
  display.print(unit); //label for unit
}

/* Function to display date */
void displayDate() {
  bool nextDay = false; 
  if (gps.time.hour() > 22) { //handling next day problem because of UTC data
    nextDay = true;
  }
  display.print(gps.date.day() + nextDay);
  display.print(F("/"));
  display.print(gps.date.month());
}

/* Function to display time */
void displayTime() {
  display.print((gps.time.hour() + 2) % 24); //conversion of time on UTC+02:00
  display.print(F(":"));
  display.print(gps.time.minute());
}

/* Function to convert LatLon to UTC coordinates */
UTMCoordinates LLtoUTM(const double Lat, const double Long)
//Written by walvok - https://github.com/walvok
{
  double a = 6378137;
  double eccSquared = 0.00669438;   //WGS-84, 6378137, 0.00669438
  double k0 = 0.9996;
  double LongOrigin;
  double eccPrimeSquared;
  double N, T, C, A, M;
  double UTMEasting;
  double UTMNorthing;
    
  //Make sure the longitude is between -180.00 .. 179.9
  double LongTemp = (Long+180)-int((Long+180)/360)*360-180; // -180.00 .. 179.9;
  double LatRad = Lat*deg2rad;
  double LongRad = LongTemp*deg2rad;
  double LongOriginRad;
  int    ZoneNumber;
    
  ZoneNumber = int((LongTemp + 180)/6) + 1;
  if( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
    ZoneNumber = 32;
  // Special zones for Svalbard
  if( Lat >= 72.0 && Lat < 84.0 ) 
  {
    if(      LongTemp >= 0.0  && LongTemp <  9.0 ) ZoneNumber = 31;
    else if( LongTemp >= 9.0  && LongTemp < 21.0 ) ZoneNumber = 33;
    else if( LongTemp >= 21.0 && LongTemp < 33.0 ) ZoneNumber = 35;
    else if( LongTemp >= 33.0 && LongTemp < 42.0 ) ZoneNumber = 37;
   }
  LongOrigin = (ZoneNumber - 1)*6 - 180 + 3;  //+3 puts origin in middle of zone
  LongOriginRad = LongOrigin * deg2rad;

  //compute the UTM Zone from the latitude and longitude
  eccPrimeSquared = (eccSquared)/(1-eccSquared);
  N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
  T = tan(LatRad)*tan(LatRad);
  C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
  A = cos(LatRad)*(LongRad-LongOriginRad);
  M = a*((1 - eccSquared/4    - 3*eccSquared*eccSquared/64  - 5*eccSquared*eccSquared*eccSquared/256)*LatRad 
        - (3*eccSquared/8 + 3*eccSquared*eccSquared/32  + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
                  + (15*eccSquared*eccSquared/256 + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad) 
                  - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));
  
  UTMEasting = (double)(k0*N*(A+(1-T+C)*A*A*A/6
          + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
          + 500000.0);
  UTMNorthing = (double)(k0*(M+N*tan(LatRad)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
         + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));
 
  //Preparation to output
  String zoneDesignator = String(ZoneNumber);
  zoneDesignator += UTMLetterDesignator(Lat);

  //Output
  UTMCoordinates utmCoords;
  utmCoords.easting = UTMEasting;
  utmCoords.northing = UTMNorthing;
  utmCoords.zoneNumber = zoneDesignator;

  return utmCoords;
}

/* Function used to get UTC Zone */
char UTMLetterDesignator(double Lat)
{
  //Written by Chuck Gantz- chuck.gantz@globalstar.com
  char LetterDesignator;
  if((84 >= Lat) && (Lat >= 72)) LetterDesignator = 'X';
  else if((72 > Lat) && (Lat >= 64)) LetterDesignator = 'W';
  else if((64 > Lat) && (Lat >= 56)) LetterDesignator = 'V';
  else if((56 > Lat) && (Lat >= 48)) LetterDesignator = 'U';
  else if((48 > Lat) && (Lat >= 40)) LetterDesignator = 'T';
  else if((40 > Lat) && (Lat >= 32)) LetterDesignator = 'S';
  else if((32 > Lat) && (Lat >= 24)) LetterDesignator = 'R';
  else if((24 > Lat) && (Lat >= 16)) LetterDesignator = 'Q';
  else if((16 > Lat) && (Lat >= 8)) LetterDesignator = 'P';
  else if(( 8 > Lat) && (Lat >= 0)) LetterDesignator = 'N';
  else if(( 0 > Lat) && (Lat >= -8)) LetterDesignator = 'M';
  else if((-8> Lat) && (Lat >= -16)) LetterDesignator = 'L';
  else if((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
  else if((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
  else if((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
  else if((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
  else if((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
  else if((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
  else if((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
  else if((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
  else LetterDesignator = 'Z'; //This is here as an error flag to show that the Latitude is outside the UTM limits
  return LetterDesignator;
}

/********** LOOP **********/
void loop() {

  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (mygps.available()) {
      if (gps.encode(mygps.read())) {
        newData = true; //Checking if GPS gets new data
      }
    }
  }

  // Check if the button is pressed
  if (digitalRead(buttonPin) == LOW) {
    switch (currentPage) { //Changing page
      case 1:
        showFirstPage = false;
        showSecondPage = true;
        currentPage = 2;
        break;
      case 2:
        showSecondPage = false;
        showThirdPage = true;
        currentPage = 3;
        break;
      case 3:
        showThirdPage = false;
        showFirstPage = true;
        currentPage = 1;
        break;
      default:
        break;
    }
    delay(100); // Debounce delay
  }

  // If GPS gets new data
  if (newData == true) {
    newData = false;

    // display.setTextColor(SSD1306_WHITE);

    if (gps.location.isValid() == 1) { //Checking GPS data
      display.clearDisplay();  // Clear the display buffer

      /* Data Display */
      if (showFirstPage) {
        displayData(F("Lat: "), gps.location.lat(), 5, F(""), 0, 0, 50, false);
        displayData(F("Lon: "), gps.location.lng(), 5, F(""), 0, 18, 50, false);
        displayData(F("Alt: "), gps.altitude.meters(), NULL, F(" m"), 0, 36, 50, false);
        displayData(F("hdop:"), gps.hdop.value(), NULL, F(""), 0, 55, 0, true);
        displayData(F("/  sats:"), gps.satellites.value(), NULL, F(""), 70, 55, 0, true);
      } else if (showSecondPage) {
        //Compute UTC conversion if on second page
        double utmEasting = LLtoUTM(gps.location.lat(), gps.location.lng()).easting;
        double utmNorthing = LLtoUTM(gps.location.lat(), gps.location.lng()).northing;
        displayData(F("E: "), utmEasting, 0, F(""), 0, 0, 25, false);
        displayData(F("N: "), utmNorthing, 0, F(""), 0, 18, 25, false);
        displayData(F("Alt: "), gps.altitude.meters(), NULL, F(" m"), 0, 36, 50, false);
        displayData(F("hdop:"), gps.hdop.value(), NULL, F(""), 0, 55, 0, true);
        displayData(F("/  sats:"), gps.satellites.value(), NULL, F(""), 70, 55, 0, true);
      } else if (showThirdPage) {
        displayData(F("Spd: "), gps.speed.kmph(), 1, F("km/h"), 0, 0, 50, false);
        displayData(F("Hdg: "), gps.course.deg(), NULL, F("deg"), 0, 18, 50, false);
        displayData(F("Dtt: "), NULL, NULL, F(""), 0, 36, 50, false);
        displayDate();
        display.print(F(" "));
        displayTime();
        displayData(F("hdop:"), gps.hdop.value(), NULL, F(""), 0, 55, 0, true);
        displayData(F("/  sats:"), gps.satellites.value(), NULL, F(""), 70, 55, 0, true);
      }

      // Blinking point
      if (onBlink) {
        display.setCursor(115, 0);
        display.setTextSize(2);
        display.print(F("*"));
        onBlink = false;
      } else {
        onBlink = true;
      }

      //Display
      display.display();
      delay(1000);
      display.clearDisplay();

    }else{ // GPS doesn't have a fix (searching)
      display.clearDisplay();
      display.setCursor(5, 10);
      display.setTextSize(2);
      display.print(F("Searching gps signal..."));
      delay(1000);
      display.display();
      display.clearDisplay();
    }

  } else { //GPS can't get new data
    display.clearDisplay();
    display.setCursor(25, 25);
    display.setTextSize(2);
    display.print(F("NO DATA"));
    delay(1000);
    display.display();
    display.clearDisplay();
  }
  
}