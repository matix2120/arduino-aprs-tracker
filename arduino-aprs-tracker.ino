// Arduino APRS Tracker (aat) with Arduino Pro Mini 3.3V/8 MHz
// Based on https://github.com/sh123/aprs_tracker
// 
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <LibAPRS.h>

// GPS SoftwareSerial
// Shares pins with (MISO 12/ MOSI 11) used for SPI
#define GPS_RX_PIN 12
#define GPS_TX_PIN 11

// LibAPRS
#define OPEN_SQUELCH false
#define ADC_REFERENCE REF_3V3

// APRS settings
#define LOW_SPEED 5 // km/h
#define HIGH_SPEED 120 // km/h
#define SLOW_RATE 30 * 60 * 1000L // 30 minutes
#define FAST_RATE 60 * 1000L // 60 seconds
#define TURN_TIME 15 * 1000L // 15 seconds
#define TURN_MIN 30
#define TURN_SLOPE 255

char APRS_CALLSIGN[]="NOCALL";
const int APRS_SSID=5;
char APRS_SYMBOL='>';

TinyGPS gps;
SoftwareSerial GPSSerial(GPS_RX_PIN, GPS_TX_PIN);

//long instead of float for latitude and longitude
long lat = 0;
long lon = 0;

int year=0;
byte month=0, day=0, hour=0, minute=0, second=0, hundredths=0;
unsigned long age=0;

unsigned long last_course;

// buffer for conversions
#define CONV_BUF_SIZE 16
static char conv_buf[CONV_BUF_SIZE];

inline unsigned long course_change_since_beacon(unsigned long current, unsigned long last)
{
  unsigned long diff;
  diff = abs(current - last);
  return (diff <= 180) ? diff : 360 - diff;
}

void setup()  
{
  Serial.begin(115200);
  GPSSerial.begin(9600);
  
  Serial.println(F("Arduino APRS Tracker"));

  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  //GPS.sendCommand(PGCMD_ANTENNA);

  APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
  APRS_setCallsign(APRS_CALLSIGN,APRS_SSID);
  APRS_setSymbol(APRS_SYMBOL);
}

void loop()
{
  bool newData = false;
  unsigned long now, beacon_rate, turn_threshold, last_tx_time, next_tx_time, course_change;
  unsigned speed = int(gps.f_speed_kmph());

  // For one second we parse GPS data
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (GPSSerial.available())
    {
      char c = GPSSerial.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, NULL, &age);
    gps.get_position(&lat, &lon, &age);

    // Serial.print(static_cast<int>(day)); Serial.print("/"); Serial.print(static_cast<int>(month)); Serial.print("/"); Serial.print(year);
    // Serial.print(" "); Serial.print(static_cast<int>(hour)); Serial.print(":"); Serial.print(static_cast<int>(minute)); Serial.print(":"); Serial.print(static_cast<int>(second));Serial.print(F(" "));

    //Serial.print(F("LAT="));
    Serial.print(lat);
    Serial.print(F(","));
    Serial.print(lon);
    Serial.print(F(","));
    // Serial.print(F(" "));
    // Serial.print(deg_to_nmea(lat, true));
    // Serial.print(F("/"));

    // Serial.print(deg_to_nmea(lon, false));
    // Serial.print(F(" "));
    Serial.print(gps.f_speed_kmph());
    Serial.print(F(","));
    now = millis();

    if (speed < LOW_SPEED) {
      beacon_rate = SLOW_RATE;
    }
    else if (speed > HIGH_SPEED) {
      beacon_rate = FAST_RATE;
    }
    else {
      beacon_rate = FAST_RATE * HIGH_SPEED / speed;
    }
    next_tx_time = last_tx_time + beacon_rate;

    Serial.print(now);
    Serial.print(F(","));
    Serial.print(gps.course()/100);

    turn_threshold = TURN_MIN + TURN_SLOPE / speed;
    course_change = course_change_since_beacon(gps.course()/100, last_course);
    if (course_change > turn_threshold
      && now >= last_tx_time + TURN_TIME
      && speed >= LOW_SPEED) {
      next_tx_time = now;
    }
    Serial.print(F(","));
    Serial.print(last_course);
    Serial.print(F(","));
    Serial.print(course_change);
    Serial.print(",");
    Serial.println(next_tx_time);

    last_course = gps.course()/100;

    if (now >= next_tx_time) {
      last_tx_time = now;
      Serial.println(F("APRS UPDATE"));
      locationUpdate();
  }

  }
}

void aprs_msg_callback(struct AX25Msg *msg) {
}

void locationUpdate() {
  char comment []= "Arduino APRS Tracker";

  APRS_setLat((char*)deg_to_nmea(lat, true));
  APRS_setLon((char*)deg_to_nmea(lon, false));
      
  // turn off SoftSerial to stop interrupting tx
  GPSSerial.end();
  
  // TX
  APRS_sendLoc(comment, strlen(comment));
 
  // read TX LED pin and wait till TX has finished (PB5) digital write 13 LED_BUILTIN
  while(bitRead(PORTB,5));

  // start SoftSerial again
  GPSSerial.begin(9600);
}

/*
**  Convert degrees in long format to APRS string format
**  DDMM.hhN for latitude and DDDMM.hhW for longitude
**  D is degrees, M is minutes and h is hundredths of minutes.
**  http://www.aprs.net/vm/DOS/PROTOCOL.HTM
*/
char* deg_to_nmea(long deg, boolean is_lat) {
  bool is_negative=0;
  if (deg < 0) is_negative=1;

  // Use the absolute number for calculation and update the buffer at the end
  deg = labs(deg);

  unsigned long b = (deg % 1000000UL) * 60UL;
  unsigned long a = (deg / 1000000UL) * 100UL + b / 1000000UL;
  b = (b % 1000000UL) / 10000UL;

  conv_buf[0] = '0';
  // in case latitude is a 3 digit number (degrees in long format)
  if( a > 9999) {
    snprintf(conv_buf , 6, "%04lu", a);
  } else {
    snprintf(conv_buf + 1, 5, "%04lu", a);
  }

  conv_buf[5] = '.';
  snprintf(conv_buf + 6, 3, "%02lu", b);
  conv_buf[9] = '\0';
  if (is_lat) {
    if (is_negative) {conv_buf[8]='S';}
    else conv_buf[8]='N';
    return conv_buf+1;
    // conv_buf +1 because we want to omit the leading zero
    }
  else {
    if (is_negative) {conv_buf[8]='W';}
    else conv_buf[8]='E';
    return conv_buf;
    }
}
