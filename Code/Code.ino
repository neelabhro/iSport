
#include <NMEAGPS.h>
#include <SPI.h>
#include <SdFat.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#include "GPSport.h"

#ifdef NeoHWSerial_h
#define DEBUG_PORT NeoSerial
#else
#define DEBUG_PORT Serial
#endif

NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This contains all the GPS fields

//#define gps_port Serial1
//static NMEAGPS  gps; // This parses the GPS characters


SdFat SD;
File  dataFile;
char filename[24];

// date 1-Oct-14
uint16_t year = 2014;
uint8_t month = 10;
uint8_t day = 1;

// time 20:30:40
uint8_t hour = 20;
uint8_t minute = 30;
uint8_t second = 40;


long buttonTimer = 0;
long longPressTime = 3000;

boolean buttonActive = false;
boolean longPressActive = false;
int button = 4;



void dateTime(uint16_t* date, uint16_t* time) {

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(year, month, day);

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(hour, minute, second);
}

float total, gtotal;

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
#define PROCESSING_VISUALIZER 1
#define SERIAL_PLOTTER  2

//  Variables
int pulsePin = 0;                 // Pulse Sensor purple wire connected to analog pin 0
int blinkPin = 13;                // pin to blink led at each beat
int fadePin = 5;                  // pin to do fancy classy fading blink at each beat
int fadeRate = 0;                 // used to fade LED on with PWM on fadePin

// Volatile Variables, used in the interrupt service routine!
volatile int BPM;                   // int that holds raw Analog in 0. updated every 2mS
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // int that holds the time interval between beats! Must be seeded!
volatile boolean Pulse = false;     // "True" when User's live heartbeat is detected. "False" when not a "live beat".
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.

// SET THE SERIAL OUTPUT TYPE TO YOUR NEEDS
// PROCESSING_VISUALIZER works with Pulse Sensor Processing Visualizer
//      https://github.com/WorldFamousElectronics/PulseSensor_Amped_Processing_Visualizer
// SERIAL_PLOTTER outputs sensor data for viewing with the Arduino Serial Plotter
//      run the Serial Plotter at 115200 baud: Tools/Serial Plotter or Command+L
static int outputType = SERIAL_PLOTTER;

void setup()  {
attachInterrupt(digitalPinToInterrupt(4), digitalInterrupt,FALLING);
#ifndef ESP8266
  while (!Serial); // for Leonardo/Micro/Zero
#endif
  Serial.begin(9600);
  Serial.println("Accelerometer Test"); Serial.println("");

  /* Initialise the sensor */
  if (!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while (1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);
  // displaySetRange(ADXL345_RANGE_8_G);
  // displaySetRange(ADXL345_RANGE_4_G);
  // displaySetRange(ADXL345_RANGE_2_G);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Display additional settings (outside the scope of sensor_t) */
  displayDataRate();
  displayRange();
  //Serial.println("");

  Serial.begin(9600);
  Serial.flush();

  gps_port.begin( 9600 );
  gps.send_P( &gps_port, F("PMTK251,115200") );  // set baud rate
  gps_port.flush();                              // wait for the command to go out
  delay( 100 );                                  // wait for the GPS device to change speeds
  gps_port.end();                                // empty the input buffer, too

  gps_port.begin( 115200 );                      // use the new baud rate
  gps.send_P( &gps_port, F("PMTK220,100") );     // set 10Hz update rate
  delay(50);
  gps.send_P( &gps_port, F("PMTK886,1") );       // Setting FR to Fitness mode
  delay(50);
  //DEBUG_PORT.begin(9600);
  while (!DEBUG_PORT)
    ;

  DEBUG_PORT.println( sizeof(gps.fix()) );
  DEBUG_PORT.println( sizeof(gps) );

  DEBUG_PORT.flush();
  pinMode(37, INPUT_PULLUP);

  if (!SD.begin(4)) {
    Serial.println( F("Card failed, or not present") );
    while (1);
    // don't do anything more:
    return;

  }
  Serial.println( F("card initialized.") );
  Serial.println( F("Code Running") );
  SdFile::dateTimeCallback(dateTime); // register callback for file create/modification date/time
  pinMode(blinkPin, OUTPUT);        // pin that will blink to your heartbeat!
  pinMode(fadePin, OUTPUT);         // pin that will fade to your heartbeat!
  interruptSetup();
  
} // setup
void(* resetFunc)(void)=0;
void firstStage()
{
  Serial.println( F("Co-ordinates Achieved") );
  sprintf(filename, "%02d-%02d-%02d-T-%02d-%02d-%02d.xml", fix.dateTime.hours , fix.dateTime.minutes, fix.dateTime.seconds, fix.dateTime.date , fix.dateTime.month , fix.dateTime.full_year());
  updateDateTime();
  dataFile = SD.open(filename , FILE_WRITE);
  Serial.println(filename);

  if (dataFile) {
    Serial.println( F("File created\n"
                      "LOGGING DATA!\n") );

//    dataFile.println( F("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
//                        "<TrainingCenterDatabase xmlns=\""
//                        "http://www.iSportsAnalysis.com/"
//                        "xmlschemas/TrainingCenterDatabase/v1">"
//                        "<Activities>\n"
//                        "<Activity Sport=\"Other\">") );
    dataFile.println( F("<Id>") );

    timing();

    dataFile.println( F("0Z"
                        "</Id>\n"
                        "<Lap StartTime=\"") );
    timing();
    dataFile.println( F("0Z\">"
                        "<Track>") );
    dataFile.println( F("<InputDevice>iHawk</InputDevice>"));                    
  }
} // firstStage

void secondStage()
{

  if (dataFile.isOpen()) {
    dataFile.println( F("<Trackpoint>\n"
                        "<Time>") );
    timing();

    dataFile.print  ( F("0Z"
                        "</Time>"
                        "<Position>\n"
                        "<LatitudeDegrees>") );
    printL( dataFile, fix.latitudeL() );
    dataFile.print  ( F("</LatitudeDegrees>\n"
                        "<LongitudeDegrees>") );
    printL( dataFile, fix.longitudeL() );
    dataFile.print  ( F("</LongitudeDegrees>\n"
                        "</Position>\n"
                        "<HeartRateBpm>\n"
                        "<Value>") );
    dataFile.print  ( BPM );
    dataFile.print  ( F("</Value>\n"
                        "</HeartRateBpm>\n"
                        "<SensorState>"
                        "Present"
                        "</SensorState>\n") );



    sensors_event_t event;
    accel.getEvent(&event);


    total = sqrt(event.acceleration.x * event.acceleration.x + event.acceleration.y * event.acceleration.y + event.acceleration.z * event.acceleration.z);
    gtotal = total / 9.8;

    dataFile.print  ( F("<Impact>"
                        "<Gforce>") );
    dataFile.print  ( gtotal );
    dataFile.print  ( F("</Gforce>\n"
                        "<Xaxis>") );
    dataFile.print  ( event.acceleration.x / 9.8 );
    dataFile.print  ( F("</Xaxis>\n"
                        "<Yaxis>") );
    dataFile.print  ( event.acceleration.y / 9.8 );
    dataFile.print  ( F("</Yaxis>\n"
                        "<Zaxis>") );
    dataFile.print  ( event.acceleration.z / 9.8 );
    dataFile.println( F("</Zaxis>\n"
                        "</Impact>\n"
                        "</Trackpoint>") );
  }

// Implementing the LongPress
  
  if (digitalRead(button) == LOW && buttonActive == false) {

      buttonActive = true;
      buttonTimer = millis();
      if ((millis() - buttonTimer < longPressTime) && (longPressActive == false)) {
        endtask();
      }
      else if ((millis() - buttonTimer > longPressTime) && (longPressActive == false)) {
        deepSleeep();
      }
    }
buttonActive = false;
longPressActive = false;

} // secondStage

void doSomeWork() {

  if (fix.valid.date && fix.valid.time && fix.valid.location)
  {
    if (!dataFile.isOpen())
      firstStage();

    if (dataFile.isOpen())
      secondStage();
  }
  else {
    // No valid location data yet!
    Serial.print  ( '?' );
  }

  Serial.println();

} // doSomeWork

void GPSloop()
{
  while (gps.available( gps_port) ) {
    fix = gps.read();
    doSomeWork();
  }

} // GPSloop

void loop()
{
  GPSloop();

  // Testing the button could go almost anywhere.
  //    Putting it here will close dataFile the fastest,
  //    because it won't have to wait for the next
  //    GPS update.
  if (digitalRead(37) == LOW) {
    endtask();

    // stop here!
    while (1);

  }

} // loop

void endtask()
{
  if (dataFile.isOpen()) {
    dataFile.println( F("</Track>\n"
                        "</Lap>\n"
                        "</Activity>\n"
                        "</Activities>\n"
                        "</TrainingCenterDatabase>") );
    dataFile.close();
    Serial.println( F("file closed") );
  }
  while(1){
  if (digitalRead(button) == LOW){
    resetFunc();
  }
  }
} // endtask
void timing () {
  if ( fix.dateTime.full_year() < 10 )
    dataFile.print( '0' );
  dataFile.print  ( fix.dateTime.full_year() );
  dataFile.print  ( '-' );
  if ( fix.dateTime.month < 10 )
    dataFile.print( '0' );
  dataFile.print  ( fix.dateTime.month );
  dataFile.print  ( '-' );
  if ( fix.dateTime.date  < 10 )
    dataFile.print( '0' );
  dataFile.print  ( fix.dateTime.date );
  dataFile.print  ( 'T' );
  if ( fix.dateTime.hours < 10 )
    dataFile.print( '0' );
  dataFile.print  ( fix.dateTime.hours );
  dataFile.print  ( ':' );
  if ( fix.dateTime.minutes < 10 )
    dataFile.print( '0' );
  dataFile.print  ( fix.dateTime.minutes );
  dataFile.print  ( ':' );
  if ( fix.dateTime.seconds < 10 )
    dataFile.print( '0' );
  dataFile.print  ( fix.dateTime.seconds );
  dataFile.print  ( '.' );
  if ( fix.dateTime_cs < 10 )
    dataFile.print( '0' );
  dataFile.print  ( fix.dateTime_cs );
}
void updateDateTime() {
  if (fix.valid.date) {
    year = fix.dateTime.full_year();
    month = fix.dateTime.month;
    day = fix.dateTime.date;
  }

  if (fix.valid.time) {
    second = fix.dateTime.seconds ;
    minute = fix.dateTime.minutes;
    hour = fix.dateTime.hours;
  }
}
static void printL( Print & outs, int32_t degE7 )
{
  // Extract and print negative sign
  if (degE7 < 0) {
    degE7 = -degE7;
    outs.print( '-' );
  }

  // Whole degrees
  int32_t deg = degE7 / 10000000L;
  outs.print( deg );
  outs.print( '.' );

  // Get fractional degrees
  degE7 -= deg * 10000000L;

  // Print leading zeroes, if needed
  int32_t factor = 1000000L;
  while ((degE7 < factor) && (factor > 1L)) {
    outs.print( '0' );
    factor /= 10L;
  }

  // Print fractional degrees
  outs.print( degE7 );
}
void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(100);
}
void displayDataRate(void)
{
  Serial.print  ("Data Rate:    ");

  switch (accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      Serial.print  ("3200 ");
      break;
    case ADXL345_DATARATE_1600_HZ:
      Serial.print  ("1600 ");
      break;
    case ADXL345_DATARATE_800_HZ:
      Serial.print  ("800 ");
      break;
    case ADXL345_DATARATE_400_HZ:
      Serial.print  ("400 ");
      break;
    case ADXL345_DATARATE_200_HZ:
      Serial.print  ("200 ");
      break;
    case ADXL345_DATARATE_100_HZ:
      Serial.print  ("100 ");
      break;
    case ADXL345_DATARATE_50_HZ:
      Serial.print  ("50 ");
      break;
    case ADXL345_DATARATE_25_HZ:
      Serial.print  ("25 ");
      break;
    case ADXL345_DATARATE_12_5_HZ:
      Serial.print  ("12.5 ");
      break;
    case ADXL345_DATARATE_6_25HZ:
      Serial.print  ("6.25 ");
      break;
    case ADXL345_DATARATE_3_13_HZ:
      Serial.print  ("3.13 ");
      break;
    case ADXL345_DATARATE_1_56_HZ:
      Serial.print  ("1.56 ");
      break;
    case ADXL345_DATARATE_0_78_HZ:
      Serial.print  ("0.78 ");
      break;
    case ADXL345_DATARATE_0_39_HZ:
      Serial.print  ("0.39 ");
      break;
    case ADXL345_DATARATE_0_20_HZ:
      Serial.print  ("0.20 ");
      break;
    case ADXL345_DATARATE_0_10_HZ:
      Serial.print  ("0.10 ");
      break;
    default:
      Serial.print  ("???? ");
      break;
  }
  Serial.println(" Hz");
} void displayRange(void)
{
  Serial.print  ("Range:         +/- ");

  switch (accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      Serial.print  ("16 ");
      break;
    case ADXL345_RANGE_8_G:
      Serial.print  ("8 ");
      break;
    case ADXL345_RANGE_4_G:
      Serial.print  ("4 ");
      break;
    case ADXL345_RANGE_2_G:
      Serial.print  ("2 ");
      break;
    default:
      Serial.print  ("?? ");
      break;
  }
  Serial.println(" g");
}

void deepSleeep()

{

longPressActive = true;
//Disable ADC
ADCSRA &= ~(1<<7);
//Enable Sleep
SMCR |= (1<<2);        //Power down mode
SMCR |= 1;          // Enable sleep

//BOD DISABLE
MCUCR != (3 << 5);      //Sets both BODSE & BODS at the same time
MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6);   
__asm__ __volatile("sleep");



}

void digitalInterrupt(){
  }
