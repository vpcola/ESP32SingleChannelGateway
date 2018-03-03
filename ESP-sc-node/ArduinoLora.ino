#include <CayenneLPP.h>
#include <DHTesp.h>
#include <Adafruit_GPS.h>
#include <HardwareSerial.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include "SSD1306.h"


// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0xD6, 0x6D, 0xC7, 0x16, 0xE7, 0x90, 0xB8, 0x39, 0x8A, 0x6A, 0xE9, 0xA4, 0x15, 0x48, 0x21, 0x69 };
// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0x29, 0x0E, 0x5C, 0xF7, 0xA1, 0x41, 0x3C, 0x5A, 0x8B, 0xF9, 0x12, 0x96, 0xE3, 0xBA, 0x54, 0x40 };
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = { 0x26041938 };
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;
// Schedule data trasmission in every this many seconds (might become longer due to duty
// cycle limitations).
// we set 10 seconds interval
const unsigned TX_INTERVAL = 10; // Fair Use policy of TTN requires update interval of at least several min. We set update interval here of 1 min for testing

// Pin mapping according to Cytron LoRa Shield RFM
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32},
};

/** Pin number for DHT11 data pin */
int dhtPin = 17;

// Adafruit GPS
#define SERIAL1_RXPIN 12
#define SERIAL1_TXPIN 13

HardwareSerial gpsSerial(2);  // use ESP32's 2nd serial port (default serial 1 is for debug)
Adafruit_GPS GPS(&gpsSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
//#define GPSECHO  true


CayenneLPP lpp(15);
DHTesp dht;
SSD1306  display(0x3c, 4, 15);


void onEvent (ev_t ev) 
{
  Serial.print(os_getTime());
  Serial.print(": ");
  switch(ev) 
  {
    case EV_TXCOMPLETE:
      Serial.printf("EV_TXCOMPLETE (includes waiting for RX windows)\r\n");
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
      break;  
    case EV_RXCOMPLETE:
      if (LMIC.dataLen)
      {
        Serial.printf("Received %d bytes\n", LMIC.dataLen);
      }
      break;
    default:
      Serial.printf("Unknown event\r\n");
      break;
  }
}

void do_send(osjob_t* j)
{
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) 
  {
    Serial.printf("OP_TXRXPEND, not sending\r\n");
  } 
  else
  if (!(LMIC.opmode & OP_TXRXPEND)) 
  {
    TempAndHumidity newValues = dht.getTempAndHumidity();
    
    lpp.reset();
    lpp.addTemperature(1, newValues.temperature);
    lpp.addRelativeHumidity(2, newValues.humidity);
    
    Serial.printf("Temperature : %.2f, Humidity : %.2f\r\n", newValues.temperature, newValues.humidity);

    display.drawString(0, 0, "Temperature: ");
    display.drawString(90, 0, String(newValues.temperature));
    display.drawString(0, 20, "Humidity  : ");
    display.drawString(90,20, String(newValues.humidity));
    display.display();    

#if 0    
    // Now for GPS
    if (GPS.fix)
    {
      float lat = GPS.latitudeDegrees;
      float lon = GPS.longitudeDegrees;
      float alt = GPS.geoidheight;
      Serial.printf("GPS lat = %.2f, lon = %.2f, alt = %.2f\n", lat, lon, alt);
      lpp.addGPS(3, lat, lon, alt);
    }
#endif
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
         
    Serial.printf("Packet queued\r\n");
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() 
{
  Serial.begin(115200);
  gpsSerial.begin(115200, SERIAL_8N1, SERIAL1_RXPIN, SERIAL1_TXPIN);
  Serial.printf("Starting...\r\n");


  pinMode(16,OUTPUT);
  digitalWrite(16, LOW); // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH);
  display.init();
  display.setFont(ArialMT_Plain_10);
  
  dht.setup(dhtPin, DHTesp::DHT11);
  
#if 0
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
#endif
  
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
  // Select frequencies range
  LMIC_selectSubBand(0);
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;
  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14);
  Serial.printf("LMIC setup done!\r\n");
  // Start job
  do_send(&sendjob);
}

void loop() 
{
#if 0
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  //if ((c) && (GPSECHO))
  //  Serial.write(c);
     
   // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) 
  {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      ; //return;  // we can fail to parse a sentence in which case we should just wait for another
  } 
#endif    
  // Make sure LMIC is ran too
  os_runloop_once();
}
