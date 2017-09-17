/***************************************************
  This is an example for our Adafruit FONA Cellular Module
  Designed specifically to work with the Adafruit FONA
  ----> http://www.adafruit.com/products/1946
  ----> http://www.adafruit.com/products/1963
  ----> http://www.adafruit.com/products/2468
  ----> http://www.adafruit.com/products/2542
  These cellular modules use TTL Serial to communicate, 2 pins are
  required to interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!
  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

/*
THIS CODE IS STILL IN PROGRESS!
Open up the serial console on the Arduino at 115200 baud to interact with FONA
This code will receive an SMS, identify the sender's phone number, and automatically send a response
For use with FONA 800 & 808, not 3G
*/

#include "Adafruit_FONA.h"

#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4

// this is a large buffer for replies
char replybuffer[255];

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

char APN[] = "epc.tmobile.com"; //Set APN for Mobile Service
char response[255]; //global variable for pulling AT command responses from inside functions (there has to be a better way to do this)
unsigned long ATtimeOut = 10000; // How long we will give an AT command to comeplete
int SLEEP_MINUTES = 1; //Sleep time

// Hardware serial is also possible!
//  HardwareSerial *fonaSerial = &Serial1;

uint8_t type;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

unsigned long previousMillis = 0;        // will store last time
char fonaInBuffer[64];          //for notifications from the FONA

void setup() {
  while (!Serial);

  Serial.begin(115200);
  Serial.println(F("FONA SMS caller ID test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  // make it slow so its easy to read!
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while(1);
  }
  Serial.println(F("FONA is OK"));

  // Print SIM card IMEI number.
  char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("SIM card IMEI: "); Serial.println(imei);
  }

    //Prepare GPS
  Serial.println("Enabling GPS");
  if (!fona.enableGPS(true)) {
    Serial.println(F("Failed to turn on"));
  }
  int8_t smsnum = fona.getNumSMS();
  uint16_t smslen;
  int8_t smsn=0;
  
  Serial.println("Waiting for GPS");
  delay(30000);
  for ( ; smsn <= smsnum; smsn++) {
    if (fona.deleteSMS(smsn)) {
        Serial.println(F("OK!"));
      } else {
        Serial.println(F("Couldn't delete"));
      }
  }
  Serial.println("FONA Ready");

}

void loop() {
  //char api[]="&key=AIzaSyDF-is7rbgmAUV_AuW8nh_so1yFGJeCyGQ";
  char* bufPtr = fonaInBuffer;    //handy buffer pointer

  float latitude, longitude, speed_kph, heading, speed_mph, altitude, dest, mi, d;
  unsigned long currentMillis = millis();
  const long interval = 120000;           // interval at which to blink (milliseconds)
  /**Global GPS Data **/
  float dlong = -121.9503118;
  float dlat = 37.407171;

  dest = 1.60934;

  // Thing name
  String yourThing = "8g62gp";
  
  if (fona.available())      //any data available from the FONA?
  {
    int slot = 0;            //this will be the slot number of the SMS
    int charCount = 0;
    //Read the notification into fonaInBuffer
    do  {
      *bufPtr = fona.read();
      Serial.write(*bufPtr);
      delay(1);
    } while ((*bufPtr++ != '\n') && (fona.available()) && (++charCount < (sizeof(fonaInBuffer)-1)));
    
    //Add a terminal NULL to the notification string
    *bufPtr = 0;
    
    //Scan the notification string for an SMS received notification.
    //  If it's an SMS message, we'll get the slot number in 'slot'
    if (1 == sscanf(fonaInBuffer, "+CMTI: \"SM\",%d", &slot)) {
      Serial.print("slot: "); Serial.println(slot);
      
      char callerIDbuffer[32];  //we'll store the SMS sender number in here
      
      // Retrieve SMS sender address/phone number.
      if (! fona.getSMSSender(slot, callerIDbuffer, 31)) {
        Serial.println("Didn't find SMS message in slot!");
      }
      Serial.print(F("FROM: ")); Serial.println(callerIDbuffer);
      
      //Send back an automatic response
      boolean gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
  
      //Distance Calculation
      d = distance(latitude, longitude, dlat, dlong);
      mi = d/(1000*dest);

      char mylon[80];
      char mylat[80];
      char comb2[80];
      char str1[] = "Longitude: ";
      char str2[] = "Latitude: ";
      char myGoog[] = "http://maps.google.com/?q=@";
      //char myGoog[] = "https://maps.googleapis.com/maps/api/staticmap?size=400x400&path=color:0xff0000ff|weight:5|";
      char myURL[80];

      ftoa(mylon,longitude,100000000);
      ftoa(mylat,latitude,100000000);
      //ftoa(delong,dlong,100000000);
      //ftoa(delat,dlat,100000000);

      //String Preparation
      //sprintf(comb1,"%s %s",str1,mylon);
      //sprintf(comb2,"%s %s",str2,mylat);
      //sprintf(myURL,"%s%s,%s|%s,%s%s",myGoog,mylat,mylon,delat,delong,api);
      sprintf(myURL,"%s%s,%s",myGoog,mylat,mylon);

    if (currentMillis - previousMillis >= interval) {
      setupGPRS(); //Setup a GPRS context
      previousMillis = currentMillis;
      boolean gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
  
      //Distance Calculation
      d = distance(latitude, longitude, dlat, dlong);
      mi = d/(1000*dest);
      
      // Log
      Serial.print("Latitude: ");
      Serial.println(latitude);
      Serial.print("Longitude: ");
      Serial.println(longitude);
      Serial.print("Distance: ");
      Serial.println(mi);
      Serial.print("Time: ");
      Serial.println(currentMillis);
    
      // Prepare request
      uint16_t statuscode;
      int16_t length;
      String url = "http://dweet.io/dweet/for/";
      url += yourThing;
      url += "?longitude=";
      url += String(longitude);
      url += "&latitude=";
      url += String(latitude);
      url += "&distance=";
      url += String(mi);
      url += "&time=";
      url += String(currentMillis);
      char buf[80];
      url.toCharArray(buf, url.length());
    
      Serial.print("Request: ");
      Serial.println(buf);
      
      // Send location to Dweet.io
      if (!fona.HTTP_GET_start(buf, &statuscode, (uint16_t *)&length)) {
        Serial.println("Failed!");
      }
      while (length > 0) {
        while (fona.available()) {
          char c = fona.read();
               
          // Serial.write is too slow, we'll write directly to Serial register!
          #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
            loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
            UDR0 = c;
          #else
            Serial.write(c);
          #endif
          length--;
        }
      }
      fona.HTTP_GET_end();
    }
  }
}

char *ftoa(char *a, double f, int precision)
{
 long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
 
 char *ret = a;
 long heiltal = (long)f;
 itoa(heiltal, a, 10);
 while (*a != '\0') a++;
 *a++ = '.';
 long desimal = abs((long)((f - heiltal) * p[precision]));
 itoa(desimal, a, 10);
 return ret;
}

float distance(float flat1, float flon1, float flat2, float flon2)
{
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;
  
  //I've to spplit all the calculation in several steps. If i try to do it in a single line the arduino will explode.
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
  //Serial.println(dist_calc);
  return dist_calc;
}

void setupGPRS() { //all the commands to setup a GPRS context and get ready for HTTP command
    //the sendATCommand sends the command to the FONA and waits until the recieves a response before continueing on. 
    Serial.print("Disable echo: ");
    if(sendATCommand("ATE0")) { //disable local echo
        Serial.println(response);
    }
    Serial.print("Set to TEXT Mode: ");
    if(sendATCommand("AT+CMGF=1")){ //sets SMS mode to TEXT mode....This MIGHT not be needed. But it doesn't break anything with it there. 
        Serial.println(response);
    }
    Serial.print("Disable GPRS PDP Content...: ");
    if(sendATCommand("AT+CIPSHUT")){ //turns off GPRS PDP content 
        Serial.println(response);
    }
    Serial.print("Attach GPRS: ");
    if(sendATCommand("AT+CGATT=1")){ //Attach to GPRS service (1 - attach, 0 - disengage)
        Serial.println(response);
    }
    Serial.print("Set Connection Type To GPRS: "); //AT+SAPBR - Bearer settings for applications based on IP
    if(sendATCommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"")){ //3 - Set bearer perameters
        Serial.println(response);
    }
    Serial.print("Set APN: ");
    if(setAPN()) {
        Serial.println(response);
    }
    Serial.print("Start Task to Set APN...: ");
    if(sendATCommand("AT+CSTT=\"epc.tmobile.com\"")){ //Starts Task to Set APN 
        Serial.println(response);
    }
    if(sendATCommand("AT+SAPBR=1,1")) { //Open Bearer
        if(response == "OK") {
            Serial.println("Engaged GPRS");
        } else {
            Serial.println("GPRS Already on");
        }
    Serial.print("Bring Up Wireless Connection for GPRS...: ");
    if(sendATCommand("AT+CMGF=1")){ //Brings up wireless connection for 
        Serial.println(response);
        }    
    }
}

boolean sendATCommand(char Command[]) { //Send an AT command and wait for a response
    int complete = 0; // have we collected the whole response?
    char c; //capture serial stream
    char content[255]; //place to save serial stream
    unsigned long commandClock = millis(); //timeout Clock
    fonaSS.println(Command); //Print Command
    delay(500);
    while(!complete && commandClock <= millis() + ATtimeOut) { //wait until the command is complete
        while(!fonaSS.available() && commandClock <= millis()+ATtimeOut); //wait until the Serial Port is opened
        while(fonaSS.available()) { //Collect the response
            c = fonaSS.read(); //capture it
            if(c == 0x0A || c == 0x0D); //disregard all new lines and carrige returns (makes the String matching eaiser to do)
            else sprintf(content,"%s%s",content,c); //concatonate the stream into a String
            
        }
        //Serial.println(content); //Debug
        strncpy(response,content,255); //Save it out to a global Variable (How do you return a String from a Function?)
        complete = 1;  //Lable as Done.
    }
    if (complete ==1) return 1; //Is it done? return a 1
    else return 0; //otherwise don't (this will trigger if the command times out) 
    /*
        Note: This function may not work perfectly...but it works pretty well. I'm not totally sure how well the timeout function works. It'll be worth testing. 
        Another bug is that if you send a command that returns with two responses, an OK, and then something else, it will ignore the something else and just say DONE as soon as the first response happens. 
        For example, HTTPACTION=0, returns with an OK when it's intiialized, then a second response when the action is complete. OR HTTPREAD does the same. That is poorly handled here, hence all the delays up above. 
    */
}
boolean setAPN() { //Set the APN. See sendATCommand for full comments on flow
    int complete = 0;
    char c;
    char content[255];
    unsigned long commandClock = millis();                      // Start the timeout clock
    fonaSS.print("AT+SAPBR=3,1,\"APN\",\"");
    fonaSS.print(APN);
    fonaSS.print("\"");
    fonaSS.println();
    delay(500);
    while(!complete && commandClock <= millis() + ATtimeOut) {
        while(!fonaSS.available() && commandClock <= millis()+ATtimeOut);
        while(fonaSS.available()) {
            c = fonaSS.read();
            if(c == 0x0A || c == 0x0D);
            else sprintf(content,"%s%s",content,c);
        }
        strncpy(response,content,255);
        complete = 1; 
    }
    if (complete ==1) return 1;
    else return 0;
}

void flushFONA() { //if there is anything is the fonaSS serial Buffer, clear it out and print it in the Serial Monitor.
    char inChar;
    while (fonaSS.available()){
        inChar = fonaSS.read();
        Serial.write(inChar);
        delay(20);
    }
}

void sendData(float data) {
  if (client.connect(server,80)) {
   Serial.println("Connected to the server....");
   String jsonData = "{\"protocol\":\"v2\",\"device\":\""+DEVICE_ID+
                     "\",\"at\":\"now\",\"data\":{\"soil\":\""+
                     String(data)+"\"}}";  // Make a HTTP request
  client.println("POST /streams HTTP/1.1");
  client.println("Host: api.carriots.com");
  client.println("Accept: application/json");
  client.println("User-Agent: Arduino-Carriots");
  client.println("Content-Type: application/json");
  client.print("carriots.apikey: ");
  client.println(API_KEY);
  client.print("Content-Length: ");
  int thisLength = jsonData.length();
  client.println(thisLength);
  client.println("Connection: close");
  client.println();
  client.println(jsonData);
 }
}

