//setup for GSM/GPS module
#define TINY_GSM_MODEM_SIM7000
#define SerialMon Serial
#define SerialAT Serial1
#define TINY_GSM_DEBUG SerialMon
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#define TINY_GSM_RX_BUFFER 1024

// infromation from simcard given from network provider for each bike lock
const char apn[]      = "telenor.smart";// apn name from network provider
const char gprsUser[] = "information from telenor";
const char gprsPass[] = "information from telenor";

const char *broker = "name of server that would be used";
//folder lockations for where this database is located
const char *GPSTopic = "lockation of where the GPS data is saved";
const char *BatTopic = "Place where battery percentage is saved";

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <SD.h>
#include <Ticker.h>

//for if the module has been used before to make sure all settings are correct
//has to be run once and then doesn't need to be in the code anymore
#if TINY_GSM_USE_GPRS && not defined TINY_GSM_MODEM_HAS_GPRS
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS false
#define TINY_GSM_USE_WIFI true
#endif
#if TINY_GSM_USE_WIFI && not defined TINY_GSM_MODEM_HAS_WIFI
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#endif

//to make sure if a problem it is possible to debugg its self
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif
TinyGsmClient client(modem);
PubSubClient  mqtt(client);

//setup for GPS/GSM pin usage 
#define uS_TO_S_FACTOR      1000000ULL  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP       60         // this time is in seconds 

#define UART_BAUD           9600
#define PIN_DTR             33
#define PIN_TX              27
#define PIN_RX              26
#define PWR_PIN             4
#define BAT_ADC             36
#define GSM_PIN             39

#define SD_MISO             2
#define SD_MOSI             15
#define SD_SCLK             14
#define SD_CS               13
#define LED_PIN             12

// to constantly change the varibale of the battery integer with the correct value
const char* Bat_value;
RTC_DATA_ATTR int bootCount = 0;
float readBattery(uint8_t pin)
{
  int vref = 1100;
  uint16_t volt = analogRead(pin);
  float battery_voltage = ((float)volt / 4095.0) * 2.0 * 3.3 * (vref);
  return battery_voltage;
}

int counter, lastIndex, numberOfPieces = 24;
String pieces[24], input;

// startup of GPS, possibletie to startup and shut down
void enableGPS(void)
{
    // Set Modem GPS Power Control Pin to HIGH ,turn on GPS power
    modem.sendAT("+CGPIO=0,48,1,1");
    if (modem.waitResponse(10000L) != 1) {
        DBG("Set GPS Power HIGH Failed");
    }
    modem.enableGPS();
}

void disableGPS(void)
{
    // Set Modem GPS Power Control Pin to LOW ,turn off GPS power
    modem.sendAT("+CGPIO=0,48,1,0");
    if (modem.waitResponse(10000L) != 1) {
        DBG("Set GPS Power LOW Failed");
    }
    modem.disableGPS();
}

//make sure the connection module can turn on, of and restart
void modemPowerOn()
{
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, HIGH);
    delay(1000);    //Datasheet Ton mintues = 1S
    digitalWrite(PWR_PIN, LOW);
}

void modemPowerOff()
{
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, HIGH);
    delay(1500);    //Datasheet Ton mintues = 1.2S
    digitalWrite(PWR_PIN, LOW);
}

void modemRestart()
{
    modemPowerOff();
    delay(1000);
    modemPowerOn();
}

// to read and send GPS data trough GSM to database
void transCoordinates()
{
  while (lat <= 0 || lon <= 0)
  {
    modem.sendAT("+SGPIO=0,4,1,1");
    if (modem.waitResponse(10000L) != 1) {
      Serial.println(" SGPIO=0,4,1,1 false ");
    }
    modem.enableGPS();
    Serial.println("Requesting current GPS/GNSS/GLONASS location");
    if (modem.getGPS(&lat, &lon))
    {
      Serial.println("Latitude: " + String(lat, 8) + "\tLongitude: " + String(lon, 8));
    }
  }
  char *p = sendbuffer;
  // add speed value
  dtostrf(speed_mph, 2, 6, p);
  p += strlen(p);
  p[0] = ','; p++;

  // concat latitude
  dtostrf(lat, 2, 6, p);
  p += strlen(p);
  p[0] = ','; p++;

  // concat longitude
  dtostrf(lon, 3, 6, p);
  p += strlen(p);
  p[0] = ','; p++;

  // concat altitude
  dtostrf(altitude, 2, 6, p);
  p += strlen(p);

  // null terminate
  p[0] = 0;

  Serial.print("Sending: "); Serial.println(sendbuffer); // (Speed,Latitude,Longitude,Altitude)
  mqtt.publish(GPSTopic, sendbuffer);

  // for sending battery data to device
   float mv = readBattery(BAT_ADC);
  Serial.println(mv);

  String TEMP = (String)mv;
  Bat_value = (char*) TEMP.c_str();
  Serial.println(Bat_value);
  mqtt.publish(BatTopic, Bat_value);
}
//SMS
char incomingChar;

#include <SoftwareSerial.h>

//buzzer
int buzzer = 34;
bool shockmemorie = false;

//shock
int shock = 32;
int val;

//motor
int motoropen1 = 35;
int motorclose1 = 25;
bool moving = false;

//bluetooth
bool lockedorunlocked = false;
bool previouslyconnected = false;

void setup() {

 //buzzer
 pinMode(buzzer, OUTPUT);

 //Shock  
 pinMode (shock, INPUT); 

 //motor
 pinMode( motoropen1,OUTPUT);
 pinMode( motorclose1,OUTPUT);

 //bluetooth
 Serial.begin(9600);
 bluetooth.begin(9600);
 pinMode(bluetoothconnected, INPUT_PULLUP);

 //GPS and GSM
 // startup
 SerialMon.begin(115200);
 delay(10);
  
 pinMode(LED_PIN, OUTPUT);
 digitalWrite(LED_PIN, HIGH);

 pinMode(PWR_PIN, OUTPUT);
 digitalWrite(PWR_PIN, HIGH);

 delay(1000);
 digitalWrite(PWR_PIN, LOW);

 modemPowerOn();

 SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

 delay(10000);

 //making use if SD slot to use more memorie to calculate and communicate instead of saving the code
 SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS)) {
        Serial.println("SDCard MOUNT FAIL");
    } else {
        uint32_t cardSize = SD.cardSize() / (1024 * 1024);
        String str = "SDCard Size: " + String(cardSize) + "MB";
        Serial.println(str);
    }
 }
 
 //to read SMS from user
 //putting SIM7000G in SMS mode
 SIM7000G.begin(19200)
 SIM7000G.print("AT+CMGF=1\r"); 
  delay(100);


void loop() {
 
 //buzzer and Shock
 val = digitalRead (shock); 
	if (val == LOW ) { 
    shockmemorie = true;
    if(shockmemorie){
     digitalWrite(buzzer, HIGH); 
     modem.sendSMS(thelephoneNumber(s) of connected consumer, Heavy shcok detected, check on bike! )
    }
	} 
	   
 //motor
 if (bluetooth.available()) {
    char receivedChar = bluetooth.read();
    Serial.println(receivedChar);

    
    if (receivedChar == '1') {
      digitalWrite(motoropen1, HIGH);
      digitalWrite(motorclose1, LOW);
      bluetooth.println("locking bike");
      moving = true;
      delay(2000);
      digitalWrite(motoropen1, LOW);
      lockedorunlocked = true;
      if (moving){
        int sensorValue = analogRead(A0);
  Serial.println(sensorValue);
      if (sensorValue > 1020) {
       digitalWrite(motoropen1, LOW);;
       digitalWrite(motorclose1, HIGH);
       bluetooth.println("Locking failed. Remove blockage and lock again");
  }
  }
      moving = false;
    } else if (receivedChar == '0') {
      digitalWrite(motoropen1, LOW);
      digitalWrite(motorclose1, LOW);
      bluetooth.println("LED is OFF");
    } else if (receivedChar == '2') {
       digitalWrite(motoropen1, LOW);
      digitalWrite(motorclose1, HIGH);
      bluetooth.println("opening bike");
      moving = true;
      delay(2000);
      digitalWrite(motorclose1, LOW);
      lockedorunlocked = false;
      if (moving){
        int sensorValue = analogRead(A0);
        Serial.println(sensorValue);
        if (sensorValue > 100) {
        digitalWrite(motoropen1, LOW);
        digitalWrite(motorclose1, HIGH);
        bluetooth.println("Locking failed. Remove blockage and lock again");
  }
  }
      moving = false;
    } 
    if (receivedChar == '3'){
      shockmemorie = false;
      digitalWrite(buzzer, LOW);
      bluetooth.println("Alarm Muted");
    }
  }

  //SMS alarm off
   if (SMSRequest()){
    if(readData()){
      delay(10);
      if (incomingChar == 'S'){
      shockmemorie = false;
      digitalWrite(buzzer, LOW);
      modem.sendSMS(thelephoneNumber(s) of connected consumer, Alarm Silenced. )
      }
  
 
  int outputstate = digitalRead(bluetoothconnected);
  if (outputstate == HIGH) {
    delay(1000);
    if (lockedorunlocked && !previouslyconnected) {
      bluetooth.println("Bike is locked");
      previouslyconnected = true;
    } else if (!lockedorunlocked && !previouslyconnected){
      bluetooth.println("Bike is unlocked");
      previouslyconnected = true;
    } 
  } else {
   previouslyconnected = false;
  }

 //GPS
  if (!modem.testAT()) {
        Serial.println("Failed to restart modem, attempting to continue without restarting");
        modemRestart();
        return;
    }

    enableGPS();
    
    //latitude and longitude getting send to database
    float lat,  lon;
    while (1) {
        if (modem.getGPS(&lat, &lon)) {
            Serial.print("latitude:"); Serial.println(lat);
            Serial.print("longitude:"); Serial.println(lon);
            break;
        }
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(2000);
    }

    disableGPS();

    while (1) {
        while (SerialAT.available()) {
            SerialMon.write(SerialAT.read());
        }
        while (SerialMon.available()) {
            SerialAT.write(SerialMon.read());
        }
    }
  //GSM
   // connecting specific modem to server database
    String name = modem.getModemName();
    delay(500);
    Serial.println("Modem Name: " + name);

    String modemInfo = modem.getModemInfo();
    delay(500);
    Serial.println("Modem Info: " + modemInfo);  

   //unlocking SIM card
   #if TINY_GSM_TEST_GPRS
    // Unlock your SIM card with a PIN 
    if ( GSM_PIN && modem.getSimStatus() != 3 ) {
        modem.simUnlock(GSM_PIN);
    }
   #endif

    modem.sendAT("+CFUN=0");
    if (modem.waitResponse(10000L) != 1) {
        DBG(" +CFUN=0  false ");
    }
    delay(200);

   //setting modem to networkmode
    bool res = modem.setNetworkMode(2);
    if (!res) {
        DBG("setNetworkMode  false ");
        return ;
    }
    delay(200);

   // setting networkmode as main mode of the modem
    res = modem.setPreferredMode(3);
    if (!res) {
        DBG("setPreferredMode  false ");
        return ;
    }
    delay(200);

    modem.sendAT("+CFUN=1");
    if (modem.waitResponse(10000L) != 1) {
        DBG(" +CFUN=1  false ");
    }
    delay(200);    
 //GSM 
#if TINY_GSM_TEST_GPRS

    SerialAT.println("AT+CGDCONT?");
    delay(500);
    if (SerialAT.available()) {
        input = SerialAT.readString();
        for (int i = 0; i < input.length(); i++) {
            if (input.substring(i, i + 1) == "\n") {
                pieces[counter] = input.substring(lastIndex, i);
                lastIndex = i + 1;
                counter++;
            }
            if (i == input.length() - 1) {
                pieces[counter] = input.substring(lastIndex, i);
            }
        }
        // Reset for reuse
        input = "";
        counter = 0;
        lastIndex = 0;

        for ( int y = 0; y < numberOfPieces; y++) {
            for ( int x = 0; x < pieces[y].length(); x++) {
                char c = pieces[y][x];  //gets one byte from buffer
                if (c == ',') {
                    if (input.indexOf(": ") >= 0) {
                        String data = input.substring((input.indexOf(": ") + 1));
                        if ( data.toInt() > 0 && data.toInt() < 25) {
                            modem.sendAT("+CGDCONT=" + String(data.toInt()) + ",\"IP\",\"" + String(apn) + "\",\"0.0.0.0\",0,0,0,0");
                        }
                        input = "";
                        break;
                    }
                    // Reset for reuse
                    input = "";
                } else {
                    input += c;
                }
            }
        }
    } else {
        Serial.println("Failed to get PDP!");
    }


    Serial.println("\n\n\nWaiting for network...");
    if (!modem.waitForNetwork()) {
        delay(10000);
        return;
    }

    if (modem.isNetworkConnected()) {
        Serial.println("Network connected");
    }

    Serial.println("\n---Starting GPRS TEST---\n");
    Serial.println("Connecting to: " + String(apn));
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        delay(10000);
        return;
    }

    Serial.print("GPRS status: ");
    if (modem.isGprsConnected()) {
        Serial.println("connected");
    } else {
        Serial.println("not connected");
    }

    String ccid = modem.getSimCCID();
    Serial.println("CCID: " + ccid);

    String imei = modem.getIMEI();
    Serial.println("IMEI: " + imei);

    String cop = modem.getOperator();
    Serial.println("Operator: " + cop);

    IPAddress local = modem.localIP();
    Serial.println("Local IP: " + String(local));

    int csq = modem.getSignalQuality();
    Serial.println("Signal quality: " + String(csq));

    SerialAT.println("AT+CPSI?");     //Get connection type and band
    delay(500);
    if (SerialAT.available()) {
        String r = SerialAT.readString();
        Serial.println(r);
    }

    Serial.println("\n---End of GPRS TEST---\n");
#endif

#if TINY_GSM_TEST_GPRS
    modem.gprsDisconnect();
    if (!modem.isGprsConnected()) {
        Serial.println("GPRS disconnected");
    } else {
        Serial.println("GPRS disconnect: Failed.");
    }
#endif
 // for when GSM needs to powerd down
#if TINY_GSM_POWERDOWN
    
    modem.sendAT("+CPOWD=1");
    if (modem.waitResponse(10000L) != 1) {
        DBG("+CPOWD=1");
    }
    modem.poweroff();
    Serial.println("Poweroff.");
#endif

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    delay(200);
    esp_deep_sleep_start();
}
