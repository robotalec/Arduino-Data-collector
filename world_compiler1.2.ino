
#include <Adafruit_BMP085.h>
#include <dht.h>
//#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>
File myFile;
dht DHT;
#define DHT11_PIN 3
 int buttonPin = 2;
TinyGPSPlus gps;
// Choose two Arduino pins to use for software serial
//const int RXPin = 5;
//const int TXPin = 4;
//SoftwareSerial gpsSerial(RXPin, TXPin);
//Default baud of NEO-6M is 9600
 int GPSBaud = 9600;
bool gpsready = false;
float sensor = A0;
float gas_value;
float sensor2 = A1;
float gas_value2;
Adafruit_BMP085 bmp;
bool button_pushed = false;
void setup() {

  Serial.begin(115200);
  // Define pin #12 as input and activate the internal pull-up resistor
  pinMode(buttonPin, INPUT_PULLUP);
  // Define pin #13 as output, for the LED
  Serial1.begin(GPSBaud);
  pinMode(sensor2, INPUT);
  pinMode(sensor, INPUT);
  Serial.println("Running...");


}

void loop() {

  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial1.available() > 0)
    if (gps.encode(Serial1.read())) {
      if (gps.location.isValid() && gpsready == false) {
        Serial.println("GPS AQuired");
        gpsready = true;
      }
      //displayInfo();
    }
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    gpsready = false;
    Serial.println("No GPS detected");
    while (true);
  }



  // Read the value of the input. It can either be 1 or 0
  int buttonValue = digitalRead(buttonPin);

  if (buttonValue == LOW && gpsready == true) {
    //if(button_pushed == false){
    //displayInfo();
    int chk = DHT.read11(DHT11_PIN);
    Serial.print("Temperature = ");
    Serial.println(DHT.temperature);
    Serial.print("Humidity = ");
    Serial.println(DHT.humidity);


    gas_value2 = analogRead(sensor2);
    gas_value = analogRead(sensor);
    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" meters");
    Serial.print("Real altitude = ");
    Serial.print(bmp.readAltitude(101500));
    Serial.println(" meters");

    Serial.print(gas_value2);
    Serial.println(" ppm C0");
    Serial.print(gas_value);
    Serial.println(" ppm LPG/CH4");
          Serial.print("Latitude: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 6);
      Serial.print("Altitude: ");
      Serial.println(gps.altitude.meters());
     delay(250);
    


    delay(500);
    savedata();

  }
}

void savedata(){
    Serial.println("Initializing SD card...");
  if (!SD.begin(53)) {
    Serial.println("initialization failed!");
  }
SD.begin(53);
if (gps.location.isValid()) {
      myFile = SD.open("readings.txt", FILE_WRITE);
      Serial.println("Writing...");
      int chk = DHT.read11(DHT11_PIN);
      myFile.print("Temperature = ");
      myFile.println(DHT.temperature);
      myFile.print("Humidity = ");
      myFile.println(DHT.humidity);


      gas_value2 = analogRead(sensor2);
      gas_value = analogRead(sensor);
      myFile.print("Pressure = ");
      myFile.println(bmp.readPressure());
      myFile.println(" Pa");
      myFile.print("Altitude = ");
      myFile.println(bmp.readAltitude());
      myFile.println(" meters");
      myFile.print("Real altitude = ");
      myFile.println(bmp.readAltitude(101500));
      myFile.println(" meters");

      myFile.println(gas_value2);
      myFile.println(" ppm C0");
      myFile.println(gas_value);
      myFile.println(" ppm LPG/CH4");



      myFile.print("Latitude: ");
      myFile.println(gps.location.lat(), 6);
      myFile.print("Longitude: ");
      myFile.println(gps.location.lng(), 6);
      myFile.print("Altitude: ");
      myFile.println(gps.altitude.meters());
      myFile.print(gps.date.month());
      myFile.print("/");
      myFile.print(gps.date.day());
      myFile.print("/");
      myFile.println(gps.date.year());
      if (gps.time.hour() < 10) myFile.print(F("0"));
      myFile.print(gps.time.hour());
      myFile.print(":");
      if (gps.time.minute() < 10) myFile.print(F("0"));
      myFile.print(gps.time.minute());
      myFile.print(":");
      if (gps.time.second() < 10) myFile.print(F("0"));
      myFile.print(gps.time.second());
      myFile.print(".");
      if (gps.time.centisecond() < 10) myFile.print(F("0"));
      myFile.println(gps.time.centisecond());

      myFile.close();
    }

}
