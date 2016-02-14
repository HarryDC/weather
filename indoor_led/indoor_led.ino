#include "LedControl.h"
#include <Adafruit_GFX.h>
#include <gfxfont.h>

/*
  Now we need a LedControl to work with.
 ***** These pin numbers will probably not work with your hardware *****
  pin 10 is connected to the DataIn
  pin 9 is connected to the CLK
  pin 8 is connected to LOAD
 ***** Please set the number of devices you have *****
  But the maximum default of 8 MAX72XX wil also work.
*/

/* we always wait a bit between updates of the display */
unsigned long delaytime = 500;

class LedGrafix : public Adafruit_GFX
{
  public:
    LedGrafix(int16_t dataIn, int16_t clk, int16_t load, int16_t num) :
      Adafruit_GFX(8 * num, 8),
      lc(dataIn, clk, load, num)
    {
      //we have already set the number of devices when we created the LedControl
      int devices = lc.getDeviceCount();
      //we have to init all devices in a loop
      for (int address = 0; address < devices; address++) {
        /*The MAX72XX is in power-saving mode on startup*/
        lc.shutdown(address, false);
        /* Set the brightness to a medium values */
        lc.setIntensity(address, 8);
        /* and clear the display */
        lc.clearDisplay(address);
      }
    }
    // For now orient horizontally
    // overridden from adafruit, ignore color parameter
    virtual void drawPixel(int16_t x, int16_t y, uint16_t c)
    {
      int16_t devices = lc.getDeviceCount();
      int16_t address = (devices - 1) - x / 8 ;
      int16_t realX = x % 8;
      lc.setLed(address, y, realX, c != 0);
    }

  private:
    LedControl lc;
};

LedGrafix gfx = LedGrafix(10, 9, 8, 4);

// initialize the dht senso
#include <DHT.h>
DHT dht(7, DHT22);

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// We'll use SoftwareSerial to communicate with the XBee:
#include <SoftwareSerial.h>
SoftwareSerial XBee(2, 3); // RX, TX

void setup() {
  Serial.begin(9600);
 
  /* Initialise the sensor */
  if (!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");
  Serial.println("------------------------------------");
  Serial.println("");

  gfx.setTextColor(1,0);

  delay(2000);
}

void readData(float* temp, float* humidity, float* pressure) {
  *temp = dht.readTemperature();
  *humidity = dht.readHumidity();

  // Process bmp
  sensors_event_t event;
  bmp.getEvent(&event);
  if (event.pressure)
  {
    *pressure = event.pressure;
  }  
}

void loop() {
  float humidity;
  float temp;
  float pressure;

  readData(&temp, &humidity, &pressure);

  String tempS(temp,1);
  String humS(humidity,1);
  String pressureS(pressure,0);

  String message = tempS + "C " + humS +"% " + pressureS + "mb ";

  int size = message.length() * 5;

  gfx.fillScreen(0);
  gfx.setCursor(0,0);
  gfx.print(tempS.c_str());
  delay(1000);
  
  gfx.fillScreen(0);
  gfx.setCursor(0,0);
  gfx.print(humS.c_str());
  delay(1000);
 
  gfx.fillScreen(0);
  gfx.setCursor(0,0);
  gfx.print(pressureS.c_str());
  delay(1000);
  
 /* 
  for (int i = 0; i < size + 32; ++i)
  {
    gfx.setCursor(-i,0);
    gfx.print(message.c_str());
    delay(50);    
  }
*/
  
  // Push Data
  message = tempS+"|"+humS+"|"+pressureS; 
  Serial.print(message);
  /*
  Serial.println(message.c_str());
  if (XBee.available())
  { // If data comes in from XBee, send it out to serial monitor
    Serial.write(XBee.read());
  }
  */
 
}

