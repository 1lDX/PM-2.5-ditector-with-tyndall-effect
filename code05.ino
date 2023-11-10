#include <Wire.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LiquidCrystal_I2C.h>
#include <Button.h>

LiquidCrystal_I2C lcd(0x20, 16, 2);   // 0x3F , 0x27

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

unsigned long delayTime = 300;

#define MQ135_PIN A0

// กำหนดตัวแปรเก็บค่าที่อ่านได้จากเซ็นเซอร์
int SENSOR_VALUE;

int measurePin = A1; //Connect dust sensor to Arduino A0 pin
int ledPower = 7;   //Connect 3 led driver pins of dust sensor to Arduino D2

int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 15000;

float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

float temp_read = 0;
float humi_read = 0;


int laser_pin = 9;
Button button_mode(8); // Connect your button between pin 2 and GND
int mode=1;
int mode_old=0;

int count=0;
bool update = false;

int new_mode = 0;
int old_mode = 0;


ISR(TIMER1_COMPA_vect)
{  
    count++;
    if(count > 6)
       count = 0;

    update = true;
}

float get_distance_cm()
{
  // Setup the connection
  static byte WireWasBegun = false;
  if(WireWasBegun == false)
  {
    Wire.begin();
    WireWasBegun = true;
  }
  // We need to delay by at least 30ms between readings
  //  so we don't get a false reading
  static unsigned long lastread = 0;
  if(millis() - lastread < 30)
  {
    delay(millis()-lastread);  
  }
  lastread = millis();  
  // Send the trigger command 0x01 to the I2C address 0x57
  //  (the address can not be changed)
  Wire.beginTransmission(0x57);
  Wire.write(0x01);
  Wire.endTransmission();
 
  // Wait 150mS for the ping to complete
  delay(150);
 
  // Read 3 bytes from 0x57
  byte response[3];
  Wire.requestFrom(0x57,3);
  for(byte i = 0; Wire.available() && (i <= 2); i++)
  {
    response[i] = Wire.read();
  }  
  // Those three bytes get assembled like this to get the
  //  distance in micrometers
  float micrometers = ((response[0]*65536UL)+(response[1]*256UL)+response[2]);  
  // And now we can return that in centimeters
  //  um / 1000000 = m
  //  m * 100      = cm
  return micrometers / 1000000 * 100;
}

void printValues() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    // Serial.print("Pressure = ");

    // Serial.print(bme.readPressure() / 100.0F);
    // Serial.println(" hPa");

    // Serial.print("Approx. Altitude = ");
    // Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    // Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}

void clear_lcd(){
      if(new_mode != old_mode){
         lcd.clear();
         old_mode = new_mode;
      }   

}


void setup()
{
	 Serial.begin(9600);
   pinMode(ledPower,OUTPUT);
   pinMode(laser_pin,OUTPUT);
  // initialize the LCD
  button_mode.begin();
	lcd.begin(16,2);

	// Turn on the blacklight and print a message.
	lcd.backlight();
	
    unsigned status;
    
    // default settings
    // (you can also pass in a Wire library object like &Wire2)
    status = bme.begin();  

    TCCR1A = 0;            // undo the Arduino's timer configuration
    TCCR1B = 0;            // ditto
    TCNT1  = 0;            // reset timer
    OCR1A  = 62500 - 1;    // period = 62500 clock tics
    TCCR1B = _BV(WGM12)    // CTC mode, TOP = OCR1A
           | _BV(CS12);    // clock at F_CPU/256
    TIMSK1 = _BV(OCIE1A);  // interrupt on output compare A


}

void loop()
{ 

  if (button_mode.pressed()){
    mode++;
    if(mode == 2)
       count = 0;
    if(mode > 2)
      mode = 1;
    Serial.println("mode: "+ String(mode));   
  }  



  if(mode==1){
    new_mode = 1;
     if(update == true){
          clear_lcd();
          digitalWrite(laser_pin, HIGH);
          lcd.setCursor(0,0);    
          lcd.print("Dist: ");
          float distance = get_distance_cm(); 
               distance = distance / 100.0;
          if(distance < 100.0)
            lcd.print(" ");   
          if(distance < 10.0)
            lcd.print(" ");            

          lcd.print(distance + String("  M."));

          // Serial.print("Distance:");
          // Serial.print(distance);                    
          // Serial.println("cm");    
          // delay(150);
      update = false;
     }
       
  }
  if((mode==2)&&(count <= 3)){
    new_mode = 2;
     if(update == true){
          clear_lcd();
          digitalWrite(laser_pin, LOW);     
          
        //  BME280
        // printValues();         
          
          lcd.setCursor(0,0);  
          lcd.print("Temp: ");
          temp_read = bme.readTemperature();          
              if(temp_read < 100.0)
                lcd.print(" ");   
              if(temp_read < 10.0)
                lcd.print(" ");  
          lcd.print(temp_read);
          lcd.print("  "+ String((char)223)+"C.");
          Serial.println(temp_read); 



          lcd.setCursor(0,1);  
          lcd.print("Humi: ");
          humi_read = bme.readHumidity();
              if(humi_read < 100.0)
                lcd.print(" ");   
              if(humi_read < 10.0)
                lcd.print(" ");
          lcd.print(humi_read);
          lcd.print("  %");  
          Serial.println(humi_read); 
      update = false;
     } 

  }

  if((mode==2)&&(count > 3)){
     new_mode = 3;
      if(update == true){
            clear_lcd();
                // MQ-135
            SENSOR_VALUE = analogRead(MQ135_PIN);
            //  Serial.print("Air quality value : ");
            //  Serial.println(SENSOR_VALUE);

              lcd.setCursor(0,0);    
              lcd.print("  Air: ");
                  if(SENSOR_VALUE < 100)
                    lcd.print(" ");   
                  if(SENSOR_VALUE < 10)
                    lcd.print(" ");  
              lcd.print(SENSOR_VALUE);

            // PM2.5
              digitalWrite(ledPower,LOW); // power on the LED
              delayMicroseconds(samplingTime);
              Serial.println(analogRead(measurePin));
              voMeasured = analogRead(measurePin); // read the dust value

              delayMicroseconds(deltaTime);
              digitalWrite(ledPower,HIGH); // turn the LED off
              delayMicroseconds(sleepTime);

              // 0 - 5V mapped to 0 - 1023 integer values
              // recover voltage
              calcVoltage = voMeasured * (5.0 / 1024.0);

              // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
              // Chris Nafis (c) 2012
              dustDensity = calcVoltage * (170) - 0.1;
              if(dustDensity < 1.0)
                dustDensity = 1.0;
              
            Serial.println("dustDensity:" + String(dustDensity)); // unit: ug/m3
            Serial.println("");
              lcd.setCursor(0,1);    
              lcd.print("PM2.5: ");
                  if(dustDensity < 100.0)
                    lcd.print(" ");   
                  if(dustDensity < 10.0)
                    lcd.print(" ");          
              lcd.print(dustDensity);
        update = false;
      }  

  }

    delay(100);
    
}
