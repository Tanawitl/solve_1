//#include "FirebaseESP8266.h"
//#include <ESP8266WiFi.h>

#if defined(ESP32)
#include <WiFi.h>
#include <FirebaseESP32.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#endif

#include <SPI.h>
#include <TFT_eSPI.h> // Hardware-specific library

#include <time.h>
//#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include "MAX30105.h" //sparkfun MAX3010X library
#include "heartRate.h"

#include <Wire.h>
#include <VL53L0X.h>

String device = "device001";
#define SSID        "BANK"
#define PASSWORD    "10086010"

#define FIREBASE_HOST "esp1-146ec-default-rtdb.firebaseio.com" 
#define FIREBASE_AUTH "TUnpCwP8iv2XcKfDIvW8Rnf3eIY9S3MPNrvdbDjT"

TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

//#define I2C_SPEED_STANDARD        1000
MAX30105 particleSensor;

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

VL53L0X sensor;

double avered = 0; double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int i = 0;
int Num = 50;//calculate SpO2 by this sampling interval 100

double ESpO2 = 95.0;//initial value of estimated SpO2
double FSpO2 = 0.7; //filter factor for estimated SpO2
double frate = 0.95; //low pass filter for IR/red LED value to eliminate AC component
#define TIMETOBOOT 3000 // wait for this time(msec) to output SpO2
#define SCALE 88.0 //adjust to display heart beat and SpO2 in the same scale
#define SAMPLING 5 //if you want to see heart beat more precisely , set SAMPLING to 1
#define FINGER_ON 3000 // if red signal is lower than this , it indicates your finger is not on the sensor
#define MINIMUM_SPO2 80.0

TaskHandle_t Task0;
//TaskHandle_t Task1;
//TaskHandle_t Task2;
//TaskHandle_t Task3;

const byte RATE_SIZE = 5; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
uint32_t ir, red , green;
#define USEFIFO

float temp;

int r=0;

int selec_dsp = 0;

int phase;

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

unsigned long previousMillis = 0;
unsigned long previousMillisA = 0;

FirebaseData firebaseData;

void setup()
{
  pinMode(13,OUTPUT);
  digitalWrite(13,0);//buzer off
  Serial.begin(115200);

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  
  WiFi.begin(SSID, PASSWORD);
  Serial.printf("WiFi connecting to %s\n",  SSID);
  while (WiFi.status() != WL_CONNECTED) 
  {
    Serial.print(".");
    delay(400);
  }
  Serial.printf("\nWiFi connected\nIP : ");
  Serial.println(WiFi.localIP());
  digitalWrite(2,0);//LED on
  delay(2000);

  
  Serial.println("Initializing...");
  while (!particleSensor.begin(Wire, 100000)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper at MH-ET LIVE MAX30102 board. ");
    delay(1000);
  }

  //Setup to sense a nice looking saw tooth on the plotter
  byte ledBrightness = 0x7F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 16384; //Options: 2048, 4096, 8192, 16384
  // Set up the wanted parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  particleSensor.enableDIETEMPRDY();

  mlx.begin();

  Wire.begin();
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);


  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);


  xTaskCreatePinnedToCore(loop_T0,"Task0",10000,NULL,0,NULL,0);

    
 // xTaskCreatePinnedToCore(loop_T3,"Task3",4096,NULL,3,NULL,1);
 // delay(1000);
 // xTaskCreatePinnedToCore(loop_T2,"Task2",4096,NULL,2,NULL,0);
 // delay(500);
 // xTaskCreatePinnedToCore(loop_T1,"Task1",10000,NULL,1,NULL,1);
 // delay(500);

  digitalWrite(13,0);//buzer on
  delay(100);
  digitalWrite(13,1);//buzer off
  delay(100);
}

void loop_T0(void * parameter) 
{
     while(1)
     {       
      if(selec_dsp == 0)
         {
           Serial.println("Place finger");     
           tft.fillRect(0, 100, 320, 320, tft.color24to16(TFT_BLACK));
           tft.setTextSize(3);
           tft.setTextColor(TFT_WHITE);
           tft.setCursor(100, 200);
           tft.print(F("Place finger"));
           tft.setCursor(10, 100);
           tft.print(temp);          
           tft.setCursor(10, 300);
           tft.print(phase);
           
           tft.setTextSize(2);
           tft.setCursor(10, 230);
           tft.print(AcX);
           tft.setCursor(90, 230);
           tft.print(AcY);
           tft.setCursor(170, 230);
           tft.print(AcZ);
           
           delay(100);                    
         }
       else if(selec_dsp == 1)
         {
           tft.fillRect(0, 10, 320, 320, tft.color24to16(TFT_BLACK));
           tft.setTextSize(3);
           tft.setTextColor(TFT_WHITE);
           tft.setCursor(10, 20);
           tft.print(beatAvg);
           tft.setCursor(10, 60);
           tft.print(ESpO2);
           tft.setCursor(10, 100);
           tft.print(temp);
           tft.setCursor(10, 300);
           tft.print(phase);

           tft.setTextSize(2);
           tft.setCursor(10, 230);
           tft.print(AcX);
           tft.setCursor(90, 230);
           tft.print(AcY);
           tft.setCursor(170, 230);
           tft.print(AcZ);
           
           delay(100);      
         }
      else if(r == 14)
        {
           Serial.println("-- SUCCESS OK --");      
           tft.setTextSize(3);
           tft.setTextColor(TFT_WHITE);
           tft.setCursor(100, 200);
           tft.print(F("-- SUCCESS OK --"));                  
        }     
    }
}

/*void loop_T1(void * parameter) 
{
   
}*/


void loop()
{  
    read_bpm();
    read_spo2();
    read_temp();    
    read_phase();
    read_axg();

          
    if(ir >=FINGER_ON)
      {
      selec_dsp = 1;
      Serial.println("r : "+String(r)+" ir : "+ String(ir)+"   BPM = : "+ String(beatAvg)+" Oxygen = " + String(ESpO2)+" % Temp = " + String(temp)+" *C phase = " + String(phase));
      Serial.println("AcX : "+String(AcX)+" AcY : "+ String(AcY)+" AcZ = : "+ String(AcZ));
      }
    else
      {
      selec_dsp = 0; 
      }  

      
                    
      unsigned long currentMillisA = millis();
      if (currentMillisA - previousMillisA  >= 10000) {      
         Firebase.setString(firebaseData,"/"+device+"/"+"bpm", String(beatAvg));
         Firebase.setString(firebaseData,"/"+device+"/"+"oxygen", String(ESpO2));
         Firebase.setString(firebaseData,"/"+device+"/"+"temp", String(temp));                         
      previousMillisA = currentMillisA;              
      }
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis  >= 3000) {            
       
         Firebase.setString(firebaseData,"/"+device+"/"+"phase", String(phase));   
         Firebase.setString(firebaseData,"/"+device+"/"+"acx", String(AcX));
         //Firebase.setString(firebaseData,"/"+device+"/"+"acy", String(AcY));
         //Firebase.setString(firebaseData,"/"+device+"/"+"acz", String(AcZ));
      previousMillis = currentMillis;              
      }  
       
}


void read_bpm()
{
  long irValue = particleSensor.getIR();
   if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
      
      r++;//เอาไว้นับเวลาให้การวัดหยุด
    }
  }
}


void read_temp()
{
  temp = mlx.readObjectTempC();
}

void read_spo2()
{
  
  double fred, fir;
  double SpO2 = 0; //raw SpO2 before low pass filtered

  #ifdef USEFIFO
    particleSensor.check(); //Check the sensor, read up to 3 samples
  
    while (particleSensor.available()) 
    {//do we have new data
  #ifdef MAX30105
      red = particleSensor.getFIFORed(); //Sparkfun's MAX30105
      ir = particleSensor.getFIFOIR();  //Sparkfun's MAX30105
  #else
      red = particleSensor.getFIFOIR(); //why getFOFOIR output Red data by MAX30102 on MH-ET LIVE breakout board
      ir = particleSensor.getFIFORed(); //why getFIFORed output IR data by MAX30102 on MH-ET LIVE breakout board
  #endif

    i++;
    fred = (double)red;
    fir = (double)ir;
    avered = avered * frate + (double)red * (1.0 - frate);//average red level by low pass filter
    aveir = aveir * frate + (double)ir * (1.0 - frate); //average IR level by low pass filter
    sumredrms += (fred - avered) * (fred - avered); //square sum of alternate component of red level
    sumirrms += (fir - aveir) * (fir - aveir);//square sum of alternate component of IR level
        
    if (ir < FINGER_ON) ESpO2 = MINIMUM_SPO2; //indicator for finger detached
        
    if ((i % Num) == 0) 
    {
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
      // Serial.println(R);
      SpO2 = -23.3 * (R - 0.4) + 100; 
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;//low pass filter
      if(ESpO2>=100){ESpO2=100;}
      //  Serial.print(SpO2);Serial.print(",");Serial.println(ESpO2);
      sumredrms = 0.0; sumirrms = 0.0; i = 0;
      break;
    }
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
    //Serial.println(SpO2); 
  }
#endif
}

void read_phase()
{
   phase = sensor.readRangeContinuousMillimeters(); 
}

void read_axg()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
}
