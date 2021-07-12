#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <HIH61XX.h>


// Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
Adafruit_ADS1015 ads0(0x48);     /* Use thi for the 12-bit version */
Adafruit_ADS1015 ads1(0x49);     /* Use thi for the 12-bit version */
Adafruit_ADS1015 ads2(0x4A);     /* Use thi for the 12-bit version */
Adafruit_ADS1015 ads3(0x4B);     /* Use thi for the 12-bit version */

const int RedLedPin = 2;       // pin that the LED is attached to
const int GreenLedPin = 3;       // pin that the LED is attached to
long previousMillis =0;
long interval = 2000;
bool ilock_tripped = 0;
bool ilock_status = 1;

const int threshold = 1200;   // an arbitrary threshold level that's in the range of the analog input
const float temp_low_threshold = -50;
const float temp_high_threshold = 25;

HIH61XX hih(0x27);
const int ButtonPin = 4; // pin that the button is connected to. 
void setup(void) 
{
  Serial.begin(9600);
  
  analogReference(DEFAULT);
  Wire.begin();
  
  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");
    Serial.println("RH\tRH, raw\t\tTemp (C)\tTemp, raw\tTemp (F)\t\tADC0-1\tADC1-1tADC2-1\tADC3-1\tADC0-2\tADC1-2\tADC2-2\tADC3-2");

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
    pinMode(RedLedPin, OUTPUT);
    pinMode(GreenLedPin, OUTPUT);

    pinMode(ButtonPin, INPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);
 


    
    digitalWrite(RedLedPin, LOW);
    digitalWrite(GreenLedPin, HIGH);
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
    digitalWrite(9, HIGH);
    digitalWrite(10, HIGH);
    digitalWrite(11, HIGH);
    digitalWrite(12, HIGH);
    digitalWrite(13, HIGH);

 


  ads0.begin();
  ads1.begin();
    ads2.begin();
  ads3.begin();

  ads0.setGain(GAIN_ONE);
  ads1.setGain(GAIN_ONE);
  ads2.setGain(GAIN_ONE);
  ads3.setGain(GAIN_ONE);

}

void loop(void) 
{
  int16_t adc0, adc1, adc2, adc3, adc4, adc5, adc6, adc7;
  int16_t adc8, adc9, adc10, adc11, adc12, adc13, adc14, adc15;
  int16_t adc_array[16];

// int i_refvoltage = analogRead(A5);
//  float f_refvoltage = (5/1024)*(float)i_refvoltage;
  //Serial.print("i_ref: "); Serial.print(i_refvoltage); Serial.print("\t f_ref: "); Serial.print(f_refvoltage); Serial.print("\n");
  
  if(digitalRead(ButtonPin) == HIGH)
   { 
        digitalWrite(5, HIGH);
        digitalWrite(6, HIGH);
        digitalWrite(7, HIGH);
        digitalWrite(8, HIGH);
        digitalWrite(9, HIGH);
        digitalWrite(10, HIGH);
        digitalWrite(11, HIGH);
        digitalWrite(12, HIGH);
        digitalWrite(13, HIGH);

        digitalWrite(GreenLedPin, HIGH);
        digitalWrite(RedLedPin, LOW);
   }

  

  adc0 = ads0.readADC_SingleEnded(0);
  adc1 = ads0.readADC_SingleEnded(1);
  adc2 = ads0.readADC_SingleEnded(2);
  adc3 = ads0.readADC_SingleEnded(3);
  adc4 = ads1.readADC_SingleEnded(0);
  adc5 = ads1.readADC_SingleEnded(1);
  adc6 = ads1.readADC_SingleEnded(2);
  adc7 = ads1.readADC_SingleEnded(3);
  adc8 = ads2.readADC_SingleEnded(0);
  adc9 = ads2.readADC_SingleEnded(1);
  adc10 = ads2.readADC_SingleEnded(2);
  adc11 = ads2.readADC_SingleEnded(3);
  adc12 = ads3.readADC_SingleEnded(0);
  adc13 = ads3.readADC_SingleEnded(1);
  adc14 = ads3.readADC_SingleEnded(2);
  adc15 = ads3.readADC_SingleEnded(3);

  adc_array[0] = adc0;
  adc_array[1] = adc1;
  adc_array[2] = adc2;
  adc_array[3] = adc3;
  adc_array[4] = adc4;
  adc_array[5] = adc5;
  adc_array[6] = adc6;
  adc_array[7] = adc7;
  adc_array[8] = adc8;
  adc_array[9] = adc9;
  adc_array[10] = adc10;
  adc_array[11] = adc11;
  adc_array[12] = adc12;
  adc_array[13] = adc13;
  adc_array[14] = adc14;
  adc_array[15] = adc15;

  //Use adc10 as the reference voltage.  It should be measure as 1/2 of the actual input voltage
  float f_refVoltage = 2*adc10*(0.002);
  //Serial.print("ref_Voltage: "); Serial.print(f_refVoltage); Serial.print("\n");


  // Find the temp of each NTC
  double temp_array[16];
  for(int i=0;i<16;i++)
   {
    float voltage=adc_array[i]*0.002;
    float resistance= (10000/voltage)*(f_refVoltage-voltage);
   
    // Calculate temperatures from ADC values using steinhart hard
    // Coeffiencients
    float f_A = 0.0021085;
    float f_B = 0.000079792;
    double f_C = 0.00000065351; 
    double lnR = log(resistance);
    double in_temp = f_A + (f_B*lnR)+(f_C*lnR*lnR*lnR);
    temp_array[i]=(1/in_temp)-273;
//    Serial.print(i);
//    Serial.print("\t ADC: "); Serial.print(adc_array[i]);Serial.print("\t voltage: "); Serial.print(voltage); Serial.print("\t resistance: "); Serial.print(resistance); Serial.print("\t temp: "); Serial.print(temp_array[i]); Serial.print("\n");
   }

//  Serial.print("AIN0: "); Serial.println(adc0);
//  Serial.print("AIN1: "); Serial.println(adc1);
 // Serial.print("AIN2: "); Serial.println(adc2);
 // Serial.print("AIN3: "); Serial.println(adc3);
//  Serial.println(" ");
  bool out_of_range = 0;
  for(int i=0;i<16;i++)
   {
    if(i!=10)
    {
    if(temp_array[i]>temp_high_threshold || temp_array[i]<temp_low_threshold){out_of_range=1;}
    }
   }

  if(out_of_range){
/*  if (adc0 > threshold ||adc1 > threshold || adc2 > threshold || adc3 > threshold || 
      adc4 > threshold|| adc5 > threshold || adc6 > threshold || adc7 > threshold ||
      adc8 > threshold ||adc9 > threshold || adc11 > threshold || 
      adc12 > threshold|| adc13 > threshold || adc14 > threshold || adc15 > threshold ) {*/
    digitalWrite(RedLedPin, HIGH);
    digitalWrite(GreenLedPin, LOW);

    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);
    digitalWrite(12, LOW);
    digitalWrite(13, LOW);
    //Serial.print("----------------------------------Interlock----------------------------------------------------------\n");
    ilock_status = 0;
    ilock_tripped = 1;
  } else {
    digitalWrite(GreenLedPin, HIGH);
    ilock_status = 1;
  }

 /* if (adc0 > threshold ) {
    digitalWrite(ledPin, HIGH);
    digitalWrite(2, LOW);
    digitalWrite(7, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
  */
 // hih.start();
  //  request an update of the humidity and temperature
 // hih.update();
 /*
  Serial.print(hih.humidity()); Serial.print("\t");
  Serial.print(hih.humidity_Raw()); Serial.print("\t\t");

  Serial.print(hih.temperature()); Serial.print("\t\t");
  Serial.print(hih.temperature_Raw()); Serial.print("\t\t");

  float temperature_F = hih.temperature() * 1.8 + 32;
  Serial.print(temperature_F); Serial.print("\t\t"); */
  if(millis()-previousMillis > interval)
  {
  for(int i=0;i<16;i++)
  {
    Serial.print(temp_array[i]);Serial.print(" ");
  }
  Serial.print(ilock_status); Serial.print(" ");
  Serial.print(ilock_tripped); Serial.print(" ");
  Serial.print("\n");
  /*
  //Serial.print(digitalRead(ButtonPin)); Serial.print("\t");
  Serial.print(adc0); Serial.print("\t");
  Serial.print(adc1); Serial.print("\t");
  Serial.print(adc2); Serial.print("\t");
  Serial.print(adc3); Serial.print("\t");
  Serial.print(adc4); Serial.print("\t");
  Serial.print(adc5); Serial.print("\t");
  Serial.print(adc6); Serial.print("\t");
  Serial.print(adc7); Serial.print("\t");
  Serial.print(adc8); Serial.print("\t");
  Serial.print(adc9); Serial.print("\t");
  Serial.print(adc10); Serial.print("\t");
  Serial.print(adc11); Serial.print("\t");
  Serial.print(adc12); Serial.print("\t");
  Serial.print(adc13); Serial.print("\t");
  Serial.print(adc14); Serial.print("\t");
  Serial.print(adc15); Serial.print("\t");
  Serial.print("endline\n");
 // Serial.print(previousMillis); Serial.print("\t");
  //Serial.print(millis()); Serial.print("\n");
*/
    previousMillis = millis();
  }
  
 
  
  //delay(1000);
}
