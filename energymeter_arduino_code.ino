const int analogInputPin = A0;
const int analogInPinC = A1;
const unsigned long sampleWindow = 1000; // Time window for peak detection in milliseconds
const float threshold = 1.0; 
unsigned long frequency = 0;
int count = 0;
float previousValue = 0;
unsigned long previousMillis = 0;
unsigned long previousMillisp = 0; // Variable to store the time of the previous ADC conversion
unsigned long interval=0.1;
unsigned long sumSquaredVoltage=0;
unsigned long sumSquaredCurrent=0;
float sumVoltage=0;
float sumCurrent=0;



unsigned int numSamples = 0;
unsigned int wantedSamples;
float sumPower=0;

bool freqReady=false;
#include <LiquidCrystal_AIP31068_I2C.h>
LiquidCrystal_AIP31068_I2C lcd(0x3E,16,2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  lcd.init();
}
//no.of zero crossings in a time intervals
unsigned long FREQ(){
  //runs time in micro sec clock
  unsigned long currentMillis = millis();
  //ADC fucn
  int sensorValue = analogRead(analogInputPin);
  //digitals values
  // sensor value+sensorvalue*(5/1023);
  float voltage=(sensorValue-(1023.0/2.0))*(5.0/1023.0);

  if ((voltage >= threshold && previousValue < threshold) || (voltage < threshold && previousValue >= threshold)) {
    count++; // Increment count only when there's a transition across threshold
  }

  previousValue = voltage;

  if (currentMillis - previousMillis >= sampleWindow) {
    frequency = count / (2 * (sampleWindow / 1000));
    previousMillis = currentMillis;
    count = 0; // Reset count after each sample window
    freqReady=true;
    return frequency;
  }  
}

void PF(float frequency){
  if (freqReady){
    if(frequency>10){
      wantedSamples=20000/(2*(frequency));
      
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= interval)
      {
        previousMillis = currentMillis;
        // Perform ADC conversion
        int sensorValueC= analogRead(analogInPinC);
        float current=sensorValueC-(1023.0/2.0);
        float actualCurrent=(current)*(250.0/1023.0);
        sumSquaredCurrent += actualCurrent*actualCurrent;

        int sensorValueV= analogRead(analogInputPin);
        float voltageV=sensorValueV-(1023.0/2.0);
        float actualVoltage=(voltageV)*(1000.0/1023.0);
        sumSquaredVoltage += actualVoltage*actualVoltage;

        float power = actualVoltage * actualCurrent;
        sumPower += power;
        
        numSamples++;


        if(numSamples == wantedSamples){
          Serial.println("------------- Measurements ---------------");
          float vrms = sqrt(sumSquaredVoltage / numSamples);
          float irms = sqrt(sumSquaredCurrent / numSamples);
          float Prms = vrms * irms;
          float Pavg = sumPower / numSamples;
          float pf = Pavg / Prms;
          lcd.print("FREQUENCY:");
          lcd.println(frequency);
          lcd.print("RMS Voltage:");
          lcd.println(vrms);
          Serial.print("RMS Current:");
          Serial.println(irms);
          Serial.print("RMS Power:");
          Serial.println(Prms);
          Serial.print("AVG POWER :");
          Serial.println(Pavg);
          Serial.print("POWER FACTOR :");
          Serial.println(pf);

          sumSquaredVoltage = 0;
          sumSquaredCurrent = 0;
          sumPower = 0;
          numSamples = 0;
          freqReady = false; // Reset frequency ready flag

          previousMillis = currentMillis;

        }  
      }
    }
    else{
      wantedSamples=200;
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= interval)
      {
        previousMillis = currentMillis;
        // Perform ADC conversion
        int sensorValueV= analogRead(analogInputPin);
        float voltageV=sensorValueV*(5.0/1023.0);
        sumVoltage += voltageV;

        int sensorValueC= analogRead(analogInPinC);
        float current=(sensorValueC)*(5.0/1023.0);
        sumCurrent += current;
        
        numSamples++;


        if(numSamples == wantedSamples){
          Serial.println("------------- Measurements ---------------");
          float vdc = (sumVoltage / numSamples);
          float idc = (sumCurrent / numSamples);
          float P = vdc * idc;
          Serial.print("FREQUENCY:");
          Serial.println(frequency);
          Serial.print("Avg Voltage:");
          Serial.println(vdc);
          lcd.print("Avg Current:");
          lcd.println(idc);
          Serial.print("Power:");
          Serial.println(P);
          sumVoltage = 0;
          sumCurrent = 0;
          sumPower = 0;
          numSamples = 0;
          freqReady = false; // Reset frequency ready flag

          previousMillis = currentMillis;

        }  
      }
    }  
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentFrequency = FREQ(); 
  PF(currentFrequency);
}
