/*
  Emon.cpp - Library for openenergymonitor
  Created by Trystan Lea, April 27 2010
  GNU GPL
  modified to use up to 12 bits ADC resolution (ex. Arduino Due)
  by boredman@boredomprojects.net 26.12.2013
  Low Pass filter for offset removal replaces HP filter 1/1/2015 - RW
*/

// Proboscide99 10/08/2016 - Added ADMUX settings for ATmega1284 e 1284P (644 / 644P also, but not tested) in readVcc function

//#include "WProgram.h" un-comment for use on older versions of Arduino IDE
#include "EmonLib.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//--------------------------------------------------------------------------------------
// Sets the pins to be used for voltage and current sensors
//--------------------------------------------------------------------------------------
void EnergyMonitor::voltage(unsigned int _inPinV, double _VCAL, double _PHASECAL)
{
  inPinV = _inPinV;
  VCAL = _VCAL;
  PHASECAL = _PHASECAL;
  offsetV = ADC_COUNTS>>1;
}

void EnergyMonitor::current(size_t channel, unsigned int _inPinI, double _ICAL)
{
  if (channel >= EMON_I_CHANNELS) return;
  inPinI[channel] = _inPinI;
  ICAL[channel] = _ICAL;
  offsetI[channel] = ADC_COUNTS>>1;
}

// bootstrap low-pass filters
void EnergyMonitor::bootstrap()
{
  const unsigned int crossings = 100;
  const unsigned int timeout = 5000;

  unsigned long start = millis();

  unsigned int crossCount = 0;
  unsigned int numberOfSamples = 0;
  int startV = 0;

  // wait for a mid-point (zero) crossing to start:
  while(1)
  {
    startV = analogRead(inPinV);
    if ((startV < (ADC_COUNTS*0.55)) && (startV > (ADC_COUNTS*0.45))) break;
    if ((millis()-start)>timeout) break;
  }

  start = millis();
  double sumV = 0.0d;
  double sumI[EMON_I_CHANNELS] = {0};
  boolean lastVCross = false, checkVCross = false;

  while ((crossCount < crossings) && ((millis()-start)<timeout))
  {
    numberOfSamples++;
    sampleV = analogRead(inPinV);
    for (size_t i = 0; i < EMON_I_CHANNELS; ++i) {
      sampleI[i] = analogRead(inPinI[i]);
    }

    sumV += sampleV;
    for (size_t i = 0; i < EMON_I_CHANNELS; ++i) {
      sumI[i] += sampleI[i];
    }

    lastVCross = checkVCross;
    if (sampleV > startV) checkVCross = true;
                     else checkVCross = false;
    if (numberOfSamples==1) lastVCross = checkVCross;

    if (lastVCross != checkVCross) crossCount++;
  }

  offsetV = sumV / numberOfSamples;

  for (size_t i = 0; i < EMON_I_CHANNELS; ++i) {
    offsetI[i] = sumI[i] / numberOfSamples;
  }

  Serial.print(offsetV);
  Serial.print(' ');
  for (size_t i = 0; i < EMON_I_CHANNELS; ++i) {
    Serial.print(offsetI[i]);
    Serial.print(',');
  }
  Serial.print(' ');
  Serial.print(numberOfSamples);
  Serial.print(' ');
  Serial.print(sumV);
  Serial.print(' ');
  for (size_t i = 0; i < EMON_I_CHANNELS; ++i) {
    Serial.print(sumI[i]);
    Serial.print(',');
  }
  Serial.print(' ');
  Serial.println(" (offsetV, offsetI[], n, sumV, sumI[])");
}

//--------------------------------------------------------------------------------------
// emon_calc procedure
// Calculates realPower,apparentPower,powerFactor,Vrms,Irms,kWh increment
// From a sample window of the mains AC voltage and current.
// The Sample window length is defined by the number of half wavelengths or crossings we choose to measure.
//--------------------------------------------------------------------------------------
void EnergyMonitor::calcVI(unsigned int crossings, unsigned int timeout)
{
  #if defined emonTxV3
  int SupplyVoltage=3300;
  #elif defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)
  int SupplyVoltage=5000;
  #else
  int SupplyVoltage = readVcc();
  #endif
  double sqV = 0.0d,
         sumV = 0.0d,
         phaseShiftedV = 0.0d,
         sqI[EMON_I_CHANNELS] = {0},
         sumI[EMON_I_CHANNELS] = {0},
         instP[EMON_I_CHANNELS] = {0},
         sumP[EMON_I_CHANNELS] = {0};
         //sumPp[EMON_I_CHANNELS] = {0},
         //sumPn[EMON_I_CHANNELS] = {0}
   //sq = squared, sum = Sum, inst = instantaneous

  double filteredV = 0.0d,
         lastFilteredV = 0.0d;

  boolean lastVCross = false, checkVCross = false;         //Used to measure number of times threshold is crossed.
  int startV = 0;

  unsigned int crossCount = 0;                             //Used to measure number of times threshold is crossed.
  unsigned int numberOfSamples = 0;                        //This is now incremented

  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
  //-------------------------------------------------------------------------------------------------------------------------
  unsigned long start = millis();    //millis()-start makes sure it doesnt get stuck in the loop if there is an error.

  while(1)                                   //the while loop...
  {
    startV = analogRead(inPinV);                    //using the voltage waveform
    if ((startV < (ADC_COUNTS*0.55)) && (startV > (ADC_COUNTS*0.45))) break;  //check its within range
    if ((millis()-start)>timeout) break;
  }

  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Main measurement loop
  //-------------------------------------------------------------------------------------------------------------------------
  start = millis();

  while ((crossCount < crossings) && ((millis()-start)<timeout))
  {
    numberOfSamples++;                       //Count number of times looped.
    lastFilteredV = filteredV;               //Used for delay/phase compensation

    //-----------------------------------------------------------------------------
    // A) Read in raw voltage and current samples
    //-----------------------------------------------------------------------------
    sampleV = analogRead(inPinV);                 //Read in raw voltage signal
    for (size_t i = 0; i < EMON_I_CHANNELS; ++i) {
      sampleI[i] = analogRead(inPinI[i]);               //Read in raw current signal
    }

    //-----------------------------------------------------------------------------
    // B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
    //     then subtract this - signal is now centred on 0 counts.
    //-----------------------------------------------------------------------------
    offsetV = offsetV + ((sampleV-offsetV)/8192);
    filteredV = sampleV - offsetV;
    for (size_t i = 0; i < EMON_I_CHANNELS; ++i) {
      offsetI[i] = offsetI[i] + ((sampleI[i]-offsetI[i])/8192);
      filteredI[i] = sampleI[i] - offsetI[i];

      //-----------------------------------------------------------------------------
      // D) Root-mean-square method current
      //-----------------------------------------------------------------------------
      sqI[i] = filteredI[i] * filteredI[i];          //1) square current values
      sumI[i] += sqI[i];                             //2) sum
    }

    //-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------
    sqV= filteredV * filteredV;                 //1) square voltage values
    sumV += sqV;                                //2) sum

    //-----------------------------------------------------------------------------
    // E) Phase calibration
    //-----------------------------------------------------------------------------
    phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV);

    //-----------------------------------------------------------------------------
    // F) Instantaneous power calc
    //-----------------------------------------------------------------------------
    for (size_t i = 0; i < EMON_I_CHANNELS; ++i) {
      instP[i] = phaseShiftedV * filteredI[i];          //Instantaneous Power
      sumP[i] += instP[i];                              //Sum

      // sum +ve and -ve flows separately, so we can get more accurate bidirectional flows:
      //if (instP[i] > 0) sumPp[i] += instP[i];
      //if (instP[i] < 0) sumPn[i] += instP[i];
    }

    //-----------------------------------------------------------------------------
    // G) Find the number of times the voltage has crossed the initial voltage
    //    - every 2 crosses we will have sampled 1 wavelength
    //    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
    //-----------------------------------------------------------------------------
    lastVCross = checkVCross;
    if (filteredV > 0) checkVCross = true;
                  else checkVCross = false;
    if (numberOfSamples==1) lastVCross = checkVCross;

    if (lastVCross != checkVCross) crossCount++;
  }

  //-------------------------------------------------------------------------------------------------------------------------
  // 3) Post loop calculations
  //-------------------------------------------------------------------------------------------------------------------------
  //Calculation of the root of the mean of the voltage and current squared (rms)
  //Calibration coefficients applied.

  double V_RATIO = VCAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Vrms = V_RATIO * sqrt(sumV / numberOfSamples);

  for (size_t i = 0; i < EMON_I_CHANNELS; ++i) {
    double I_RATIO = ICAL[i] *((SupplyVoltage/1000.0) / (ADC_COUNTS));
    Irms[i] = I_RATIO * sqrt(sumI[i] / numberOfSamples);

    //Calculation power values
    realPower[i] = V_RATIO * I_RATIO * sumP[i] / numberOfSamples;
    //positiveRealPower[i] = V_RATIO * I_RATIO * sumPp[i] / numberOfSamples;
    //negativeRealPower[i] = V_RATIO * I_RATIO * sumPn[i] / numberOfSamples;
    apparentPower[i] = Vrms * Irms[i];
    powerFactor[i]=realPower[i] / apparentPower[i];
  }
//--------------------------------------------------------------------------------------
}

//--------------------------------------------------------------------------------------
// double EnergyMonitor::calcIrms(unsigned int Number_of_Samples)
// {
//
//   #if defined emonTxV3
//     int SupplyVoltage=3300;
//   #else
//     int SupplyVoltage = readVcc();
//   #endif
//
//
//   for (unsigned int n = 0; n < Number_of_Samples; n++)
//   {
//     sampleI = analogRead(inPinI);
//
//     // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
//     //  then subtract this - signal is now centered on 0 counts.
//     offsetI = (offsetI + (sampleI-offsetI)/16384);
//     filteredI = sampleI - offsetI;
//
//     // Root-mean-square method current
//     // 1) square current values
//     sqI = filteredI * filteredI;
//     // 2) sum
//     sumI += sqI;
//   }
//
//   double I_RATIO = ICAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
//   Irms = I_RATIO * sqrt(sumI / Number_of_Samples);
//
//   //Reset accumulators
//   sumI = 0;
//   //--------------------------------------------------------------------------------------
//
//   return Irms;
// }

void EnergyMonitor::serialprint()
{
  Serial.print("V=");
  Serial.print(Vrms);
  for (size_t i = 0; i < EMON_I_CHANNELS; ++i) {
    Serial.print("C:");
    Serial.print(i);
    Serial.print(" P=");
    Serial.print(realPower[i]);
    // Serial.print(" P+=");
    // Serial.print(positiveRealPower[i]);
    // Serial.print(" P-=");
    // Serial.print(negativeRealPower[i]);
    Serial.print(" VA=");
    Serial.print(apparentPower[i]);
    Serial.print(" I=");
    Serial.print(Irms[i]);
    Serial.print(" phi=");
    Serial.print(powerFactor[i]);
  }
  Serial.println(' ');
  // delay(100);
}

//thanks to http://hacking.majenko.co.uk/making-accurate-adc-readings-on-arduino
//and Jérôme who alerted us to http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/

long EnergyMonitor::readVcc() {
  long result;

  //not used on emonTx V3 - as Vcc is always 3.3V - eliminates bandgap error and need for calibration http://harizanov.com/2013/09/thoughts-on-avr-adc-accuracy/

  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined(__AVR_ATmega644__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_AT90USB1286__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  ADCSRB &= ~_BV(MUX5);   // Without this the function always returns -1 on the ATmega2560 http://openenergymonitor.org/emon/node/2253#comment-11432
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);

  #endif


  #if defined(__AVR__)
  delay(2);                                        // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);                             // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = READVCC_CALIBRATION_CONST / result;  //1100mV*1024 ADC steps http://openenergymonitor.org/emon/node/1186
  return result;
  #elif defined(__arm__)
  return (3300);                                  //Arduino Due
  #else
  return (3300);                                  //Guess that other un-supported architectures will be running a 3.3V!
  #endif
}

