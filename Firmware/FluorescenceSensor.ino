#include <IntervalTimer.h>
#include <math.h>
#include <ADC.h>
#include <ADC_Module.h>
#include <elapsedMillis.h>
#define pi 3.1415926535897932384626433832795 //a very round number lmao
#define nHarmonicsSin 1 //number of sin harmonics to calculate (at least 1)       
#define nHarmonicsCos 1 //number of cos harmonics to calculate (at least 1)

/////////////////////////////////////SETTINGS///////////////////////////////////////
                                    
double Fsig = 410; //choose modulation frequency (Hz)

double Fsample = 200000; //choose sampling frequency (Hz)

double phaseDiff = 0; //Choose phase offset (radians)

unsigned int outputTime = 100; //number of milliseconds between outputs

///////////////////////////////////////////////////////////////////////////////////

IntervalTimer lockInTimer;
elapsedMillis timeElapsed;
ADC *adc = new ADC();
bool useSyncFilter = false; //use synchronous filter only for very low frequencies
int ledPin = 23;
int i;
volatile int j;
double FcutOff = 0.25; //filter cutoff frequency
double wsig = 2*pi*Fsig; // signal angular frequency
double dt = 1/Fsample; //sample time in seconds
int dt_micros = (int)(dt*1000000); //sample time in microseconds
const int samplesPerPeriod = (int)(Fsample/Fsig);
int sampleIndex = 0; //what sample within the current period are we at
bool wait = true;
double phasSig; //reference signal values in phase
double quadSig; //ditto, quadrature
volatile double delta; //filtering things
volatile double incr;
volatile double xiFilt1[nHarmonicsSin];
volatile double xqFilt1[nHarmonicsCos];
volatile double xiFilt2[nHarmonicsSin];
volatile double xqFilt2[nHarmonicsCos];
volatile double xiVar2[nHarmonicsSin];
volatile double xiMean2[nHarmonicsSin];
volatile double xqVar2[nHarmonicsCos];
volatile double xqMean2[nHarmonicsCos];
double grabQuad[nHarmonicsCos];
double grabPhas[nHarmonicsSin];
double grabVar;
double lag;
double rmsMeasured;
double noise;
double xSig; //input signal
double xi0;
double xq0;
double xio0;
double xqo0;
//weighting coeffs for filter
double g = cos((2.0 * pi * FcutOff) / Fsample);
double alpha = g - 1.0 + sqrt(g * g - 4.0 * g + 3.0); // how unusual
double alpha_min = 1.0 - alpha;
//sync filter circular buffer
int xBuffer[20000] = {};
int Io0 = 1;
int In0 = 0;


void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(2000000);
  delay(1000);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); 
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
  adc->adc0->setResolution(12);
  adc->adc0->setAveraging (1);
  adc->adc0->startContinuous(A0);
  lockInTimer.begin(calculate, dt_micros);
  delay(1000);
}


void loop() {
  while(wait == true){
    if (timeElapsed > outputTime){
      timeElapsed = 0;
      wait = false;   
    } 
  }
  
  noInterrupts();
  for (i = 0; i < nHarmonicsSin; ++i){
    grabPhas[i] = xiFilt2[i];
  }
  for (i = 0; i < nHarmonicsCos; ++i){
    grabQuad[i] = xqFilt2[i];
  }
  grabVar = xqVar2[0];
  interrupts();

  lag = atan(grabQuad[0]/grabPhas[0]);
  rmsMeasured = sqrt(grabQuad[0]*grabQuad[0] + grabPhas[0]*grabPhas[0]);
  noise = sqrt(grabVar);

  

  Serial.println(rmsMeasured, 5); //total signal amplitude
//Serial.print(" "); 
//  Serial.print(lag, 5); //phase measurement
//  Serial.print(" ");
//  Serial.print(noise, 5);  //noise estimate
//  Serial.print(" ");
//
//  //output harmonics
//
//  for (i = 0; i < nHarmonicsSin; ++i){
//    Serial.print(grabPhas[i], 5);  //in phase 
//    Serial.print(" ");
//  }
//  for (i = 0; i < nHarmonicsCos; ++i){
//    Serial.print(grabQuad[i], 5);  //quad
//    Serial.print(" ");
//  }
//  
//Serial.println("");

  wait = true;
  
}

void calculate(){
  xSig = (double)adc->analogReadContinuous(A0);

  //Using standard exponential filter
  if (useSyncFilter == false){

    //in-phase harmonics
    for (j = 0; j < nHarmonicsSin; ++j){
      //calculate reference value and multiply
      phasSig = sin(((2.0 * (j + 1) * pi * sampleIndex) / samplesPerPeriod) - ((j + 1) * phaseDiff));
      xi0 = (xSig*phasSig); 

      //first smoothing
      delta = xi0 - xiFilt1[j]; 
      incr = alpha*delta;
      xiFilt1[j] = xiFilt1[j] + incr;
      
      //second smoothing
      delta = xiFilt1[j] - xiFilt2[j]; 
      incr = alpha*delta;
      xiFilt2[j] = xiFilt2[j] + incr;       
    }
    //same for quadrature harmonics
    for (j = 0; j < nHarmonicsCos; ++j){
      
      //calculate reference value and multiply
      quadSig = cos(((2.0 * (j + 1) * pi * sampleIndex) / samplesPerPeriod) - ((j + 1) * phaseDiff)); 
      xq0 = (xSig*quadSig);

      //first smoothing
      delta = xq0 - xqFilt1[j];
      incr = alpha*delta;
      xqFilt1[j] = xqFilt1[j] + incr;
      
      //second smoothing
      delta = xqFilt1[j] - xqFilt2[j];
      incr = alpha*delta;
      xqFilt2[j] = xqFilt2[j] + incr; 
    }
    
    //noise estimate
    delta = xqFilt2[0] - xqMean2[0];
    incr = alpha*delta;
    xqMean2[0] = xqMean2[0] + incr;
    xqVar2[0] = alpha_min*(xqVar2[0] + delta*incr);
    
    ++sampleIndex;
  }

  
  //using optional synchronous filter instead (only for very low frequencies)
  if (useSyncFilter == true){
    for (j = 0; j < nHarmonicsSin; ++j){
      phasSig = sin(((2.0 * (j + 1) * pi * sampleIndex) / samplesPerPeriod) - ((j + 1) * phaseDiff));
      xi0 = (xSig*phasSig); //multiplication by reference signal
      xio0 = (xBuffer[Io0]*phasSig); //ditto for oldest buffered value
      
      //smooth once
      xiFilt1[j] = xiFilt1[j] + ((xi0 - xio0)/samplesPerPeriod);
      delta = xiFilt1[j] - xiFilt2[j];
      incr = alpha*delta;
      xiFilt2[j] = xiFilt2[j] + incr;
      
      //smooth twice
      delta = xiFilt2[j] - xiMean2[j];
      incr = alpha*delta;
      xiMean2[j] = xiMean2[j] + incr;
      xiVar2[j] = (1 - alpha) * (xiVar2[j] + delta*incr);
      
    }
    for (j = 0; j < nHarmonicsCos; ++j){
      quadSig = cos(((2.0 * (j + 1) * pi * sampleIndex) / samplesPerPeriod) - ((j + 1) * phaseDiff)); 
      xq0 = (xSig*quadSig); 
      xqo0 = (xBuffer[Io0]*quadSig);
      
      xqFilt1[j] = xqFilt1[j] + ((xq0 - xqo0)/samplesPerPeriod);      
      delta = xqFilt1[j] - xqFilt2[j];
      incr = alpha*delta;
      xqFilt2[j] = xqFilt2[j] + incr;

      delta = xqFilt2[j] - xqMean2[j];
      incr = alpha*delta;
      xqMean2[j] = xqMean2[j] + incr;
      xqVar2[j] = (1 - alpha) * (xqVar2[j] + delta*incr);   
    }

    //advance circular buffers
    In0 = In0 + 1;
    if (In0 >= samplesPerPeriod){
      In0 = In0 - samplesPerPeriod;
    }
    Io0 = In0 + 1;
    if (Io0 >= samplesPerPeriod){
      Io0 = Io0 - samplesPerPeriod;
    }
    //store newest value in buffer
    xBuffer[In0] = xSig;
    
    ++sampleIndex; 
  }

  

  //LED control
  if (sampleIndex < (samplesPerPeriod/2)){
    digitalWrite(ledPin, HIGH);
  }
  else {
    digitalWrite(ledPin, LOW);
  }
  
  if (sampleIndex == samplesPerPeriod){
    sampleIndex = 0;
  }
}

//ayy lmao
