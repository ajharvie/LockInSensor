#include <IntervalTimer.h>
#include <math.h>
#include <ADC.h>
#include <ADC_Module.h>
#include <elapsedMillis.h>
#define pi 3.1415926535897932384626433832795
#define nHarmonicsSin 1 //number of sin harmonics to calculate (at least 1)       
#define nHarmonicsCos 1 //number of cos harmonics to calculate (at least 1)

/////////////////////////////////////SETTINGS///////////////////////////////////////
                                    
double Fsig = 410; //choose modulation frequency (Hz)

double Fsample = 200000; //choose sampling frequency (Hz)

double phaseDiff = 0; //Choose phase offset (radians)

unsigned int outputTime = 100; //number of milliseconds between outputs

///////////////////////////////////////////////////////////////////////////////////

IntervalTimer lockInTimer; //object for interrupt loop used for lock-in algorithm
elapsedMillis timeElapsed; //counter for time between output samples
ADC *adc = new ADC();  //adc object
bool useSyncFilter = false; //use synchronous filter only for very low frequencies (less than 20 Hz)
int ledPin = 23; //digital pin for LED driver
int i; //index for reading from live values
volatile int j; //index used for calculation within intervalTimer
double FcutOff = 0.25; //filter cutoff frequency
double wsig = 2*pi*Fsig; // signal angular frequency
double dt = 1/Fsample; //sample time in seconds
int dt_micros = (int)(dt*1000000); //sample time in microseconds
const int samplesPerPeriod = (int)(Fsample/Fsig); //number of input samples to measure per reference signal period
int sampleIndex = 0; //what sample within the current period are we at
bool wait = true;
double phasSig; //reference signal values in phase
double quadSig; //ditto, quadrature
volatile double delta; //filtering things
volatile double incr; //small increment
volatile double xiFilt1[nHarmonicsSin]; //in phase (first filter)
volatile double xqFilt1[nHarmonicsCos]; //quadrature (first filter)
volatile double xiFilt2[nHarmonicsSin]; //in phase (second filter)
volatile double xqFilt2[nHarmonicsCos]; //quadrature (second filter)
volatile double xiVar2[nHarmonicsSin]; //variances and means
volatile double xiMean2[nHarmonicsSin];
volatile double xqVar2[nHarmonicsCos];
volatile double xqMean2[nHarmonicsCos];
double grabQuad[nHarmonicsCos]; //"snapshot" of current values in volatile array
double grabPhas[nHarmonicsSin];
double grabVar; //snapshot of variance
double lag; //phase value
double rmsMeasured; //rms value
double noise; //est. noise
double xSig; //input signal
double xi0; //input * ref signal in phase
double xq0; //input * ref signal quadrature
double xio0; //buffered values of above for sync filter
double xqo0;
//calculation of weighting coeffs for filter - see: R.G. Lyons, Understanding digital signal processing, 3rd ed., Prentice Hall Publishing, 2011
double g = cos((2.0 * pi * FcutOff) / Fsample);
double alpha = g - 1.0 + sqrt(g * g - 4.0 * g + 3.0); 
double alpha_min = 1.0 - alpha;
//sync filter circular buffer
int xBuffer[20000] = {};
int Io0 = 1; //oldest value index
int In0 = 0; //newest


void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(2000000);
  delay(1000); //wait for serial port to initialise
  
  //set ADC for fastest operation + max resolution and start measurement
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); 
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
  adc->adc0->setResolution(12);
  adc->adc0->setAveraging (1);
  adc->adc0->startContinuous(A0);
  
  //start calculation
  lockInTimer.begin(calculate, dt_micros);
  delay(1000);
}


void loop() {
  //calculations happen in calculate(). This loop grabs the "live" values from the array and outputs via serial
  
  //wait until next output time
  while(wait == true){
    if (timeElapsed > outputTime){
      timeElapsed = 0;
      wait = false;   
    } 
  }
  
  //grab values from array. Disallow interrupts (and therefore updating of the values) while doing so
  noInterrupts();
  for (i = 0; i < nHarmonicsSin; ++i){
    grabPhas[i] = xiFilt2[i];
  }
  for (i = 0; i < nHarmonicsCos; ++i){
    grabQuad[i] = xqFilt2[i];
  }
  grabVar = xqVar2[0];
  interrupts();
  
  
  //calculating output values
  lag = atan(grabQuad[0]/grabPhas[0]);
  rmsMeasured = sqrt(grabQuad[0]*grabQuad[0] + grabPhas[0]*grabPhas[0]);
  noise = sqrt(grabVar);

  

  Serial.println(rmsMeasured, 5); // output total signal amplitude

  //waiting until next output
  wait = true;
  
}

void calculate(){
  //acquire input sample
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

  
  //if using optional synchronous filter instead (only for very low frequencies)
  if (useSyncFilter == true){
    //in-phase harmonics first
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
    //as before but in quadrature
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
    //store newest input signal value in buffer
    xBuffer[In0] = xSig;
    
    ++sampleIndex; 
  }

  

  //LED control. Toggles every half period
  if (sampleIndex < (samplesPerPeriod/2)){
    digitalWrite(ledPin, HIGH);//LED on
  }
  else {
    digitalWrite(ledPin, LOW);//LED off
  }
  
  //go to beginning if reached end of period
  if (sampleIndex == samplesPerPeriod){
    sampleIndex = 0;
  }
}
