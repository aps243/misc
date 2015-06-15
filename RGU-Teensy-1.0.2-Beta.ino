
#include <ADC.h>
#include "utility/sqrt_integer.h"
#include "utility/dspinst.h"
#include "Arduino.h"
#define AUDIO_BLOCK_SAMPLES  128
#define AUDIO_SAMPLE_RATE    44117.64706
#define AUDIO_SAMPLE_RATE_EXACT 44117.64706

#define LOG_OUT 1 // use the log output function
#define FHT_N 25600 // set to 256 point fht

#include "arm_math.h"

// windows.c
extern "C" {
  extern const int16_t AudioWindowHanning256[];
  extern const int16_t AudioWindowBartlett256[];
  extern const int16_t AudioWindowBlackman256[];
  extern const int16_t AudioWindowFlattop256[];
  extern const int16_t AudioWindowBlackmanHarris256[];
  extern const int16_t AudioWindowNuttall256[];
  extern const int16_t AudioWindowBlackmanNuttall256[];
  extern const int16_t AudioWindowWelch256[];
  extern const int16_t AudioWindowHamming256[];
  extern const int16_t AudioWindowCosine256[];
  extern const int16_t AudioWindowTukey256[];
}

// #include <FHT.h> // include the library

typedef struct audio_block_struct {
  unsigned char ref_count;
  unsigned char memory_pool_index;
  unsigned char reserved1;
  unsigned char reserved2;
  int16_t data[512];
} audio_block_t;

uint16_t output[128] __attribute__ ((aligned (4)));
const int16_t *window;
audio_block_t *block;
int16_t buffer[512] __attribute__ ((aligned (4)));
uint32_t sum[128];
uint8_t count;
uint8_t naverage;
bool outputflag;
audio_block_t *inputQueueArray[1];
arm_cfft_radix4_instance_q15 fft_inst;

audio_block_t FullBlock;

const int readPin = A9; // ADC0
const int readPin2 = A2; // ADC1

ADC *adc = new ADC(); // adc object;

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(readPin, INPUT);
  pinMode(readPin2, INPUT);

  Serial.begin(9600);

  Serial.println("Begin setup");

  ///// ADC0 ////
  // reference can be ADC_REF_3V3, ADC_REF_1V2 (not for Teensy LC) or ADC_REF_EXT.
  //adc->setReference(ADC_REF_1V2, ADC_0); // change all 3.3 to 1.2 if you change the reference to 1V2

  adc->setAveraging(1); // set number of averages
  adc->setResolution(8); // set bits of resolution

  // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED_16BITS, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
  // see the documentation for more information
  adc->setConversionSpeed(ADC_VERY_HIGH_SPEED); // change the conversion speed
  // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
  adc->setSamplingSpeed(ADC_VERY_HIGH_SPEED); // change the sampling speed

  adc->enableInterrupts(ADC_0); // it's necessary to enable interrupts for PDB to work (why?)

  adc->analogRead(readPin, ADC_0); // call this to setup everything before the pdb starts


  ////// ADC1 /////
#if defined(ADC_TEENSY_3_1)
  adc->setAveraging(1, ADC_1); // set number of averages
  adc->setResolution(8, ADC_1); // set bits of resolution
  adc->setConversionSpeed(ADC_VERY_HIGH_SPEED, ADC_1); // change the conversion speed
  adc->setSamplingSpeed(ADC_VERY_HIGH_SPEED, ADC_1); // change the sampling speed

  adc->enableInterrupts(ADC_1);

  adc->analogRead(readPin2, ADC_1); // call this to setup everything before the pdb starts
#endif

  Serial.println("End setup");
block = &FullBlock;

}

char c = 0;
int value;
int value2;

ADC::Sync_result result;
void loop() {

  if (Serial.available()) {
    c = Serial.read();
    if (c == 'v') { // value

      Serial.print("Value ADC0: ");
      runFHT();
      // value = (uint16_t)adc->analogReadContinuous(ADC_0); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
      result = adc->readSynchronizedContinuous();
      // if using 16 bits and single-ended is necessary to typecast to unsigned,
      // otherwise values larger than 3.3/2 will be interpreted as negative
      result.result_adc0 = (uint16_t)result.result_adc0;
      value = result.result_adc0;
      result.result_adc1 = (uint16_t)result.result_adc1;
      value2 = result.result_adc1;
      Serial.println(value * 3.3 / adc->getMaxValue(ADC_0), DEC);
#if defined(ADC_TEENSY_3_1)
      Serial.print("Value ADC1: ");
      //value2 = (uint16_t)adc->analogReadContinuous(ADC_1); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
      Serial.println(value2 * 3.3 / adc->getMaxValue(ADC_1), DEC);
#endif
    } else if (c == 's') { // start pdb, before pressing enter write the frequency in Hz
      uint32_t freq = Serial.parseInt();
      Serial.print("Start pdb with frequency ");
      Serial.print(freq);
      Serial.println(" Hz.");
      adc->adc0->stopPDB();
      adc->adc0->startPDB(freq); //frequency in Hz
#if defined(ADC_TEENSY_3_1)
      adc->adc1->stopPDB();
      adc->adc1->startPDB(freq); //frequency in Hz
#endif
    } else if (c == 'p') { // pbd stats
      Serial.print("Prescaler:");
      Serial.println( (PDB0_SC & 0x7000) >> 12 , HEX);
      Serial.print("Mult:");
      Serial.println( (PDB0_SC & 0xC) >> 2, HEX);
    }

  }



  /* fail_flag contains all possible errors,
      They are defined in  ADC_Module.h as

      ADC_ERROR_OTHER
      ADC_ERROR_CALIB
      ADC_ERROR_WRONG_PIN
      ADC_ERROR_ANALOG_READ
      ADC_ERROR_COMPARISON
      ADC_ERROR_ANALOG_DIFF_READ
      ADC_ERROR_CONT
      ADC_ERROR_CONT_DIFF
      ADC_ERROR_WRONG_ADC
      ADC_ERROR_SYNCH

      You can compare the value of the flag with those masks to know what's the error.
  */

  if (adc->adc0->fail_flag) {
    Serial.print("ADC0 error flags: 0x");
    Serial.println(adc->adc0->fail_flag, HEX);
    if (adc->adc0->fail_flag == ADC_ERROR_COMPARISON) {
      adc->adc0->fail_flag &= ~ADC_ERROR_COMPARISON; // clear that error
      Serial.println("Comparison error in ADC0");
    }
  }
#if defined(ADC_TEENSY_3_1)
  if (adc->adc1->fail_flag) {
    Serial.print("ADC1 error flags: 0x");
    Serial.println(adc->adc1->fail_flag, HEX);
    if (adc->adc1->fail_flag == ADC_ERROR_COMPARISON) {
      adc->adc1->fail_flag &= ~ADC_ERROR_COMPARISON; // clear that error
      Serial.println("Comparison error in ADC1");
    }
  }
#endif

  //digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));

  //delay(100);


}

void runFHT() {
  int k = 0;

  for (int i = 0 ; i < 512 ; i++) { // save 256 samples
    while (adc->isComplete(ADC_0) && adc->isComplete(ADC_1)); // wait for adc to be ready
    result = adc->readSynchronizedContinuous();
    // if using 16 bits and single-ended is necessary to typecast to unsigned,
    // otherwise values larger than 3.3/2 will be interpreted as negative
    result.result_adc0 = (uint16_t)result.result_adc0;
    block->data[i] = result.result_adc0;
    
    //value = result.result_adc0;

    Serial.print("Value ADC0: ");
    value2 = result.result_adc1;
    Serial.println(value * 3.3 / adc->getMaxValue(ADC_0), DEC);

    Serial.print("Value ADC1: ");
    //value2 = (uint16_t)adc->analogReadContinuous(ADC_1); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
    Serial.println(value2 * 3.3 / adc->getMaxValue(ADC_1), DEC);
    //Serial.println(value * 3.3 / adc->getMaxValue(ADC_0), DEC);
  }


//This function can be used to get the 256 datapoints(or is it 512) then call the updater()
}

static void copy_to_fft_buffer(void *destination, const void *source)
{
  const uint16_t *src = (const uint16_t *)source;
  uint32_t *dst = (uint32_t *)destination;

  for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
    *dst++ = *src++;  // real sample plus a zero for imaginary
  }
}

static void apply_window_to_fft_buffer(void *buffer, const void *window)
{
  int16_t *buf = (int16_t *)buffer;
  const int16_t *win = (int16_t *)window;;

  for (int i = 0; i < 256; i++) {
    int32_t val = *buf * *win++;
    //*buf = signed_saturate_rshift(val, 16, 15);
    *buf = val >> 15;
    buf += 2;
  }

}



/*This is where the actual Math occurs
* ideally this function could be passed in any block left or right channel 
*/
void updater(void)
{
  audio_block_t *block; //it addresses a block

  //block = receiveReadOnly(); //gets pointer to audio_block_struct This may be only 128 samples at a time thats why it does it twice
  if (!block) return;
  if (!prevblock) {
    prevblock = block;
    return;
  }
  
//void calcFFT(audio_block_t *block){
//copy_to_fft_buffer(buffer,block->data);

//remove next line
copy_to_fft_buffer(buffer, prevblock->data); //puts the audio_block_struct->data into fft buffer
  //copy_to_fft_buffer(buffer + 256, block->data); // looks like its putting 512 datapoints into the block->data


//pick a windowFunction
  //window = AudioWindowBlackmanNuttall256;
  //window = NULL;
  

if (window) apply_window_to_fft_buffer(buffer, window); //applies the window
  arm_cfft_radix4_q15(&fft_inst, buffer);  //DSP accelerated FFT
  // G. Heinzel's paper says we're supposed to average the magnitude
  // squared, then do the square root at the end.
  if (count == 0) {
    for (int i = 0; i < 128; i++) {
      uint32_t tmp = *((uint32_t *)buffer + i);
      uint32_t magsq = multiply_16tx16t_add_16bx16b(tmp, tmp);
      sum[i] = magsq / naverage;
    }
  } else {
    for (int i = 0; i < 128; i++) {
      uint32_t tmp = *((uint32_t *)buffer + i);
      uint32_t magsq = multiply_16tx16t_add_16bx16b(tmp, tmp);
      sum[i] += magsq / naverage;
    }
  }
  if (++count == naverage) {
    count = 0;
    for (int i = 0; i < 128; i++) {
      output[i] = sqrt_uint32_approx(sum[i]); //puts everything neatly in output array
    }
    outputflag = true; //sets the flag 
  }
  //release(prevblock);//dealloc, does this calloc(sizeof(audio_block_t)); ?
  prevblock = block;
}




bool available() {
  if (outputflag == true) {
    outputflag = false;
    return true;
  }
  return false;
}
float read(unsigned int binNumber) {
  if (binNumber > 127) return 0.0;
  return (float)(output[binNumber]) * (1.0 / 16384.0);
}

void averageTogether(uint8_t n) {
  if (n == 0) n = 1;
  naverage = n;
}
void windowFunction(const int16_t *w) {
  window = w;
}

// If you enable interrupts make sure to call readSingle() to clear the interrupt.
void adc0_isr() {
  adc->adc0->readSingle();
  //digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN) );
}

#if defined(ADC_TEENSY_3_1)
void adc1_isr() {
  adc->adc1->readSingle();
  digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN) );
}
#endif

// pdb interrupt is enabled in case you need it.
void pdb_isr(void) {
  PDB0_SC &= ~PDB_SC_PDBIF; // clear interrupt
  //digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN) );
}
