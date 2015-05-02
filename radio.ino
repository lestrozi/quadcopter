// TODO: bug: auto-calibrating not working (possibly due to interference with motors and/or physical shaking)
// TODO: possible bug: this is working without "locks": we might be reading a value while parts of its bytes is being rewritten. This shouldn't work, but workaround (bUpdatedFlags) to fix that caused more problems than solved

#include <ByteBuffer.h>
#include <MemoryFree.h>
#include <PinChangeInt.h>

#define CH5_NO_SIGNAL_1 1200
#define CH5_NO_SIGNAL_2 1700

#define MIN_VAL 800
#define MAX_VAL 2300

uint8_t pins[NUM_CHANNELS]={ CH1, CH2, CH3, CH4, CH5, CH6 };
volatile int chvalue[NUM_CHANNELS] = { 0, 0, 0, 0, 0, 0 };
volatile unsigned long rising[NUM_CHANNELS]={ 0, 0, 0, 0, 0, 0 };
//volatile int chmin[NUM_CHANNELS]={ 1200, 1200, 1200, 1200, 1200, 1200 };
//volatile int chmax[NUM_CHANNELS]={ 1800, 1800, 1800, 1800, 1800, 1800 };
volatile int chmin[NUM_CHANNELS]={ 916, 1064, 916, 840, 868, 852 };
volatile int chmax[NUM_CHANNELS]={ 2024, 2208, 2036, 1928, 1800, 1800 };
volatile uint8_t bUpdatedFlags;
int discardValues = 0;


void initRadio() {
  for(int i=0; i<NUM_CHANNELS; i++) {
    pinMode(pins[i], INPUT);
    digitalWrite(pins[i], HIGH);

//    //PCintPort::attachInterrupt(pins[i], &chint, CHANGE);
    PCintPort::attachInterrupt(pins[i], &chrising, RISING);
  }
}

int getNormalizedRadioValue(int i) {
  /*
  if (i == 1)
    return chvalue[1];
  else if (i == 0)
    return chmin[1];
  else if (i == 2)
    return chmax[1];
  else if (i == 3)
    return map(chvalue[1], chmin[1], chmax[1], 0, 1000);
  */
  //return chvalue[i];
  return map(chvalue[i], chmin[i], chmax[i], 0, 1000);
}

int indexFromPin(int pin) {
  for(int i=0; i<NUM_CHANNELS; i++) {
    if (pins[i] == pin)
      return i;
  }  
  
  return -1;
}

boolean isUpdatedValue(int i) {
  return (bUpdatedFlags & (1 << i));
}

void clearUpdates() {
  bUpdatedFlags = 0;
}

void chrising() {
  uint8_t interrupted_pin = PCintPort::arduinoPin;
  uint8_t state = PCintPort::pinState;

  unsigned long now = micros();
  
  int pinIndex = indexFromPin(interrupted_pin);
  
  rising[pinIndex] = now;

  PCintPort::detachInterrupt(interrupted_pin);
  PCintPort::attachInterrupt(interrupted_pin, &chfalling, FALLING);
}

void chfalling() {
  uint8_t interrupted_pin = PCintPort::arduinoPin;
  uint8_t state = PCintPort::pinState;
  
  unsigned long now = micros();
  
  int pinIndex = indexFromPin(interrupted_pin);
  
  int newVal = now - rising[pinIndex];

  if (newVal < MIN_VAL)
    goto exit;
  if (newVal > MAX_VAL)
    goto exit;

  if (rising[pinIndex] != 0)
    chvalue[pinIndex] = newVal;

  if (interrupted_pin == CH5)
    lostSignal = ((newVal > CH5_NO_SIGNAL_1) && (newVal < CH5_NO_SIGNAL_2));

  #ifdef CONSTANT_CALIBRATING
  if (discardValues < 100) {
    discardValues++;
  }
  else {
    if (newVal < chmin[pinIndex])
      chmin[pinIndex] = newVal;
    if (newVal > chmax[pinIndex])
      chmax[pinIndex] = newVal;
  }
  #endif

  exit:  
  bUpdatedFlags |= (1 << pinIndex);
  
  PCintPort::detachInterrupt(interrupted_pin);
  PCintPort::attachInterrupt(interrupted_pin, &chrising, RISING);
}

/*
void chint() {
  uint8_t interrupted_pin = PCintPort::arduinoPin;
  uint8_t state = PCintPort::pinState;
  
  unsigned long now = micros();
  
  int pinIndex = indexFromPin(interrupted_pin);
  
  switch(state) {
    case HIGH:
      rising[pinIndex] = now;
      break;
      
    case LOW:
      int newVal = now - rising[pinIndex];
    
      if (rising[pinIndex] != 0)
        chvalue[pinIndex] = newVal;
      
      if (interrupted_pin == CH5)
        lostSignal = ((newVal > CH5_NO_SIGNAL_1) && (newVal < CH5_NO_SIGNAL_2));

      #ifdef CONSTANT_CALIBRATING
      if (discardValues < 100) {
        discardValues++;
      }
      else {
        if (newVal < chmin[pinIndex])
          chmin[pinIndex] = newVal;
        if (newVal > chmax[pinIndex])
          chmax[pinIndex] = newVal;
      }
      #endif

      break;
  }
}
*/
