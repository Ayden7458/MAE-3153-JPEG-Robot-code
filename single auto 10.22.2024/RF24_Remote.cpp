
#include "RF24_Remote.h"

RF24 radio(PIN_SPI_CE, PIN_SPI_CSN);

    // Radio Address: each team should choose a unique address string
    // I suggest "TM0xx" where xx is your team number  ex:  TM001, TM014
const byte addresses[6] = "TM03";
    // Radio Channel: each team should choose a unique channel number between 20 and 127
    // I suggest you use:   (TeamNumber * 5) + 15  ex:  team 4 would use channel 35
const int RF24channel = 35;  //each team should choose a unique channel

int nrfDataRead[8];
bool nrfComplete = false;

bool nrf24L01Setup() {
  // NRF24L01
  if (radio.begin()) {                      // initialize RF24
    radio.setPALevel(RF24_PA_MAX);      // set power amplifier (PA) level
    radio.setDataRate(RF24_1MBPS);      // set data rate through the air
    radio.setRetries(0, 15);            // set the number and delay of retries
    radio.openWritingPipe(addresses);   // open a pipe for writing
    radio.openReadingPipe(1, addresses);// open a pipe for reading
    radio.setChannel(RF24channel);    
    radio.startListening();             // start monitoringtart listening on the pipes opened

    FlexiTimer2::set(20, 1.0 / 1000, checkNrfReceived); // call every 20 1ms "ticks"
    FlexiTimer2::start();

    return true;
  }
  return false;
}

void checkNrfReceived() {
  delayMicroseconds(1000);
  if (radio.available()) {             // if receive the data
    while (radio.available()) {         // read all the data
      radio.read(nrfDataRead, sizeof(nrfDataRead));   // read data
    }
    nrfComplete = true;
    return;
  }
  nrfComplete = false;
}

bool getNrf24L01Data()
{
  return nrfComplete;
}

void clearNrfFlag() {
  nrfComplete = 0;
}

void updateCarActionByNrfRemote() {
  
  float speed = nrfDataRead[1] /1023.0;  // 1023.0 not 1023 

  int rightturn = nrfDataRead[2] - 512;
  int forward = -(nrfDataRead[3] - 512);
  int leftmotor, rightmotor;

  if ( nrfDataRead[7] == 0) {
    forward = -forward;
  }

  rightturn=rightturn * speed;
  //forward = forward * speed;

  leftmotor = (forward + rightturn) / 2;
  rightmotor = (forward - rightturn) / 2;
    
  motorRun(leftmotor, rightmotor);

  if (nrfDataRead[4] == 0) {
    setBuzzer(true);
  }
  else {
    setBuzzer(false);
  }
}

void resetNrfDataBuf() {
  nrfDataRead[0] = 0;
  nrfDataRead[1] = 0;
  nrfDataRead[2] = 512;
  nrfDataRead[3] = 512;
  nrfDataRead[4] = 1;
  nrfDataRead[5] = 1;
  nrfDataRead[6] = 1;
  nrfDataRead[7] = 1;
}

u8 updateNrfCarMode() {
  // nrfDataRead [5 6 7] --> 111
  return ((nrfDataRead[5] == 1 ? 1 : 0) << 2) | ((nrfDataRead[6] == 1 ? 1 : 0) << 1) | ((nrfDataRead[7] == 1 ? 1 : 0) << 0);
}
