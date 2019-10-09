// Defining segments.
#define SYNC_SEG 0
#define PROP_SEG 1
#define PHASE_SEG1 2
#define PHASE_SEG2 3

/**
/* Bit timing configuration parameters.
/**/

// Bit and segments lengths (time quanta).
#define SYNC_SEG_LEN 1
#define PROP_SEG_LEN 1
#define PHASE_SEG1_LEN 7
#define PHASE_SEG2_LEN 7
#define BIT_LEN (SYNC_SEG_LEN + PROP_SEG_LEN + PHASE_SEG1_LEN + PHASE_SEG2_LEN)

#define SJW 1 // Synchronization Jump Width (dafault: 1 TQ).
#define BAUD_RATE 10 // Baud rate (default: 10 bps).

// Defining time quantum length (milliseconds).
// For a 10bps baud rate and 16 TQ bit length, time quantum must be 6,25 milliseconds.
#define TQ 1000.0/(BAUD_RATE*BIT_LEN)

// Another way to define TQ:
//#define BRP 50000 // Baud rate prescaler (default: 50000).
//#define OSC_FRQ 16000000 // Oscillator frequency (default: 16 MHz).
//#define TQ (2/OSC_FRQ)*BRP

#define TRUE 1
#define FALSE 0

bool samplePoint = false;
bool writingPoint = false;
const byte hardSyncIntPin = 2;
const byte resyncIntPin = 3;
volatile unsigned char currentSegment = PROP_SEG;
volatile unsigned char tqCnt = 0;

void setup() {
  Serial.begin(4800);
  pinMode(hardSyncIntPin, INPUT_PULLUP);
  pinMode(resyncIntPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hardSyncIntPin), hardSync, FALLING);
  attachInterrupt(digitalPinToInterrupt(resyncIntPin), resync, FALLING);
}

void loop() {
  plotValues();
  bitTimingStateMachine();
}

void bitTimingStateMachine() {
  switch(currentSegment) {
    case SYNC_SEG:
      if(tqCnt == SYNC_SEG_LEN) {
        tqCnt = 0;
        writingPoint = false;
        currentSegment = PROP_SEG;
      }
      break; 
    case PROP_SEG:
      if(tqCnt == PROP_SEG_LEN) {
        tqCnt = 0;
        currentSegment = PHASE_SEG1;
      }
      break;
    case PHASE_SEG1:
      if(tqCnt == PHASE_SEG1_LEN) {
        // TODO: Sample bit.
        samplePoint = true;
        tqCnt = 0;
        currentSegment = PHASE_SEG2;
      }
      break;
    case PHASE_SEG2:
      if(samplePoint) samplePoint = false;
      if(tqCnt == PHASE_SEG2_LEN) {
        writingPoint = true;
        tqCnt = 0;
        currentSegment = SYNC_SEG;
      }
      break;
    default:
      Serial.println("Unknown segment!");
      break;
  }
  delay(TQ);
  tqCnt++;
}

void hardSync() {
  currentSegment = PROP_SEG;
  tqCnt = 0;
}

void resync() {
  switch(currentSegment){
    case PROP_SEG:
    case PHASE_SEG1:
      // TODO: Lengthen segment to compensate phase error (max. SJW).
      break;
    case PHASE_SEG2:
      // TODO: Shorten segment to compensate phase error (max. SJW).
      break;
    default:
      Serial.println("Unknown segment!");
      break;
  }
}

void plotValues() {
  for (int i = 0; i < 5; i++) {
    Serial.print(writingPoint ? (TRUE + 0) : (FALSE + 0));
    Serial.print(' ');
    Serial.print(samplePoint ? (TRUE - 2) : (FALSE - 2));
    Serial.print(' ');
    Serial.print(currentSegment + 2);
    Serial.print('\n');
  }
}
