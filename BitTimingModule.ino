/**
/* Bit timing module.
/**/

// Defining segments.
#define SYNC_SEG 0
#define PROP_SEG 1
#define PHASE_SEG1 2
#define PHASE_SEG2 3

// Bit and segments lengths (time quanta).
#define SYNC_SEG_LEN 1
#define PROP_SEG_LEN 1
#define PHASE_SEG1_LEN 8
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

volatile bool hardSyncBool = false;
volatile bool resyncBool = false;
bool samplePoint = false;
bool writingPoint = false;
const byte hardSyncIntPin = 2;
const byte resyncIntPin = 3;
volatile unsigned char currentSegment = PROP_SEG;
volatile unsigned char tqSegCnt = 0;
volatile unsigned char phaseError = 0;
volatile unsigned char phaseSeg1Len = PHASE_SEG1_LEN;
volatile unsigned char phaseSeg2Len = PHASE_SEG2_LEN;

void setup() {
  Serial.begin(2400);
  pinMode(hardSyncIntPin, INPUT_PULLUP);
  pinMode(resyncIntPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hardSyncIntPin), hardSync, FALLING);
  attachInterrupt(digitalPinToInterrupt(resyncIntPin), resync, FALLING);
}

void loop() {
  plotValues();
  bitTimingStateMachine();
  delay(TQ);
  tqSegCnt++;
}

void bitTimingStateMachine() {
  switch(currentSegment) {
    case SYNC_SEG:
      if(tqSegCnt == SYNC_SEG_LEN) {
        tqSegCnt = 0;
        writingPoint = false; // Disable writing point.
        currentSegment = PROP_SEG;
      }
      break; 
    case PROP_SEG:
      if(tqSegCnt == PROP_SEG_LEN) {
        tqSegCnt = 0;
        writingPoint = false;
        currentSegment = PHASE_SEG1;
      }
      break;
    case PHASE_SEG1:
      if(tqSegCnt == phaseSeg1Len) {
        samplePoint = true; // Enable sample point.
        tqSegCnt = 0;
        currentSegment = PHASE_SEG2;
        restoreSegsDefaultLen(); // Restore PHASE_SEG1 default lenght.
      }
      break;
    case PHASE_SEG2:
      if(samplePoint) samplePoint = false; // Disable sample point.
      if(tqSegCnt == phaseSeg2Len) {
        writingPoint = true; // Enable writing point.
        tqSegCnt = 0;
        currentSegment = SYNC_SEG;
        restoreSegsDefaultLen(); // Restore PHASE_SEG2 default lenght.
      }
      break;
    default:
      Serial.println("Unknown segment!");
      break;
  }
}

void restoreSegsDefaultLen() {
  phaseSeg1Len = PHASE_SEG1_LEN;
  phaseSeg2Len = PHASE_SEG2_LEN;
  hardSyncBool = false;
  resyncBool = false;
}

void hardSync() {
  hardSyncBool = true;
  writingPoint = true;
  currentSegment = PROP_SEG;
  tqSegCnt = 0;
}

void resync() {
  resyncBool = true;
  switch(currentSegment){
    case SYNC_SEG:
      break;
    case PROP_SEG:
      // Lengthen PHASE_SEG1 to compensate phase error (max. SJW).
      phaseError = min(tqSegCnt + SYNC_SEG_LEN, PROP_SEG_LEN);
      phaseSeg1Len = PHASE_SEG1_LEN + ((phaseError <= SJW) ? phaseError : SJW);
    case PHASE_SEG1:
      // Lengthen segment to compensate phase error (max. SJW).
      phaseError = min((tqSegCnt + PROP_SEG_LEN + SYNC_SEG_LEN), PHASE_SEG1_LEN);
      phaseSeg1Len = PHASE_SEG1_LEN + ((phaseError <= SJW) ? phaseError : SJW);
      break;
    case PHASE_SEG2:
      // Shorten segment to compensate phase error (max. SJW).
      phaseError = min((PHASE_SEG2_LEN - tqSegCnt), PHASE_SEG2_LEN);
      phaseSeg2Len = PHASE_SEG2_LEN - ((phaseError <= SJW) ? phaseError : SJW);
      break;
    default:
      Serial.println("Unknown segment!");
      break;
  }
}

void plotValues() {
  for (int i = 0; i < 5; i++) {
    Serial.print(tqSegCnt + 6);
    Serial.print(' ');
    Serial.print(writingPoint ? (TRUE + 0) : (FALSE + 0));
    Serial.print(' ');
    Serial.print(samplePoint ? (TRUE - 2) : (FALSE - 2));
    Serial.print(' ');
    Serial.print(currentSegment + 2);
    Serial.print(' ');
    Serial.print(hardSyncBool ? (TRUE - 4) : (FALSE - 4));
    Serial.print(' ');
    Serial.print(resyncBool ? (TRUE - 6) : (FALSE - 6));
    Serial.print(' ');
    Serial.print('\n');
  }
}
