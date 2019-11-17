/**
/* CAN Controller.
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

/********** Interframe Space ***********/
#define INTERFRAME_SPACE                0
#define INTERFRAME_SPACE_INTERMISSION   1
#define INTERFRAME_SPACE_BUS_IDLE       2
/***************************************/

/**** Start of Frame ***/
#define START_OF_FRAME  3
/***********************/

/************* Arbitration *************/
#define ARBITRATION                     4
/*** Standard/Extended format fields ***/
#define ARBITRATION_IDENTIFIER_11_BIT   5
#define ARBITRATION_RTR                 6
/**** Extended format extra fields *****/
#define ARBITRATION_SRR                 7
#define ARBITRATION_IDE                 8
#define ARBITRATION_IDENTIFIER_18_BIT   9
/***************************************/

/************* Control *************/
#define CONTROL     10
/* Standard/Extended format fields */
#define CONTROL_IDE 11
#define CONTROL_r0  12
#define CONTROL_DLC 13
/*** Extended format extra fields **/
#define CONTROL_r1  14
/***********************************/

/*** Data ****/
#define DATA 15
/*************/

/******* CRC check ******/
#define CRC             16
#define CRC_SEQUENCE    17
#define CRC_DELIMITER   18
/************************/

/********** ACK *********/
#define ACK             19
#define ACK_SLOT        20
#define ACK_DELIMITER   21
/************************/

/****** End of Frame ****/
#define END_OF_FRAME    22
/************************/

/****** Bit Stuffing ****/
#define BIT_STUFFING    23
/************************/

/****** Error frame *****/
#define ERROR           24
#define ERROR_FLAG      25
#define ERROR_DELIMITER 26
/************************/

/***** Overload frame ******/
#define OVERLOAD           27
#define OVERLOAD_FLAG      28
#define OVERLOAD_DELIMITER 29
/***************************/

#define MAX_FRAME_SIZE 127

unsigned char currentFrameField    = INTERFRAME_SPACE;
unsigned char currentFrameSubField = INTERFRAME_SPACE_BUS_IDLE;
unsigned char prevFrameField;

unsigned char frameBuf[MAX_FRAME_SIZE];
unsigned char sampledBit;
unsigned char writingBit;
unsigned char previousBit;
unsigned char bitIndex = 0;
unsigned char bitCnt   = 0;
unsigned char hasError = 0;
unsigned char crcError = 0;
unsigned char isTransmitter      = 0;
unsigned char bitFieldIndex      = 0;
unsigned char overloadFrameCnt   = 0;
unsigned char samePolarityBitCnt = 1;

unsigned char dlc;
unsigned char crc[] = "000000000000000";
const unsigned char generatorPolynomial[] = "100010110011001"; // 0x4599

typedef struct{
    unsigned char idA[11];
    unsigned char rtr;
    unsigned char srr;
    unsigned char ide;
    unsigned char idB[18];
    unsigned char r1;
    unsigned char r0;
    unsigned char dlc[4];
    unsigned char data[64];
    unsigned char crc[15];
} Frame;

Frame frame;
Frame receivedframe;

const unsigned char errorOverloadFrame[] = "00000011111111";
bool samplePoint = false;
bool writingPoint = false;
const byte hardSyncIntPin = 2;
const byte resyncIntPin   = 3;
volatile bool hardSyncBool  = false;
volatile bool resyncBool    = false;
volatile unsigned char currentSegment = PROP_SEG;
volatile unsigned char tqSegCnt       = 0;
volatile unsigned char phaseError     = 0;
volatile unsigned char phaseSeg1Len   = PHASE_SEG1_LEN;
volatile unsigned char phaseSeg2Len   = PHASE_SEG2_LEN;

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
    hardSyncBool = false;
    resyncBool = false;
    if (writingPoint && isTransmitter) {
        encoderStateMachine();
        // Write bit.
    }
    if (samplePoint) {
        // Sample bit.
        decoderStateMachine();
    }
}

void checkBitStuffing() {
    sampledBit == previousBit ? samePolarityBitCnt++ : (samePolarityBitCnt = 1);
    previousBit = sampledBit;
    if (samePolarityBitCnt == 5) {
        printf("Destuffing next bit at index %d.\n", bitIndex);
        samePolarityBitCnt = 1;
        prevFrameField = currentFrameField;
        currentFrameField = BIT_STUFFING;
    }
}

void bitStuffingStateMachine() {
    if (sampledBit == previousBit) {
        printf("Bit stuffing error at index %d.\n", bitIndex);
        hasError = 1;
    } else {
        printf("Stuffed bit: %c\n", sampledBit);
        samePolarityBitCnt = 1;
        previousBit = sampledBit;
        currentFrameField = prevFrameField;
    }
}

void computeCrcSequence() {
    int i, j;
    unsigned char nxtBit, crcNxt;
    
    nxtBit = sampledBit;
    crcNxt = nxtBit ^ crc[0];

    // Shift left by one position.
    for (j = 0; j < 14; j++) {
        crc[j] = crc[j+1];
    }
    crc[14] = '0';

    if (crcNxt) {
        for (j = 0; j < 15; j++) {
          crc[j] = crc[j] ^ generatorPolynomial[j] ? '1' : '0';
        }
    }
}

int validateCrcSequence() {
    int j;
    for (j = 0; j < 15 && !crcError; j++) {
        crcError = crc[j] != receivedframe.crc[j];
    }
}

void interframeSpaceStateMachine() {
    printf("Interframe space\n");
    switch(currentFrameSubField) {
        case INTERFRAME_SPACE_INTERMISSION:
            if (sampledBit == '0') {
                if (bitCnt == 2) {
                    /***
                    /* If a CAN node has a message waiting for transmission and it samples a
                    /* dominant bit at the third bit of INTERMISSION, it will interpret this as
                    /* a START OF FRAME bit, and, with the next bit, start transmitting its message
                    /* with the first bit of its IDENTIFIER without first transmitting a START OF FRAME bit
                    /* and without becoming receiver.
                    /***/
                    currentFrameField = START_OF_FRAME;
                    decoderStateMachine();
                } else {
                    overloadFrameCnt++;
                    if (overloadFrameCnt <= 2) {
                        // Overload frame.
                        currentFrameField = OVERLOAD;
                        currentFrameSubField = OVERLOAD_FLAG;
                        if (bitCnt == 0) {
                            bitCnt = 6; // Overload flag length.
                            decoderStateMachine();
                        } else {
                            bitCnt = 6; // Overload flag length.
                        }
                    } else {
                        printf("Overload error: ");
                        printf("Maximum of 2 Overload frames allowed to delay Data/Remote frame.\n");
                        hasError = 1;
                    }
                }
            } else if (sampledBit == '1') {
                bitCnt++;
                if (bitCnt == 3) {
                    bitCnt = 0;
                    // TODO: Create logic to decide wheter or not to transmit a frame after receiving one.
                    if ((isTransmitter = 1)) { // Forcing transmission after receiving a frame.
                        hardSync();
                        currentFrameField = START_OF_FRAME;
                        frame = receivedframe;  // Send back the received frame.
                    } else {
                        currentFrameField = INTERFRAME_SPACE;
                        currentFrameSubField = INTERFRAME_SPACE_BUS_IDLE;
                    }
                }
            } else {
                printf("Interframe space error: ");
                printf("Expecting 3 recessive bits during Intermission.\n");
                hasError = 1;
            }
            break;
        case INTERFRAME_SPACE_BUS_IDLE:
            if (sampledBit == '0') {
                hardSync();
                currentFrameField = START_OF_FRAME;
                decoderStateMachine();
            }
            break;
        default:
            printf("Interframe space error: invalid sub-frame field.\n");
            return;
    }
};

void startOfFrameStateMachine() {
    printf("Start of Frame\n");
    int j;
    dlc      = 0;
    bitCnt   = 0;
    bitIndex = 0;
    crcError = 0;
    hasError = 0;
    receivedframe.ide ?: 0;  // Assuming Standard format when in Receiver mode.
    bitFieldIndex     = 0;
    overloadFrameCnt   = 0;
    samePolarityBitCnt = 1;
    previousBit = sampledBit;
    frameBuf[bitIndex++] = sampledBit;
    currentFrameField = ARBITRATION;
    currentFrameSubField = ARBITRATION_IDENTIFIER_11_BIT;
    // Reset CRC sequence.
    for (j = 0; j < 15; j++) {
        crc[j] = '0';
    }
    computeCrcSequence();
};

void arbitrationStateMachine() {
    printf("Arbitration\n");
    int skipState = 0;
    switch (currentFrameSubField) {
        case ARBITRATION_IDENTIFIER_11_BIT:
            frameBuf[bitIndex++] = sampledBit;
            receivedframe.idA[bitFieldIndex++] = sampledBit;
            if (bitFieldIndex == 11) {
                bitFieldIndex = 0;
                if (isTransmitter) {
                    currentFrameSubField = (receivedframe.ide == '0') ? ARBITRATION_RTR : ARBITRATION_SRR;
                } else {
                    currentFrameSubField = ARBITRATION_RTR; // Assuming Standard format.
                }
            }
            break;
        case ARBITRATION_RTR:
            receivedframe.rtr = sampledBit;
            frameBuf[bitIndex++] = sampledBit;
            currentFrameField = CONTROL;
            if (receivedframe.ide == '1') currentFrameSubField = CONTROL_r1;  // Extended format.
            else currentFrameSubField = CONTROL_IDE;    // Standard format.
            break;
        case ARBITRATION_SRR: // Empty transition to IDE bit field.
            currentFrameSubField = ARBITRATION_IDE;
            if (isTransmitter) {
                receivedframe.srr = sampledBit;
            } else {
                receivedframe.srr = receivedframe.rtr;
                skipState = 1; // Prevent from doing further evaluation without having sampled a new bit.
                arbitrationStateMachine();
            }
            break;
        case ARBITRATION_IDE: // Empty transition to 18 bit identifier.
            bitFieldIndex = 0;
            currentFrameSubField = ARBITRATION_IDENTIFIER_18_BIT;
            break;
        case ARBITRATION_IDENTIFIER_18_BIT:
            frameBuf[bitIndex++] = sampledBit;
            receivedframe.idB[bitFieldIndex++] = sampledBit;
            if (bitFieldIndex == 18) currentFrameSubField = ARBITRATION_RTR; // Assuming Standard format.
            break;
        default:
            printf("Arbitration error: invalid sub-frame field.\n");
            return;
    }
    // Compute CRC sequence and check bit stuffing.
    if (!hasError && !skipState) {
        computeCrcSequence();
        checkBitStuffing();
    }
}

void controlStateMachine() {
    printf("Control\n");
    int skipState = 0;
    switch (currentFrameSubField) {
        case CONTROL_IDE:
            receivedframe.ide = sampledBit;
            frameBuf[bitIndex++] = sampledBit;
            if (receivedframe.ide == '0') { // Standard format.
                currentFrameSubField = CONTROL_r0;
            } else { // Extended format.
                currentFrameField = ARBITRATION;
                currentFrameSubField = ARBITRATION_SRR;
                skipState = 1; // Prevent from doing further evaluation without having sampled a new bit.
                arbitrationStateMachine();
            }
            break;
        case CONTROL_r1:
            receivedframe.r1 = sampledBit;
            frameBuf[bitIndex++] = sampledBit;
            currentFrameSubField = CONTROL_r0;
            break;
        case CONTROL_r0:
            receivedframe.r0 = sampledBit;
            frameBuf[bitIndex++] = sampledBit;
            currentFrameSubField = CONTROL_DLC;
            bitFieldIndex = 0;
            bitCnt = 4;
            break;
        case CONTROL_DLC:
            frameBuf[bitIndex++] = sampledBit;
            receivedframe.dlc[bitFieldIndex++] = sampledBit;
            bitCnt--;
            dlc += ((sampledBit - '0') << bitCnt);
            if (bitCnt == 0) {
                dlc = min(dlc, 8); // Maximum number of data bytes: 8.
                printf("%d\n", dlc);
                bitFieldIndex = 0;
                if (receivedframe.rtr == '0' && dlc != 0) currentFrameField = DATA; // Data frame.
                else {
                    // Remote frame. Transitioning to CRC sequence. 
                    bitCnt = 15;
                    currentFrameField = CRC;
                    currentFrameSubField = CRC_SEQUENCE;
                }
            }
            break;
        default:
            printf("Control error: invalid sub-frame field.\n");
            return;
    }
    // Compute CRC sequence and check bit stuffing.
    if (!hasError && !skipState) {
        computeCrcSequence();
        checkBitStuffing();
    }
}

void dataStateMachine() {
    printf("Data\n");
    frameBuf[bitIndex++] = sampledBit;
    receivedframe.data[bitFieldIndex++] = sampledBit;
    bitCnt++;
    if (bitCnt == 8 * dlc) {
        bitCnt = 15;
        bitFieldIndex = 0;
        currentFrameField = CRC;
        currentFrameSubField = CRC_SEQUENCE;
    }
    // Compute CRC sequence and check bit stuffing.
    if (!hasError) {
        computeCrcSequence();
        checkBitStuffing();
    }
}

void crcStateMachine() {
    printf("CRC\n");
    switch (currentFrameSubField) {
        case CRC_SEQUENCE:
            frameBuf[bitIndex++] = sampledBit;
            receivedframe.crc[bitFieldIndex++] = sampledBit;
            bitCnt--;
            if (bitCnt == 0) {
                validateCrcSequence();
                currentFrameSubField = CRC_DELIMITER;
            }
            // Check bit stuffing.
            if (!hasError)
                checkBitStuffing();
            break;
        case CRC_DELIMITER:
            if (sampledBit != '1') {
                printf("CRC delimiter error: ");
                printf("Must be a recessive bit.\n");
                hasError = 1;
            } else {
                frameBuf[bitIndex++] = sampledBit;
                currentFrameField = ACK;
                currentFrameSubField = ACK_SLOT;
            }
            break;
        default:
            printf("CRC error: invalid sub-frame field.\n");
            return;
    }
}

void ackStateMachine() {
    printf("ACK\n");
    switch (currentFrameSubField) {
        case ACK_SLOT:
            if (sampledBit == '1') { // None of the stations has acknowledged the message.
                printf("Acknowledgment error: ");
                printf("Failed to validade the message correctly.\n");
                hasError = 1;
            } else {
                frameBuf[bitIndex++] = sampledBit;
                currentFrameSubField = ACK_DELIMITER;
            }
            break;
        case ACK_DELIMITER:
            if (crcError) {
                printf("CRC error: ");
                printf("The calculated result is not the same as that received in the CRC sequence.\n");
                hasError = 1;
            } else if (sampledBit != '1') {
                printf("Acknowledgment delimiter error: ");
                printf("Must be a recessive bit.\n");
                hasError = 1;
            } else {
                bitCnt = 0;
                frameBuf[bitIndex++] = sampledBit;
                currentFrameField = END_OF_FRAME;
            }
            break;
        default:
            printf("Ack error: invalid sub-frame field.\n");
            return;
    }
}

void endOfFrameStateMachine() {
    printf("End of frame\n");
    if (sampledBit == '1') {
        frameBuf[bitIndex++] = sampledBit;
        bitCnt++;
        if (bitCnt == 7) {
            bitCnt = 0;
            printFrameInfo(receivedframe);
            currentFrameField = INTERFRAME_SPACE;
            currentFrameSubField = INTERFRAME_SPACE_INTERMISSION;
            isTransmitter = 0;  // Disabling transmission.
        }
    } else {
        printf("End of frame error: ");
        printf("Expecting a flag sequence consisting of 7 recessive bits.\n");
        hasError = 1;
    }
}

void errorStateMachine() {
    printf("Error frame\n");
    switch(currentFrameSubField) {
        case ERROR_FLAG:
            if (sampledBit == '0') {
                bitCnt++;
            } else if (sampledBit == '1') {
                if (bitCnt < 6) {
                    printf("Error flag error: ");
                    printf("Expecting at least 6 equal bits during error flag.\n");
                    hasError = 1;
                } else if (bitCnt >= 6 && bitCnt <= 12) {
                    bitCnt = 7;
                    currentFrameSubField = ERROR_DELIMITER;
                }
            }
            if (bitCnt > 12) {
                printf("Error flag error: ");
                printf("Expecting maximum of 12 equal bits during error flag.\n");
                hasError = 1;
            }
            break;
        case ERROR_DELIMITER:
            if (sampledBit == '1') {
                bitCnt--;
                if (bitCnt == 0) {
                    currentFrameField = INTERFRAME_SPACE;
                    currentFrameSubField = INTERFRAME_SPACE_INTERMISSION;
                }
            } else {
                printf("Error delimiter error: ");
                printf("Expecting 8 recessive bits during error delimiter.\n");
                hasError = 1;
            }
            break;
        default:
            printf("Error frame error: invalid sub-frame field.\n");
            return;
    }
}

void overloadStateMachine() {
    printf("Overload frame\n");
    switch(currentFrameSubField) {
        case OVERLOAD_FLAG:
            if (sampledBit == '0') {
                bitCnt--;
                if (bitCnt == 0) {
                    bitCnt = 8;
                    currentFrameSubField = OVERLOAD_DELIMITER;
                }
            } else if (sampledBit == '1') {
                printf("Overload flag error: ");
                printf("Expecting 6 dominant bits during overload flag.\n");
                hasError = 1;
            }
            break;
        case OVERLOAD_DELIMITER:
            if (sampledBit == '1') {
                bitCnt--;
                if (bitCnt == 0) {
                    currentFrameField = INTERFRAME_SPACE;
                    currentFrameSubField = INTERFRAME_SPACE_INTERMISSION;
                }
            } else {
                printf("Overload delimiter error: ");
                printf("Expecting 8 recessive bits during overload delimiter.\n");
                hasError = 1;
            }
            break;
        default:
            printf("Overload frame error: invalid sub-frame field.\n");
            return;
    }
}

void decoderStateMachine() {
    switch(currentFrameField) {
        case INTERFRAME_SPACE:
            interframeSpaceStateMachine();
            break;
        case START_OF_FRAME:
            startOfFrameStateMachine();
            break;
        case ARBITRATION:
            arbitrationStateMachine();
            break;
        case CONTROL:
            controlStateMachine();
            break;
        case DATA:
            dataStateMachine();
            break;
        case CRC:
            crcStateMachine();
            break;
        case ACK:
            ackStateMachine();
            break;
        case END_OF_FRAME:
            endOfFrameStateMachine();
            break;
        case BIT_STUFFING:
            bitStuffingStateMachine();
            break;
        case ERROR:
            errorStateMachine();
            break;
        case OVERLOAD:
            overloadStateMachine();
            break;
        default:
            printf("Decoder error: invalid frame field.\n");
            break;
    }
    if (hasError) {
        printFrameInfo(receivedframe);
        printf("Start receiving error flag...\n");
        bitCnt = 0;
        hasError = 0;
        isTransmitter = 1;
        bitFieldIndex = 0;
        currentFrameField = ERROR;
        currentFrameSubField = ERROR_FLAG;
    }
}

void encoderStateMachine() {
    switch (currentFrameField) {
        case START_OF_FRAME:
            printFrameInfo(frame);
            writingBit = '0';
            break;
        case ARBITRATION:
            switch (currentFrameSubField) {
                case ARBITRATION_IDENTIFIER_11_BIT:
                    writingBit = frame.idA[bitFieldIndex];
                    break;
                case ARBITRATION_RTR:
                    writingBit = frame.rtr;
                    break;
                case ARBITRATION_SRR:
                    writingBit = frame.srr;
                    break;
                case ARBITRATION_IDE:
                    writingBit = frame.ide;
                    break;
                case ARBITRATION_IDENTIFIER_18_BIT:
                    writingBit = frame.idB[bitFieldIndex];
                    break;
            }
            break;
        case CONTROL:
            switch (currentFrameSubField) {
                case CONTROL_IDE:
                    writingBit = frame.ide;
                    break;
                case CONTROL_r1:
                    writingBit = frame.r1;
                    break;
                case CONTROL_r0:
                    writingBit = frame.r0;
                    break;
                case CONTROL_DLC:
                    writingBit = frame.dlc[bitFieldIndex];
                    break;
            }
            break;
        case DATA:
            writingBit = frame.data[bitFieldIndex];
            break;
        case CRC:
            switch (currentFrameSubField) {
                case CRC_SEQUENCE:
                    writingBit = frame.crc[bitFieldIndex];
                    break;
                case CRC_DELIMITER:
                    writingBit = '1';
                    break;
            }
            break;
        case ACK:
            switch (currentFrameSubField) {
                case ACK_SLOT:
                    // Forcing ACK for the sake of testing.
                    // This value should be set to '1' by the encoder.
                    writingBit = '0';
                    break;
                case ACK_DELIMITER:
                    writingBit = '1';
                    break;
            }
            break;
        case END_OF_FRAME:
            writingBit = '1';
            break;
        case BIT_STUFFING:
            writingBit = (!(previousBit - '0')) + '0'; // The opposite polarity from the previous bit.
            break;
        case ERROR:
        case OVERLOAD:
            writingBit = errorOverloadFrame[bitFieldIndex++];
            if (bitFieldIndex == 14) {
                bitFieldIndex = 0;
                isTransmitter = 0;
            }
            break;
        default:
            printf("Encoder error: invalid frame field %d.\n", currentFrameField);
            break;
    }
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


void printFrameInfo(Frame frame) {
    int i;
    printf("\n------- FRAME INFO -------\n");
    printf("ID (11-bit): ");
    for (i = 0; i < 11; i++) {
        printf("%c", frame.idA[i]);
    }
    printf("\n");

    printf("RTR: %c\n", frame.rtr);

    printf("IDE: %c\n", frame.ide);

    if (frame.ide == '1') {
        printf("SRR: %c\n", frame.srr);
        printf("ID (18-bit): ");
        for (i = 0; i < 18; i++) {
            printf("%c", frame.idB[i]);
        }
        printf("\n");
        printf("r1: %c\n", frame.r1);
    }

    printf("r0: %c\n", frame.r0);

    printf("DLC: ");
    for (i = 0; i < 4; i++) {
        printf("%c", frame.dlc[i]);
    }
    printf("\n");


    if (frame.rtr == '0') {
        printf("Data: ");
        for (i = 0; i < 8 * dlc; i++) {
            printf("%c", frame.data[i]);
        }
        printf("\n");
    }

    printf("CRC: ");
    for (i = 0; i < 15; i++) {
        printf("%c", frame.crc[i]);
    }
    printf("\n");

    printf("Frame (destuffed): ");
    for (i = 0; i < bitIndex; i++) {
        printf("%c", frameBuf[i]);
    }

    printf("\n\n");
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
