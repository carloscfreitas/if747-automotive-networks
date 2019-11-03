#include <stdio.h>
#include <math.h>

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

#define MAX_FRAME_SIZE 127

unsigned char currentFrameField    = INTERFRAME_SPACE;
unsigned char currentFrameSubField = INTERFRAME_SPACE_BUS_IDLE;
unsigned char prevFrameField;

unsigned char frameBuf[MAX_FRAME_SIZE];
unsigned char sampledBit;
unsigned char previousBit;
unsigned char bitIndex = 0;
unsigned char bitCnt   = 0;
unsigned char hasError    = 0;
unsigned char samePolarityBitCnt = 1;

unsigned char RTR;
unsigned char SRR;
unsigned char IDE;
unsigned char DLC;

void decoderStateMachine();

void checkBitStuffing() {
    sampledBit == previousBit ? samePolarityBitCnt++ : (samePolarityBitCnt = 1);
    previousBit = sampledBit;
    if (samePolarityBitCnt == 5) {
        printf("Destuffing bit at index %d.\n", bitIndex);
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
        samePolarityBitCnt = 1;
        previousBit = sampledBit;
        currentFrameField = prevFrameField;
    }
}

// TODO: Implement CRC15 check.
int validateCrcChecksum() {
    return 1;
}

void interframeSpaceStateMachine() {
    printf("Interframe space\n");
    switch(currentFrameSubField) {
        case INTERFRAME_SPACE_INTERMISSION:
            /***
            /* If a CAN node has a message waiting for transmission and it samples a
            /* dominant bit at the third bit of INTERMISSION, it will interpret this as
            /* a START OF FRAME bit, and, with the next bit, start transmitting its message
            /* with the first bit of its IDENTIFIER without first transmitting a START OF FRAME bit
            /* and without becoming receiver.
            /***/
            if (samePolarityBitCnt == 2 && sampledBit == '0') {
                currentFrameField = START_OF_FRAME;
                decoderStateMachine();
            } else if (sampledBit == '1') {
                samePolarityBitCnt++;
                if (samePolarityBitCnt == 3) {
                    samePolarityBitCnt = 1;
                    currentFrameField = INTERFRAME_SPACE;
                    currentFrameSubField = INTERFRAME_SPACE_BUS_IDLE;
                }
            } else {
                printf("Interframe space error: ");
                printf("Expecting 3 recessive bits during Intermission.\n");
                hasError = 1;
            }
            break;
        case INTERFRAME_SPACE_BUS_IDLE:
            if (sampledBit == '0') {
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
    // TODO: Enable hard synchronisation.
    IDE = 0;  // Assuming Standard format.
    DLC = 0;
    bitIndex = 0;
    bitCnt = 0;
    samePolarityBitCnt = 1;
    previousBit = sampledBit;
    frameBuf[bitIndex++] = sampledBit;
    currentFrameField = ARBITRATION;
    currentFrameSubField = ARBITRATION_IDENTIFIER_11_BIT;
};

void arbitrationStateMachine() {
    printf("Arbitration\n");
    int shouldCheckBitStuffing = 1;
    switch (currentFrameSubField) {
        case ARBITRATION_IDENTIFIER_11_BIT:
            frameBuf[bitIndex++] = sampledBit;
            if (bitIndex == 12) currentFrameSubField = ARBITRATION_RTR; // Assuming Standard format.
            break;
        case ARBITRATION_RTR:
            RTR = sampledBit;
            frameBuf[bitIndex++] = sampledBit;
            currentFrameField = CONTROL;
            if (IDE == '1') currentFrameSubField = CONTROL_r1;  // Extended format.
            else currentFrameSubField = CONTROL_IDE;    // Standard format.
            break;
        case ARBITRATION_SRR: // Empty transition to IDE bit.
            SRR = RTR;
            currentFrameSubField = ARBITRATION_IDE;
            shouldCheckBitStuffing = 0; // Prevent from checking bit stuffing without having sampled a new bit.
            arbitrationStateMachine();
            break;
        case ARBITRATION_IDE: // Empty transition to 18 bit identifier.
            currentFrameSubField = ARBITRATION_IDENTIFIER_18_BIT;
            break;
        case ARBITRATION_IDENTIFIER_18_BIT:
            frameBuf[bitIndex++] = sampledBit;
            if (bitIndex == 32) currentFrameSubField = ARBITRATION_RTR; // Assuming Standard format.
            break;
        default:
            printf("Arbitration error: invalid sub-frame field.\n");
            return;
    }
    // Check bit stuffing.
    if (!hasError && shouldCheckBitStuffing) {
        checkBitStuffing();
    }
}

void controlStateMachine() {
    printf("Control\n");
    int shouldCheckBitStuffing = 1;
    switch (currentFrameSubField) {
        case CONTROL_IDE:
            IDE = sampledBit;
            frameBuf[bitIndex++] = sampledBit;
            if (IDE == '0') { // Standard format.
                currentFrameSubField = CONTROL_r0;
            } else { // Extended format.
                currentFrameField = ARBITRATION;
                currentFrameSubField = ARBITRATION_SRR;
                shouldCheckBitStuffing = 0; // Prevent from checking bit stuffing without having sampled a new bit.
                arbitrationStateMachine();
            }
            break;
        case CONTROL_r1:
            frameBuf[bitIndex++] = sampledBit;
            currentFrameSubField = CONTROL_r0;
            break;
        case CONTROL_r0:
            frameBuf[bitIndex++] = sampledBit;
            currentFrameSubField = CONTROL_DLC;
            bitCnt = 4;
            break;
        case CONTROL_DLC:
            frameBuf[bitIndex++] = sampledBit;
            bitCnt--;
            DLC += ((sampledBit - '0') << bitCnt);
            if (bitCnt == 0) {
                DLC = fmin(DLC, 8); // Maximum number of data bytes: 8.
                printf("%d\n", DLC);
                if (RTR == '0') currentFrameField = DATA; // Data frame.
                else {
                    // Remote frame.
                    currentFrameField = CRC;
                    currentFrameSubField = CRC_SEQUENCE;
                }
            }
            break;
        default:
            printf("Control error: invalid sub-frame field.\n");
            return;
    }
    // Check bit stuffing.
    if (!hasError && shouldCheckBitStuffing) {
        checkBitStuffing();
    }
}

void dataStateMachine() {
    printf("Data\n");
    frameBuf[bitIndex++] = sampledBit;
    bitCnt++;
    if (bitCnt == 8 * DLC) {
        bitCnt = 15;
        currentFrameField = CRC;
        currentFrameSubField = CRC_SEQUENCE;
    }
    // Check bit stuffing.
    if (!hasError) {
        checkBitStuffing();
    }
}

void crcStateMachine() {
    printf("CRC\n");
    switch (currentFrameSubField) {
        case CRC_SEQUENCE:
            frameBuf[bitIndex++] = sampledBit;
            bitCnt--;
            if (bitCnt == 0) {
                if (validateCrcChecksum()) {
                    currentFrameSubField = CRC_DELIMITER;
                } else {
                    hasError = 1;
                }
            }
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
    // Check bit stuffing.
    if (!hasError) {
        checkBitStuffing();
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
            if (hasError) {
                printf("CRC error: ");
                printf("The calculated result is not the same as that received in the CRC sequence.\n");
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
            currentFrameField = INTERFRAME_SPACE;
            currentFrameSubField = INTERFRAME_SPACE_BUS_IDLE;

            // Print destuffed frame.
            int i;
            for (int i = 0; i < bitIndex; i++) {
                printf("%c", frameBuf[i]);
            }
            printf("\n");
        }
    } else {
        printf("End of frame error: ");
        printf("Expecting a flag sequence consisting of 7 recessive bits.\n");
        hasError = 1;
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
        default:
            printf("Decoder error: invalid frame field.\n");
            break;
    }
}

int main() {
    FILE *fp;
    fp = fopen("can_bus.txt", "r");
    
    if (fp == NULL) {
        printf("Error while opening the file.\n");
        return 1;
    }

    do {
        sampledBit = fgetc(fp);
        if (sampledBit == '0' || sampledBit == '1')
            decoderStateMachine();
    } while(feof(fp) == 0 && !hasError);

    fclose(fp);
    return 0;
}