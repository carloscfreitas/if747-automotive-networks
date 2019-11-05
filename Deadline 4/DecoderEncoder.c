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

/****** Error frame *****/
#define ERROR           24
#define ERROR_FLAG      25
#define ERROR_DELIMITER 26
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
unsigned char hasError = 0;
unsigned char crcError = 0;
unsigned char samePolarityBitCnt = 1;

unsigned char rtrBit;
unsigned char srrBit;
unsigned char ideBit;
unsigned char dlc;
unsigned char crc[15] = "000000000000000";
unsigned char generatorPolynomial[15] = "100010110011001"; // 0x4599

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

void computeCrcSequence() {
    int i, j;
    unsigned char nxtBit, crcNxt;
    
    nxtBit = sampledBit;
    crcNxt = nxtBit ^ crc[0];

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
        crcError = crc[j] != frameBuf[(bitIndex - 1) - 14 + j];
    }
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
            if (bitCnt == 2 && sampledBit == '0') {
                currentFrameField = START_OF_FRAME;
                decoderStateMachine();
            } else if (sampledBit == '1') {
                bitCnt++;
                if (bitCnt == 3) {
                    bitCnt = 0;
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
    dlc      = 0;
    ideBit   = 0;  // Assuming Standard format.
    bitCnt   = 0;
    bitIndex = 0;
    crcError = 0;
    hasError = 0;
    samePolarityBitCnt = 1;
    previousBit = sampledBit;
    frameBuf[bitIndex++] = sampledBit;
    currentFrameField = ARBITRATION;
    currentFrameSubField = ARBITRATION_IDENTIFIER_11_BIT;
    // Reset CRC sequence.
    int j;
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
            if (bitIndex == 12) currentFrameSubField = ARBITRATION_RTR; // Assuming Standard format.
            break;
        case ARBITRATION_RTR:
            rtrBit = sampledBit;
            frameBuf[bitIndex++] = sampledBit;
            currentFrameField = CONTROL;
            if (ideBit == '1') currentFrameSubField = CONTROL_r1;  // Extended format.
            else currentFrameSubField = CONTROL_IDE;    // Standard format.
            break;
        case ARBITRATION_SRR: // Empty transition to IDE bit field.
            srrBit = rtrBit;
            currentFrameSubField = ARBITRATION_IDE;
            skipState = 1; // Prevent from doing further evaluation without having sampled a new bit.
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
            ideBit = sampledBit;
            frameBuf[bitIndex++] = sampledBit;
            if (ideBit == '0') { // Standard format.
                currentFrameSubField = CONTROL_r0;
            } else { // Extended format.
                currentFrameField = ARBITRATION;
                currentFrameSubField = ARBITRATION_SRR;
                skipState = 1; // Prevent from doing further evaluation without having sampled a new bit.
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
            dlc += ((sampledBit - '0') << bitCnt);
            if (bitCnt == 0) {
                dlc = fmin(dlc, 8); // Maximum number of data bytes: 8.
                printf("%d\n", dlc);
                if (rtrBit == '0') currentFrameField = DATA; // Data frame.
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
    bitCnt++;
    if (bitCnt == 8 * dlc) {
        bitCnt = 15;
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
            currentFrameField = INTERFRAME_SPACE;
            currentFrameSubField = INTERFRAME_SPACE_INTERMISSION;

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

void errorStateMachine() {
    printf("Error frame\n");
    switch(currentFrameSubField) {
        case ERROR_FLAG:
            // TODO: Support error flag superposition (max: 12 bits).
            if (sampledBit == '0') {
                bitCnt--;
                if (bitCnt == 0) {
                    bitCnt = 8;
                    currentFrameSubField = ERROR_DELIMITER;
                }
            } else {
                printf("Error flag error: ");
                printf("Expecting 6 dominant bits during error flag.\n");
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
                printf("Expecting 8 dominant bits during error flag.\n");
                hasError = 1;
            }
            break;
        default:
            printf("Error frame error: invalid sub-frame field.\n");
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
        default:
            printf("Decoder error: invalid frame field.\n");
            break;
    }
    if (hasError) {
        printf("Start receiving error flag.\n");
        bitCnt = 6; // Size of the error flag.
        hasError = 0;
        currentFrameField = ERROR;
        currentFrameSubField = ERROR_FLAG;
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
        decoderStateMachine();
    } while(feof(fp) == 0);

    fclose(fp);
    return 0;
}