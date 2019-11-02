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
#define CRC_DELIMITER   17
/************************/

/********** ACK *********/
#define ACK             18
#define ACK_SLOT        19
#define ACK_DELIMITER   20
/************************/

/****** End of Frame ****/
#define END_OF_FRAME    21
/************************/

#define MAX_FRAME_SIZE 127

unsigned char currentFrameField    = INTERFRAME_SPACE;
unsigned char currentFrameSubField = INTERFRAME_SPACE_BUS_IDLE;

unsigned char frameBuf[MAX_FRAME_SIZE];
unsigned char sampledBit;
unsigned char bitIndex = 0;
unsigned char bitCnt   = 0;

unsigned char RTR;
unsigned char SRR;
unsigned char IDE;
unsigned char DLC;

void decoderStateMachine();

void interframeSpaceStateMachine() {
    printf("Interframe space\n");
    switch(currentFrameSubField) {
        case INTERFRAME_SPACE_INTERMISSION:
            break;
        case INTERFRAME_SPACE_BUS_IDLE:
            if (sampledBit == '0') {
                currentFrameField = START_OF_FRAME;
                decoderStateMachine();
            }
            break;
        default:
            printf("Interframe space error: invalid sub-frame field.\n");
            break;
    }
};

void startOfFrameStateMachine() {
    printf("Start of Frame\n");
    // TODO: Trigger hard synchronisation.
    bitIndex = 0;
    IDE = 0;  // Assuming Standard format.
    frameBuf[bitIndex++] = sampledBit;
    currentFrameField = ARBITRATION;
    currentFrameSubField = ARBITRATION_IDENTIFIER_11_BIT;
};

void arbitrationStateMachine() {
    printf("Arbitration\n");
    switch (currentFrameSubField) {
        case ARBITRATION_IDENTIFIER_11_BIT:
            frameBuf[bitIndex++] = sampledBit;
            if (bitIndex == 12) currentFrameSubField = ARBITRATION_RTR; // Assuming Standard format.
            break;
        case ARBITRATION_RTR:
            RTR = sampledBit;
            frameBuf[bitIndex++] = sampledBit;
            currentFrameField = CONTROL;
            if (IDE == '1') {
                currentFrameSubField = CONTROL_r1;
            } else {
                currentFrameSubField = CONTROL_IDE;
            }
            break;
        case ARBITRATION_SRR: // Empty transition to IDE bit.
            SRR = RTR;
            currentFrameSubField = ARBITRATION_IDE;
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
            break;
    }
}

void controlStateMachine() {
    printf("Control\n");
    switch (currentFrameSubField) {
        case CONTROL_IDE:
            IDE = sampledBit;
            frameBuf[bitIndex++] = sampledBit;
            if (IDE == '0') { // Standard format.
                currentFrameSubField = CONTROL_r0;
            } else { // Extended format.
                currentFrameField = ARBITRATION;
                currentFrameSubField = ARBITRATION_SRR;
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
                else currentFrameField = CRC; // Remote frame.
            }
            break;
        default:
            printf("Control error: invalid sub-frame field.\n");
            break;
    }
}

void dataStateMachine() {
    printf("Data\n");
    frameBuf[bitIndex++] = sampledBit;
    bitCnt++;
    if (bitCnt == 8 * DLC) currentFrameField = CRC;
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
            break;
        case ACK:
            break;
        case END_OF_FRAME:
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
        decoderStateMachine();
    } while(feof(fp) == 0);

    fclose(fp);
    return 0;
}