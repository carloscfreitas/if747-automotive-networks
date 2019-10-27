#include <stdio.h>

/********** Interframe Space ***********/
#define INTERFRAME_SPACE 				0
#define INTERFRAME_SPACE_INTERMISSION 	1
#define INTERFRAME_SPACE_BUS_IDLE 		2
/***************************************/

/**** Start of Frame ***/
#define START_OF_FRAME	3
/***********************/

/************* Arbitration *************/
#define ARBITRATION						4
/*** Standard/Extended format fields ***/
#define ARBITRATION_IDENTIFIER_11_BIT	5
#define ARBITRATION_RTR					6
/**** Extended format extra fields *****/
#define ARBITRATION_SRR					7
#define ARBITRATION_IDE					8
#define ARBITRATION_IDENTIFIER_18_BIT	9
/***************************************/

/************* Control *************/
#define CONTROL 	10
/* Standard/Extended format fields */
#define CONTROL_IDE	11
#define CONTROL_r0	12
#define CONTROL_DLC	13
/*** Extended format extra fields **/
#define CONTROL_r1	14
/***********************************/

/*** Data ****/
#define DATA 15
/*************/

/******* CRC check ******/
#define CRC 			16
#define CRC_DELIMITER	17
/************************/

/********** ACK *********/
#define ACK				18
#define ACK_SLOT		19
#define ACK_DELIMITER	20
/************************/

/****** End of Frame ****/
#define END_OF_FRAME	21
/************************/

#define MAX_FRAME_SIZE 127

unsigned char currentFrameField 	= INTERFRAME_SPACE;
unsigned char currentFrameSubField 	= INTERFRAME_SPACE_BUS_IDLE;

unsigned char frameBuf[MAX_FRAME_SIZE];
unsigned char bitIndex 		= 0;
unsigned char sampledBit	= 0;

void decoderStateMachine();

void interframeSpaceStateMachine() {
	switch(currentFrameSubField) {
		case INTERFRAME_SPACE_INTERMISSION:
			break;
		case INTERFRAME_SPACE_BUS_IDLE:
			if (sampledBit == '0') {
				printf("INTERFRAME_SPACE_BUS_IDLE -> START_OF_FRAME\n");
				currentFrameField = START_OF_FRAME;
				decoderStateMachine();
			} else if(sampledBit == '1') {
				printf("INTERFRAME_SPACE_BUS_IDLE\n");
			}
			break;
		default:
			printf("Interframe space error: invalid sub-frame field.\n");
			break;
	}
};

void startOfFrameStateMachine() {
	printf("START_OF_FRAME\n");
	bitIndex = 0;
	frameBuf[bitIndex++] = sampledBit;
	// TODO: Trigger hard synchronisation.
	currentFrameField = ARBITRATION;
	printf("%c", frameBuf[0]);
};

void decoderStateMachine() {
	switch(currentFrameField) {
		case INTERFRAME_SPACE:
			interframeSpaceStateMachine();
			break;
		case START_OF_FRAME:
			startOfFrameStateMachine();
			break;
		case ARBITRATION:
			break;
		case CONTROL:
			break;
		case DATA:
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