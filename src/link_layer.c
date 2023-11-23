// Link layer protocol implementation

#include "link_layer.h"

#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define ESC         0x7D // During stuffing escape flag with 0x7D 0x5E or escape escape with 0x7D 0x5D

// F
#define F_FLAG      0x7E // Synchronisation: start or end of frame
// A
#define A_TRA_FLAG  0x03 // Address field in frames that are commands sent by the Transmitter or replies sent by the Receiver
#define A_REC_FLAG  0x01 // Address field in frames that are commands sent by the Receiver or replies sent by the Transmitter
// C
#define SET_FLAG    0x03 // SET frame: sent by the transmitter to initiate a connection
#define UA_FLAG     0x07 // UA frame: confirmation to the reception of a valid supervision frame
#define RR0_FLAG    0x05 // RR0 frame: indication sent by the Receiver that it is ready to receive an information frame number 0
#define RR1_FLAG    0x85 // RR1 frame: indication sent by the Receiver that it is ready to receive an information frame number 1
#define REJ0_FLAG   0x01 // REJ0 frame: indication sent by the Receiver that it rejects an information frame number 0 (detected an error)
#define REJ1_FLAG   0x81 // REJ1 frame: indication sent by the Receiver that it rejects an information frame number 1 (detected an error)
#define DISC_FLAG   0x0B // DISC frame to indicate the termination of a connection
#define INFO_0_FLAG 0x00 // Information frame number 0
#define INFO_1_FLAG 0x40 // Information frame number 1

// C MACROS
#define FLAG_RR(N)  ((N << 7) | 0x05) // RR(N) frame: indication sent by the Receiver that it is ready to receive an information frame number N
#define FLAG_REJ(N) ((N << 7) | 0x01) // REJ(N) frame: indication sent by the Receiver that it rejects an information frame number N (detected an error)
#define INFO_FLAG(N)(N << 6) // Information frame number N


// States of the State Machine
enum states{START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP, READING_DATA, DATA_FOUND_ESC};

// Link layer parameters
LinkLayer layer;

// Cable file descriptor
int fd;

// Alarm state
int alarmEnabled = FALSE;
int alarmCount = 0;

/// @brief Alarm function handler
/// @param signal 
void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;

    //printf("Alarm #%d\n", alarmCount); // Last print is purely visual.
}

// Save the old Port setting to use later.
struct termios oldtio;

/// @brief Configure Port connection
/// @param serialPort 
/// @return file descriptor on success or "-1" on error.
int connection(const char *serialPortName, int BAUDRATE) {

    // Open serial port device for reading and writing, and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    fd = open(serialPortName, O_RDWR | O_NOCTTY);

    if (fd < 0) {
        perror(serialPortName);
        return -1;
    }

    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        return -1;
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0;  // Blocking read until 1 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        return -1;
    }

    printf("New termios structure set\n");

    return fd;
}

/// @brief Sends Supervision Frame to Receiver/Transmitter
/// @param A 
/// @param C 
/// @return number of bytes writen.
int sendSupervisionFrame(unsigned char A, unsigned char C){
    unsigned char FRAME[5] = {F_FLAG, A, C, A ^ C, F_FLAG};
    return write(fd, FRAME, 5);
}

/// @brief State machine to receive responses from Receiver/Transmitter
/// @param state
/// @param A 
/// @param C 
/// @return "1" if connection sucessfull, "0" if no error or "-1" if error.
int fsm(enum states *state, unsigned char A, unsigned char C){
    
    unsigned char buf[5];

    int bytes = read(fd, buf, 1);
    
    if (bytes == 0) {
        return 0;
    }

    //printf("%d bytes read\n", bytes);
    switch (*state){
        case START:{
            if (buf[0] == F_FLAG){
                *state = FLAG_RCV;
                //printf("0:FLAG read\n");
            }
            //printf("0x%02X\n", buf[0]);
            break;
        }
        case FLAG_RCV:{
            if (buf[0] == A){
                *state = A_RCV;
                //printf("1:ADDRESS read\n");
            }
            else if (buf[0] == F_FLAG){
                *state = FLAG_RCV;
                //printf("1:FLAG read\n");
            }
            else{
                *state = START;
            }
            //printf("0x%02X\n", buf[0]);
            break;
        }
        case A_RCV:{
            if (buf[0] == C){
                *state = C_RCV;
                //printf("2:CONTROL read\n");
            }
            else if (buf[0] == F_FLAG){
                *state = FLAG_RCV;
                //printf("2:FLAG read\n");
            }
            else{
                *state = START;
            }
            //printf("0x%02X\n", buf[0]);
            break;
        }
        case C_RCV:{
            if (buf[0] == (A ^ C)){
                *state = BCC_OK;
                //printf("3:BCC read\n");
            }
            else if (buf[0] == F_FLAG){
                *state = FLAG_RCV;
                //printf("3:FLAG read\n");
            }
            else{
                *state = START;
            }
            //printf("0x%02X\n", buf[0]);
            break;
        }
        case BCC_OK:{
            if (buf[0] == F_FLAG){
                *state = STOP;
                //printf("4:FLAG read\n");
                alarm(0);
                return 1;
            }
            else{
                *state = START;
            }
            //printf("0x%02X\n", buf[0]);
            break;
        }
        default: return -1;
    }
    return 0;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    layer = connectionParameters;

    fd = connection(layer.serialPort, layer.baudRate);
    if (fd == -1) return -1;

    // Set alarm function handler
    (void)signal(SIGALRM, alarmHandler);

    enum states state = START;
    
    switch (layer.role){
        case LlTx:{
            //sleep(3);
            while ((state != STOP) && (alarmCount < (1 + layer.nRetransmissions))) {
                if (alarmEnabled == FALSE) {
                    alarm(layer.timeout); // Set alarm to be triggered in timeout seconds
                    alarmEnabled = TRUE;
                    
                    sendSupervisionFrame(A_TRA_FLAG, SET_FLAG);
                }
                else {
                    int result = fsm(&state, A_TRA_FLAG, UA_FLAG);
                    if (result == 1) {
                        printf("Connection established!\n");
                    } else if (result == -1) {
                        return -1;
                    }
                }
                
                if (alarmCount == layer.nRetransmissions){
                    printf("Couldn't reach\n");
                    return -1;
                }
            }
            break;
        }
        case LlRx:{
            //sleep(3);
            while (state != STOP) {
                int result = fsm(&state, A_TRA_FLAG, SET_FLAG);
                if (result == 1) {
                    printf("Connection established!\n");
                } else if (result == -1) {
                    return -1;
                }
            }

            sendSupervisionFrame(A_TRA_FLAG, UA_FLAG);
            
            break;
        }
        default:{
            return -1;
        }
    }

    return 1;
}

/// @brief State machine to receive responses from Receiver
/// @param state
/// @return cField 
unsigned char readControlFrame(enum states *state){

    unsigned char byte, cField = 0;
    
    if (read(fd, &byte, 1) > 0) {
        //printf("read something -> 0x%02X\n", byte);
        switch (*state) {
            case START:
                //printf("START_RCV -> 0x%02X\n", byte);
                if (byte == F_FLAG) *state = FLAG_RCV;
                break;
            case FLAG_RCV:
                //printf("FLAG_RCV -> 0x%02X\n", byte);
                if (byte == A_REC_FLAG) *state = A_RCV;
                else if (byte != F_FLAG) *state = START;
                break;
            case A_RCV:
                //printf("A_RCV -> 0x%02X\n", byte);
                if (byte == RR0_FLAG || byte == RR1_FLAG || byte == REJ0_FLAG || byte == REJ1_FLAG || byte == DISC_FLAG){
                    *state = C_RCV;
                    cField = byte;
                }
                else if (byte == F_FLAG) *state = FLAG_RCV;
                else *state = START;
                break;
            case C_RCV:
                //printf("C_RCV -> 0x%02X\n", byte);
                if (byte == (A_REC_FLAG ^ cField)) *state = BCC_OK;
                else if (byte == F_FLAG) *state = FLAG_RCV;
                else *state = START;
                break;
            case BCC_OK:
                //printf("BCC_OK -> 0x%02X\n", byte);
                if (byte == F_FLAG){
                    *state = STOP;
                }
                else *state = START;
                break;
            default: 
                break;
        }
    }
    return cField;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    //sleep(3);
    unsigned char tramaTx = 0;

    int frameSize = 6 + bufSize;
    unsigned char *frame = (unsigned char *) malloc(frameSize);
    frame[0] = F_FLAG;
    frame[1] = A_TRA_FLAG;
    frame[2] = INFO_0_FLAG;
    frame[3] = frame[1] ^frame[2];
    memcpy(frame + 4, buf, bufSize);
    unsigned char BCC2 = buf[0];
    for (unsigned int i = 1 ; i < bufSize ; i++) BCC2 ^= buf[i];

    int j = 4;
    for (unsigned int i = 0 ; i < bufSize ; i++) {
        if(buf[i] == F_FLAG || buf[i] == ESC) {
            frame = realloc(frame, ++frameSize);
            frame[j++] = ESC;
        }
        frame[j++] = buf[i];
    }
    frame[j++] = BCC2;
    frame[j++] = F_FLAG;

    int accepted = 0;
    alarmCount = 0;
    alarmEnabled = FALSE;

    enum states state = START;
    while ((state != STOP) && (alarmCount < (1 + layer.nRetransmissions) && !accepted)) {
        if (alarmEnabled == FALSE){
            alarm(layer.timeout); // Set alarm to be triggered in timeout seconds
            alarmEnabled = TRUE;
            
            write(fd, frame, j);

            accepted = 0;
        } else {
            unsigned char result = readControlFrame(&state);
            
            if(!result){
                continue;
            }
            else if(result == RR0_FLAG || result == RR1_FLAG) {
                accepted = 1;
                tramaTx = (tramaTx + 1) % 2;
            }
            else continue;
        }
    }
    
    free(frame);
    if(accepted) return frameSize;
    else {
        llclose(FALSE);
        return -1;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    //sleep(3);
    unsigned char tramaRx = 1;
    unsigned char byte, cField;
    int i = 0;
    enum states state = START;

    while (state != STOP) {
        if (read(fd, &byte, 1) > 0) {
            //printf("read something\n");
            //printf("0x%02X\n", byte);
            switch (state) {
                case START:
                    //printf("START_RCV\n");
                    //printf("0x%02X\n", byte);
                    if (byte == F_FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    //printf("FLAG_RCV\n");
                    //printf("0x%02X\n", byte);
                    if (byte == A_TRA_FLAG) state = A_RCV;
                    else if (byte != F_FLAG) state = START;
                    break;
                case A_RCV:
                    //printf("A_RCV\n");
                    //printf("0x%02X\n", byte);
                    if (byte == INFO_FLAG(0) || byte == INFO_FLAG(1)){
                        state = C_RCV;
                        cField = byte;   
                    }
                    else if (byte == F_FLAG) state = FLAG_RCV;
                    else if (byte == DISC_FLAG) {
                        //printf("DISC\n");
                        return 0;
                    }
                    else state = START;
                    break;
                case C_RCV:
                    //printf("C_RCV\n");
                    //printf("0x%02X\n", byte);
                    if (byte == (A_TRA_FLAG ^ cField)) state = READING_DATA;
                    else if (byte == F_FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case READING_DATA:
                    //printf("READING_RCV\n");
                    //printf("0x%02X\n", byte);
                    if (byte == ESC) state = DATA_FOUND_ESC;
                    else if (byte == F_FLAG){
                        //printf("FLAG FOUND\n");
                        unsigned char bcc2 = packet[i-1];
                        i--;
                        packet[i] = '\0';
                        unsigned char acc = packet[0];
                        //printf("bcc2 = 0x%02X\n", bcc2);
                        //printf("acc = 0x%02X\n", acc);

                        for (unsigned int j = 1; j < i; j++)
                            acc ^= packet[j];

                        //printf("bcc2 = 0x%02X\n", bcc2);
                        //printf("acc = 0x%02X\n", acc);

                        if (bcc2 == acc){
                            state = STOP;
                            sendSupervisionFrame(A_REC_FLAG, FLAG_RR(tramaRx));
                            //printf("INFO\n");
                            tramaRx = (tramaRx + 1) % 2;
                            return i;
                        }
                        else{
                            printf("Error: retransmition\n");
                            sendSupervisionFrame(A_REC_FLAG, FLAG_REJ(tramaRx));
                            return -1;
                        };

                    }
                    else{
                        packet[i++] = byte;
                    }
                    break;
                case DATA_FOUND_ESC:
                    //printf("DATA_ESC_RCV\n");
                    //printf("0x%02X\n", byte);
                    state = READING_DATA;
                    if (byte == ESC || byte == F_FLAG) packet[i++] = byte;
                    else{
                        packet[i++] = ESC;
                        packet[i++] = byte;
                    }
                    break;
                default:
                    //printf("DEFAULT\n");
                    break;
            }
        }
    }
    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // Set alarm function handler
    (void)signal(SIGALRM, alarmHandler);

    enum states state = START;
    //int bytes;
    alarmCount = 0;
    alarmEnabled = FALSE;

    switch (layer.role){
        case LlTx:{
            //sleep(3);
            while ((state != STOP) && (alarmCount < (1 + layer.nRetransmissions))) {
                if (alarmEnabled == FALSE) {
                    alarm(layer.timeout); // Set alarm to be triggered in timeout seconds
                    alarmEnabled = TRUE;
                    
                    sendSupervisionFrame(A_TRA_FLAG, DISC_FLAG);
                    //bytes = sendSupervisionFrame(A_TRA_FLAG, DISC_FLAG);
                    //printf("%d bytes written.\n", bytes);
                }
                else {
                    int result = fsm(&state, A_REC_FLAG, DISC_FLAG);
                    if (result == 1) {
                        sendSupervisionFrame(A_REC_FLAG, UA_FLAG);
                        //int bytes = sendSupervisionFrame(A_REC_FLAG, UA_FLAG);
                        //printf("%d bytes written\n", bytes);
                        printf("Disconnecting...\n");
                    } else if (result == -1) {
                        return -1;
                    }
                }
                
                if (alarmCount > layer.nRetransmissions){
                    printf("Couldn't reach. Disconnecting...\n");
                    break;
                }
            }
            break;
        }
        case LlRx:{
            //sleep(3);
            while ((state != STOP) && (alarmCount < (1 + layer.nRetransmissions))) {
                if (alarmEnabled == FALSE) {
                    alarm(layer.timeout); // Set alarm to be triggered in timeout seconds
                    alarmEnabled = TRUE;
                    
                    sendSupervisionFrame(A_REC_FLAG, DISC_FLAG);
                    //bytes = sendSupervisionFrame(A_REC_FLAG, DISC_FLAG);
                    //printf("%d bytes written.\n", bytes);
                }
                else {
                    int result = fsm(&state, A_REC_FLAG, UA_FLAG);
                    if (result == 1) {
                        printf("Disconnecting...\n");
                    } else if (result == -1) {
                        return -1;
                    }
                }
                
                if (alarmCount > layer.nRetransmissions){
                    printf("Couldn't reach. Disconnecting...\n");
                    break;
                }
            }
            break;
        }
        default:{
            return -1;
        }
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        return -1;
    }

    close(fd);

    return 1;
}
