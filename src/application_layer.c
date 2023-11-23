// Application layer protocol implementation

#include "application_layer.h"

#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>


/// @brief Calculate the number of octates needed. 
/// @param x 
/// @return ceil(log2l(x)/8)
int octate(long int x) {
    int log2Value = 0;
    
    // Calculate log2(x) using bitwise manipulation
    while (x > 1) {
        x >>= 1;
        log2Value++;
    }

    // Ceiling operation
    int result = (log2Value >> 3); // Divide by 8 using right shift by 3 bits
    if (log2Value % 3 != 0) {
        result++; // Add 1 if there is a remainder after dividing by 8
    }

    return result;
}

/// @brief Parse the control packet for the transferred file name and size.
/// @param packet 
/// @param size 
/// @param fileSize 
/// @return name of transferred file
unsigned char* parseControlPacket(unsigned char* packet, int size, unsigned long int *fileSize) {

    // File Size
    unsigned char fileSizeNBytes = packet[2];
    unsigned char *fileSizeAux = (unsigned char*)malloc(fileSizeNBytes);
    memcpy(fileSizeAux, packet + 3, fileSizeNBytes);
    for (unsigned int i = 0; i < fileSizeNBytes; i++)
        *fileSize |= (fileSizeAux[fileSizeNBytes - i - 1] << (8 * i));

    // File Name
    unsigned char fileNameNBytes = packet[3 + fileSizeNBytes + 1];
    unsigned char *name = (unsigned char*)malloc(fileNameNBytes);
    memcpy(name, packet + 3 + fileSizeNBytes + 2, fileNameNBytes);

    free(fileSizeAux);

    return name;
}

/// @brief Create Control Packet(Start or End)
/// @param c 
/// @param filename 
/// @param length 
/// @param size 
/// @return packet
unsigned char * getControlPacket(const unsigned int c, const char* filename, long int length, unsigned int* size){

    const int L1 = octate(length);
    const int L2 = strlen(filename);
    *size = 1 + 2 + L1 + 2 + L2; // c + (T1 + L1) + L1 + (T2 + L2) + L2
    unsigned char *packet = (unsigned char*)malloc(*size);
    
    unsigned int pos = 0;
    packet[pos++] = c; // 2-Start 3-End 
    packet[pos++] = 0; // Type 1
    packet[pos++] = L1; // Length 1

    for (unsigned char i = 0; i < L1; i++) {
        packet[2 + L1 - i] = length & 0xFF;
        length >>= 8;
    }
    pos += L1;
    packet[pos++] = 1; // Type 2
    packet[pos++] = L2; // Length 2
    memcpy(packet + pos, filename, L2);
    return packet;
}

/// @brief Create a Data Packet
/// @param sequence 
/// @param data 
/// @param dataSize 
/// @param packetSize 
/// @return packet
unsigned char * getDataPacket(unsigned char sequence, unsigned char *data, int dataSize, int *packetSize){

    *packetSize = 1 + 1 + 2 + dataSize; // C + L2 + L1 + P
    unsigned char* packet = (unsigned char*)malloc(*packetSize);

    packet[0] = 1; // 1-Data
    packet[1] = sequence; // L2
    packet[2] = dataSize >> 8 & 0xFF; // L1
    packet[3] = dataSize & 0xFF; // P
    memcpy(packet + 4, data, dataSize);

    return packet;
}

/// @brief Get Data to send in the packets
/// @param fd 
/// @param fileLength 
/// @return data
unsigned char * getData(FILE* fd, long int fileLength) {
    unsigned char* content = (unsigned char*)malloc(sizeof(unsigned char) * fileLength);
    fread(content, sizeof(unsigned char), fileLength, fd);
    return content;
}

/// @brief Parse the data received from the packets
/// @param packet 
/// @param packetSize 
/// @param buffer 
void parseDataPacket(const unsigned char* packet, const unsigned int packetSize, unsigned char* buffer) {
    memcpy(buffer, packet + 4, packetSize - 4);
    buffer += packetSize + 4;
}


void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer linkLayer;
    LinkLayerRole linkLayerRole;

    if (strcmp(role,"tx") == 0){
        linkLayerRole = LlTx;
    }
    else if (strcmp(role,"rx") == 0){
        linkLayerRole = LlRx;
    }
    else {
        printf("Unexpected role!\n");
        exit(-1);
    }

    strcpy(linkLayer.serialPort, serialPort);
    linkLayer.role = linkLayerRole;
    linkLayer.baudRate = baudRate;
    linkLayer.nRetransmissions = nTries;
    linkLayer.timeout = timeout;

    if (llopen(linkLayer) == -1){
        llclose(FALSE);
        perror("Opening connection error!\n");
        exit(-1);
    }


    // Start clock to measure data transmittion time
    struct timespec start_timer, end_timer;
    clock_gettime(CLOCK_REALTIME, &start_timer);


    //int chars = 0;

    switch (linkLayerRole) {
        case LlTx: {
            FILE* fileTx = fopen(filename, "rb");
            if (fileTx == NULL) {
                llclose(FALSE);
                perror("File not found\n");
                exit(-1);
            }

            // Find file size
            fseek(fileTx, 0, SEEK_END); 
            long int fileSize = ftell(fileTx);
            fseek(fileTx, 0, SEEK_SET);

            unsigned int controlPacketSize;
            unsigned char *controlPacketStart = getControlPacket(2, filename, fileSize, &controlPacketSize);

            int frameSize = llwrite(controlPacketStart, controlPacketSize);
            if(frameSize == -1){
                llclose(FALSE);
                printf("Exit: error in start packet\n");
                exit(-1);
            }
            //chars += frameSize;
            free(controlPacketStart);

            unsigned char sequence = 0;
            unsigned char* content = getData(fileTx, fileSize);
            long int bytesLeft = fileSize;

            while (bytesLeft >= 0) { 

                int dataSize = bytesLeft > (long int) MAX_PAYLOAD_SIZE ? MAX_PAYLOAD_SIZE : bytesLeft;
                //printf("Data Size = %d\n", dataSize);
                unsigned char* data = (unsigned char*) malloc(dataSize);
                memcpy(data, content, dataSize);
                int packetSize;
                unsigned char* packet = getDataPacket(sequence, data, dataSize, &packetSize);
                
                frameSize = llwrite(packet, packetSize);
                if(frameSize == -1) {
                    llclose(FALSE);
                    printf("Exit: error in data packets\n");
                    exit(-1);
                }
                
                bytesLeft -= (long int) MAX_PAYLOAD_SIZE; 
                content += dataSize;
                sequence = (sequence + 1) % 255;
                
                //chars += frameSize;
                free(data);
            }
            unsigned char *controlPacketEnd = getControlPacket(3, filename, fileSize, &controlPacketSize);

            frameSize = llwrite(controlPacketEnd, controlPacketSize);
            if(frameSize == -1) { 
                llclose(FALSE);
                printf("Exit: error in end packet\n");
                exit(-1);
            }
            //chars += frameSize;
            free(controlPacketEnd);

            fclose(fileTx);
            
            //printf("%d chars written after byte stuffing.\n", chars);
            break;
        }
        case LlRx: {
            unsigned char *packet = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);
            int packetSize = -1;
            while ((packetSize = llread(packet)) < 0);
            //chars += packetSize;
            
            unsigned long int rxFileSize = 0;
            unsigned char* name = parseControlPacket(packet, packetSize, &rxFileSize);
            
            if (strcmp((char *)(name), filename) != 0){
                printf("Filename received different from expected.\n");
            }
            else {
                printf("Filename received and expected match.\n");
            }

            //FILE* fileRx = fopen((char *) name, "wb+");
            FILE* fileRx = fopen(filename, "wb+");
            while (1) {
                while ((packetSize = llread(packet)) < 0);
                //chars += packetSize;

                if(packetSize == 0) break;
                else if(packet[0] != 3){
                    unsigned char *buffer = (unsigned char*)malloc(packetSize);
                    parseDataPacket(packet, packetSize, buffer);
                    fwrite(buffer, sizeof(unsigned char), packetSize-4, fileRx);
                    free(buffer);
                }
                else continue;
            }
            free(packet);

            fclose(fileRx);

            //printf("%d chars read after byte destuffing\n", chars);
            break;
        }
        
        default: {
            llclose(FALSE);
            exit(-1);
        }
    }

    clock_gettime(CLOCK_REALTIME, &end_timer);

    double time_running = (end_timer.tv_sec - start_timer.tv_sec) + (end_timer.tv_nsec - start_timer.tv_nsec) / 1E9;

    printf("Data transmittion time: %f\n", time_running);

    if (llclose(FALSE) == -1){
        perror("Closing connection error!\n");
        exit(-1);
    }
}
