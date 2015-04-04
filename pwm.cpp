#include "mbed.h"
Serial device(p9, p10);         // tx = P9, rx = P10
DigitalOut REDE(p11);     
Serial pc(USBTX, USBRX);

void init(void){
    device.baud(115200);        // baud Rate = 115.2kbps [Futaba default]
    REDE = 0;                   // RS485 Transmit disable
}



void torque (unsigned char ID, unsigned char data){
 
    unsigned char TxData[9];    // TransmitByteData [9byte]
    unsigned char CheckSum = 0; // CheckSum calculation
    
    TxData[0] = 0xFA;           // Header
    TxData[1] = 0xAF;           // Header
    TxData[2] = ID;             // ID
    TxData[3] = 0x00;           // Flags
    TxData[4] = 0x24;           // Address
    TxData[5] = 0x01;           // Length
    TxData[6] = 0x01;           // Count
    TxData[7] = data;           // Data
    
    
    // CheckSum calculation
    for(int i=2; i<=7; i++){
        CheckSum = CheckSum ^ TxData[i];                // XOR from ID to Data
    }
    
    TxData[8] = CheckSum;       // Sum
    
    // Send Packet 
    REDE = 1;                   // RS485 Transmit Enable
    for(int i=0; i<=8; i++){
        device.putc(TxData[i]);
    }
    wait_us(250);               // Wait for transmission
    REDE = 0;                   // RS485 Transmitt disable
}

void long_1_8 (int pos1,int pos2,int pos3,int pos4,
int pos5,int pos6,int pos7,int pos8, int spd){
 
    unsigned char TxData[48];    // TransmitByteData [9byte]
    unsigned char CheckSum = 0; // CheckSum calculation
    
    TxData[0] = 0xFA;           // Header
    TxData[1] = 0xAF;           // Header
    TxData[2] = 0x00;             // ID
    TxData[3] = 0x00;           // Flags
    TxData[4] = 0x1E;           // Address
    TxData[5] = 0x05;           // Length
    TxData[6] = 0x08;           // Count
 
    TxData[7] = 1; 
    TxData[8] = (unsigned char)0x00FF & pos1;           // Low byte
    TxData[9] = (unsigned char)0x00FF & (pos1 >> 8);    // Hi  byte
    TxData[10] = (unsigned char)0x00FF & spd;
    TxData[11] = (unsigned char)0x00FF & (spd >> 8);


    TxData[12] = 2; 
    TxData[13] = (unsigned char)0x00FF & pos2;           // Low byte
    TxData[14] = (unsigned char)0x00FF & (pos2 >> 8);    // Hi  byte
    TxData[15] = (unsigned char)0x00FF & spd;
    TxData[16] = (unsigned char)0x00FF & (spd >> 8);
 
 
    TxData[17] = 3; 
    TxData[18] = (unsigned char)0x00FF & pos3;           // Low byte
    TxData[19] = (unsigned char)0x00FF & (pos3 >> 8);    // Hi  byte
    TxData[20] = (unsigned char)0x00FF & spd;
    TxData[21] = (unsigned char)0x00FF & (spd >> 8);
 
 
    TxData[22] = 4; 
    TxData[23] = (unsigned char)0x00FF & pos4;           // Low byte
    TxData[24] = (unsigned char)0x00FF & (pos4 >> 8);    // Hi  byte
    TxData[25] = (unsigned char)0x00FF & spd;
    TxData[26] = (unsigned char)0x00FF & (spd >> 8);
 
 
    TxData[27] = 5; 
    TxData[28] = (unsigned char)0x00FF & pos5;           // Low byte
    TxData[29] = (unsigned char)0x00FF & (pos5 >> 8);    // Hi  byte
    TxData[30] = (unsigned char)0x00FF & spd;
    TxData[31] = (unsigned char)0x00FF & (spd >> 8);
 
 
    TxData[32] = 6; 
    TxData[33] = (unsigned char)0x00FF & pos6;           // Low byte
    TxData[34] = (unsigned char)0x00FF & (pos6 >> 8);    // Hi  byte
    TxData[35] = (unsigned char)0x00FF & spd;
    TxData[36] = (unsigned char)0x00FF & (spd >> 8);
 
    TxData[37] = 7; 
    TxData[38] = (unsigned char)0x00FF & pos7;           // Low byte
    TxData[39] = (unsigned char)0x00FF & (pos7 >> 8);    // Hi  byte
    TxData[40] = (unsigned char)0x00FF & spd;
    TxData[41] = (unsigned char)0x00FF & (spd >> 8);
    
    
    TxData[42] = 8; 
    TxData[43] = (unsigned char)0x00FF & pos8;           // Low byte
    TxData[44] = (unsigned char)0x00FF & (pos8 >> 8);    // Hi  byte
    TxData[45] = (unsigned char)0x00FF & spd;
    TxData[46] = (unsigned char)0x00FF & (spd >> 8);
    
 
    
    // CheckSum calculation
    for(int i=2; i<=46; i++){
        CheckSum = CheckSum ^ TxData[i];                // XOR from ID to Data
    }
    
    TxData[47] = CheckSum;       // Sum
    
    // Send Packet 
    REDE = 1;                   // RS485 Transmit Enable
    for(int i=0; i<=47; i++){
        device.putc(TxData[i]);
    }
    wait_us(250);               // Wait for transmission
    REDE = 0;                   // RS485 Transmitt disable
    wait((float)spd/100);
}


void GoalPosition (unsigned char ID, int data){
 
    unsigned char TxData[10];   // TransmitByteData [10byte]
    unsigned char CheckSum = 0; // CheckSum calculation
    
    TxData[0] = 0xFA;           // Header
    TxData[1] = 0xAF;           // Header
    TxData[2] = ID;             // ID
    TxData[3] = 0x00;           // Flags
    TxData[4] = 0x1E;           // Address
    TxData[5] = 0x02;           // Length
    TxData[6] = 0x01;           // Count
                                // Data
    TxData[7] = (unsigned char)0x00FF & data;           // Low byte
    TxData[8] = (unsigned char)0x00FF & (data >> 8);    // Hi  byte
    
    // CheckSum calculation
    for(int i=2; i<=8; i++){
        CheckSum = CheckSum ^ TxData[i];                // XOR from ID to Data
    }
    TxData[9] = CheckSum;       // Sum
    // Send Packet
    REDE = 1;                   // RS485 Transmitt Enable
    for(int i=0; i<=9; i++){
        device.putc(TxData[i]);
    }
    wait_us(250);               // Wait for transmission
    REDE = 0;                   // RS485 Transmit disable
}

void Speed (unsigned char ID, int pos, int spd){ //hayasa iti 
 
    unsigned char TxData[12];   // TransmitByteData [10byte]
    unsigned char CheckSum = 0; // CheckSum calculation
    
    TxData[0] = 0xFA;           // Header
    TxData[1] = 0xAF;           // Header
    TxData[2] = ID;             // ID
    TxData[3] = 0x00;           // Flags
    TxData[4] = 0x1E;           // Address
    TxData[5] = 0x04;           // Length
    TxData[6] = 0x01;           // Count
                                // Data
    TxData[7] = (unsigned char)0x00FF & pos;           // Low byte
    TxData[8] = (unsigned char)0x00FF & (pos >> 8);    // Hi  byte
    TxData[9] = (unsigned char)0x00FF & spd;
    TxData[10] = (unsigned char)0x00FF & (spd >> 8);
    // CheckSum calculation
    for(int i=2; i<=10; i++){
        CheckSum = CheckSum ^ TxData[i];                // XOR from ID to Data
    }
    TxData[11] = CheckSum;       // Sum
    
    for(int i=0; i<=11; i++){
        device.putc(TxData[i]);
    }
    wait((float)spd/100);
}




int main() {
     init();                     // initialize
        // ID = 1(0x01) , torque = OFF (0x00)
    torque(0xFF, 0x01);
      wait(1);   
                           
   
       while(1){
       
   char c = pc.getc();
       if(c == '1'){
                     pc.putc(c);
        init();
        torque(0xFF, 0x01);
         wait(0.1);
                 long_1_8(450,450,450,-450,450,-450,450,-450,50);
               }

      } 
    }
     