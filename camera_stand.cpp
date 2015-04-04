#include "mbed.h"
Serial pc(USBTX, USBRX);
Serial device(p9, p10);         // tx = P9, rx = P10
DigitalOut REDE(p11);           // RS485 Transmit Enable

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
    init();                     
    torque(0xFF, 0x01);                             
    wait(1);                    


    while(1){      
        char p = pc.getc(); 
        char s = pc.getc();     
        pc.putc(p); 
        pc.putc(s); 
          
        Speed(0x01,p,s); // ID = 1(0x01) , GoalPosition = 30.0deg(300)
        wait(1);                // wait (1sec)

    }
}
      