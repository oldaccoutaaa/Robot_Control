#include "mbed.h"

Serial pc(USBTX, USBRX); // tx, rx
BusOut led(p5,p6);



int main() {
    pc.printf("Press 'u' to turn LED1 brightness up, 'd' to turn it down\n");
    
    while(1){
         char c = pc.getc();
         
         if(c == 'u') {
            pc.putc(c);
            led = 1;    
            }
          if(c == 'd'){
            pc.putc(c);
            led = 2;
            }
      }
        
 } 