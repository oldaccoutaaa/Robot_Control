#include "mbed.h"
 
Serial pc(USBTX, USBRX); // tx, rx
Serial device(p9, p10);  // tx, rx
char buf[] = "Hello World!";
int i = 0;
 
void pc_rx () {
    device.putc(pc.getc());
}
 
void dev_rx () {
    pc.putc(device.getc());
}
 
void pc_tx () {
    pc.putc(buf[i]);
    i ++;
    if (i >= strlen(buf)) {
       pc.attach(NULL, TxIrq);
    }
}
 
int main() {
    pc.attach(pc_rx, RxIrq);
    pc.attach(dev_rx, RxIrq);
    pc.attach(pc_tx, TxIrq);
 
    while(1);
}
