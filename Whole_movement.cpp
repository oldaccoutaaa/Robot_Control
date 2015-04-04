#include "mbed.h"
#include "TextLCD.h"
#include "QEI.h"
Serial pc(USBTX, USBRX);
Serial device(p9, p10);         // tx = P9, rx = P10
DigitalOut REDE(p11);          //output
int x1;

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

Ticker flipper1; //timer
Ticker flipper2; //timer
//rotary----------------------------------------------------------------------------------
DigitalIn sw1(p27);//
DigitalIn sw2(p28);
DigitalIn sw3(p29);
DigitalIn sw4(p30);

DigitalOut move(p6);
DigitalOut  motor1(p26); //L
DigitalOut  motor2(p25); //R

int up = 0.0;
int down = 0.0;

TextLCD lcd(p15, p16, p17, p18, p19, p20);  // RS, E, DB4, DB5, DB6, DB7

#define ROTATE_PER_REVOLUTIONS  100

QEI wheelL(p11, p8, NC, ROTATE_PER_REVOLUTIONS, QEI::X4_ENCODING);
QEI wheelR(p15, p14, NC, ROTATE_PER_REVOLUTIONS, QEI::X4_ENCODING);

//QEI wheel (p30, p29, NC, 624);

//----------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------
//lower body servo
//----------------------------------------------------------------------------------------



float pwm_max=0.0024;
float pwm_min=0.0007;
float pwm_mid=0.0015;
unsigned char direction1=0,direction2=0,direction3=0,direction4=0;
 
PwmOut servo1(p21);
PwmOut servo2(p22);
PwmOut servo3(p23);
PwmOut servo4(p24);

void NYX_PWM_1 (float width)
{
    if(width>pwm_max){width=pwm_max;}
    if(width<pwm_min){width=pwm_min;}
    servo1.pulsewidth(width);
}

void NYX_PWM100_1 (float w1)
{
    float width1;
    unsigned char direction_temporary;

    if(w1>100){w1=100;} 
    if(w1<(-100)){w1=-100;}

    if(w1>0){direction_temporary=1;}
    else if(w1<0){direction_temporary=2;}
    else{direction_temporary=0;}

    if(direction1!=2 &&direction_temporary==2)
    {
        servo1.pulsewidth(pwm_mid-0.0005);
        wait(0.02);
        servo1.pulsewidth(pwm_mid);
        wait(0.02);
    }

    else if(direction1!=1 &&direction_temporary==1)
    {
        servo1.pulsewidth(pwm_mid);
        wait(0.02);
        servo1.pulsewidth(pwm_mid+0.00001);
        wait(0.02);
    }

    direction1=direction_temporary;


    if(w1>0)
    {width1=pwm_mid+((pwm_max-pwm_mid)*0.001*w1);NYX_PWM_1(width1);direction1=1;}
    else if(w1==0)
    {width1=pwm_mid;NYX_PWM_1(width1);direction1=0;}
    else
    {width1=pwm_mid+((pwm_mid-pwm_min)*0.001*w1);NYX_PWM_1(width1);direction1=2;}

}


void NYX_PWM_2 (float width)
{
    if(width>pwm_max){width=pwm_max;}
    if(width<pwm_min){width=pwm_min;}
    servo2.pulsewidth(width);
}


void NYX_PWM100_2 (float w2)
{
    float width2;
    unsigned char direction_temporary;

    if(w2>100){w2=100;} 
    if(w2<(-100)){w2=-100;}

    if(w2>0){direction_temporary=1;}
    else if(w2<0){direction_temporary=2;}
    else{direction_temporary=0;}

    if(direction2!=2 &&direction_temporary==2)
    {
        servo2.pulsewidth(pwm_mid-0.0005);
        wait(0.02);
        servo2.pulsewidth(pwm_mid);
        wait(0.02);
    }

    else if(direction2!=1 &&direction_temporary==1)
    {
        servo2.pulsewidth(pwm_mid);
        wait(0.02);
        servo2.pulsewidth(pwm_mid+0.00001);
        wait(0.02);
    }

    direction2=direction_temporary;


    if(w2>0)
    {width2=pwm_mid+((pwm_max-pwm_mid)*0.001*w2);NYX_PWM_2(width2);direction2=1;}
    else if(w2==0)
    {width2=pwm_mid;NYX_PWM_2(width2);direction2=0;}
    else
    {width2=pwm_mid+((pwm_mid-pwm_min)*0.001*w2);NYX_PWM_2(width2);direction2=2;}

}

void NYX_PWM_3 (float width)
{
    if(width>pwm_max){width=pwm_max;}
    if(width<pwm_min){width=pwm_min;}
    servo3.pulsewidth(width);
}

void NYX_PWM100_3 (float w3)
{
    float width3;
    unsigned char direction_temporary;

    if(w3>100){w3=100;} 
    if(w3<(-100)){w3=-100;}

    if(w3>0){direction_temporary=1;}
    else if(w3<0){direction_temporary=2;}
    else{direction_temporary=0;}

    if(direction3!=2 &&direction_temporary==2)
    {
        servo3.pulsewidth(pwm_mid-0.0005);
        wait(0.02);
        servo3.pulsewidth(pwm_mid);
        wait(0.02);
    }

    else if(direction3!=1 &&direction_temporary==1)
    {
        servo3.pulsewidth(pwm_mid);
        wait(0.02);
        servo3.pulsewidth(pwm_mid+0.00001);
        wait(0.02);
    }

    direction3=direction_temporary;


    if(w3>0)
    {width3=pwm_mid+((pwm_max-pwm_mid)*0.001*w3);NYX_PWM_3(width3);direction3=1;}
    else if(w3==0)
    {width3=pwm_mid;NYX_PWM_3(width3);direction3=0;}
    else
    {width3=pwm_mid+((pwm_mid-pwm_min)*0.001*w3);NYX_PWM_3(width3);direction3=2;}

}



void NYX_PWM_4 (float width)
{
    if(width>pwm_max){width=pwm_max;}
    if(width<pwm_min){width=pwm_min;}
    servo4.pulsewidth(width);
}

void NYX_PWM100_4 (float w4)
{
    float width4;
    unsigned char direction_temporary;

    if(w4>100){w4=100;} 
    if(w4<(-100)){w4=-100;}

    if(w4>0){direction_temporary=1;}
    else if(w4<0){direction_temporary=2;}
    else{direction_temporary=0;}

    if(direction4!=2 &&direction_temporary==2)
    {
        servo4.pulsewidth(pwm_mid-0.0005);
        wait(0.02);
        servo4.pulsewidth(pwm_mid);
        wait(0.02);
    }

    else if(direction4!=1 &&direction_temporary==1)
    {
        servo4.pulsewidth(pwm_mid);
        wait(0.02);
        servo4.pulsewidth(pwm_mid+0.00001);
        wait(0.02);
    }

    direction4=direction_temporary;


    if(w4>0)
    {width4=pwm_mid+((pwm_max-pwm_mid)*0.001*w4);NYX_PWM_4(width4);direction4=1;}
    else if(w4==0)
    {width4=pwm_mid;NYX_PWM_4(width4);direction4=0;}
    else
    {width4=pwm_mid+((pwm_mid-pwm_min)*0.001*w4);NYX_PWM_4(width4);direction4=2;}

}



void Servo_init()
{
    servo1.period_ms(20);
    servo1.pulsewidth(pwm_mid);
    servo2.period_ms(20);
    servo2.pulsewidth(pwm_mid);
    servo3.period_ms(20);
    servo3.pulsewidth(pwm_mid);
    servo4.period_ms(20);
    servo4.pulsewidth(pwm_mid);
}

void Servo_run( int n1, int n2, int n3,int n4 ){

    NYX_PWM100_1(n1);        
    NYX_PWM100_2(n2);
    NYX_PWM100_3(n3);
    NYX_PWM100_4(n4);
    //   wait(w1);

} 

void Servo_s(double x,double s,double i){
    double v1;
    double v2;
    //    double v3;
    //  double v4;

    x = 3.14159265 * x / 180.0;
    v1 = (100.0-i)*(cos(x)+sin(x));
    v2 = (100.0-i)*(cos(x)-sin(x));
    Servo_run( (int)(v1+i),(int)(v2+i),(int)(-v1+i),(int)(-v2+i));
    //wait(s);
}


void go_motor(){

    move = 1;
    motor1 = 1;
} 

void turn_motor(){

    NYX_PWM100_1(100);        
    NYX_PWM100_2(100);
    NYX_PWM100_3(100);
    NYX_PWM100_4(100);
    //   wait(w1);
} 

void pc_rx1(){
    char c = pc.getc();   
    if( c == 'j'){
        pc.putc(c);
        x1 += 0.1;
        wait(x1);
    }
}
 
 
//---------------------------------------------------------------------------------------
//rotary
//----------------------------------------------------------------------------------------
    int right_arm; //1=move 0=stop
    int dir;
    int pw; //p§Œä‚©‚¯‚Ä‚é
    float total_pw;
    int timer=0;
    int mokuhyou;
    int f_init_right_arm =1;
    
void start_right_arm(int _mokuhyou,float _total_pw){

    mokuhyou=_mokuhyou;
    total_pw=_total_pw;
    right_arm = 1;
}

void move_right_arm(int dx,float _total_pw){

    mokuhyou += dx;
    total_pw=_total_pw;
    right_arm = 1;
}

void stop_right_arm(){
    right_arm=0;
}

void init_right_arm(){
    start_right_arm(-800000,0.25);//-80000 ‚Í˜r‚Ì’·‚³‚Ì‘S‘Ì
    f_init_right_arm = 1;
}

void go_init_right_arm(){
    start_right_arm(800000,0.25);//80000 ‚Í˜r‚Ì’·‚³‚Ì‘S‘Ì
    f_init_right_arm = 1;
}

void motor_printf(){    
    int x;
   
    x = wheelL.getPulses();  
        pc.printf("Pulses: %07d\r\n", x);
        pc.printf("Rotate: %04.3f\r\n", (double)x/(ROTATE_PER_REVOLUTIONS*4));   

        pc.printf("pw %07d\r\n", pw);
        pc.printf("dir %07d\r\n", dir); 
        pc.printf("timer %07d\r\n",timer );                     
        pc.printf("right_arm %d\r\n\r\n",right_arm);  
        

        //     pc.printf("motor1 %07d\n", motor1); 
   //     pc.printf("move: %07d\n", move); 

        }

void tatiagari1(){
    stop_right_arm();
    wheelL.reset();
    led1 = 1;
}

void tatiagari2(){
    stop_right_arm();
    led2 = 1;
   // wheelL.reset();
}

void tatiagari3(){
    stop_right_arm();
    led3 = 1;
   // wheelL.reset();
}

void tatiagari4(){
    stop_right_arm();
    led4 = 1;
 //   wheelL.reset();
}

int swc1;
int swc2;
int swc3;
int swc4;

void flip(){
    
    int x;
    x = wheelL.getPulses();      
    //   pw = x/2;
    timer++;   
    if(timer>100){
        timer = 0;
    }
    if(dir == 0 || right_arm == 0 ){
        move = 0;
        motor1= 0;

    }else if(dir == 1){
        if(pw > timer){
            move = 1;
            motor1 = 1;
        }
        else{
            move = 0;
            motor1 = 0;
        }
    }else if(dir == 2){
        if(pw > timer){
            move = 0;
            motor1 = 1;
        }
        else{
            move = 0;
            motor1 = 0;
        }
    }


    if(x<mokuhyou-500){
        dir = 1;
    

    }else if(x>mokuhyou+500){
        dir = 2;

    }else{
        dir = 0;
        stop_right_arm();
    }
    //pw = std::min(abs(x-mokuhyou)/2,100)*total_pw;
    if(abs(x-mokuhyou)/2 < 100){
    pw = (abs(x-mokuhyou)/2)*total_pw;
    
    }
    else{
    pw = 100*total_pw;
    }
    if(x<3000 && f_init_right_arm != 1 && dir != 1){
    pw *= x/3000;
    
    }else if(x>77000 && f_init_right_arm != 1 && dir != 2){
    pw *= (80000-x)/3000;
    
    }
    //flipper.attach(&motor_printf,2.0);       
 
    if(sw1 == 1 ){
        swc1 += 1;
        if(swc1 >= 100){
            tatiagari1();
            f_init_right_arm = 0;
            move = 0;
            motor1 = 0;
            wait(0.00002);
            while(sw1==1){
            
            move = 1;
            motor1 = 1;
            }
            
        }
    }else{
        swc1 = 0;
        led1 = 0;
    }

    if(sw2 == 1 ){
        swc2 += 1;
        if(swc2 >= 100){
            tatiagari2();
            move = 0;
            motor1 = 0;
            wait(0.00001);
            while(sw2==1){
            move = 0;
            motor1= 1;
            }
        }
    }else{
        swc2 = 0;
        led2 = 0;
    }

    if(sw3 == 1 ){
        swc3 += 1;
        if(swc3 >= 100){
            tatiagari3();
        }
    }else{
        swc3 = 0;
        led3 = 0;
    }

    if(sw4 == 1 ){
        swc4 += 1;
        if(swc4 >= 100){
            tatiagari4();
        }
    }else{
        swc4 = 0;
        led4 = 0;
    }
}

//void motor_printf(){
  //      pc.printf("Pulses: %07d\n", x);
    //    pc.printf("Rotate: %04.3f\n", (double)x/(ROTATE_PER_REVOLUTIONS*4));        
      //  }



/*
void start_right_arm(int _mokuhyou,float _total_pw){

    mokuhyou=_mokuhyou;
    total_pw=_total_pw;
    right_arm = 1;
}

void stop_right_arm(){
    right_arm=0;
}
*/
//----------------------------------------------------------------------------------------
//upper body
//----------------------------------------------------------------------------------------


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

        unsigned char TxData[48];    // TransmitByteData [48byte]
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

        TxData[47] = CheckSum;      

        // Send Packet 
        REDE = 1;                   // RS485 Transmit Enable
        for(int i=0; i<=47; i++){
            device.putc(TxData[i]);
        }
        wait_us(250);               // Wait for transmission
        REDE = 0;                  
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

    TxData[7] = (unsigned char)0x00FF & data;          
    TxData[8] = (unsigned char)0x00FF & (data >> 8);    

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
    TxData[11] = CheckSum;       

    for(int i=0; i<=11; i++){
        device.putc(TxData[i]);
    }
    wait((float)spd/100);
}


int main() {
    init();                     // initialize
    torque(0xFF, 0x01); // ID, torque = OFF (0x00)
    wait(1);   
    Servo_init();                    
  
    flipper1.attach(&flip, 0.001);
    flipper2.attach(&motor_printf,1.0);
  //  sw1.rise(&tatiagari1);
   // sw2.rise(&tatiagari2);
   // sw3.rise(&tatiagari3);
   // sw4.rise(&tatiagari4);


    while(1){      
        char c = pc.getc();      
        int x;
        x = wheelL.getPulses();                      
;
//----------------------------------------------------------------------------------------
//R go l
//----------------------------------------------------------------------------------------
        if(c == 'l'){
            pc.putc(c);
            Servo_run(-100,0,100,0); //lower body servo command
            wait(0.4);
        }

//----------------------------------------------------------------------------------------
//R back 2
//----------------------------------------------------------------------------------------
        if(c == '2'){
            pc.putc(c);
            Servo_run(0,100,0,-100); //lower body servo command
            wait(0.4);
        }

//----------------------------------------------------------------------------------------
//L go 3
//----------------------------------------------------------------------------------------
        if(c == '3'){
            pc.putc(c);
            Servo_run(0,-100,0,100); //lower body servo command
            wait(0.4);
        }

//----------------------------------------------------------------------------------------
//L back 4
//----------------------------------------------------------------------------------------
        if(c == '4'){
            pc.putc(c);
            Servo_run(100,0,-100,0); //lower body servo command
            wait(0.4);
        }
//----------------------------------------------------------------------------------------
//go 5
//----------------------------------------------------------------------------------------
        if(c == '5'){
            pc.putc(c);

            Servo_run(-100,-100,100,100); //lower body servo command
            wait(1);

        }
        
//----------------------------------------------------------------------------------------
//back 6
//----------------------------------------------------------------------------------------

        if(c == '6'){
            pc.putc(c);
            Servo_run(100,100,-100,-100); //lower body servo command
            wait(0.4);
        }

//----------------------------------------------------------------------------------------
//go and turn 1
//----------------------------------------------------------------------------------------
        if( c == '1'){
            pc.putc(c);
            double i;
            double v1;
            double v2;
            double q;
            double n=0;
            for(i=0.0;i<(360.0);i+=5.625){ //10 2 wait
                //Servo_s(i+180.0,1.0/360.0,0);
                //void Servo_s(double x,double s,double i){
                //    double v1;
                //     double v2;
                //    double v3;
                //  double v4;

                q = 3.14159265 * i / 180.0;            
                // q = 3.14159265 * (floor(i/10.0)*10.0-90.0) / 180.0;
                v1 = (70.0-n)*(cos(q)+sin(q));
                v2 = (70.0-n)*(cos(q)-sin(q));
                Servo_run( (int)(v1+n),(int)(v2+n),(int)(-v1+n),(int)(-v2+n));

                wait(0.2083333333333344);  

                //   wait((10.0/360.0)*20.0);
                Servo_run(100,100,100,100);
                //wait(0.0022);
                //wait(0.0001);
                wait(0.0455625);
                // wait(0.0615625);
                //wait((2.7/36.0)*2.0);
            }

            Servo_run(0,0,0,0);
            wait(0.1);

        }



//----------------------------------------------------------------------------------------
//go & turn 8
//----------------------------------------------------------------------------------------


        if(c == '8'){        
            pc.putc(c);       
            int n = 5;
            while( n > 0 ){ 
                Servo_run(100,100,100,100);
                wait(0.5);
                Servo_run(-100,-100,100,100);         
                wait(0.2);
                Servo_run(100,100,100,100);            
                wait(1.2);
                Servo_run(100,100,-100,-100);
                wait(0.4);
                Servo_run(100,100,100,100);
                wait(0.91);
                Servo_run(0,0,0,0); 
                //    flipper.detach(); 
                //  Servo_run(100,100,100,100);  
                //  wait(1.52);                   
                n--;
                //  flipper.attach(&turn_motor,0.5); 

                //  flipper.detach();

                // wait(0.4 );
            } 
        }       
        
//----------------------------------------------------------------------------------------
//turn ccw 9
//----------------------------------------------------------------------------------------
        if(c == '9'){
            pc.putc(c);

            Servo_run(100,100,100,100); //lower body servo command
            wait(0.4);
        }

//----------------------------------------------------------------------------------------
//turn  cw q
//----------------------------------------------------------------------------------------
        if(c == 'q'){
            pc.putc(c);

            Servo_run(-100,-100,-100,-100); //lower body servo command
            wait(0.4);
        }


//----------------------------------------------------------------------------------------
//stop 0
//----------------------------------------------------------------------------------------      
        if(c == '0'){
            pc.putc(c);

            Servo_run(0,0,0,0); //lower body servo command
            wait(0.4);
        }
 

//----------------------------------------------------------------------------------------
//swing  before j
//----------------------------------------------------------------------------------------

        if(c == 'j'){
            pc.putc(c);
            torque(0xFF, 0x01);  // torque = OFF(0x00), ON(0x01), BRAKE(0x02)
            //            wait(1);                   
            //           long_1_8(450,450,450,-450,450,-450,450,-450,5);//upper body command
            wait(1);
            long_1_8(350,450,150,1150,0,-150,250,0,5);//upper body command 
            // migi  kosi  hidari up doen 
            wait(1);
            //  long_1_8(350,450,450,550,0,-150,250,0,5);//upper body command
        }

//----------------------------------------------------------------------------------------
//swing  after k
//----------------------------------------------------------------------------------------

        if(c == 'k'){
            pc.putc(c);


            torque(0xFF, 0x01);  // torque = OFF(0x00), ON(0x01), BRAKE(0x02)
            wait(1);                   
            //          long_1_8(550,450,450,250,450,0,0,0,5);//upper body command
            wait(1);    
            long_1_8(550,450,450,550,450,0,0,0,5);//upper body command
            // long_1_8(550,450,900,550,450,0,0,0,5);//upper body command
        }


//----------------------------------------------------------------------------------------
//initialize  h 
//----------------------------------------------------------------------------------------    
        if(c == 'h'){
            pc.putc(c);

            torque(0xFF, 0x01);  // torque = OFF(0x00), ON(0x01), BRAKE(0x02)
            wait(1);                   
            long_1_8(450,450,450,-450,450,-450,450,-450,5);//upper body command
        }

//----------------------------------------------------------------------------------------
//new  p
//-----------------------------------------------------------------------------------------
        if(c == 'p'){
            pc.putc(c);

            torque(0xFF, 0x01);  // torque = OFF(0x00), ON(0x01), BRAKE(0x02)
            wait(1);                   
            long_1_8(450,450,450,-450,450,-450,450,-450,5);//upper body command  j
            wait(1);
            long_1_8(350,450,150,1150,0,-150,250,0,5);//upper body command      k
            wait(1);
            move = 0;
            motor1 = 1;
            //flipper.attach(&go_motor,1.0);
            long_1_8(550,450,450,550,450,0,0,0,5);//upper body command
            wait(0.4);
        }



//----------------------------------------------------------------------------------------    
//----------------------------------------------------------------------------------------
//rotary  
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------
//rotary reset
//----------------------------------------------------------------------------------------
        //L
        if( c == 'd'){
            pc.putc(c);        
            wheelL.reset();
            pc.printf("Pulses: %07d\n", x);
            //           lcd.printf("Pulses: %07d\n", y); 
        }                
          

//----------------------------------------------------------------------------------------
//rotary up 
//----------------------------------------------------------------------------------------
        //L
        if( c == 't'){
            pc.putc(c);        

            move = 1;
            motor1 = 1;

            //           wheel.reset();
            pc.printf("Pulses: %07d\n", x);
            pc.printf("Rotate: %04.3f\n", (double)x/(ROTATE_PER_REVOLUTIONS*4));
        }

//-----------------------------------------------------------------------------------------
//rotary down    
//-----------------------------------------------------------------------------------------

        //L
        if( c == 'c'){
            pc.putc(c);        
            move = 0;
            motor1 = 1;
            pc.printf("Pulses: %07d\n", x);
            pc.printf("Rotate: %04.3f\n", (double)x/(ROTATE_PER_REVOLUTIONS*4));
        
        }



//----------------------------------------------------------------------------------------
//rotary stop
//----------------------------------------------------------------------------------------

        //L
        if( c == 'f'){
            pc.putc(c);        
            move = 0;
            motor1 = 0;
            //         wheel.reset();
            lcd.printf("Pulses: %07d\n", x);
            lcd.printf("Rotate: %04.3f\n", (double)x/(ROTATE_PER_REVOLUTIONS*4));
        }
                
 
//------------------------------------------------------------------------------------------------------             
//arm rotary m
//-------------------------------------------------------------------------------------------------------         
        if( c ==  'm'){   
        pc.putc(c);
        start_right_arm(15000,1.0);
        }
        if( c ==  'e'){   
        pc.putc(c);
        start_right_arm(-15000,1.0);
        } 
        
        if( c ==  'x'){   
        pc.putc(c);
        start_right_arm(10000,0.5); 
        }     
        
        if( c ==  'z'){   
        pc.putc(c);
        start_right_arm(-10000,1.0);
        }
  
        if( c ==  'g'){   
        pc.putc(c);
        start_right_arm(1000,0.1);
        }

        if( c ==  'r'){   
        pc.putc(c);
        start_right_arm(1000,0.5);
        } 


        if( c ==  'w'){   
        pc.putc(c);
        start_right_arm(1000,1.0);
        } 
              
  
        if( c ==  'v'){   
        pc.putc(c);
        start_right_arm(-40000,1.0);
        }
                                
        if( c == 'b'){
            init_right_arm();
        }
        if( c == 'a'){
            go_init_right_arm();
        }
    }
}
