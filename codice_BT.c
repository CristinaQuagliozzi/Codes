#include <16F887.h>
#device ICD=TRUE
#fuses HS,NOLVP,NOWDT
#use delay (clock=20000000)
#use rs232 (stream=PROMI,baud=9600,  xmit=PIN_C3, rcv=PIN_C2)
#define Motoredx PIN_B4
#define Motoresx PIN_B5



int1 ciclo;
int8 tmp;
int8 comando;

#int_timer2
void timer2_isr(){
   if (ciclo==true){
      if (comando==1){
         output_high(Motoredx);
         delay_us(1300);
         output_low(Motoredx);
         output_high(Motoresx);
         delay_us(1700);
         output_low(Motoresx);
      }
      if (comando==2){
         output_high(Motoredx);
         delay_us(1500);
         output_low(Motoredx);
         output_high(Motoresx);
         delay_us(1700);
         output_low(Motoresx);
      }
      if (comando==3){
         output_high(Motoredx);
         delay_us(1300);
         output_low(Motoredx);
         output_high(Motoresx);
         delay_us(1500);
         output_low(Motoresx);
      }
      if (comando==4){
         output_high(Motoredx);
         delay_us(1700);
         output_low(Motoredx);
         output_high(Motoresx);
         delay_us(1300);
         output_low(Motoresx);
      }
      if (comando==5){
         output_high(Motoredx);
         delay_us(1500);
         output_low(Motoredx);
         output_high(Motoresx);
         delay_us(1500);
         output_low(Motoresx);
      }
      ciclo=false;
   } 
   else{
      ciclo = true;
   }
}


void main () {  
   setup_timer_2(T2_DIV_BY_16, 209, 15);
   enable_interrupts(int_timer2);
   enable_interrupts(global);
   
   ciclo=true;
   while(TRUE){
      tmp = getc();;
      if (tmp=='a'){
         comando=1;
      }
      if (tmp=='d'){
         comando=2;
      }
      if (tmp=='s'){
         comando=3;
      }
      if (tmp=='b'){
         comando=4;
      }
      if (tmp=='p'){
         comando=5;
      }
   }
}
