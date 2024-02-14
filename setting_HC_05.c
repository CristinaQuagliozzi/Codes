#include <16F887.h>
#device ICD=TRUE
#fuses HS,NOLVP,NOWDT
#use delay (clock=20000000)
#use rs232 (stream=BT,baud=9600,  xmit=PIN_A1, rcv=PIN_A3)
#use rs232 (stream=PC,baud=9600,  xmit=PIN_C6, rcv=PIN_C7)





void main () {   

   
   while(true){
      if(kbhit(PC)){
         fputc(fgetc(PC),BT);
      }
      
      if(kbhit(BT)){
         fputc(fgetc(BT),PC);
      }
   }

}
