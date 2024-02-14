#include <16F887.h>
#device ICD=TRUE
#fuses HS,NOLVP,NOWDT
#use delay (clock=20000000)
#use rs232 (DEBUGGER)

// PIN_C1 corresponds to ccp1. It captures the rising edge of signal.
// PIN_C2 corresponds to ccp2. It captures the falling edge of signal.

#define SENSORE PIN_A1      //definiamo il pin a cui colleghiamo l'output del sensore

// Questa funzione � usata per attivare il sensore. Come da datasheet � necessario eseguire le seguenti operazioni
void attiva_sensore() {                   
     output_high(SENSORE); // definisco il pin come output e lo metto alto
     delay_us(10);
     output_float(SENSORE); // definisco il pin come input
}


#int_ccp2 //These directive specify the following function is an interrupt function, called when ccp2 detects the event of a falling edge.
          //When the interrupt is detected, the compiler will generate code to jump to the function here below.
void isr(){ //Interrupt Service Routine
     disable_interrupts(INT_CCP2); //Disable this interrupt, in order to avoid that a new event recalls // it. At the end of isr interrupt will be renabled.  
     long rise, fall, p_w, time; // definisco le variabili per trovare il tempo di decadimento
     
     rise=CCP_1; //CCP_1 register contains the timer value when the rising edge has been detected // by the ccp1 module
     fall=CCP_2; //CCP_2 register contains the timer value when the falling edge has been detected // by the ccp2 module
     p_w=fall-rise; //time of flight, measured in clock counts
     time=p_w/(20/4);    //tempo in microsecondi, svolge una istruzione ogni /4 di clock     // time of flight, measured in us. 20MHz is the clock frequency, 4 is the  // number of clock counts needed to execute a single instruction.     
     
     printf("\r\n Time in us is: %lu",time);
     
     enable_interrupts(INT_CCP2); // renable ccp2 interrupt
}

void main() {
      setup_ccp1(CCP_CAPTURE_RE);     // Configure CCP1 to capture rise edges
      setup_ccp2(CCP_CAPTURE_FE);     // Configure CCP2 to capture fall
      setup_timer_1(T1_INTERNAL);     // Initializes timer 1, sets the internal clock as source
      enable_interrupts(INT_CCP2);    // Enables the specified interrupt
      enable_interrupts(GLOBAL);      //The GLOBAL level will not enable any of the specific interrupts, but will allow any of the
                          //specified interrupts previously enabled to become active.
   
      while(TRUE){                    
            attiva_sensore(); // send a trigger to the sensor and activates the distance detection
            delay_ms(60);   // dopo 60 ms riattivo il sensore e prendo un'altra misura
      }
}
