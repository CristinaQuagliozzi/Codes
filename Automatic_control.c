#include <16F887.h>
#device ICD=TRUE //genera codici compatibili col pic,ICD (In Circuit Debug) è un modulo integrato nei PIC che permette di effettuare un debug senza richiedere hardware speciale.
#fuses HS,NOLVP,NOWDT //direttive del chip 
#use delay (clock=20000000)
#use rs232 (stream=PROMI,baud=115200, xmit=PIN_C6, rcv=PIN_C7)//baud=115200 per aumentare la velocita di trasmissione 
// stream= imposta la porta(tipo do stream della comunzione)
//xmit = piedino di trasmissione
//rcv= piedino di ricezione
#define SENSOR_1 PIN_A1 //definiamo il pin a cui colleghiamo l'output del sensore 
#define SENSOR_2 PIN_A2
#define SENSOR_3 PIN_A3
#define Motoredx PIN_B4 //pin a cui collego il motore
#define Motoresx PIN_B5

//La direttiva #fuses HS,NOWDT,NOPROTECT,NOLVP determina il settaggio della Configuration Word interna al PICmicro. Questo particolare registro contiene dei flag che devono essere settati opportunamente durante la programmazione del PICmicro per consentire un corretto funzionamento. Ogni flag o fuses può essere settato con questa direttiva inserendo il nome relativo. In questo caso:

//HS L'oscillatore lavora con un cristallo esterno con frequenza superiore a 4Mhz
//NOWDT Il watch dog timer è disabilitato
//NOLVP La programmazione in modalitò Low Power è disattivata



int1 sens_1,sens_2,sens_3; //variabili binarie 
int1 curva;
//Calibrazione motori, si setta il fododiodo alto e vediamo il tempo che ci mette per andare basso, il tempo dipende dal colore che legge.
//Quando definisco il pin come input è come se staccassi il pin di output dall’alimentazione: da questo momento quindi inizio a contare per capire qual è il tempo di decadimento. 
void sens_read () {
   set_tris_a (0b00000000);//imposto i vari bit 0 output //con questa linea diciamo che tuD i pin da A0 a A7 devono essere definiti come output
   output_high(SENSOR_1); 
   output_high(SENSOR_2); 
   output_high(SENSOR_3);
   delay_us(10);  // serve per non mandare il segnale subito basso. (tempo di scarica condensatore)
   set_tris_a (0b00001110); //1 input imposto i pin A1, A2, A3 come input
   delay_us(250); // sopra 250 non è bianco (250 è il tempo di decadimento minimo (quando vedo bianco))microsecondi
   if (input_state(SENSOR_1)){
      sens_1=1; //se sens1 = true, la variabile sens è true=nero, 1 è nero e 0 è bianco 
   }
   else{
      sens_1=0; }   
   if (input_state(SENSOR_2)){
      sens_2=1;
   }
   else{
      sens_2=0;}   
   if (input_state(SENSOR_3)){
      sens_3=1;
   }
   else{
      sens_3=0;}
   delay_us(500);
} // se è ancora alto, pongo questa variabile a 1, quindi sono su una superficie nera, al contrario la pongo a zero
void main(){
   sens_1 = 0; //INIZIALIZZO I SENSORI
   sens_2 = 0;
   sens_3 = 0;
   while(true){
      sens_read(); 
      if (((sens_3==0) && (sens_2==1) && (sens_1==0))) { //sensore 3 = 0; sensore 2 = 1 --> sensore1 = 0 vado dritto
         output_high(Motoredx);
         delay_us(1300); // orario 
         output_low(Motoredx);
         delay_ms(20); //basso per 20ms,seconda parte del duty cicle
         output_high(Motoresx);
         delay_us(1700);// antiorario
         output_low(Motoresx);
         delay_ms(20);
         
      }
      if ((((sens_3==1) && (sens_2==0) && (sens_1==0)))) { //sensore 3 =1; sensore2= 0; sensore1 =0 --> giro a destra
         output_high(Motoredx);
         delay_us(1300);
         output_low(Motoredx);
         delay_ms(20);
         output_high(Motoresx);
         delay_us(1700);
         output_low(Motoresx);
         delay_ms(20);
         curva = TRUE;
         
      }
      if ((((sens_3==0) && (sens_2==0) && (sens_1==1)))) { // sensore3=0; sensore2=0; sensore1=1 --> giro a sinsitra
         output_high(Motoredx);
         delay_us(1300);
         output_low(Motoredx);
         delay_ms(20);
         output_high(Motoresx);
         delay_us(1700);
         output_low(Motoresx);
         delay_ms(20);
         curva = FALSE;
         
       }
       if ((sens_3==0) && (sens_2==0) && (sens_1==0) && curva==TRUE) { //Il sensore si trova fuori dalla guida
         output_high(Motoredx);
         delay_us(1500); //inizio senso antiorario a 1500
         output_low(Motoredx);
         delay_ms(20); ??
         output_high(Motoresx);
         delay_us(1600); //riduzione velocit� in curva
         output_low(Motoresx);
         delay_ms(20);
         
       } 
       if ((sens_3==0) && (sens_2==0) && (sens_1==0) && curva==FALSE) { //Il sensore si trova fuori dalla guida
         output_high(Motoredx);
         delay_us(1400);
         output_low(Motoredx);
         delay_ms(20);
         output_high(Motoresx);
         delay_us(1500);
         output_low(Motoresx);
         delay_ms(20);
         
         } 
         if ((sens_3==1) && (sens_2==1) && (sens_1==1)){ //fine si ferma, vede tutto nero 
         output_high(Motoredx);
         delay_us(1500);
         output_low(Motoredx);
         delay_ms(20);
         output_high(Motoresx);
         delay_us(1500);
         output_low(Motoresx);
         delay_ms(20);
         }
   }
}
