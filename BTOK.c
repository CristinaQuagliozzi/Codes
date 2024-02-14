#include <16F887.h> //microcontrollore
#device ICD=TRUE
#fuses HS,NOLVP,NOWDT 
#use delay (clock=20000000) // freq oscillatore primario 20 MHz
#use rs232 (stream=PROMI,baud=9600,  xmit=PIN_C3, rcv=PIN_C2) //baudrate 9600, xmit = pin trasmissione , rcv = pin ricezione, PROMI = parità
#define Motoredx PIN_B4 // pin ingresso motore dx
#define Motoresx PIN_B5 // pin ingresso motore sx



int1 ciclo; // 1bit (1 o 0), variabile che può avere un bit al massimo, oppure true o flase 
int8 tmp; // converte la stringa in un numero a 8bit che mi identifica un carattere 
int8 comando;


#int_timer2 //entro nell'interrupt 
void timer2_isr(){ //leggo registro di stato timer2 interrupt
   if (ciclo==true){
      if (comando==1){  //comando 1 = avanti
         output_high(Motoredx); 
         delay_us(1300);
         output_low(Motoredx);
         output_high(Motoresx);
         delay_us(1700);
         output_low(Motoresx);
      }
      if (comando==2){  //comando 2 = destra, il motore di destra va avanti con 1300 e sinistra va avanti con 1700
         output_high(Motoredx);
         delay_us(1500);
         output_low(Motoredx);
         output_high(Motoresx);
         delay_us(1700);
         output_low(Motoresx);
      }
      if (comando==3){   // comando 3 = sinistra
         output_high(Motoredx);
         delay_us(1300);
         output_low(Motoredx);
         output_high(Motoresx);
         delay_us(1500);
         output_low(Motoresx);
      }
      if (comando==4){ // comando 4 = indietro
         output_high(Motoredx);
         delay_us(1700);
         output_low(Motoredx);
         output_high(Motoresx);
         delay_us(1300);
         output_low(Motoresx);
      } 
      if (comando==5){  //comando 5 = stop
         output_high(Motoredx);
         delay_us(1500);
         output_low(Motoredx);
         output_high(Motoresx);
         delay_us(1500);
         output_low(Motoresx);
      }
      ciclo=false; //Esce dal 'ciclo' in quanto sono finite le istruzioni da eseguire
   } 
   else{
      ciclo = true;  
   }
}

// Si entra nell'interrupt e si esegue cosa c'� all'interno dell'interrupt, per cui se il ciclo � true vengono eseguiti
// i comandi (1,2,3,4,5), alla fine dei comandi il ciclo � false (0), si esce dall'interrupt perch� i comandi sono finiti, 
//la prossima volta che si entra nell'interrupt il ciclo � false, per cui non entra nel primo if, ma nell'else in cui il ciclo
//diventa true. Sostanzialmente quando entro nell'interrupt la prima volta ho il segnale alto, se rientro il ciclo � false 
//quindi basso. Nell'else non viene svolto nessun comando, viene solamente cambiata la varibiale ciclo, lo faccio perch�
//i motori devono stare bassi per 20ms dopo l'istruzione che i motori sono alti. Entro nell'interrupt ogni 10 ms, muovo i motori
//esco dall'interrupt, il prossimo interrupt � tra 10 ms, rientra nell'interrupt (else ciclo=true, viene settata la variabile),
// esce ed il prossimo interrupt avviene tra altri 10 ms, in questo modo sono passati 20 ms in cui i motori sono bassi.


//inizializzo timer2 + interrupt
void main () {  
   setup_timer_2(T2_DIV_BY_16, 209, 15); //prescaler = 16 aumento ogni 16 ciclo istruzioni di 1, period = 209, postscaler = 15 (10 ms= 16*15*200ns*x), lo faccio per contare 10ms, altrimenti conta 50ns (200ns*255)
   enable_interrupts(int_timer2);  //abilito enable interrupt
   enable_interrupts(global);// si puo usare sempre nella funzione, in ogni punto 
   ciclo=true;
   while(TRUE){
      tmp = getc();; //prende la variabile tmp, prendo quello che clicco sul bt e la salvo per fare eseguire il comando 
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
