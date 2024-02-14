/********************************************************************
          Include Wire Library for I2C, OLED and graphics Library
********************************************************************/
#include <Wire.h> //libreria per I2C
#include <Adafruit_SSD1306.h> //librerie per schermo oled 
#include <Adafruit_GFX.h>

/********************************************************************
          BASIC INTERFACE SETTINGS
********************************************************************/
const int pinDir = 2; //variabili per il motore definite cosi in classe
const int pinStep = 3;

#define OLED_ADDR 0x3C //definizione dello schermetto 
Adafruit_SSD1306 display(-1); //inizializzazione dello s hermo fornita dal datasheet 

/********************************************************************
         Arduino pins
********************************************************************/
//Buttons Pull-down configuration: 10kohm resistor
//so that the pin is connected to gnd when the switch isn't pressed.
//Prevents the floating of the pin 

const int buttonPin5 = 5; // Right (+) 
const int buttonPin6 = 6; // MenuCount, scrollo le opzioni per il menu
const int buttonPin7 = 7; // Left (-)

const int buttonPin8 =8; // (ok)

/********************************************************************
         Variables
********************************************************************/
float encoder0Pos = 0; //ci serve per incrementare di 0.1 i valori relativi al volume e al tempo che selzioniamo  

byte clk; //su questa leggiamo lo stato del bottone associato al pin 6 per scrollare il menu 

int buttonState_ok = 0; // Current state of the button, variabili relativi ai bottini per vedere se è premuto o no 
int buttonflag = 0;    // Variable for the button press state

String  stato = "start"; // String variable to lock the states on the Nested Menu, variabile definita come stringa e inizializzata a start defnisce il menu principale di tutto 


int menuCount = 1; // Position Menu Cursor counter, verra aggiornatat ogni volta che premo il tasto oper lo scorrimento associato al pin 6 

int progress = 0;// relativa alla siringa, disegno della siringa.

/********************************************************************
          BASIC STEPPER SETTINGS--Maths
    ********************************************************************/

const float pi = 3.14159; 
const int step_angle_in_deg = 1.8; //non ci serve, all'inizio facevamo un controllo con l'angolo ma poi abbiamo deciso di andare a controllare i giri.
const int screw_lead_in_mm = 8; // pitch * number of starts = 8mm; pitch=2mm ,number of starts=4, avanzamento lineare 


int micro_step = 16;

int step_for_revolution = 200; //step necessari per un giro 

float velocita_desiderata = 10; //velocità in giri al minuto 

long velocita = velocita_desiderata * micro_step; // motor speed [RPM]
long delayStepToStep = (60*pow(10,6)) / (velocita * 200) ; //tempo che intercorre tra due fronti alti di salita, nell'onda quadra che va a piotare il motore.
long t = delayStepToStep/2; 

float syringe_volume_1mL = 1;
float syringe_volume_5mL = 5;
float syringe_volume_10mL = 10;

float syringe_inner_diameter_1mL = 4.7; //diametri delle siringhe 
float syringe_inner_diameter_5mL = 12.3;
float syringe_inner_diameter_10mL = 15.6;


float corresponding_height_1mL =  ( (pow(10,-3)) * syringe_volume_1mL * (pow(10,6)) )/(pi * (pow(syringe_inner_diameter_1mL/2,2))) ; //altezze ottenute dalla formula inversa del volume 
float corresponding_height_5mL =  ( (pow(10,-3)) * syringe_volume_5mL * (pow(10,6))  )/(pi * (pow(syringe_inner_diameter_5mL/2,2))) ;// 10^6 serve per adattare il numeratore ai mL
float corresponding_height_10mL =  ( (pow(10,-3)) * syringe_volume_10mL * (pow(10,6)) )/(pi * (pow(syringe_inner_diameter_10mL/2,2))) ;


float corresponding_number_revolutions_1mL = ( (corresponding_height_1mL)/(screw_lead_in_mm) ); //rotazioni necessarie per erogare 1 mL
float corresponding_number_revolutions_5mL = ( (corresponding_height_5mL)/(screw_lead_in_mm) );
float corresponding_number_revolutions_10mL = ( (corresponding_height_10mL)/(screw_lead_in_mm) );


float required_steps_1mL = (corresponding_number_revolutions_1mL * step_for_revolution) ;// moltiplico tutto per una rivoluzione e ottengo i giri che servono per iniettare un certo volume 
float required_steps_5mL = (corresponding_number_revolutions_5mL * step_for_revolution) ;
float required_steps_10mL = (corresponding_number_revolutions_10mL * step_for_revolution) ;


float step_under_mL_in_n_1mL = ((required_steps_1mL)*(micro_step)); //moltiplico per il microstepping number 
float step_under_mL_in_n_5mL = ((required_steps_5mL)*(micro_step));
float step_under_mL_in_n_10mL = ((required_steps_10mL)*(micro_step));

/********************************************************************
         Selection Variables
********************************************************************/

int selected_syringe_mL = 0;
float selectedTime;
float volume_selected;
float current_step_under_mL_in_n;
int step_under_mL_in_n_for_selected_volume;

/********************************************************************
           Setup
********************************************************************/

void setup() {
  
  pinMode(pinStep, OUTPUT); //dichiaro le modalita dei pin 
  pinMode(pinDir, OUTPUT);
  
  delay(100);
  pinMode(buttonPin5, INPUT);
  pinMode(buttonPin6, INPUT);
  pinMode(buttonPin7, INPUT);
  pinMode(buttonPin8, INPUT);

  Wire.begin(); //inizializzazione libreria per I2C 
  Serial.begin(9600);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR); //inizializzazione del display 
  display.display(); //display stesso
  display.clearDisplay(); //pulizia del display 

}

void loop() {

  clk = digitalRead(6); //nel loop viene letto continuamente lo stato del bottone associato al pin 6 associato alla variabile clk definita come un byte 
  
  buttonStatus();
  menuCheck();
  doEncoder();

  custom_delivery_volume();
  custom_load_volume();
  time_changer();

  Menu();
  drawcursor();

  esecuzione();

  delay(50); //solving Debounce Problem

  //Serial.println(buttonflag);
  //Serial.println(stato);
  //Serial.println(menuCount);
  //Serial.println(selectedTime);
  //Serial.println(selected_syringe_mL);
  //Serial.println(encoder0Pos);
  //Serial.println(step_under_mL_in_n_5mL)
}

/********************************************************************
           Main Code
********************************************************************/

void Menu() { //il menu segue la logica degli automi: numero di stati finiti che si legano tra loro mediante transizioni 

  /********************************************************************
           Layer Start
  ********************************************************************/
  if(stato =="start"){ //per passare da uno stato all'altro ci sara una transizione che nel nostro caso sarà la premuta di un tasto e si passera da un menu ad un altro 
   //questo è il primo menu definito dalla variabile start, che era quella iniziale. Quando è posto su start faccio tutto questo:
    display.clearDisplay(); //pulisco lo schermo 
    display.setTextColor(SSD1306_WHITE); //setto colore
    display.setTextSize(1); //setto dimensione testo 

    display.setCursor(10, 0); display.println("Infusion System");//posiziono le scritte tramite una funzione che prende in ingresso le x e y dei pixel
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE); // disegna una linea da un punto all'altro 
    display.setCursor(10, 8); display.println("Delivery"); //sono dei sottomenu in cui entriamo dopo 
    display.setCursor(10, 16); display.println("Loading");
    display.setCursor(10, 24); display.println("Settings");
  }
  /********************************************************************
           Layer System
  ********************************************************************/
  if(buttonflag == 1 && menuCount == 1 && stato == "start"){ //'buttonflag' relativo alla premuta del tasto relativo al pin 8 che diceva 'ok', se questo button passa ad 1 passiamo alla 
  //schermata sotto e si aggiorna lo stato inf_1 che ci serve per dopo 
    stato="inf_1";
    
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("Infusion System");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Version: Final");
    display.setCursor(10, 16); display.println("Group: 5");
    display.setCursor(10, 24); display.println("Test: Final");
  }
  //quando si lascia il bottone ok dobbiamo fare in modo che la schermata rimanga e lo facciamo imponendo un if e imponendo il button pari a 0 e lo stato pari a inf1
  if(buttonflag == 0 && menuCount == 1 && stato == "inf_1"){
    stato = "inf_1_1"; //aggiorniamo lo stato per la prossima condizione if 
    menuCount = 1; //ci serve per tornare poi alla condizione inziale 
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("Infiusion System");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Version: Final");
    display.setCursor(10, 16); display.println("Group: 5");
    display.setCursor(10, 24); display.println("Test: Final");
  }

  if(buttonflag == 1 && menuCount == 1 && stato == "inf_1_1"){
    display.clearDisplay();
    stato = "start";
  }

  /********************************************************************
           Layer Delivery, sviluppiamo dei sottomenu in cui possiamo entrare e settare dei parametri 
  ********************************************************************/
  if(buttonflag == 1 && menuCount == 2 && stato == "start"){
    stato = "del_1"; //aggiorno lo stato per il prossimo if 
    menuCount = 1; //aggiorno il menu count e lo riporto a 1 
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    //disegnamo sullo schermetto quello che vogliamo settare
    display.setCursor(10, 0); display.println("Delivery");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Volume Selected"); //possiamo selelzionare il tempo o il volume 
    display.setCursor(10, 16); display.println("Time Selected");
    display.setCursor(10, 24); display.println("---");
  }
    if(buttonflag == 0 && menuCount == 1 && stato == "del_1"){ //se rilasciamo il pulsante andiamoo ad aggiornare le variabili ma rimaniamo sempre nella stessa schermata
    stato = "del_2";
    menuCount = 1;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("Delivery");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Volume Selected");
    display.setCursor(10, 16); display.println("Time Selected");
    display.setCursor(10, 24); display.println("---");
  }
  /********************************************************************
           Layer Delivery
  ********************************************************************/
  if(buttonflag == 1 && menuCount == 2 && stato == "del_2"){
    stato = "go_1";
    menuCount = 1;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("Delivery");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Esegui full"); //si puo scegliere quale erogazione esseguire 
    display.setCursor(10, 16); display.println("Esegui custom");
    display.setCursor(10, 24); display.println("Esegui half");
  }
  if(buttonflag == 0 && menuCount == 1 && stato == "go_1"){ //mi da sempre il mantenimento della schermata quando rilascio il tasto e finiamo sempre nella schermata di sopra 
    stato = "go_2";
    menuCount = 1;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("Delivery");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Esegui full");
    display.setCursor(10, 16); display.println("Esegui custom");
    display.setCursor(10, 24); display.println("Esegui half");
  }

  /********************************************************************
           Back Doors, funzioni per tornare indietro 
  ********************************************************************/
  if(buttonflag == 1 && menuCount == 1 && stato == "go_2"){
    display.clearDisplay(); //pulisco il display e ci fa tornare alla schermata principale dei volumi 
    stato = "del_1";
  }
    if(buttonflag == 1 && menuCount == 1 && stato == "del_2"){ //se premiamo il tasto ma ci trovavamo allo stato del_2 allora torniamo  allo stato start 
    display.clearDisplay();
    stato = "start";
  }
  /********************************************************************
           Layer Loading, il discorso rimane lo stesso e il loading è il duale del delivery, varia soltanto uando impostiamo il motore perche scegliamo la oprtazione in senso contrario 
  ********************************************************************/
  if(buttonflag == 1 && menuCount == 3 && stato == "start"){
    stato = "load_1";
    menuCount = 1;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("Loading");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Volume Selected");
    display.setCursor(10, 16); display.println("Time Selected");
    display.setCursor(10, 24); display.println("---");
  }
  if(buttonflag == 0 && menuCount == 1 && stato == "load_1"){
    stato = "load_2";
    menuCount = 1;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("Loading");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Volume Selected");
    display.setCursor(10, 16); display.println("Time Selected");
    display.setCursor(10, 24); display.println("---");
  }
  /********************************************************************
           Load
  ********************************************************************/
  if(buttonflag == 1 && menuCount == 2 && stato == "load_2"){
    stato = "go2_1";
    menuCount = 1;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("Load");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Esegui full");
    display.setCursor(10, 16); display.println("Esegui custom");
    display.setCursor(10, 24); display.println("Esegui half");
  }
  if(buttonflag == 0 && menuCount == 1 && stato == "go2_1"){
    stato = "go2_2";
    menuCount = 1;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("Load");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Esegui  full");
    display.setCursor(10, 16); display.println("Esegui custom");
    display.setCursor(10, 24); display.println("Esegui  half");
  }

  /********************************************************************
           Back Doors
  ********************************************************************/
  if(buttonflag == 1 && menuCount == 1 && stato == "go2_2"){
    display.clearDisplay();
    stato = "load_1";
  }
  if(buttonflag == 1 && menuCount == 1 && stato == "load_2"){
    display.clearDisplay();
    stato = "start";
  }

  /********************************************************************
           Layer Settings 
  ********************************************************************/
  if(buttonflag == 1 && menuCount == 4 && stato == "start"){
    stato ="settings_1";
    menuCount =1;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("Settings");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Syringe Selection"); //ci da la possibilita di selezionare la siringa, i volumi e i tempi. 
    display.setCursor(10, 16); display.println("Time Selection");
    display.setCursor(10, 24); display.println("Volume Selection");
  }
  if(buttonflag == 0 && menuCount == 1 && stato == "settings_1"){
    stato ="settings_2";
    menuCount = 1;  
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("Settings");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Syringe Selection");
    display.setCursor(10, 16); display.println("Time Selection");
    display.setCursor(10, 24); display.println("Volume Selection");
  }
  /********************************************************************
           Layer Syringe Selection, sottomenu per le opzioni desiderate per la dose
  ********************************************************************/
  if(buttonflag == 1 && menuCount == 2 && stato == "settings_2"){
    stato = "syringe_selection_1";

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("Syringe Selection");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Volume: 1[mL]");
    display.setCursor(10, 16); display.println("Volume: 5[mL]");
    display.setCursor(10, 24); display.println("Volume: 10[mL]");
  }
  if(buttonflag == 0 && menuCount == 2 && stato == "syringe_selection_1"){
    stato = "syringe_selection_2";
    menuCount = 1;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("Syringe Selection");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Volume: 1[mL]");
    display.setCursor(10, 16); display.println("Volume: 5[mL]");
    display.setCursor(10, 24); display.println("Volume: 10[mL]");
  }
  if(buttonflag == 1 && menuCount == 2 && stato == "syringe_selection_2"){
    selected_syringe_mL = syringe_volume_1mL; //impostiamo le variabili che ci servono per i conti successivi 
  }else if(buttonflag == 1 && menuCount == 3 && stato == "syringe_selection_2"){
    selected_syringe_mL = syringe_volume_5mL;
  }else if(buttonflag == 1 && menuCount == 4 && stato == "syringe_selection_2"){
    selected_syringe_mL = syringe_volume_10mL;
  }

  /********************************************************************
           Layer Time Selection
  ********************************************************************/
  if(buttonflag == 1 && menuCount == 3 && stato == "settings_2"){
    stato = "time_selection_1";

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("Time Selection");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Modifica");
    display.setCursor(10, 16); display.println("  ");    // qui abbiamo la possibilita di scegliere noi un tempo qualsiasi  
    display.setCursor(10, 24); display.println("Minute");
  }
  if(buttonflag == 0 && menuCount == 3 && stato == "time_selection_1"){
    stato = "time_selection_2";
    menuCount = 1;
    encoder0Pos = 0;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("Time Selection");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Modifica");
    display.setCursor(10, 16); display.println("   ");    
    display.setCursor(10, 24); display.println("Minute");
  }
  if(stato == "time_selection_2" ){
    display.setCursor(10, 16); display.println(encoder0Pos);   //per stamapre il tempo selezionato lo ricaviamo dalla variabile encoder0pos, viene definita nle fondo del codice come 
    //una funzione e legge gli incrementi che diamo  se premiamo il tasto + o -, quindi raccoglie i valroi della variabile e li posiziona sopra nello spazio tra le virgolette. 
  }
  /********************************************************************
            Custom Volume Selection, si fa la stessa cosa di sopra solo per la scelta del volume 
  ********************************************************************/
  if(buttonflag == 1 && menuCount == 4 && stato == "settings_2"){
    stato = "custom_1";
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("Inserire Volume");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Modifica");
    display.setCursor(10, 16); display.println("  ");
    display.setCursor(10, 24); display.println("[mL]");

  }
  if(buttonflag == 0 && menuCount == 4 && stato == "custom_1"){
    stato = "custom_2";
    menuCount = 1;
    encoder0Pos = 0;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("Inserire Volume");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Modifica");
    display.setCursor(10, 16); display.println("  ");
    display.setCursor(10, 24); display.println("[mL]");
  }
  if(stato == "custom_2" ){
    display.setCursor(10, 16); display.println(encoder0Pos);   
  }


  /********************************************************************
           Back Doors, servono sempre per tornare alle chermate precedenti.
  ********************************************************************/
  if(buttonflag == 1 && menuCount == 1 && stato == "settings_2"){
    display.clearDisplay();
    stato = "start";
  }
  if(buttonflag == 1 && menuCount == 1 && stato == "syringe_selection_2"){
    display.clearDisplay();
    stato = "settings_1";
  }

  if(buttonflag == 1 && menuCount == 1 && stato == "time_selection_2"){
    display.clearDisplay();
    stato = "settings_1";
  }
  if(buttonflag == 1 && menuCount == 1 && stato == "settings3_2"){
    display.clearDisplay();
    stato = "settings_1";
  }
  if(buttonflag == 1 && menuCount == 1 && stato == "custom_2"){
    display.clearDisplay();
    stato = "settings_1";
  }

  /********************************************************************
           Error, si apre quando bypassiamo una schetmata senza selezionare qualcosa.
  ********************************************************************/
  if(stato == "errore"){
    menuCount = 1;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("ERRORE");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Selezionare");
    display.setCursor(10, 16); display.println("Siringa");
    display.setCursor(10, 24); display.println("Richiesta");
  }
   
  if(buttonflag == 1 && menuCount == 1 && stato == "errore"){ //se premiamo ok torniamo al menu che ci da la possbilità di selezionare la siringa 
    display.clearDisplay();
    stato = "settings_1";
  }

  if(stato == "errore_custom_time"){
    menuCount = 1;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("ERRORE");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Selezionare");
    display.setCursor(10, 16); display.println("Siringa e Tempo");
    display.setCursor(10, 24); display.println("Richiesti");
  }
  if(buttonflag == 1 && menuCount == 1 && stato == "errore_custom_time"){
    display.clearDisplay();
    stato = "settings_1";
  }

  if(stato == "errore_custom"){
    menuCount = 1;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("ERRORE");
    display.drawLine(7, 7, 121, 7, SSD1306_WHITE);
    display.setCursor(10, 8); display.println("Selezionare");
    display.setCursor(10, 16); display.println("Volume");
    display.setCursor(10, 24); display.println("Richiesto");
  }
  if(buttonflag == 1 && menuCount == 1 && stato == "errore_custom"){
    display.clearDisplay();
    stato = "settings_1";
  }

///********************************************************************
//          Erogazione in corso
// ********************************************************************/
    
  if(stato =="erogazione_in corso") {
    menuCount = 1;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("Task in Corso");
    do {
      display.setCursor(38, 22); display.println("Avanzamento");//serve per disegnare la siringa sul display 
      display.drawRect(40, 10, 60, 10, SSD1306_WHITE);  
      display.drawLine(25, 15, 40, 15, SSD1306_WHITE);
      display.drawLine(25, 12, 25, 18, SSD1306_WHITE);
      display.drawLine(100, 15, 105, 15, SSD1306_WHITE);
      display.fillRect(41, 12, progress, 6, SSD1306_WHITE);//incrementa il numero di pixel in modo da avere un effeto di avanzamento. 
    }
    while (  stato == " erogazione_in corso " );
      if (progress < 55) {//quando stiamo erogando andiamo a spegnere piano piano o pixel che hanno il contenuto della siringa  
        progress++;
      } else {
        progress = 0;
      }

  }
  if(buttonflag == 1 && menuCount == 1 && stato == "erogazione_in corso"){
    display.clearDisplay();
    stato = "start"; //riporta ai vecchi menu per selezionare nuove infusioni  
  }

///********************************************************************
//          Task Completato
// ********************************************************************/
    
  if(stato =="task_completato") {

    menuCount = 1;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);  
    display.setTextSize(1);

    display.setCursor(10, 0); display.println("Task Completato");
    display.setCursor(38, 22); display.println("...");
    display.drawRect(40, 10, 60, 10, SSD1306_WHITE);  
    display.drawLine(25, 15, 40, 15, SSD1306_WHITE);
    display.drawLine(25, 12, 25, 18, SSD1306_WHITE);
    display.drawLine(100, 15, 105, 15, SSD1306_WHITE);
    display.fillRect(41, 12, 55, 6, SSD1306_WHITE);

  }
  if(buttonflag == 1 && menuCount == 1 && stato == "task_completato"){
    display.clearDisplay();
    stato = "start";
  }
}
/********************************************************************
           Functions
********************************************************************/ 
void drawcursor(){ //definizioni di funzioni che vengono richiamate nel loop 

  display.setCursor(1, (menuCount * 8)-8); //scelgo le coordinate in cui stampare un carattere o una scritta nel dislay, scaliamo di 8 pixel per far scendere il cursore.
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  display.println(">");// seleziona tutte le possibilita che abbiamo nel menu
  display.display();

}

void buttonStatus(){//legge lo stato del pin8 e lo mette nella variabile, se è alto mette il buttonflag pari a 1 e faccio una transizione da un menu ad un altro 
  buttonState_ok = digitalRead(8);
  if(buttonState_ok == HIGH){
    buttonflag=1;
    delay(50);     
  }else{
    buttonflag=0;//se non rileva nulla pone la button flag a 0
    delay(50);
  }
}

void menuCheck() { //serve a tenere conto delle variazioni associate al bottone del pin6, ci permette di scrolare le azioni in un menu 
  if (clk == HIGH && menuCount < 5){ //al massimo abbiamo 5 opzioni, quindi se premiamo sotto al 5. fa un incremento del menu count
    menuCount++;
    delay(50);
  }
  if (clk == HIGH && menuCount >= 5) { //se premo un tasto >5 inizializza di nuovo il menu count pari a 1 
    menuCount = 1;
    delay(50);
  }
}

void doEncoder() { //registra gli incrementi, prende i valori della variabile inizializzatata e se premiamo il tasto fa un incremento di 0.1 ogni volta 
  //forse devo aggiungere una condizione 
  if (digitalRead(buttonPin7) == HIGH) {
   encoder0Pos = encoder0Pos - 0.1;
   delay(10);
  }
  if (digitalRead(buttonPin5) == HIGH ) {
   encoder0Pos = encoder0Pos + 0.1;
   delay(10);
  }
}

void time_changer(){ //veiamo sullo schermetto di quanto stiamo andando ad incrementare e quindi la scelta  
  if(buttonflag == 1 && menuCount == 2 && stato == "time_selection_2")
    selectedTime = encoder0Pos;
}

void custom_delivery_volume(){
  if(buttonflag == 1 && menuCount == 2 && stato == "custom_2" )
    volume_selected = encoder0Pos;
}

void custom_load_volume(){
  if(buttonflag == 1 && menuCount == 2 && stato == "cust2_2" )
    volume_selected = encoder0Pos;
}

void erogazione_full(){ //funzioni che definiscono la modalità di iniezione 
  stato = "task_in corso";
  digitalWrite(pinDir, LOW);//pin di direzione basso in modo da avere l'iniezione.
  //stato = "task_in corso";
  if(selected_syringe_mL == syringe_volume_5mL ){ //qui vengono scelti i conti fatti per la siringa 
    current_step_under_mL_in_n = step_under_mL_in_n_5mL;    
  }else if(selected_syringe_mL == syringe_volume_10mL){
    current_step_under_mL_in_n = step_under_mL_in_n_10mL;
  }else if(selected_syringe_mL == syringe_volume_1mL){
  current_step_under_mL_in_n = step_under_mL_in_n_1mL;
  }
  for (int x=0; x < current_step_under_mL_in_n; x++) { //qui si decidono il numero di step che deve fare il motore e si genera sotto un onda quadra.
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(t);
    digitalWrite(pinStep, LOW); 
    delayMicroseconds(t);
  }
  stato = "task_completato";
}

void erogazione_half(){ //il numero di step sarà la metà
  stato = "task_in corso";
  digitalWrite(pinDir, LOW);
  //stato = "task_in corso";
  if(selected_syringe_mL == syringe_volume_5mL ){
    current_step_under_mL_in_n = step_under_mL_in_n_5mL;    
  }else if(selected_syringe_mL == syringe_volume_10mL){
    current_step_under_mL_in_n = step_under_mL_in_n_10mL;
  }else if(selected_syringe_mL == syringe_volume_1mL){
  current_step_under_mL_in_n = step_under_mL_in_n_1mL;
  }
  for (int x=0; x < current_step_under_mL_in_n/2; x++) {
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(t);
    digitalWrite(pinStep, LOW);
    delayMicroseconds(t);
  }
  stato = "task_completato";
}

void erogazione_custom(){ //cambia il fatto che possiamo andare a selezionare un volume e lo calcoliamo con le formule sopra 
  stato = "task_in corso";
  digitalWrite(pinDir, LOW);
  float corresponding_height_for_selected_volume =  ( (pow(10,-3)) * volume_selected * (pow(10,-6)) )/(pi * (pow(syringe_inner_diameter_5mL/2,2)));
  float corresponding_number_revolutions_for_selected_volume = ( (corresponding_height_for_selected_volume)/(screw_lead_in_mm) );
  float required_steps_for_selected_volume = (corresponding_number_revolutions_for_selected_volume * step_for_revolution);

  float step_under_mL_in_n_for_selected_volume = ((required_steps_for_selected_volume)*(micro_step)); //calcoliamo gli step necessari 

  for (int x=0; x < step_under_mL_in_n_for_selected_volume; x++) {//imponiamo gli step nel ciclo for  
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(t);
    digitalWrite(pinStep, LOW);
    delayMicroseconds(t);
  }
  delay(5000);
  stato = "task_completato";// delay(1000);
  
}

void load_full(){
  digitalWrite(pinDir, HIGH); //il pin di direzione sara alto per far girare i motore al contrario  
  stato = "task_in corso";
  if(selected_syringe_mL == syringe_volume_5mL ){
    current_step_under_mL_in_n = step_under_mL_in_n_5mL;    
  }else if(selected_syringe_mL == syringe_volume_10mL){
    current_step_under_mL_in_n = step_under_mL_in_n_10mL;
  }else if(selected_syringe_mL == syringe_volume_1mL){
  current_step_under_mL_in_n = step_under_mL_in_n_1mL;
  }
  for (int x=0; x < current_step_under_mL_in_n; x++) {
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(t);
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(t);
  }
  delay(5000);
  stato = "task_completato";// delay(1000);
}

void load_half(){
  digitalWrite(pinDir, HIGH);
  stato = "task_in corso";
  if(selected_syringe_mL == syringe_volume_5mL ){
    current_step_under_mL_in_n = step_under_mL_in_n_5mL;    
  }else if(selected_syringe_mL == syringe_volume_10mL){
    current_step_under_mL_in_n = step_under_mL_in_n_10mL;
  }else if(selected_syringe_mL == syringe_volume_1mL){
  current_step_under_mL_in_n = step_under_mL_in_n_1mL;
  }
  for (int x=0; x < current_step_under_mL_in_n/2; x++) {
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(t);
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(t);
  }
  delay(5000);
  stato = "task_completato";// delay(1000);
}

void load_custom(){
  stato = "task_in corso";
  digitalWrite(pinDir, HIGH);
  float corresponding_height_for_selected_volume =  ( (pow(10,-3)) * volume_selected * (pow(10,-6)) )/(pi * (pow(syringe_inner_diameter_5mL/2,2)));
  float corresponding_number_revolutions_for_selected_volume = ( (corresponding_height_for_selected_volume)/(screw_lead_in_mm) );
  float required_steps_for_selected_volume = (corresponding_number_revolutions_for_selected_volume * step_for_revolution);

  float step_under_mL_in_n_for_selected_volume = ((required_steps_for_selected_volume)*(micro_step));
  
  for (int x=0; x < step_under_mL_in_n_for_selected_volume; x++) {
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(t);
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(t);
  }
  delay(5000);
  stato = "task_completato";// delay(1000);
}

void delivery_with_custom_volume(){// viene definita la stessa funzione ma qui possiamo scegliere tempo e volume 
  stato = "task_in corso";
  digitalWrite(pinDir, LOW);
  float corresponding_height_for_selected_volume =  ( (pow(10,-3)) * volume_selected * (pow(10,-6)) )/(pi * (pow(syringe_inner_diameter_5mL/2,2)));
  float corresponding_number_revolutions_for_selected_volume = ( (corresponding_height_for_selected_volume)/(screw_lead_in_mm) );
  float required_steps_for_selected_volume = (corresponding_number_revolutions_for_selected_volume * step_for_revolution);

  float step_under_mL_in_n_for_selected_volume = ((required_steps_for_selected_volume)*(micro_step));

  float desired_period = (selectedTime/(corresponding_number_revolutions_for_selected_volume)); //qui scegliamo il tempo desiderato
  float velocita_time = (1/(desired_period)); //facciamo il reciproco e otteniamo la velocità da dare al motore.
  long delayStepToStep_time_selected = (60*pow(10,6)) / (velocita_time * 3200) ;
  long t_time_selected = delayStepToStep_time_selected/2; 

  for (int x=0; x < step_under_mL_in_n_for_selected_volume; x++) {
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(t_time_selected);
    digitalWrite(pinStep, LOW);
    delayMicroseconds(t_time_selected);
  }
  delay(5000);
  stato = "task_completato";
}

void loading_with_custom_volume(){ //stessa cosa solo che mettiamo il pin alto 
  stato = "task_in corso";
  digitalWrite(pinDir, HIGH);
  float corresponding_height_for_selected_volume =  ( (pow(10,-3)) * volume_selected * (pow(10,-6)) )/(pi * (pow(syringe_inner_diameter_5mL/2,2)));
  float corresponding_number_revolutions_for_selected_volume = ( (corresponding_height_for_selected_volume)/(screw_lead_in_mm) );
  float required_steps_for_selected_volume = (corresponding_number_revolutions_for_selected_volume * step_for_revolution);

  float step_under_mL_in_n_for_selected_volume = ((required_steps_for_selected_volume)*(micro_step));

  float desired_period = (selectedTime/(corresponding_number_revolutions_for_selected_volume));
  float velocita_time = (1/(desired_period));
  long delayStepToStep_time_selected = (60*pow(10,6)) / (velocita_time * 3200) ;
  long t_time_selected = delayStepToStep_time_selected/2; 

  for (int x=0; x < step_under_mL_in_n_for_selected_volume; x++) {
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(t_time_selected);
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(t_time_selected);
  }
  delay(5000);
  stato = "task_completato";
}

void esecuzione(){ //serve per vedere se abbiamo fatto un errore 
  
  //DELIVERY
  if( selected_syringe_mL != 0 ){ 
    stato = "errore";
  }else if(buttonflag == 1 && menuCount == 2 && stato == "go_2"){
    stato = "erogazione_in corso"; //se è tutto bene vado in erogazione in corso e poi entro in erogazione full
    erogazione_full(); 
  }

  if( step_under_mL_in_n_for_selected_volume != 0 ){
    stato = "errore_custom";
  }else if(buttonflag == 1 && menuCount == 3 && stato == "go_2"){
    stato = "erogazione_in corso";
    erogazione_custom();
  }

  if(selected_syringe_mL != 0){
    stato = "errore";
  }else if(buttonflag == 1 && menuCount == 4 && stato == "go_2");{
    stato = "erogazione_in corso";
    erogazione_half();
  }

  if(step_under_mL_in_n_for_selected_volume != 0 ){ 
    stato = "errore_custom_time";  
  }else if(buttonflag == 1 && menuCount == 3 && stato == "del_2" ){
    stato = "erogazione_in corso";
    delivery_with_custom_volume();
  }

  //LOAD 
  if( selected_syringe_mL != 0 ){ 
    stato = "errore";
  }else if(buttonflag == 1 && menuCount == 2 && stato == "go2_2"){ 
    load_full();
  }

  if(step_under_mL_in_n_for_selected_volume != 0 ){
    stato = "errore_custom";
  }else if (buttonflag == 1 && menuCount == 3 && stato == "go2_2" ){
    load_custom();
  }

  if( selected_syringe_mL != 0 ){
    stato = "errore";
  }else if(buttonflag == 1 && menuCount == 4 && stato == "go2_2" ){
    load_half();
  }

  if( step_under_mL_in_n_for_selected_volume != 0 ){
    stato = "errore_custom_time";
  }else if(buttonflag == 1 && menuCount == 3 && stato == "load_2" ){
    loading_with_custom_volume();
  }

}
