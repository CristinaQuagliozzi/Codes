#include <Wire.h>
#include <Arduino.h>

const int sensorPin = A0; 

int sensorValue = 0;   
int sensorMin = 1023;   // poiche abbiamo un convertitore a 10 bit 
int sensorMax = 0;     


 
 
void setup(){

  Wire.begin(); // starting communication
  delay (500); 
  pinMode(A0, INPUT);
  calibrate(); //è una sorta di tara 
  Serial.begin(9600);
}

void loop(){

  sensordata();
  outputsensordata();
  //Serial.println(sensorout);  
  delay(1000);
}                              

void calibrate(){   // funzione di calibrazione 

  while (millis() < 5000) { //per 5 s leggiamo il valore del sensore, se è piu grande del valore piu basso allora poni la misura come il nuovo massimo 
    sensorValue = analogRead(sensorPin);//adattiamo la dinamica del senore a quella del convertitore.
    // record the maximum sensor value
    if (sensorValue > sensorMax) { // se il valore è piu grande allora poni come minimo quello, serve per adattare la dinamoica del dac alla dinamica del senore 
    //perche noi abbiamo da 0 a 1023 ma è possibile che il senore lavora da 100 e 300 quindi usiamo questi come minimo e massimo.
      sensorMax = sensorValue;
    }
    // record the minimum sensor value
    if (sensorValue < sensorMin) {
      sensorMin = sensorValue;
    }
  }
}


void sensordata(){

   // read the sensor:
  sensorValue = analogRead(sensorPin);//siccome il senore è in binario lo dobbiamo convetire in tensione attraverso la funzione map 

  sensorValue_volt = map(sensorValue, sensorMin, sensorMax, 0,5); 
  sensorValue_volt = sensorValue_volt/40; //otteniamo il valore in volt e poi lo dividiamo per 40 perche era stato previsto uno stadio di amplificazione  con guadagno di 40 
}

float polinomio(){ //questo polinomio è stato trovato dalla curva di calibrazione del sensore e ci siamo messi tra 0 e 0.05 l/min che sarebbero circa 3 L/ora 
// sensorValue_volt=-700*(actualflux) + 95; questa equazione la invertiamo ed esplicitiamo i mV in modo tale che una volta letto il valore in mV quello ci corrisponde direttamente al valore di flusso 
float actualflux = (95-(sensorValue_volt))/(700);
  return actualflux;
}

void feedbacksystem(){  // facciamo vedere come modificare il codice nel caso del controllo col flussimetro 

  stato = "task_in corso";
  digitalWrite(pinDir, LOW);
  float corresponding_height_for_selected_volume =  ( (pow(10,-3)) * volume_selected * (pow(10,6)) )/(pi * (pow(syringe_inner_diameter_5mL/2,2)));
  float corresponding_number_revolutions_for_selected_volume = ( (corresponding_height_for_selected_volume)/(screw_lead_in_mm) );
  float required_steps_for_selected_volume = (corresponding_number_revolutions_for_selected_volume * step_for_revolution);

  float step_under_mL_in_n_for_selected_volume = ((required_steps_for_selected_volume)*(micro_step));

  float desired_period = (selectedTime/(corresponding_number_revolutions_for_selected_volume));
  float velocita_time = (1/(desired_period));
  long delayStepToStep_time_selected = (60*pow(10,6)) / (velocita_time * 3200) ;
  long t_time_selected = delayStepToStep_time_selected/2; 

  float valore_riferimento = ( (volume_selected )/(selectedTime));//è il valore di controllo, in hp di fluido incompribile andiamo a defnire una sorta di portata volumetrica 
  //durante il tempo di lavoro del motore anduamo sempre a controllare che il valore che aquisice il senore sia uguale al valore del flusso di riferimento 

   for (int x=0; x < step_under_mL_in_n_for_selected_volume; x++) {
     
    digitalWrite(pinStep, HIGH);
    delayMicroseconds(t_time_selected);
    digitalWrite(pinStep, LOW);
    delayMicroseconds(t_time_selected);

    if (actualflux != valore_riferimento){  // se il flusso non è uguale andiamo a bloccare il motore 
      x = step_under_mL_in_n_for_selected_volume;   
    } 
  }
  delay(5000);
   
}

