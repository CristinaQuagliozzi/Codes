#include <ezButton.h>
#define MAX_POSITION 0x7FFFFFFF // maximum of position we can set (long type), posizione dello switch 
ezButton limitSwitch(A0); // create ezButton object that attach to pin A0;posizione del pin 

String stato ;

void setup() {
  Serial.begin(9600);// inizializzazione della comunicazione 
  limitSwitch.setDebounceTime(50); // set debounce time to 50 milliseconds, per evitare il problema del debouncing 

}

void loop() {

  limitSwitch.loop(); //rileviamo quando è premuto 
  if (limitSwitch.isPressed()) {
    stato = 'limit switch: TOUCHED'; //condizione che nel codice primcipale va a bloccare il motore e fa aprtire il buzzer 
  }
}
