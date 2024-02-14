#define PIN_BUZZER 7 //associato al pin 7 

bool control_variable;


void setup(){   //inizilizziamo il setup 
  pinMode(PIN_BUZZER, OUTPUT);
}

void loop(){ //definiamo la funzione suona 
  suona();
}  

void frequenza_1() { //generiamo un suono a frequenze minori 
  for(int i=0;i<80;i++){
    digitalWrite(PIN_BUZZER,HIGH);
    delay(1);
    
    digitalWrite(PIN_BUZZER,LOW);
    delay(1);
  }
}

void frequenza_2() { //suono con frequenza diversa per avere due suoni che combinati ricordano quello della sirena 
  for(int i=0;i<100;i++){
    digitalWrite(PIN_BUZZER, HIGH);
    delay(2);//wait for 2ms
        
    digitalWrite(PIN_BUZZER, LOW);
    delay(2);//wait for 2ms
  }
}

void suona() { //altrerna l'esecuzione di frequenza uno e due 
  if (control_variable !=0 ){
    frequenza_1();
    frequenza_2();
  }
}