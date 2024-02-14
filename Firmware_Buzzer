#define PIN_BUZZER 7 //associate the exit that you want 

bool control_variable;


void setup(){   //initialize the setup 
  pinMode(PIN_BUZZER, OUTPUT);
}

void loop(){ //define the function 'suona'
  suona();
}  

void frequenza_1() { //generate a sound with lower frequency 
  for(int i=0;i<80;i++){
    digitalWrite(PIN_BUZZER,HIGH);
    delay(1);
    
    digitalWrite(PIN_BUZZER,LOW);
    delay(1);
  }
}

void frequenza_2() { //sound with a different frequency 
  for(int i=0;i<100;i++){
    digitalWrite(PIN_BUZZER, HIGH);
    delay(2);//wait for 2ms
        
    digitalWrite(PIN_BUZZER, LOW);
    delay(2);//wait for 2ms
  }
}

void suona() { //alternate 2 frequencies
  if (control_variable !=0 ){
    frequenza_1();
    frequenza_2();
  }
}
