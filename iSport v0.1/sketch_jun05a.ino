int LED1 = 12;
int LED2 = 13;
int button = 3;

boolean LED1State = false;
boolean LED2State = false;

long buttonTimer = 0;
long longPressTime = 3000;

boolean buttonActive = false;
boolean longPressActive = false;

void setup() {
  attachInterrupt(digitalPinToInterrupt(4), digitalInterrupt,FALLING);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(button, INPUT);

}

void loop() {

  if (digitalRead(button) == HIGH) {

    if (buttonActive == false) {

      buttonActive = true;
      buttonTimer = millis();

    }

    if ((millis() - buttonTimer > longPressTime) && (longPressActive == false)) {
      deepSleeep();

    }

  } else {

    if (buttonActive == true) {

      if (longPressActive == true) {

        longPressActive = false;

      } else {

        LED2State = !LED2State;
        digitalWrite(LED2, LED2State);

      }

      buttonActive = false;

    }

  }

}



void deepSleeep()

{

longPressActive = true;
//Disable ADC
ADCSRA &= ~(1<<7);
//Enable Sleep
SMCR |= (1<<2);        //Power down mode
SMCR |= 1;          // Enable sleep

//BOD DISABLE
MCUCR != (3 << 5);      //Sets both BODSE & BODS at the same time
MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6);   
__asm__ __volatile("sleep");



}

void digitalInterrupt(){
  }
