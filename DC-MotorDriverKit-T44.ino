/* 
Program for the CasuallyLoaded® Brushed DC-Motor Driver kit, Created 2022.
Compiled for use on the Attiny44 with 16Mhz external clock and 4.3v BOD. Each kit has 
this code preprogrammed into the included IC so its not necessary to do the upload yourself. 
For those looking to modify the code it has been semi-annotated. 
*/

//create and define variables
volatile long DutyMicros = 0;
volatile long pulseTime = 0;
volatile long triggerPoint = 0;
volatile long risePoint = 0;
volatile long fallPoint = 0;
volatile long FirstHigh = 0;
volatile long SecondHigh = 0;
volatile int PulseStarted = 0;
volatile int PrevState = 0;
volatile int FilteredDuty = 0;
volatile int SetDutyVariable = 0;
volatile int UpdateDuty = 0;
int DutyPercent = 0;
int SpinDirection = 0;
int SetSpinDirection = 1;

#define AHIGH B10000010;
#define BHIGH B00100010;

void setup() {

  DDRA = B11111010; //set the data direction for outputs
  PORTA = B00000000; //set all output pins low

  GIMSK = B00110000; //enable pin change interrupts
  PCMSK0 = B00000001; //set the individual port pins to trigger said interrupts
  PCMSK1 = B00000100;

  TCCR0A = 0; //8bit timer0 setup, used to control the speed indicator LEDs brightness via PWM
  TCCR0B = 0; 
  TCCR0A = B00100011;
  TCCR0B = B00000011;
  OCR0B = 0;
  
  TCCR1A = 0; //16bit timer1 setup, used to control the highside bootstrap PWM (Motor Speed) 
  TCCR1B = 0; 
  TCCR1A = B10100010;
  TCCR1B = B00010010;
  ICR1 = 400;
  OCR1A = 0;
  OCR1B = 0;

  TCCR1A = AHIGH; //set the full-bridge to default. (PWM variable VCC at PHASEA and GND at PHASEB)
  PORTA |= B00001000;

  TCCR0A &= ~B00100000; //start up function to blink the indicator LEDs 
  PORTA |= B10000010; //NOTE: |= B1 will set the bit to a 1 and &= ~B1 will set the bit to a 0 without impacting any other bits
  delay(200);
  PORTA &= ~B10000010;
  delay(100);
  PORTA |= B10000010;
  delay(200);
  PORTA &= ~B10000010;
  TCCR0A |= B00100000;

  sei(); //makes sure all interrupts are enabled
 
}

void loop() {
  
  if (UpdateDuty == 1) { //pushes duty update to the OCR1x registers used to control motor speed
    DutyPercent = FilteredDuty; //saves to a non volatile loop friendly variable
    OCR1A = DutyPercent; //sets the new duty
    OCR1B = DutyPercent;
    OCR0B = map(DutyPercent, 1, 398, 1, 255); //maps the set duty to an 8bit value for the speed indicators brightness
    UpdateDuty = 0; //so function only runs once
  }
  
  if (DutyPercent < 10){ //if the duty is below the "on" threshold 
     TCCR1A = 0; //turns off the highsides, lowsides and indicators
     TCCR0A = 0;
     PORTA &= ~B00011010;
     SetSpinDirection = 1; //spin direction can still be updated while the motor is off
  }

  //functions that set the full-bridges in order to control the spin direction
  if ((DutyPercent >= 10) && (SpinDirection == 0) && (SetSpinDirection == 1)){
      TCCR0A = B00100011;
      TCCR1A = 0; //sets PHASEA to be the highside/PWM-VCC and PHASEB to be lowside/Ground
      PORTA &= ~B00011010;
      TCCR1A = AHIGH;
      PORTA |= B00001000;
      SetSpinDirection = 0;
  }
  else if ((DutyPercent >= 10) && (SpinDirection == 1) && (SetSpinDirection == 1)){
      TCCR0A = B00100011;
      TCCR1A = 0; //sets PHASEB to be the highside/PWM-VCC and PHASEA to be lowside/Ground
      PORTA &= ~B00011000;
      TCCR1A = BHIGH;
      PORTA |= B00010010;
      SetSpinDirection = 0;
  }
     
}

ISR (PCINT0_vect) { //pin change interupt vector used to set the spin direction

  if ((PINA & B00000001)){ //checks if pin PA0 went high, activates reverse 
    SpinDirection = 1;
    SetSpinDirection = 1;
  }
  else if (!(PINA & B00000001)){ //checks if pin PA0 went low, activates forwards
    SpinDirection = 0;
    SetSpinDirection = 1;
  }
}

ISR(PCINT1_vect){ //interput vector used to do the speed input signal detecting (most complex part of program)
  
  triggerPoint = micros(); //stores interrupt trigger time as micros

  //calculates the input signals duty cycle
  if ((PINB & B00000100) && (PrevState == 0)){ //if input pin just went high and was just low
    PrevState = 1; //records that the input pin is currently high
  }
  else if ((!(PINB & B00000100)) && (PrevState == 1)){ //if the input pin just went low and was just high
    PrevState = 0; //records that the input pin is currently low
    fallPoint = triggerPoint; //sets the duty fall point
  }

  //calculates the input signals frequency using the time of a full pulse (high to low and back to high again)
  if ((PrevState == 1) && (PulseStarted == 0)){ //if input is high and its the start of a new full pulse
    FirstHigh = triggerPoint; //saves the micros time
    risePoint = triggerPoint; //sets the duty risepoint
    PulseStarted = 1; //records that the new full pulse has started
  }
  else if ((PrevState == 1) && (PulseStarted == 1)){ //if the input is high again and a full pulse was started on prev high
    SecondHigh = triggerPoint; //saves the micros time
    DutyMicros = (fallPoint - risePoint); //calcs the duty in microseconds
    pulseTime = (SecondHigh - FirstHigh); //calcs the total time it took to pulse from high (start of new pulse) to low and back to high again giving the time in micros of the full pulse
    PulseStarted = 0; //records that the next high will be a new full pulse
    SetDutyVariable = 1; //tells interupt to check and calculate the duty to be pushed to main loop for setting
  }

  if ((SetDutyVariable == 1) && (pulseTime >= 100) && (pulseTime <= 10000) && (DutyMicros < pulseTime) && (DutyMicros > 1)){ //filter to make sure the raw signal can be mapped properly
    
    FilteredDuty = map(DutyMicros, 1, pulseTime, 1, 400); //maps the duty cycle using the fequency in micros as the top
    if (FilteredDuty < 10){ //constrains duty to ensure its within the correct range
      FilteredDuty = 0;
    }
    else if (FilteredDuty > 398){
      FilteredDuty = 398;
    }
    UpdateDuty = 1; //sets an update flag to be seen by the main loop
    SetDutyVariable = 0; //ensures this loop only runs when told to
    }
  else if (SetDutyVariable == 1){ //if the values collected do not pass the filter reset the update and try again next time
    SetDutyVariable = 0; //ensures this loop only runs when told to
    }  
}
