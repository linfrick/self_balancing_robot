const int POT_input = A1;   /* assign ADC Channel */
bool d1 = HIGH;
bool d2 = LOW;

void setup() {
  pinMode(4, OUTPUT);  /* Motor control pin 1 */
  pinMode(7, OUTPUT);  /* Motor control pin 2 */
  pinMode(3, OUTPUT);  /* PWM pin for Speed Control */
  pinMode(2, INPUT_PULLUP);  /* Interrupt pin for direction control */
  attachInterrupt(digitalPinToInterrupt(2), motor, FALLING);  /* Interrupt on falling edge on pin 2 */
}

void loop() {
  int pwm_adc;
  pwm_adc = analogRead(POT_input); /* Input from Potentiometer for speed control */
  digitalWrite(4,d1);
  digitalWrite(7,d2);
  analogWrite(3, pwm_adc / 4);    
}

void motor(){
  d1 = !d1;
  d2 = !d2;
  _delay_ms(200);
}
