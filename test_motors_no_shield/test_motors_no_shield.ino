const int left1 = 8;
const int left2 = 9;
const int right1 = 6;
const int right2 = 7;

void setup() {
  // put your setup code here, to run once:
  pinMode(left1,OUTPUT) ;   //Logic pins are also set as output
  pinMode(left2,OUTPUT) ;
  pinMode(right1,OUTPUT) ;   //Logic pins are also set as output
  pinMode(right2,OUTPUT) ;
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(left1,HIGH) ;
  digitalWrite(left2,LOW) ;
}
