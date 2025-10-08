#define LED1 13U


void setup() {
  // put your setup code here, to run once:
  pinMode(LED1, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED1,LOW);
  delay(1000);
  digitalWrite(LED1,HIGH);
  delay(1000);
}
