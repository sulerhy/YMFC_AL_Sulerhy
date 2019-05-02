unsigned long loop_timer, last_timer;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  loop_timer = micros() + 4000L;
}

void loop() {
   last_timer = micros();
//   delayMicroseconds(4000);
//   while(micros() < loop_timer);
//   loop_timer = micros()+4000;
      Serial.print(F(" Hello 500 anh em"));
   Serial.println(micros()-last_timer);
}
