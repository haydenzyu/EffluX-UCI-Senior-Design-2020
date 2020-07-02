int time1;

void setup() {
  Serial.begin(9600);
}
void loop() {
  Serial.print("Time: ");
  time1 = millis()/1000;

  Serial.println(time1); //prints time since program started
  delay(1000);          // wait a second so as not to send massive amounts of data
}