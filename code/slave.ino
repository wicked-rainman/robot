int led = 13;
int dsr = 11;
void setup() {
  Serial.begin(9600);
  Serial2.begin(115200);
  pinMode(led, OUTPUT);
  pinMode(dsr, INPUT);
  digitalWrite(led, LOW);
}

void loop() {
  if (Serial2.available()>0) readMaster();
}

int readMaster() {
  String k;
  digitalWrite(led, HIGH);
  while(Serial2.available() > 0) {
    k = Serial2.readString();
    Serial.print(" Received \"");
    Serial.print(k);
    Serial.println("\"");
    Serial.flush();
    Serial2.print(k);
    Serial2.flush();
  }  
  digitalWrite(led,LOW);
}
