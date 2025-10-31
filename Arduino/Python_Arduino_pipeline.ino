int x; 

void setup() { 
  Serial.begin(115200); 
  Serial.setTimeout(1); 
  Serial.println("Arduino is ready");
  pinMode(LED_BUILTIN, OUTPUT);

} 




void loop() {
// Read serial data
if (Serial.available() > 0) {
int inByte = Serial.read();
if (inByte == '1') {
digitalWrite(LED_BUILTIN, HIGH);
} else {
digitalWrite(LED_BUILTIN, LOW);
}
}
}
