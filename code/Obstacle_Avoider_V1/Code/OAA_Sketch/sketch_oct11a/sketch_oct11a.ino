int value;
void setup()
{
  Serial.begin(9600);
}
void loop(){
  value=analogRead(A0);
  Serial.print(value);
  Serial.print("\n");
}
