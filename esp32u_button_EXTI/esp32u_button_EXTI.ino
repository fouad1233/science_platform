#define pushButton_pin   16

int flag = 1;
void IRAM_ATTR toggleLED()
{
  flag = 1;
}
void setup()
{
  Serial.begin(115200);
  pinMode(pushButton_pin, INPUT_PULLUP);
  attachInterrupt(pushButton_pin, toggleLED, RISING);
} 
void loop()
{
  if(flag){
    Serial.println("ON");
    flag = 0;
  }
}