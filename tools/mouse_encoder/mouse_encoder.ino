byte AInput = 2;
byte BInput = 7;
 
byte lastState = 0;
int steps = 0;
int  cw = 0;
byte AState = 0;
byte BState = 0;
byte State = 0;
void setup() {
  Serial.begin(9600);
  pinMode(AInput, INPUT);
  pinMode(BInput, INPUT);
}


void loop() {
  // read the input pin:
  AState = digitalRead(AInput);
  BState = digitalRead(BInput) << 1;
  State = AState | BState;
 
  if (lastState != State){
    switch (State) {
      case 0:
        if (lastState == 2){
          steps++;
          cw = 1;
        }
        else if(lastState == 1){
          steps--;
          cw = -1;
        }
        break;
      case 1:
        if (lastState == 0){
          steps++;
          cw = 1;
        }
        else if(lastState == 3){
          steps--;
          cw = -1;
        }
        break;
      case 2:
        if (lastState == 3){
          steps++;
          cw = 1;
        }
        else if(lastState == 0){
          steps--;
          cw = -1;
        }
        break;
      case 3:
        if (lastState == 1){
          steps++;
          cw = 1;
        }
        else if(lastState == 2){
          steps--;
          cw = -1;
        }
        break;
    }
    Serial.println(cw);
  }

  lastState = State;
  
  delay(1);
}
