///////////////////////////
// LAST UPDATE: 22.3.2016//
///////////////////////////
//MOTOR CONTROL
int RPwm1 = 10;
int RPwm2 = 9;

int LPwm1 = 5;
int LPwm2 = 6;

//LED & BUZZER
int Buzzer = 13;
int ArduLed = 12;

//EDGE & CONTRAST SENSORS
int Redge = A0;
int Ledge = A1;

//TRIMPOTS
int SPD = A7;
int TRN = A6;
int ETRN = A5;

//OPPONENT SENSORS
int LSens = 8;
int RSens = 2;
int MSens = 4;

// DIPSWITCH & BUTTON
int Button = 3; // Can be used as start pin too.
int DS1 = A4;
int DS2 = A3;
int DS3 = A2;

//VALUES
int Speed;
int MaxSpeed = 90; // Idle Speed while no sensor giving data.
int TurnSpeed = 45; // Left and Right Forward Turning Speed
int EdgeTurn; // Turning Time variable when minisumo sees white line
int Duration; // Turning Time at minisumo starting.
int LastValue = 5; // Last Value Variable for remembering last Opponent sensor sense.

void setup()
{
pinMode(LSens, INPUT);    // Left Opponent Sensor Input
pinMode(RSens, INPUT);    // Right Opponent Sensor Input
pinMode(MSens, INPUT);    // Middle Opponent Sensor Input
pinMode(Buzzer, OUTPUT);  // Buzzer Declared as Output
pinMode(ArduLed, OUTPUT); // Buzzer Declared as Output
pinMode(RPwm1, OUTPUT);  // Four PWM Channel Declared as Output
pinMode(RPwm2, OUTPUT); 
pinMode(LPwm1, OUTPUT); 
pinMode(LPwm2, OUTPUT); 
digitalWrite(Buzzer, LOW); // Buzzer Pin Made Low for Silence :)
digitalWrite(ArduLed, LOW);  // Arduino Mode Led Made Low
digitalWrite(DS1, HIGH); // 3 Dipswitch Pin Pullups Made
digitalWrite(DS2, HIGH);
digitalWrite(DS3, HIGH);
digitalWrite(Button, HIGH); // Button Pin Pullup Made
digitalWrite(LSens, HIGH); // 3 Opponent Sensor Pullups Made
digitalWrite(RSens, HIGH); 
digitalWrite(MSens, HIGH); 
Serial.begin(9600);
}

//Motor Control Function
void Set_Motor (float Lval, float Rval, int timex){
  Lval = Lval*2.5;
  Rval = Rval*2.5; 
  if (Lval >=0) { 
      analogWrite(LPwm1, Lval);  
      digitalWrite(LPwm2, LOW);       
      } else {
      Lval=abs(Lval); 
      digitalWrite(LPwm1, LOW);  
      analogWrite(LPwm2, Lval); 
      }
   if (Rval >=0) {    
      analogWrite(RPwm1, Rval);  
      digitalWrite(RPwm2, LOW);       
      } else {
      Rval=abs(Rval);     
      digitalWrite(RPwm1, LOW);  
      analogWrite(RPwm2, Rval); 
      }   
     // Serial.print(Rval); Serial.print("-"); Serial.println(Lval);
   delay(timex);  
}
void loop() {
 if (digitalRead(Button)==0) { // If button is pressed at beginning.
   tone(Buzzer, 18, 300); // Pin, Frequency, Duration
   while (1) {  
    if (digitalRead(DS1)==0 && digitalRead(DS2)==0 && digitalRead(DS3)==0) {
               Serial.print("Board Test");
               Set_Motor(80,80,1000);
               Set_Motor(0,0,1000);
               Set_Motor(-80,-80,1000);
                Set_Motor(0,0,1000);
                tone(Buzzer, 18, 300);
                tone(ArduLed, 18, 300);            
       }
           
        }}
//////////////////////////////////////////////
tone(Buzzer, 18, 100); // Pin, Frequency, Duration
tone(ArduLed, 8, 100); // Pin, Frequency, Duration
Wait:
     Serial.println("Button Press Waited");
 Set_Motor(0,0,1);
 /// Sensor Control While Waiting The Button Press ///
 if ( digitalRead(MSens)==LOW || digitalRead(RSens)==LOW || digitalRead(LSens)== LOW || analogRead(Redge)< 500 || analogRead(Ledge)< 500 ) { digitalWrite(ArduLed, HIGH);} 
 else { digitalWrite(ArduLed, LOW); }
 ///////////////////////////////////////////////
 if (digitalRead(Button)==0) {
     Duration=(analogRead(TRN)/4); // Duration variable based on TRN (A6) trimpot
     Serial.println("5 Sec Routine Started"); 
       for (int i = 0; i < 5; i++){         
   digitalWrite(Buzzer, HIGH); digitalWrite(ArduLed, HIGH); delay(100);
   digitalWrite(Buzzer, LOW); digitalWrite(ArduLed, LOW);  delay(900);
       }
       if (digitalRead(DS1)==0 && digitalRead(DS2)==1 && digitalRead(DS3)==1){
           Serial.print("LEFT TURN");            
           Set_Motor(-100,100,Duration); //                 
       }
          else if (digitalRead(DS1)==0 && digitalRead(DS2)==0 && digitalRead(DS3)==0) {
               Serial.print("MIDDLE DIRECT");
               Set_Motor(80,80,2);
       }
           else if (digitalRead(DS1)==1 && digitalRead(DS2)==1 && digitalRead(DS3)==0){
           Serial.print("Sag");
           Set_Motor(100,-100,Duration);           
       }       
         else if (digitalRead(DS1)==1 && digitalRead(DS2)==0 && digitalRead(DS3)==0){
            Serial.print("Left Circle");
           Set_Motor(100,36,2); 
       }       
       else if (digitalRead(DS1)==0 && digitalRead(DS2)==0 && digitalRead(DS3)==1){
           Serial.print("Right Circle");
           Set_Motor(36,100,2); 
       }       
        else if (digitalRead(DS1)==0 && digitalRead(DS2)==1 && digitalRead(DS3)==0){
           Serial.print("Reverse 180"); 
           Set_Motor(-100,100,Duration); 
          delay(Duration);  
       } 
       Serial.println("OK");
       //Set_Motor(-0,0,15000); You should open that line when calibrating Turning Duration Time based on TRN Trimpot.
       digitalWrite(Buzzer, LOW);
       EdgeTurn=(analogRead(ETRN)/5); EdgeTurn=205-EdgeTurn; 
       goto Start;
 } 
goto Wait;

//Main Loop
Start:
  /// Edge Sensor Control Routine ///
  digitalWrite(ArduLed, LOW);
 if (analogRead(Ledge)<500 && analogRead(Redge)> 500) {
   digitalWrite(Buzzer, LOW);
   digitalWrite(ArduLed, HIGH);
 // Serial.println("Left Edge Detected"); 
  // while (analogRead(Ledge)<500){
   Set_Motor(-90, -90,35); // Geri
      // }    
   Set_Motor(-90, -90, EdgeTurn); // Left Backward, Right Forward, Turning Time Based on ETRN Trimpot
  LastValue=5;
 }
   else  if (analogRead(Ledge)> 500 && analogRead(Redge)< 500) {
     digitalWrite(Buzzer, LOW);
     digitalWrite(ArduLed, HIGH);
   Set_Motor(-90, -90,35); // Back 35 Milliseconds
     //  }
   Set_Motor(-90, -90, EdgeTurn); // Right Backward, Left Forward, Turning Time Based on ETRN Trimpot
   LastValue=5;
  }
   else  if (analogRead(Ledge)< 500 && analogRead(Redge)< 500) {
     digitalWrite(Buzzer, LOW);
     digitalWrite(ArduLed, HIGH);
  //Serial.println("Both Edges Detected"); 
  // while (analogRead(Redge)<500){
   Set_Motor(-90, -90,35); // Back 35 Milliseconds
     //  }
   Set_Motor(-90, -90, EdgeTurn); // Right Backward, Left Forward, Turning Time Based on ETRN Trimpot
    LastValue=5; 
 }else
/// Opponent Sensor Control Routine ///
if (digitalRead(MSens)==LOW) {Set_Motor(MaxSpeed, MaxSpeed,1); digitalWrite(Buzzer, HIGH); LastValue=5;} else
if (digitalRead(LSens)== LOW) {Set_Motor(-40, TurnSpeed,1); digitalWrite(Buzzer, HIGH); LastValue=7;} else
if (digitalRead(RSens)==LOW) {Set_Motor(TurnSpeed, -40,1); digitalWrite(Buzzer, HIGH); LastValue=3;} else
{
  digitalWrite(Buzzer, LOW);
Speed=(analogRead(SPD)/10.3); Speed=70-Speed; 
if (LastValue==5) { Set_Motor(Speed, Speed,3);} else // Forward, Based on SPD (A7) Trimpot
if (LastValue==7) { Set_Motor(45, Speed,3);} else  // Left Turning Based on SPD (A7) Trimpot
if (LastValue==3) { Set_Motor(Speed, 45,3);}  // Right Turning Based on SPD (A7) Trimpot
}
goto Start;
}
