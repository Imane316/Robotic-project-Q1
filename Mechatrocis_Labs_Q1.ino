// Motor A, Left
int enA = 6;
int in1 = 4;
int in2 = 5;
#define Left_ENC_D 2
#define Left_ENC_A A3


 
// Motor B, Right
int enB = 9;
int in3 = 7;
int in4 = 8;
#define Right_ENC_D 3
#define Right_ENC_A A0


float _LeftEncoderTicks = 0;
float _RightEncoderTicks = 0;
boolean forward = true;
boolean backward = false;

/*************************************************************************
* PDI Motor speed variables (choose between 0 - no speed, and 255 - maximum speed)
*************************************************************************/
const uint8_t maxspeeda = 60;
const uint8_t maxspeedb = 60;
const uint8_t basespeeda = 50;
const uint8_t basespeedb = 40;



#define LechoPin 10// attach pin 6 Arduino to pin Echo of HC-SR04
#define LtrigPin 11 //attach pin 7 Arduino to pin Trig of HC-SR04
#define RechoPin 12
#define RtrigPin 13
#define DechoPin A1
#define DtrigPin A2



// Ulrasound
long Lduration, Rduration, Dduration; // variable for the duration of sound wave travel
int Ldistance, Rdistance, Ddistance; // variable for the distance 


//  PID control system variables 

float Kp = 0.1; //related to the proportional control term; 
float Ki = 0,002; //related to the integral control term;               
float Kd = 0; //related to the derivative control term;               
int P;
int I;
int D;

//PID Global variables

int lastError = 0;
boolean onoff = false;

void setup() {
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

// Encoder Interruptions
  pinMode(Right_ENC_D,INPUT_PULLUP);
  pinMode(Right_ENC_A,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Right_ENC_D), readRightEncoder, RISING);
  pinMode(Left_ENC_D,INPUT_PULLUP);
  pinMode(Left_ENC_A,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Left_ENC_D), readLeftEncoder, RISING);

  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  pinMode(LtrigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(LechoPin, INPUT); // Sets the echoPin as an INPUT
  pinMode(RtrigPin, OUTPUT); 
  pinMode(RechoPin, INPUT);
  pinMode(DtrigPin, OUTPUT); 
  pinMode(DechoPin, INPUT);
  // Speed_setting(basespeeda, basespeedb);
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
}

void loop() {
  
  long Lduration, Rduration, Dduration; // variable for the duration of sound wave travel
  int Ldistance, Rdistance, Ddistance;

  // Clears the trigPin condition
  digitalWrite(LtrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(LtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(LtrigPin, LOW);
  Lduration = pulseIn(LechoPin, HIGH);
  Ldistance = Lduration * 0.034 / 2;

  digitalWrite(RtrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(RtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(RtrigPin, LOW);
  Rduration = pulseIn(RechoPin, HIGH);
  Rdistance = Rduration * 0.034 / 2;

  digitalWrite(DtrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(DtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(DtrigPin, LOW);
  Dduration = pulseIn(DechoPin, HIGH);
  Ddistance = Dduration * 0.034 / 2;

if(Rdistance > 20 || Ldistance > 20)
{
  Speed_setting(0,0);
  if (Ldistance > 40)
    {Speed_setting(0,50);//turn left
    delay(50);
    
     }
  if (Rdistance > 40)
    {Speed_setting(50,0);//turn right
    delay(50);
    
     }  
  }

if(Ddistance < 30){
  Speed_setting(0,0);
  if (Ldistance > 40)
    {Speed_setting(0,50);//turn left
    delay(50);
    
     }
  if (Rdistance > 40)
    {Speed_setting(50,0);//turn right
    delay(50);
    
     }
}


PID_control();


  // Displays the distance on the Serial Monitor
  Serial.print("Left Distance: ");
  Serial.print(Ldistance);
  Serial.println(" cm");
  Serial.print("Right Distance: ");
  Serial.print(Rdistance);
  Serial.println(" cm");
  Serial.print("Front Distance: ");
  Serial.print(Ddistance);
  Serial.println(" cm");
  delay(50);
  Serial.println("Encodeur gauche: ");
  Serial.print(_LeftEncoderTicks);
  Serial.println("Encodeur droite: ");
  Serial.print(_RightEncoderTicks);
  
}


void readRightEncoder(){
   //(digitalRead(Right_ENC_A)>0) ? (_RightEncoderTicks--) : (_RightEncoderTicks++);
   _RightEncoderTicks++;
   
}
void readLeftEncoder(){
   //(digitalRead(Right_ENC_A)>0) ? (_LeftEncoderTicks--) : (_LeftEncoderTicks++);
   _LeftEncoderTicks++;
}

void Speed_setting(int motorspeeda, int motorspeedb) {
  //set the appropriate values for aphase and bphase so that the robot goes straight
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  //delay(5000);
  analogWrite(enA, motorspeeda);
  analogWrite(enB, motorspeedb);
}

void PID_control() {
int error = _LeftEncoderTicks-_RightEncoderTicks;


Serial.println("_LeftEncoderTicks : ");
Serial.println(_LeftEncoderTicks);
Serial.println("_RightEncoderTicks: ");
Serial.println(_RightEncoderTicks);
Serial.println("Error : ");
Serial.println(error);

_LeftEncoderTicks = 0;
_RightEncoderTicks = 0;

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd; //calculate the correction
                                       
  
  int motorspeeda = basespeeda ;
  int motorspeedb = basespeedb + motorspeed;
  //int motorspeedb = basespeedb ;
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = basespeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = basespeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
  } 
  Speed_setting(motorspeeda, motorspeedb);

  
}
