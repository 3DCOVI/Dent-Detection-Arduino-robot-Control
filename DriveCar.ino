//arduino pin declarations
int enA=5;n
int inA1=6;
int inA2=7;
int enB=11;
int inB1=12;
int inB2=13;
//***********************************************
//sonar sensor pin declarations
const int trigPin=9;
const int echoPin=10;
long duration;
int distance;
int motorIndex=1;
int a;
//************************************************
//assorted declarations for initializing loops, data reading, etc..
int incomingByte;
bool light = false;
bool start = false;
bool loopThis = false;
int speeed = 200;
int slowDownCoe = 50;
int getDist(){
    //call the distance
  digitalWrite(trigPin,LOW);
  delay(40);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
  duration=pulseIn(echoPin,HIGH);
  distance=duration*0.034/2;
  return distance;
}

void bwd(double spd){
 //move motor backwards at speed 'spd'
  digitalWrite(inA1,HIGH);
  digitalWrite(inA2,LOW);
  analogWrite(enA,spd);
  digitalWrite(inB1,HIGH);
  digitalWrite(inB2,LOW);
  analogWrite(enB,spd);
}

void fwd(double spd){
 //move motor forwards at speed 'spd'
  digitalWrite(inA1,LOW);
  digitalWrite(inA2,HIGH);
  analogWrite(enA,spd);
  digitalWrite(inB1,LOW);
  digitalWrite(inB2, HIGH);
  analogWrite(enB,spd);
}
 void stp(){
  //stop all motors
  digitalWrite(inA1,LOW);
  digitalWrite(inA2,LOW);
  analogWrite(enA,0);
  digitalWrite(inB1,LOW);
  digitalWrite(inB2,LOW);
  analogWrite(enB,0);
 }
void setup() {
 //setup serial port and output pins
`Serial.begin(9600);
 Serial.println("start");
 pinMode(enA,OUTPUT);
 pinMode(inA1,OUTPUT);
 pinMode(inA2,OUTPUT);
 pinMode(enB,OUTPUT);
 pinMode(inB1,OUTPUT);
 pinMode(inB2,OUTPUT);
 pinMode(trigPin,OUTPUT);
 pinMode(echoPin,INPUT);
 Serial.println("starting");
}
void loop() {
  //******************************
 //wait for serial trigger '1'
  if(!loopThis){
    while (!start){
   // Serial.println("idle");
      if (Serial.available() > 0) {
                    // read the incoming byte:
                    
                    //delay(100);
                    incomingByte = Serial.read();
                  Serial.println(incomingByte);
                    if (incomingByte == 49){// ASCII code 49 = 1
                      start = true;//prevent this loop from re running
                      break;// exit loop
                      }
            }
    }
   //*************************************************
    fwd(speeed);// move forwards
    //*****************************************************
    bool trig = false;
    while(start){     
    a=getDist();
      if (a % 15 <=1  && !trig ){ // every 15-16 cm send a trigger command to matlab to take picture
       // Serial.println(a);
       //Serial.println(a % 20);
       Serial.println("trigger");
       trig = true; // send trigger once act as button
       stp(); // stop motors
       delay(8000);
       Serial.println("cont");//// tell matlabs that the program is continuing
       fwd(speeed);// move forwards
      }
      else if (!(a % 20 <=1)) {
        trig = false; // allow trigger command to be sent again
      }
     //breaker
     if(getDist() >= 85 && getDist() != 0){ // set maximum scanning range to return
      stp();
      Serial.println("done"); // end matlab serial collection 
      delay(100);
      start = false;
      break;
     }
    }
    //***************************************
    
    bwd(speeed);// return
    //************************************
    while(getDist()>= 7){//Serial.println("idle"); //stop moving backwards after 7 cm
    if(getDist() <= 20){// after 20 cm slow down
        bwd(speeed - slowDownCoe);
    }
    }
    //*************************************************
    stp();
    loopThis = true;// end full loop until program reset
  }
    
  }
//************************************************



