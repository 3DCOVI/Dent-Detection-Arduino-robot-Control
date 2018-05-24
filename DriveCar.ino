int enA=5;
int inA1=6;
int inA2=7;
int enB=11;
int inB1=12;
int inB2=13;
int power=1;
//***********************************************
const int trigPin=9;
const int echoPin=10;
long duration;
int distance;
int motorIndex=1;
int a;
//************************************************
 int incomingByte;
 bool light = false;
 bool start = false;
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
  digitalWrite(inA1,HIGH);
  digitalWrite(inA2,LOW);
  analogWrite(enA,spd);
  digitalWrite(inB1,HIGH);
  digitalWrite(inB2,LOW);
  analogWrite(enB,spd);
}

void fwd(double spd){
  digitalWrite(inA1,LOW);
  digitalWrite(inA2,HIGH);
  analogWrite(enA,spd);
  digitalWrite(inB1,LOW);
  digitalWrite(inB2, HIGH);
  analogWrite(enB,spd);
}
 void stp(){
      digitalWrite(inA1,LOW);
  digitalWrite(inA2,LOW);
  analogWrite(enA,0);
  digitalWrite(inB1,LOW);
  digitalWrite(inB2,LOW);
  analogWrite(enB,0);
 }
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(13, OUTPUT);
 Serial.println("start");
 //delayMicroseconds(10); 
  // put your setup code here, to run once:

  pinMode(enA,OUTPUT);
  pinMode(inA1,OUTPUT);
  pinMode(inA2,OUTPUT);
  pinMode(enB,OUTPUT);
  pinMode(inB1,OUTPUT);
  pinMode(inB2,OUTPUT);
  //--------------------------
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  Serial.begin(9600);
  //Serial.println("CLEARDATA");
 // Serial.println("LABEL,Time(us),Distance(cm)");
  //Serial.println("RESETTIMER");
  Serial.println("starting");
}
 bool loopThis = false;
void loop() {
  //******************************
  if(!loopThis){
    while (!start){
   // Serial.println("idle");
      if (Serial.available() > 0) {
                    // read the incoming byte:
                    
                    //delay(100);
                    incomingByte = Serial.read();
                  Serial.println(incomingByte);
                    if (incomingByte == 49){
                      start = true;
                      break;
                      }
            }
    }
   //*************************************************
    fwd(speeed);
    //*****************************************************
    bool trig = false;
    while(start){
     // delayMicroseconds(100);
    //  
      
    a=getDist();
      if (a % 15 <=1  && !trig ){
        // Serial.println(a);
         //Serial.println(a % 20);
        Serial.println("trigger");
        
        
      trig = true; // send trigger once
      stp();
      delay(8000);
       Serial.println("cont");
      fwd(speeed);
     
      }
      else if (!(a % 20 <=1)) {
        trig = false;
      }
     //breaker
     if(getDist() >= 85 && getDist() != 0){
      stp();
        Serial.println("done");
      delay(100);
        start = false;
        break;
     }
    }
    //***************************************
    
    bwd(speeed);
    //************************************
    while(getDist()>= 7){//Serial.println("idle");
    if(getDist() <= 20){
        bwd(speeed - slowDownCoe);
    }
    }
    //*************************************************
    stp();
    loopThis = true;
  }
    
  }
//************************************************



