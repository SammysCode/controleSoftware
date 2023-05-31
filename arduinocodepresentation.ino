#include <SoftwareSerial.h>

//buzzer
int buzzer = 13;
bool shockmemorie = false;

//shock
int shock = 2;
int val;

//motor
int motoropen1 = 3;
int motorclose1 = 4;
int motoropen2 = 8;
int motorclose2 = 9;
bool moving = false;

//bluetooth
SoftwareSerial bluetooth(10, 11);
int bluetoothconnected = 5;
bool lockedorunlocked = false;
bool previouslyconnected = false;

void setup() {

 //buzzer
 pinMode(buzzer, OUTPUT);

 //Shock  
 pinMode (shock, INPUT); 

 //motor
 pinMode( motoropen1,OUTPUT);
 pinMode( motorclose1,OUTPUT);
 pinMode( motoropen2,OUTPUT);
 pinMode( motorclose2,OUTPUT);

 //bluetooth
 Serial.begin(9600);
 bluetooth.begin(9600);
 pinMode(bluetoothconnected, INPUT_PULLUP);

}

void loop() {
 
 //buzzer and Shock
 val = digitalRead (shock); 
	if (val == LOW ) { 
    shockmemorie = true;
    if(shockmemorie){
     digitalWrite(buzzer, HIGH); 
    }
	} 
	   

 //motor
 if (bluetooth.available()) {
    char receivedChar = bluetooth.read();
    Serial.println(receivedChar);

    
    if (receivedChar == '1') {
      digitalWrite(motoropen1, HIGH);
      digitalWrite(motoropen2, HIGH);
      digitalWrite(motorclose1, LOW);
      digitalWrite(motorclose2, LOW);
      bluetooth.println("locking bike");
      moving = true;
      delay(2000);
      digitalWrite(motoropen1, LOW);
      digitalWrite(motoropen2, LOW);
      lockedorunlocked = true;
      if (moving){
        int sensorValue = analogRead(A0);
  Serial.println(sensorValue);
      if (sensorValue > 1020) {
       digitalWrite(motoropen1, LOW);
       digitalWrite(motoropen2, LOW);
       digitalWrite(motorclose1, HIGH);
       digitalWrite(motorclose2, HIGH);
       bluetooth.println("Locking failed. Remove blockage and lock again");
  }
  }
      moving = false;
    } else if (receivedChar == '0') {
      digitalWrite(motoropen1, LOW);
      digitalWrite(motoropen2, LOW);
      digitalWrite(motorclose1, LOW);
      digitalWrite(motorclose2, LOW);
      bluetooth.println("LED is OFF");
    } else if (receivedChar == '2') {
       digitalWrite(motoropen1, LOW);
       digitalWrite(motoropen2, LOW);
      digitalWrite(motorclose1, HIGH);
      digitalWrite(motorclose2, HIGH);
      bluetooth.println("opening bike");
      moving = true;
      delay(2000);
      digitalWrite(motorclose1, LOW);
      digitalWrite(motorclose2, LOW);
      lockedorunlocked = false;
      if (moving){
        int sensorValue = analogRead(A0);
        Serial.println(sensorValue);
        if (sensorValue > 100) {
        digitalWrite(motoropen1, LOW);
        digitalWrite(motoropen2, LOW);
        digitalWrite(motorclose1, HIGH);
        digitalWrite(motorclose2, HIGH);
        bluetooth.println("Locking failed. Remove blockage and lock again");
  }
  }
      moving = false;
    } 
    if (receivedChar == '3'){
      shockmemorie = false;
      digitalWrite(buzzer, LOW);
      bluetooth.println("Alarm Muted");
    }
  }
  
 
  int outputstate = digitalRead(bluetoothconnected);
  if (outputstate == HIGH) {
    delay(1000);
    if (lockedorunlocked && !previouslyconnected) {
      bluetooth.println("Bike is locked");
      previouslyconnected = true;
    } else if (!lockedorunlocked && !previouslyconnected){
      bluetooth.println("Bike is unlocked");
      previouslyconnected = true;
    } 
  } else {
   previouslyconnected = false;
  }
}
