// Pin assignments
#define AIN1 3
#define AIN2 4
#define APWM 5
#define BIN1 12
#define BIN2 13
#define BPWM 11
#define STBY 6

// Constants for motor control functions
#define STEPTIME 600 
#define STRAIGHTSPEED 200
#define TURNSPEED 120
#define TURNTIME 300

#include <NewPing.h>
#define TRIGGER_PIN  10  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     7   // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.


#include "WiFiEsp.h"
#include <WiFiEspUdp.h>
char ssid[] = "MIKHAILS";     // your network SSID (name)
char pwd[] = "ZhabaEstKomarov";  // your network password

// Emulate Serial1 on pins 7/6 if not present
//#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(8, 9); // RX, TX
//#endif
WiFiEspUDP Udp;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// Array to track current PWM values for each channel (A and B)
int pwms[] = {APWM, BPWM};

// Offsets to be used to compensate for one motor being more powerful
byte leftOffset = 0;
byte rightOffset = 0;

// Variable to track remaining time
//unsigned long pwmTimeout = 0;

// Function to write out pwm values
void writePwms ( int left, int right) {
    analogWrite (pwms[0], left);
    analogWrite (pwms[1], right);
}

void setSpeed(float left, float right) {
    digitalWrite(STBY, HIGH);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    writePwms (left * (STRAIGHTSPEED-leftOffset), right * (STRAIGHTSPEED-rightOffset));
//    pwmTimeout = millis() + STEPTIME;
}



// Move the robot forward for STEPTIME
void goForward ( ) {
    digitalWrite(STBY, HIGH);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    writePwms (STRAIGHTSPEED-leftOffset,STRAIGHTSPEED-rightOffset);
//    pwmTimeout = millis() + STEPTIME;
}


// Move the robot backward for STEPTIME
void goBack() {
    digitalWrite(STBY, HIGH);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    writePwms (STRAIGHTSPEED-leftOffset,STRAIGHTSPEED-rightOffset);
//    pwmTimeout = millis() + STEPTIME;
}

// Turn the robot left for TURNTIME
void goRight () {
    digitalWrite(STBY, HIGH);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);

    writePwms (TURNSPEED,TURNSPEED);
//    pwmTimeout = millis() + TURNTIME;
}

// Turn the robot right for TURNTIME
void goLeft () {
    digitalWrite(STBY, HIGH);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

    writePwms (TURNSPEED,TURNSPEED);
//    pwmTimeout = millis() + TURNTIME;
}

// Stop the robot (using standby)
void stop(){
    digitalWrite(STBY, LOW); 
}

// Arduino setup function
void setup() {
    // Initialize pins as outputs
    pinMode (STBY, OUTPUT);
    pinMode (AIN1, OUTPUT);
    pinMode (AIN2, OUTPUT);
    pinMode (APWM, OUTPUT);
    pinMode (BIN1, OUTPUT);
    pinMode (BIN2, OUTPUT);
    pinMode (BPWM, OUTPUT);

    Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
    randomSeed(analogRead(0));

    Serial1.begin(9600);
    WiFi.init(&Serial1);
    WiFi.begin(ssid, pwd);
    while (true) {              
       if (WiFi.status() == WL_CONNECTED) {
         IPAddress ip = WiFi.localIP();
         Serial.print("IP Address: ");
         Serial.println(ip);
         break;
       }
       Serial.println(WiFi.status());
       delay(100);
    }
    Udp.begin(3333);    
}

void loop() {
  //forward_when_can();
  udp_control();
}


char packetBuffer[255];          // buffer to hold incoming packet
char ReplyBuffer[] = "ACK";      // a string to send back

void udp_control() {
  int packetSize = Udp.parsePacket();
  if (!packetSize) return;
  Serial.print("Received packet of size ");

  Serial.println(packetSize);
  Serial.print("From ");
  IPAddress remoteIp = Udp.remoteIP();
  Serial.print(remoteIp);
  Serial.print(", port ");
  Serial.println(Udp.remotePort());

  // read the packet into packetBufffer
  int len = Udp.read(packetBuffer, 255);
  if (len > 0) {
    packetBuffer[len] = 0;
  }
  Serial.println("Contents:");
  Serial.println(packetBuffer);

  // send a reply, to the IP address and port that sent us the packet we received
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(ReplyBuffer);
  Udp.endPacket();

  switch (packetBuffer[0]) {
    case 'f':
      setSpeed(.5,.5);
      break;
    case 's':
      stop();
      break;
    case 'b':
      goBack();
      break;  
    case 'l':
      setSpeed(.2, .6);
      break;
    case 'r':
      setSpeed(.6, .2);
      break;
  }
}

int blocked = 0;
long turn_direction = 0;
void forward_when_can() {  
  unsigned long distance = sonar.ping_cm();
  //Serial.println(distance);
  if (distance > 0 && distance < 100) {
    blocked++;
    if (blocked > 30) {
      goBack();
      delay(1000);
      blocked = 0;
    }
    if (blocked == 1) { // we just got stuck -- choose turn direction
      turn_direction = random(2);
    }
    if (turn_direction == 0) {
      setSpeed(.2, .6);    
    } else {
      setSpeed(.6, .2);    
    }
  } else {
    setSpeed(.6, .6);
    blocked = 0;
  }
  delay(50);
}

void ping_test() {
  Serial.print("Ping: ");
  Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
  delay(100);
}

// Loop (code betwen {}'s repeats over and over again)
float left = 0, right = 0;
void circles() {    
    left = .4;
    right = .8;
    setSpeed(left, right);
    delay(2000);
    left = .8;
    right = .4;
    setSpeed(left, right);
    delay(2000);
      /*
    // Make the robot go Forward.
    goForward();
    // Wait for one second
    delay(1000);
    // Make the robot stop
    stop();
    // Wait for one second
    delay(1000);
    // Make the robot go backward
    goBack();
    // Wait for one second
    delay(1000);
    // Make the robot stop
    stop();
    // Wait for one second
    delay(1000);*/
}
