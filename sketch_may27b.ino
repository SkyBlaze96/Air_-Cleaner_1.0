//2019-06-07 작성.
#include <Wire.h>
#include <SoftwareSerial.h>
#include <doxygen.h>
#include "ESP8266.h"
#include <string.h>
SoftwareSerial mySerial(5,6);
#define SSID "ASUS"  

#define PASSWORD "PASSWORD" // WiFi Password  

#define SERVERIP "192.168.0.31"

float previous_dustDensity = 0;
float alpha = 0.8;     // 깨끗한 상태의 측정 전압(기준)
float fanSpeed   = 0;  // 
float fanPWM     = 3;  // Pin D3 팬의 PWM 센서에 연결
int   fanSensor  = 8;  // Pin D8 팬의 RPM 센서에 연결   
 
int Speed = 0; //0부터 255까지 속도 지정.
 
unsigned long SensorPulsTijd;
int measurePin = 0; //먼지 센서의 센서를 아두이노 A0핀에 연결
int ledPower = 2; // 먼지 센서의 LED를 아두이노 D2 핀에 연결
int pm30_pin = 10; // 노랑 LED - PM 50 이하 OK
int pm60_pin = 11; // 빨강 LED - PM 70 이상 경고

int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;

float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;
float average_dustDensity = 0;
float previousDensity = 0.0;
int num = 50; //샘플수
float sum = 0.0;
float D[] = {
0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0,0.0,
0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0,0.0,
0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0,0.0,
0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0,0.0,
0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0,0.0
};

void setup(){
Serial.begin(9600);
pinMode(ledPower,OUTPUT);// 미세먼지 센서 내부 LED
pinMode(pm30_pin, OUTPUT);digitalWrite(pm30_pin,LOW); //PM30 이하LED
pinMode(pm60_pin, OUTPUT);digitalWrite(pm60_pin,LOW); //PM60 이상 LED
pinMode(fanSensor, INPUT);
pinMode(fanSpeed, INPUT);
digitalWrite(fanSensor, HIGH);

 Serial.setTimeout(5000);  
  mySerial.begin(9600);   
  Serial.println("ESP8266 connect");  
}


void loop() {

D[num-1] = particleSensing();
for ( int i = 0; i<num; i++ ) {
D[i] = D[i+1];
sum = sum + D[i];
}
average_dustDensity = sum/num;


int dnsty = alpha*previousDensity + (1.0-alpha)*average_dustDensity;

 if( dnsty <31 )
 {
  digitalWrite(pm30_pin, HIGH);
  digitalWrite(pm60_pin, LOW);
  delay(50);
  analogWrite(3, Speed=50);
  }
 else if( dnsty >30 && dnsty <81 ) 
   {
   digitalWrite(pm30_pin, LOW);
   digitalWrite(pm60_pin, HIGH);
   delay(50);
   analogWrite(3, Speed=155); 
   }
 else if( dnsty >90 ) 
   {
    digitalWrite(pm30_pin, HIGH);
    digitalWrite(pm60_pin, HIGH);
    delay(70);
    analogWrite(3, Speed=255);
   }



sum = 0.0;
previousDensity = average_dustDensity;

 Serial.print("Dust Level: ");    
  Serial.println((int)dnsty);   
  Serial.print("[ug/m3]");
     String sdnsty = String(dnsty);
   
     String cmd = "AT+CIPSTART=\"TCP\",\"";  
     cmd += SERVERIP;  
     cmd += "\",80";  
     Serial.println(cmd);  
     mySerial.println(cmd);  
     if(mySerial.find("Error"))  
    {  
      Serial.println( "TCP connect error" );  
      return;  
    }  
     
     cmd = "GET /test.php?dust="+sdnsty+"\r\n";  
     mySerial.print("AT+CIPSEND=");  
     mySerial.println(cmd.length());  
          
       
     Serial.println(cmd);  
       
       
     if(mySerial.find(">"))  
     {  
       Serial.print(">");  
       }else  
       {  
         mySerial.println("AT+CIPCLOSE");  
         Serial.println("connect timeout");  
         delay(1000);  
         return;  
       }  
         
       mySerial.print(cmd);  
       delay(2000);  
       //Serial.find("+IPD");  
       while (Serial.available())  
       {  
         char c = Serial.read();  
         mySerial.write(c);  
         if(c=='\r') mySerial.print('\n');  
       }  
       Serial.println("====");  
       delay(1000);  

}


float particleSensing() {

digitalWrite(ledPower,LOW); // power on the LED
delayMicroseconds(samplingTime);
voMeasured = analogRead(measurePin);
delayMicroseconds(deltaTime);
digitalWrite(ledPower,HIGH); // turn the LED off
delayMicroseconds(sleepTime);

// 0 - 5V mapped to 0 - 1023 integer values
calcVoltage = voMeasured * (5.0 / 1024.0);
if( calcVoltage > 0.6 ) {
// linear eqn from http://www.howmuchsnow.com/arduino/airquality/
dustDensity =1000.0*( 0.172 * calcVoltage - 0.1);
delay(190);
}
return dustDensity;
}

boolean connectWiFi()  
{  
     
   String cmd="AT+CWJAP=\"";  
   cmd+=SSID;  
   cmd+="\",\"";  
   cmd+=PASSWORD;  
   cmd+="\"";  
   mySerial.println(cmd);  
   Serial.println(cmd);  
   delay(3000);  
    
   if(mySerial.find("OK"))  
   {  
     Serial.println("OK, Connected to WiFi.");  
     return true;  
   }  
   else  
   {  
     Serial.println("Can not connect to the WiFi.");  
     return false;  
   }  
 }  
