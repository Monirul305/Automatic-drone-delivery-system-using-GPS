#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";     //Byte of array representing the address. This is the address where we will send the data. This should be same on the receiving side.



int pitch_pin = A7, roll_pin = A6, yaw_pin = A4, throttle_pin = A5, start_pin = A2, stop_pin = A3;
int pitch_val, roll_val, yaw_val, th_val, start_val, stop_val, servo_val;
int x, y, roll, th;
int pitch_f = 21, roll_f = 31, yaw_f = 41, th_f = 11, s0 = 61;
int pitch_b = 20, roll_b = 30, yaw_b = 40, th_b = 10, s90 = 60;
int s_t = 99, s_p = 88;
double longtitude, latitude;


void setup() {
//  Serial.begin(9600);

  radio.begin();                  //Starting the Wireless communication
  radio.openWritingPipe(address); //Setting the address where we will send the data
  radio.setPALevel(RF24_PA_MIN);  //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.stopListening();          //This sets the module as transmitter
}





void loop()
{
  roll_val = analogRead(roll_pin);
  pitch_val = analogRead(pitch_pin);
  yaw_val = analogRead(yaw_pin);
  th_val = analogRead(throttle_pin);
  start_val = analogRead(start_pin);
  stop_val = analogRead(stop_pin);
  servo_val = analogRead(A0);

  //////////////////////////////////////////////////////////////////////////////////////send 3(High) 2(no action) 1(low)

  if (roll_val == 1023)
    radio.write(&roll_f, sizeof(roll_f));
  else if (roll_val == 0)
    radio.write(&roll_b, sizeof(roll_b));


  if (pitch_val == 1023)
    radio.write(&pitch_f, sizeof(pitch_f));
  else if (pitch_val == 0)
    radio.write(&pitch_b, sizeof(pitch_b));


  if (yaw_val == 1023)
    radio.write(&yaw_f, sizeof(yaw_f));
  else if (yaw_val == 0)
    radio.write(&yaw_b, sizeof(yaw_b));

  if (th_val == 1023)
    radio.write(&th_f, sizeof(th_f));
  else if (th_val <= 10)
    radio.write(&th_b, sizeof(th_b));

  if (start_val == 0)
    radio.write(&s_t, sizeof(s_t));

  if (stop_val == 0)
    radio.write(&s_p, sizeof(s_p));




    if (servo_val >= 650)
    radio.write(&s0, sizeof(s0));
  else if (servo_val <= 300)
    radio.write(&s90, sizeof(s90));

//  if (Serial.available())
//  {
//    longtitude = Serial.parseFloat();
//    latitude = Serial.parseFloat();
//    Serial.println(longtitude,8);
//    Serial.println(latitude,8);
//    radio.write(&longtitude, sizeof(longtitude));
//    delay(100);
//    radio.write(&latitude, sizeof(latitude));
//  }



  delay(100);


  //    Serial.print("roll_val=");
  //    Serial.print(roll_val);
  //    Serial.print("  pitch_val=");
  //    Serial.print(pitch_val);
  //    Serial.print("  yaw_val=");
  //    Serial.print(yaw_val);
  //    Serial.print("  th_val=");
  //    Serial.print(th_val);
  //    Serial.print("  start_val=");
  //    Serial.print(start_val);
  //    Serial.print("  stop_val=");
  //    Serial.println(stop_val);



}
