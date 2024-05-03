//Includes
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
int button_state;
int esc1 = 2, esc2 = 3, esc3 = 4, esc4 = 5;
int count = 0;

#include <Servo.h>
Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;     // create servo object to control the ESC

//Variables
float elapsedTime, time, timePrev;        //Variables for time control
long acc_x, acc_y, acc_z, acc_total_vector[20], acc_av_vector, vibration_total_result;
byte eeprom_data[36], start, data;
int loop_counter, gyro_address, vibration_counter;



void setup() {
    Wire.begin();                           //begin the wire comunication
    Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)
    Wire.write(0x6B);                       //make the reset (place a 0 into the 6B register)
    Wire.write(0x00);
    Wire.endTransmission(true);             //end the transmission
  
    Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)
    Wire.write(0x1B);                       //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x10);                       //Set the register bits as 00010000 (1000dps full scale)
    Wire.endTransmission(true);             //End the transmission with the gyro

  Serial.begin(9600);                     //Remember to set this same baud rate to the serial monitor
    time = millis();                        //Start counting time in milliseconds


  radio.begin();
  radio.openReadingPipe(0, address);   //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MIN);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.startListening();              //This sets the module as receiver

  ESC1.attach(esc1, 1000, 2000);
  ESC2.attach(esc2, 1000, 2000);
  ESC3.attach(esc3, 1000, 2000);
  ESC4.attach(esc4, 1000, 2000); // (pin, min pulse width, max pulse width in microseconds)


}//end of setup void


void loop() {
  Wire.beginTransmission(0x68);                                           //Start communication with the gyro.
  Wire.write(0x3B);                                                               //Start reading @ register 43h and auto increment with every read.
  Wire.endTransmission();                                                         //End the transmission.
  Wire.requestFrom(0x68, 6);                                              //Request 6 bytes from the gyro.
  while (Wire.available() < 6);                                                   //Wait until the 6 bytes are received.
  acc_x = Wire.read() << 8 | Wire.read();                                         //Add the low and high byte to the acc_x variable.
  acc_y = Wire.read() << 8 | Wire.read();                                         //Add the low and high byte to the acc_y variable.
  acc_z = Wire.read() << 8 | Wire.read();                                         //Add the low and high byte to the acc_z variable.

  acc_total_vector[0] = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector.

  acc_av_vector = acc_total_vector[0];                                            //Copy the total vector to the accelerometer average vector variable.

  for (start = 16; start > 0; start--) {                                          //Do this loop 16 times to create an array of accelrometer vectors.
    acc_total_vector[start] = acc_total_vector[start - 1];                        //Shift every variable one position up in the array.
    acc_av_vector += acc_total_vector[start];                                     //Add the array value to the acc_av_vector variable.
  }

  acc_av_vector /= 17;                                                            //Divide the acc_av_vector by 17 to get the avarage total accelerometer vector.

  if (vibration_counter < 20) {                                                   //If the vibration_counter is less than 20 do this.
    vibration_counter ++;                                                         //Increment the vibration_counter variable.
    vibration_total_result += abs(acc_total_vector[0] - acc_av_vector);           //Add the absolute difference between the avarage vector and current vector to the vibration_total_result variable.
  }
  else {
    vibration_counter = 0;                                                        //If the vibration_counter is equal or larger than 20 do this.
    Serial.println(vibration_total_result / 50);                                  //Print the total accelerometer vector divided by 50 on the serial monitor.
    vibration_total_result = 0;                                                   //Reset the vibration_total_result variable.
  }


  //////////////////////////////////////////////////////////
  if (radio.available())              //Looking for the data.
  {
    radio.read(&button_state, sizeof(button_state));    //Reading the data
    button_state = map(button_state, 0, 375, 0, 60);
    ESC1.write(button_state);
    ESC2.write(button_state+5);
    ESC3.write(button_state);
    ESC4.write(button_state);
//        Serial.print("    |    ");
//        Serial.println(button_state);
  }
}
