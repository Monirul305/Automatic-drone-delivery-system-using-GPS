#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(2,3); // CE, CSN
const byte address[6] = "00001";
int rc = 0;

void setup() {
  Serial.begin(57600);

  radio.begin();
  radio.openReadingPipe(0, address);   //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MIN);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.startListening();              //This sets the module as receiver
}
void loop()
{
  if (radio.available()) {             //Looking for the data.
    radio.read(&rc, sizeof(rc));    //Reading the data
    Serial.print("rc=");
    Serial.println(rc);
  }

}
