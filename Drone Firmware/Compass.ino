#include <Wire.h>

#define Compass_addr 0x1E //0011110b, I2C 7bit Compass_addr of HMC5883
int x, y, z; //triple axis data
int xmin, xmax, ymin, ymax, zmin, zmax;
float angle_r, angle_d, dec = -0.6;
float angle_flt;
int curr_angle;

void setup() {
  //Initialize Serial and I2C communications
  Serial.begin(57600);
  Wire.begin();
  //Put the HMC5883 IC into the correct operating mode
  setup_compass();

}

void loop() {

  read_compass_data();


  //Print out values of each axis
  Serial.print("Angle in degrees: ");
  curr_angle = angle_flt;
  Serial.println(curr_angle);

}


void setup_compass()
{
  Wire.beginTransmission(Compass_addr); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
}

void read_compass_data()
{
  xmin = 0; xmax = 0; ymax = 0; ymin = 0; zmin = 0; zmax = 0;
  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(Compass_addr);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();

  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(Compass_addr, 6);

  if (6 <= Wire.available()) {
    x = Wire.read() << 8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read() << 8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read() << 8; //Y msb
    y |= Wire.read(); //Y lsb
  }

  angle_r = atan2(y, x);

  angle_r += dec;

  if (angle_r < 0) angle_r += 2 * PI;   // Correcting when signs are reveresed
  if (angle_r > 2 * PI)angle_r -= 2 * PI; // Correcting due to the addition of the declination angle

  angle_d = angle_r * 180 / PI; // The heading in Degrees unit

  angle_flt = angle_flt * 0.00 + angle_d * 1; // Smoothing the output angle / Low pass filter
}
