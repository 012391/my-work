
#include <MQ135.h>
#include <MQUnifiedsensor.h>
#include <dht.h> //DHT11
#include <NOKIA5110_TEXT.h>
#include <SoftwareSerial.h>
SoftwareSerial pmsSerial(13, 1);


NOKIA5110_TEXT mylcd(5, 6, 7, 8, 9);
#define inverse  false
#define contrast 0xBE
#define bias 0x13
#define FontNumber 1


#define dht_dpin A5 // im using analog pin 0 for this you can change it to any analog pin
#define placa "Arduino Mega"
#define Voltage_Resolution 5
#define pin A1 //Analog input 0 of your arduino
#define type "MQ-135" //MQ135
#define ADC_Bit_Resolution 10 // For arduino UNO/MEGA/NANO
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm  

MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

void setup() {
  Serial.begin(115200);

  pmsSerial.begin(9600);
  Serialinit();

  mylcd.LCDInit(inverse, contrast, bias);
  mylcd.LCDClear();
  mylcd.LCDFont(FontNumber);
  // to make the lcd light up/ work.

}

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms5003data data;


void loop() {
  if (readPMSdata(&pmsSerial)) {
    // reading data was successful!
    Serial.println();
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (standard)");
    Serial.print("PM 1.0: "); Serial.print(data.pm10_standard);
    Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_standard);
    Serial.print("\t\tPM 10: "); Serial.println(data.pm100_standard);
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (environmental)");
    Serial.print("PM 1.0: "); Serial.print(data.pm10_env);
    Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_env);
    Serial.print("\t\tPM 10: "); Serial.println(data.pm100_env);
    Serial.println("---------------------------------------");
    Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(data.particles_03um);
    Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(data.particles_05um);
    Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(data.particles_10um);
    Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(data.particles_25um);
    Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(data.particles_50um);
    Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(data.particles_100um);
    Serial.println("---------------------------------------");
  }
}

boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }

  /* debugging
    for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
    }
    Serial.println();
  */

  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}


void DisplayMQ135() {


  MQ135.update(); // Update data, the arduino will be read the voltage on the analog pin

  MQ135.setA(605.18); MQ135.setB(-3.937); // Configurate the ecuation values to get CO concentration
  float CO = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup

  MQ135.setA(77.255); MQ135.setB(-3.18); // Configurate the ecuation values to get Alcohol concentration
  float Alcohol = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup

  MQ135.setA(110.47); MQ135.setB(-2.862); // Configurate the ecuation values to get CO2 concentration
  float CO2 = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup

  MQ135.setA(44.947); MQ135.setB(-3.445); // Configurate the ecuation values to get Tolueno concentration
  float Tolueno = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup

  MQ135.setA(102.2 ); MQ135.setB(-2.473); // Configurate the ecuation values to get NH4 concentration
  float NH4 = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup

  MQ135.setA(34.668); MQ135.setB(-3.369); // Configurate the ecuation values to get Acetona concentration
  float Acetona = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup

  Serial.print("|   "); Serial.print(CO);
  Serial.print("   |   "); Serial.print(Alcohol);
  Serial.print("   |   "); Serial.print(CO2);
  Serial.print("   |   "); Serial.print(Tolueno);
  Serial.print("   |   "); Serial.print(NH4);
  Serial.print("   |   "); Serial.print(Acetona);
  Serial.println("   |");
  /*
    Exponential regression:
    GAS      | a      | b
    CO       | 605.18 | -3.937
    Alcohol  | 77.255 | -3.18
    CO2      | 110.47 | -2.862
    Tolueno  | 44.947 | -3.445
    NH4      | 102.2  | -2.473
    Acetona  | 34.668 | -3.369
  */

  delay(500); //Sampling frequency
}


void Serialinit()
{
  Serial.begin(9600);
  MQ135.setRegressionMethod(1);
  MQ135.init();
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i ++)
  {
    MQ135.update();
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0 / 10);
  Serial.println("  done!.");

  if (isinf(calcR0)) {
    Serial.println("Warning: Conection issue founded, R0 is infite (Open circuit detected) please check your wiring and supply");
    while (1);
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Conection issue founded, R0 is zero (Analog pin with short circuit to ground) please check your wiring and supply");
    while (1);
  }
  /*****************************  MQ CAlibration ********************************************/
  Serial.println("** Lectures from MQ-135 ****");
  Serial.println("|    CO   |  Alcohol |   CO2  |  Tolueno  |  NH4  |  Acteona  |");
  delay(100);
  Serial.println("This string is only here for now");

}


void DisplayDHT()
{ // use this to make the code work with the temp sensor
  dht DHT;
  static char outstrHum[8];
  static char outstrTemp[8];
  {
    DHT.read11(dht_dpin);
    dtostrf(DHT.humidity, 6, 2, outstrHum);
    mylcd.LCDgotoXY(0, 0);
    mylcd.LCDString("Humdity");
    mylcd.LCDgotoXY(0, 1);
    mylcd.LCDString(outstrHum);
    delay(1000);


    dtostrf(DHT.temperature, 6, 2, outstrTemp);
    mylcd.LCDgotoXY(0, 2);
    mylcd.LCDString("Temperature");
    mylcd.LCDgotoXY(0, 3);
    mylcd.LCDString(outstrTemp);
    delay(1000);
  }
  delay(5000);
  mylcd.LCDClear();
  // for themp ill use 20c and 24c to be normal anything above or below will be cold or hot.
  //  for humidity  30to 50.
  DHT.read11(dht_dpin);
  dtostrf(DHT.humidity, 6, 2, outstrHum);
  dtostrf(DHT.temperature, 6, 2, outstrTemp);
  {
    if (19 < outstrTemp < 25); {
      mylcd.LCDgotoXY(0, 1);
      mylcd.LCDString("good ");
    }
    if (outstrTemp < 20)
      mylcd.LCDgotoXY(0, 3);
    mylcd.LCDString(" cold ");

    if (outstrTemp > 20); {
      mylcd.LCDgotoXY(0, 4);
      mylcd.LCDString("hot ");
    }
  }
  MQ135.update(); // Update data, the arduino will be read the voltage on the analog pin

  MQ135.setA(605.18); MQ135.setB(-3.937); // Configurate the ecuation values to get CO concentration
  float CO = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup

  MQ135.setA(77.255); MQ135.setB(-3.18); // Configurate the ecuation values to get Alcohol concentration
  float Alcohol = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup

  MQ135.setA(110.47); MQ135.setB(-2.862); // Configurate the ecuation values to get CO2 concentration
  float CO2 = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup

  MQ135.setA(44.947); MQ135.setB(-3.445); // Configurate the ecuation values to get Tolueno concentration
  float Tolueno = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup

  MQ135.setA(102.2 ); MQ135.setB(-2.473); // Configurate the ecuation values to get NH4 concentration
  float NH4 = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup

  MQ135.setA(34.668); MQ135.setB(-3.369); // Configurate the ecuation values to get Acetona concentration
  float Acetona = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup


  mylcd.LCDClear();
  mylcd.LCDgotoXY(0, 0);
  mylcd.LCDString("Co");
  mylcd.LCDgotoXY(0, 1);
  String b;

  b = String(CO);
  mylcd.LCDString(const_cast<char*>(b.c_str()));
  delay(1000);
  mylcd.LCDClear();
  mylcd.LCDgotoXY(0, 0);
  mylcd.LCDString("Alcohol");
  mylcd.LCDgotoXY(0, 1);
  String a;

  a = String(Alcohol);
  mylcd.LCDString(const_cast<char*>(a.c_str()));
  delay(1000);


}
