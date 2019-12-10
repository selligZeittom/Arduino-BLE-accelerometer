/* Include libraries for the sensors */
#include <Arduino_LSM9DS1.h> /* 9 axis IMU */
/* #include <Arduino_APDS9960.h> Gesture and motion */
/* #include <Arduino_HTS221.h> Humidity and pressure*/

/* Include library for BLE */
#include <ArduinoBLE.h>

/* Define some values for bit reduction */
#define MAX_VIBRATION_VALUE (4.0f)
#define MIN_VIBRATION_VALUE (0.0f)
#define DIVIDOR ((float)(MAX_VIBRATION_VALUE - MIN_VIBRATION_VALUE))
#define FACTOR ((float)(0xff))

/* Create the variables for the BLE exchanges */
BLEService accelerometerService("19B10010-E8F2-537E-4F6C-D104768A1214");
BLEByteCharacteristic accelerometerCharacteristicX("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEByteCharacteristic accelerometerCharacteristicY("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEByteCharacteristic accelerometerCharacteristicZ("19B10013-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);


/*
  Performs a normalization of the acceleration to save memory
  -2.0 -> 0
  +2.0 -> 255
*/
uint8_t normalize_acceleration(float fAcc)
{
  /* Put an offset to the value */
  float temp = fAcc + 2.0f;

  /* Check for limits */
  if(temp > MAX_VIBRATION_VALUE)
  {
    temp = MAX_VIBRATION_VALUE;
  }
  else if (temp < MIN_VIBRATION_VALUE)
  {
    temp = MIN_VIBRATION_VALUE;
  }
  temp *= FACTOR;
  temp /= DIVIDOR;
  uint8_t ret_val = (uint8_t)(temp);
  return ret_val;
}

void setup() 
{
  /* Serial communication set up at 115200 bauds */
  Serial.begin(9600);
  while(!Serial){};

  /* Initialize the 9 axis IMU */
  if (!IMU.begin()) 
  {
    Serial.println("Failed to initialize IMU...");
    while (1);
  }

  /* Initialize BLE */
  if (!BLE.begin()) 
  {
    Serial.println("starting BLE failed!");
    while (1);
  }
  
  /* Set the name of the device and the service */
  BLE.setLocalName("AccelerometerNano33");
  BLE.setAdvertisedService(accelerometerService);
  
  /* Add the characteristics to the service */
  accelerometerService.addCharacteristic(accelerometerCharacteristicX);
  accelerometerService.addCharacteristic(accelerometerCharacteristicY);
  accelerometerService.addCharacteristic(accelerometerCharacteristicZ);

  
  /* Add the service */
  BLE.addService(accelerometerService);

  /* Set an initial value */
  accelerometerCharacteristicX.writeValue(0);
  accelerometerCharacteristicY.writeValue(0);
  accelerometerCharacteristicZ.writeValue(0);
  
  /* Start advertising */
  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() 
{
  /* Poll for BLE events */
  BLE.poll();
  
  /* Get the accelerations */
  float x, y, z;
  uint8_t bX, bY, bZ;
  bX = bY = bZ = 0;

  if (IMU.accelerationAvailable()) 
  {
    IMU.readAcceleration(x, y, z);
    bX = normalize_acceleration(x);
    bY = normalize_acceleration(y);
    bZ = normalize_acceleration(z);
    Serial.print("X : ");
    Serial.print(bX);
    Serial.print(", Y : ");
    Serial.print(bY);
    Serial.print(", Z : ");
    Serial.println(bZ);
    
    /* Update the characteristics */
    accelerometerCharacteristicX.writeValue(bX);
    accelerometerCharacteristicY.writeValue(bY);
    accelerometerCharacteristicZ.writeValue(bZ);
  }
}
