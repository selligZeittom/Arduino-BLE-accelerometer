/* Include libraries for the sensors */
#include <Arduino_LSM9DS1.h> /* 9 axis IMU */
/* #include <Arduino_APDS9960.h> Gesture and motion */
/* #include <Arduino_HTS221.h> Humidity and pressure*/

/* Include library for BLE */
#include <ArduinoBLE.h>

/* Create the variables for the BLE exchanges */
BLEService accelerometerService("19B10010-E8F2-537E-4F6C-D104768A1214");
BLEFloatCharacteristic accelerometerCharacteristicX("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic accelerometerCharacteristicY("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic accelerometerCharacteristicZ("19B10013-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

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
    Serial.println("Failed to initialize BLE...");
    while (1);
  }
  
  /* Set the name of the device and the service */
  BLE.setLocalName("Nano33");
  BLE.setAdvertisedService(accelerometerService);
  
  /* Add the characteristics to the service */
  accelerometerService.addCharacteristic(accelerometerCharacteristicX);
  accelerometerService.addCharacteristic(accelerometerCharacteristicY);
  accelerometerService.addCharacteristic(accelerometerCharacteristicZ);

  /* Add the service */
  BLE.addService(accelerometerService);

  /* Set an initial value */
  accelerometerCharacteristicX.writeValue(0.0f);
  accelerometerCharacteristicY.writeValue(0.0f);
  accelerometerCharacteristicZ.writeValue(0.0f);
  
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
  if (IMU.accelerationAvailable()) 
  {
    IMU.readAcceleration(x, y, z);
    
    /* Update the characteristics */
    accelerometerCharacteristicX.writeValue(x);
    accelerometerCharacteristicY.writeValue(y);
    accelerometerCharacteristicZ.writeValue(z);
  }
}
