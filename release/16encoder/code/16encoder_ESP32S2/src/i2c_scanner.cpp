

// #include <Arduino.h>
// #include <Wire.h>

// void I2C_ScannerWire();
// void I2C_ScannerWire1();

// void setup()
// {
//   Serial.begin(115200);

//   Wire.begin();        // default: 21, 22
//   Wire1.begin(19, 18); // I don't know the default pins !
// }

// void loop()
// {
//   Serial.println("---------- Scanning Wire -------------");
//   I2C_ScannerWire();

//   Serial.println("---------- Scanning Wire1 ------------");
//   I2C_ScannerWire1();
//   delay(2000);
// }

// void I2C_ScannerWire()
// {
//   byte error, address;
//   int nDevices;

//   Serial.println("Scanning...");

//   nDevices = 0;
//   for (address = 1; address < 127; address++)
//   {
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();

//     if (error == 0)
//     {
//       Serial.print("I2C device found at address 0x");
//       if (address < 16)
//         Serial.print("0");
//       Serial.print(address, HEX);
//       Serial.println("  !");

//       nDevices++;
//     }
//     else if (error == 4)
//     {
//       Serial.print("Unknown error at address 0x");
//       if (address < 16)
//         Serial.print("0");
//       Serial.println(address, HEX);
//     }
//   }
//   if (nDevices == 0)
//     Serial.println("No I2C devices found\n");
//   else
//     Serial.println("done\n");
// }

// void I2C_ScannerWire1()
// {
//   byte error, address;
//   int nDevices;

//   Serial.println("Scanning...");

//   nDevices = 0;
//   for (address = 1; address < 127; address++)
//   {
//     Wire1.beginTransmission(address);
//     error = Wire1.endTransmission();

//     if (error == 0)
//     {
//       Serial.print("I2C device found at address 0x");
//       if (address < 16)
//         Serial.print("0");
//       Serial.print(address, HEX);
//       Serial.println("  !");

//       nDevices++;
//     }
//     else if (error == 4)
//     {
//       Serial.print("Unknown error at address 0x");
//       if (address < 16)
//         Serial.print("0");
//       Serial.println(address, HEX);
//     }
//   }
//   if (nDevices == 0)
//     Serial.println("No I2C devices found\n");
//   else
//     Serial.println("done\n");
// }
 


