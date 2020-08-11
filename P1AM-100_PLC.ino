/*
  MRS-611 P1AM-100_PLC.ino
  P1AM Module, P1 IO Modules, P1AM-ETH module.
  Ethernet Modbus TCP Slave/Server communicates with
  Python program on PC.
  Ethernet Modbus TCP Master/Client communicates with
  P1AM sampling Arduino.

  The P1AM-100 cpu will require external 24vdc power
  for the IO modules to function.

  Required Libraries which need to be installed.
  https://github.com/arduino-libraries/ArduinoModbus
  https://github.com/arduino-libraries/ArduinoRS485
  https://github.com/facts-engineering/P1AM

   _____  _____  _____  _____  _____  
   |  P  ||  P  ||  S  ||  S  ||  S  | Slot 1:  P1-16ND3
   |  1  ||  1  ||  L  ||  L  ||  L  | Slot 2:  P1-16TR
   |  A  ||  A  ||  O  ||  O  ||  O  | Slot 3:  P1-08DAL-2
   |  M  ||  M  ||  T  ||  T  ||  T  |
   |  -  ||  -  ||     ||     ||     |
   |  E  ||  1  ||  0  ||  0  ||  0  |
   |  T  ||  0  ||  1  ||  2  ||  3  |
   |  H  ||  0  ||     ||     ||     |
    ¯¯¯¯¯  ¯¯¯¯¯  ¯¯¯¯¯  ¯¯¯¯¯  ¯¯¯¯¯  
*/

#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include <P1AM.h>

const int numCoils = 11;
const int numDiscreteInputs = 1;
const int numInputRegisters = 1;
const int numHoldingRegisters = 2000;

byte mac[] = { //Use the Reserved MAC printed on the right-side label of your P1AM-ETH.
   0xDE,
   0xAD,
   0xBE,
   0xEF,
   0xFE,
   0x30
};
IPAddress ip(192, 168, 0, 30); //IP Address of the P1AM-ETH module.
IPAddress sampleserver(192, 168, 0, 40); //IP Addresses of the Server

unsigned int pedalselect = 0;                    // Pedal #
unsigned int airflowrate;                    // Air Flow Rate
unsigned int maxpresspedaltime;                   // Max Press Time
unsigned int maxholdpedaltime;                    // Max Hold Time
unsigned int maxnumpresses;               // Max # numpresses     need max 21 presses
unsigned int strextension;                // Fully pressed position on string encoder
unsigned int strcontraction;                // Fully released position on string encoder

unsigned int numpresses;                  // Return Actual # presses
unsigned int returnerrorcode;             // Return error code
unsigned int timetotabledown;                  // Return Time to complete press
unsigned int readp1;                      // Temporary variable to read P1-16ND3

boolean upsensor;
boolean downsensor;
boolean mechdock;
boolean elecdock;
boolean timerdone;
boolean endoftravel;
boolean upreached;

unsigned int maxnumsamples;
unsigned int numsamples;
unsigned int numsamplesloc;
unsigned int datapointerloc;
unsigned int dataloc;
unsigned long startTime = 0;
unsigned long elapsedTime = 0;

unsigned int capturedsamples = 0;
unsigned int sampleready = 0;
unsigned int airpressure = 0;
unsigned int stringposition = 0;

EthernetServer server(502); //Standard Modbus Port is 502
EthernetClient client;
EthernetClient client2;
ModbusTCPServer modbusTCPServer;
ModbusTCPClient modbusTCPClient(client2);

void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200);
   while (!Serial) {} // wait for serial port to connect.
   Serial.println("MRS-611 P1AM PLC module setup");
   Serial.println("Rev. 1");
   while (!P1.init()){} ; //Wait for P1 Modules to Sign on   
   P1.printModules();  //print out all currently signed-on modules to the console
   P1.getFwVersion();  //print the Base Controller's firmware version to the console
   Serial.println();   //print a blank line to look more organized

//   You can use Ethernet.init(pin) to configure the CS pin
//   Ethernet.init(10);  // Most Arduino shields
   Ethernet.init(5);   // MKR ETH shield
   Ethernet.begin(mac, ip);
   modbusTCPClient.setTimeout(500); //Adjust Response Timeout from 30 seconds to 500 ms.
   server.begin(); // start the server to begin listening

   if (!modbusTCPServer.begin()) { // start the Modbus TCP server
      Serial.println("Failed to start Modbus TCP Server!");
      while (1); //If it can't be started no need to contine, stay here forever.
   }

   modbusTCPServer.configureCoils(0x00, numCoils); //Coils
   modbusTCPServer.configureDiscreteInputs(0x00, numDiscreteInputs); //Discrete Inputs
   modbusTCPServer.configureHoldingRegisters(0x00, numHoldingRegisters); //Holding Register Words
   modbusTCPServer.configureInputRegisters(0x00, numInputRegisters); //Input Register Words

   Serial.println("Done with setup()");
}

void loop() {
// put your main code here, to run repeatedly:
   EthernetClient newClient = server.accept(); //listen for incoming clients

   if (newClient) { //newest connection is always accepted
      client = newClient;
      Serial.println("");
      Serial.print("Client Accept:"); //a new client connected
      Serial.println(client.remoteIP());
   }

   if (client.available()) { //if data is available
      modbusTCPServer.accept(client); //accept that data
      modbusTCPServer.poll(); //service any Modbus TCP requests, while client connected
      Serial.println("Modbus poll");
   }

   if (client && !client.connected()) { //Stop any clients which are disconnected
      client.stop();
      Serial.println("Client Stopped"); //the client disconnected
   }

   if (!modbusTCPClient.connected()) { // client not connected, start the Modbus TCP client
      Serial.print("Attempting to connect to Modbus TCP server at IP:");
      Serial.println(sampleserver);
      if (!modbusTCPClient.begin(sampleserver)) {
         Serial.println("Modbus TCP Client failed to connect!");
         delay(1000);
      } else {
         Serial.println("Modbus TCP Client connected");
      }
   } // client connected

// check for Sensor Go signal
   if (modbusTCPServer.coilRead(4) == 1) { // Sensor Go signal

      modbusTCPServer.coilWrite(4, 0); // reset Sensor Go signal
      modbusTCPServer.poll();

      readp1 = P1.readDiscrete(1,11);  //Read value slot 1 channel 11 home
      if (readp1 == 0)
         modbusTCPServer.coilWrite(6, 0); // reset home
      else
         modbusTCPServer.coilWrite(6, 1); // set home

      readp1 = P1.readDiscrete(1,12);  //Read value slot 1 channel 12 eot
      if (readp1 == 0)
         modbusTCPServer.coilWrite(7, 0); // reset eot
      else
         modbusTCPServer.coilWrite(7, 1); // set eot

      readp1 = P1.readDiscrete(1,13);  //Read value slot 1 channel 13 egress
      if (readp1 == 0)
         modbusTCPServer.coilWrite(8, 0); // reset egress
      else
         modbusTCPServer.coilWrite(8, 1); // set egress

      readp1 = P1.readDiscrete(1,9);  //Read value slot 1 channel 9 mech dock
      if (readp1 == 0)
         modbusTCPServer.coilWrite(9, 0); // reset mech dock
      else
         modbusTCPServer.coilWrite(9, 1); // set mech dock

      readp1 = P1.readDiscrete(1,10);  //Read value slot 1 channel 10 elec dock
      if (readp1 == 0)
         modbusTCPServer.coilWrite(10, 0); // reset elec dock
      else
         modbusTCPServer.coilWrite(10, 1); // set elec dock

      modbusTCPServer.coilWrite(5, 1); // set Sensor Data Ready
// start loop to wait until python script resets sensor_data_ready
      do {
         modbusTCPServer.poll();
      } while (modbusTCPServer.coilRead(5) == 1);
   }

// check for Press Go signal
   if (modbusTCPServer.coilRead(0) == 1) { // Press Go signal

      modbusTCPServer.coilWrite(0, 0); // reset Press Go signal
      modbusTCPServer.poll();

      pedalselect = modbusTCPServer.holdingRegisterRead(0);
      modbusTCPServer.poll();

      airflowrate = modbusTCPServer.holdingRegisterRead(1);
      modbusTCPServer.poll();

      P1.writeAnalog(airflowrate, 3, 1); //writes analog data to P1 output module airflow rate to proportional valve

      maxpresspedaltime = modbusTCPServer.holdingRegisterRead(2);
      modbusTCPServer.poll();

      maxholdpedaltime = modbusTCPServer.holdingRegisterRead(3);
      modbusTCPServer.poll();

      maxnumpresses = modbusTCPServer.holdingRegisterRead(4);
      modbusTCPServer.poll();

      strextension = modbusTCPServer.holdingRegisterRead(5);
      modbusTCPServer.poll();

      strcontraction = modbusTCPServer.holdingRegisterRead(6);
      modbusTCPServer.poll();

      numpresses = 0;
      returnerrorcode = 80;
      timetotabledown = 0;
      numsamplesloc = 0x0010;
      datapointerloc = 0x0025;
      dataloc = 0x003A;
      upreached = false;

// start loop for each pedal press
      do {

         P1.writeDiscrete(HIGH,2,pedalselect);  //Turn slot 2 channel based on pedalselect on to close solenoid for pedal press
         Serial.println("");
         Serial.print("pedal solenoid ");      Serial.print(pedalselect);      Serial.println(" started pressing");         Serial.println("");
         Serial.println("Start press pedal time");
         startTime = millis();
         timerdone = false;
         numpresses++;                                                           // increment number of presses to actual completed
         modbusTCPServer.holdingRegisterWrite(datapointerloc, dataloc);          // (HR38-HR58) stores pointers to data
         modbusTCPServer.poll();
         numsamples = 0;
         upsensor = false;
         endoftravel = false;
// start loop for each sample
         do {                                                                    // loop to collect samples from arduino
            numsamples++;                                                        // increment number of samples to actual collected
            if (modbusTCPClient.connected()) {
               if (!modbusTCPClient.holdingRegisterWrite(0xFF, 0x00, pedalselect)) { // write pedalselect value to sampling arduino
                  modbusTCPServer.poll();
                  Serial.print("Failed to write! 0xFF, 0x00, pedalselect");                  Serial.println(modbusTCPClient.lastError());
               }
               if (!modbusTCPClient.coilWrite(0xFF, 0x02, 0x01)) {				// send Start Sample write the value of 0x01, to the coil at address 0x00
                  modbusTCPServer.poll();
                  Serial.print("Failed to write coil! 0xFF, 0x02, 0x01");                  Serial.println(modbusTCPClient.lastError());
               }
// start loop to wait for Sample Ready
               do {
                  sampleready = modbusTCPClient.coilRead(0xFF, 0x03); // read Sample Ready
                  modbusTCPServer.poll();
                  if (sampleready == -1) {
                     Serial.print("Failed to read! 0xFF, 0x03");                     Serial.println(modbusTCPClient.lastError());
                  }
               } while (sampleready < 1);
// save sample data
               airpressure = modbusTCPClient.holdingRegisterRead(0xFF, 0x07);				// read air pressure
               modbusTCPServer.poll();
               modbusTCPServer.holdingRegisterWrite(dataloc, airpressure);
               modbusTCPServer.poll();
               if (airpressure == -1) {
                  Serial.print("Failed to read! 0xFF, 0x07");                  Serial.println(modbusTCPClient.lastError());
               }
               dataloc++;
               stringposition = modbusTCPClient.holdingRegisterRead(0xFF, 0x08);			// read string position
               modbusTCPServer.poll();
               modbusTCPServer.holdingRegisterWrite(dataloc, stringposition);
               modbusTCPServer.poll();
               if (stringposition == -1) {
                  Serial.print("Failed to read! 0xFF, 0x08");                  Serial.println(modbusTCPClient.lastError());
               }
               dataloc++;
               if (!modbusTCPClient.coilWrite(0xFF, 0x03, 0x00)) {        // clear Sample Ready
                  Serial.print("Failed to write coil! 0xFF, 0x03, 0x00");                  Serial.println(modbusTCPClient.lastError());
               }
            }
            elapsedTime = millis() - startTime;
            if (elapsedTime >= maxpresspedaltime) {
               returnerrorcode = 81;
               Serial.println("loop exit reason - returnerrorcode = 81 pedal press took too much time");
               Serial.println("");
            }
            if (stringposition > strextension) {
               endoftravel = true;
               Serial.println("loop exit reason - End of string extension reached");
               Serial.println("");
            }
            readp1 = P1.readDiscrete(1,1);  //Read value slot 1 channel 1 upsensor
            if (readp1 == 0)
               upsensor = false;
            else
               upsensor = true;
            if ((pedalselect==4) && (upsensor == true)) {
               endoftravel = true;
               upreached = true;
               Serial.println("loop exit reason - Up reached");
               Serial.println("");
            }
         } while ((endoftravel == false) && (returnerrorcode == 80));         // stop on position of pedal
// pedal is down
         modbusTCPServer.holdingRegisterWrite(numsamplesloc, numsamples);
         modbusTCPServer.poll();
         numsamplesloc++;
         datapointerloc++;

         P1.writeDiscrete(LOW,2,pedalselect);   //Turn slot 2 channel based on pedalselect off to open solenoid for pedal press
         Serial.print("pedal solenoid ");      Serial.print(pedalselect);      Serial.println(" stopped pressing");         Serial.println("");
         Serial.println("Start hold down pedal time");
         startTime = millis();
         timerdone = false;
         downsensor = false;
// start loop to wait for hold down pedal time
         do {
            modbusTCPServer.poll();
            elapsedTime = millis() - startTime;

            readp1 = P1.readDiscrete(1,2);  //Read value slot 1 channel 2 downsensor
            if (readp1 == 0)
               downsensor = false;
            else
               downsensor = true;

            if ((pedalselect == 5) && (downsensor == true)) {
               timetotabledown = elapsedTime; // store time that down was reached
               Serial.print("loop exit reason - downsensor == true at elapsedTime = ");               Serial.println(elapsedTime);
               Serial.println("");
            }
            if ((pedalselect == 5) && (downsensor == false)) {
               // continue to wait
            }
            if ((pedalselect == 5) && (elapsedTime >= maxholdpedaltime)) { // reaching maxholdpedaltime is an error on down pedal but an expected timeout on other pedals
               returnerrorcode = 83; // table did not reach down sensor within 25 seconds
               Serial.println("loop exit reason - returnerrorcode = 83 table did not reach down sensor within 25 seconds");
               Serial.println("");
            }
            if ((pedalselect != 5) && (elapsedTime >= maxholdpedaltime)) { // reaching maxholdpedaltime is an error on down pedal but an expected timeout on other pedals
               timerdone = true;
               Serial.println("loop exit reason - timerdone = true");
               Serial.println("");
            }
         } while ((downsensor == false) && (timerdone == false) && (returnerrorcode == 80));
// done hold down time

         Serial.println("Start delay between hold down and release pedal time");
         startTime = millis();
         timerdone = false;
// start loop to wait short delay between hold down pedal and release
         do {
            modbusTCPServer.poll();
            elapsedTime = millis() - startTime;
            if (elapsedTime >= 500) {
               timerdone = true;
               Serial.println("loop exit reason - timerdone = true");
               Serial.println("");
            }
         } while ((timerdone == false) && ((returnerrorcode == 80) || (returnerrorcode == 83)));
// done wait between end of hold down and release time
         P1.writeDiscrete(HIGH,2,pedalselect + 8);  //Turn slot 2 channel based on pedalselect on to close solenoid for pedal release
         Serial.print("pedal solenoid ");      Serial.print(pedalselect+8);      Serial.println(" started releasing");         Serial.println("");
         Serial.println("Start release pedal time");
         startTime = millis();
         timerdone = false;
         endoftravel = false;
// start loop for each sample
         do {                                                                    // loop to collect samples from arduino
            if (modbusTCPClient.connected()) {
               if (!modbusTCPClient.holdingRegisterWrite(0xFF, 0x00, pedalselect)) { // write pedalselect value to sampling arduino
                  modbusTCPServer.poll();
                  Serial.print("Failed to write! 0xFF, 0x00, pedalselect");                  Serial.println(modbusTCPClient.lastError());
               }
               if (!modbusTCPClient.coilWrite(0xFF, 0x02, 0x01)) {				// send Start Sample write the value of 0x01, to the coil at address 0x00
                  modbusTCPServer.poll();
                  Serial.print("Failed to write coil! 0xFF, 0x02, 0x01");                  Serial.println(modbusTCPClient.lastError());
               }
      // start loop to wait for Sample Ready
               do {
                  sampleready = modbusTCPClient.coilRead(0xFF, 0x03); // read Sample Ready
                  modbusTCPServer.poll();
                  if (sampleready == -1) {
                     Serial.print("Failed to read! 0xFF, 0x03");                     Serial.println(modbusTCPClient.lastError());
                  }
               } while (sampleready < 1);
      // save sample data
               stringposition = modbusTCPClient.holdingRegisterRead(0xFF, 0x08);			// read string position
               modbusTCPServer.poll();
               modbusTCPServer.holdingRegisterWrite(dataloc, stringposition);
               modbusTCPServer.poll();
               if (stringposition == -1) {
                  Serial.print("Failed to read! 0xFF, 0x08");                  Serial.println(modbusTCPClient.lastError());
               }
               if (!modbusTCPClient.coilWrite(0xFF, 0x03, 0x00)) {        // clear Sample Ready
                  Serial.print("Failed to write coil! 0xFF, 0x03, 0x00");                  Serial.println(modbusTCPClient.lastError());
               }
            }
            elapsedTime = millis() - startTime;
            if (elapsedTime >= maxpresspedaltime) {
               returnerrorcode = 84;
               Serial.println("loop exit reason - returnerrorcode = 84 pedal release took too much time");
               Serial.println("");
            }
            if (stringposition < strcontraction) {
               endoftravel = true;
               Serial.println("loop exit reason - End of string contraction reached");
               Serial.println("");
            }
         } while ((endoftravel == false) && ((returnerrorcode == 80) || (returnerrorcode == 83)));         // stop on position of pedal
// pedal is up

         P1.writeDiscrete(LOW,2,pedalselect + 8);   //Turn slot 2 channel based on pedalselect off to open solenoid for pedal release
         Serial.print("pedal solenoid ");      Serial.print(pedalselect+8);      Serial.println(" stopped releasing");         Serial.println("");
         Serial.println("Start delay between end of pedal press and next pedal press");
         startTime = millis();
         timerdone = false;
// start loop to wait short delay between end of pedal press and next pedal press
         do {
            modbusTCPServer.poll();
            elapsedTime = millis() - startTime;
            if (elapsedTime >= 500) {
               timerdone = true;
               Serial.println("loop exit reason - timerdone = true");
               Serial.println("");
            }
         } while ((timerdone == false) && ((returnerrorcode == 80) || (returnerrorcode == 83)));
// done wait between end of pedal press and next pedal press

         Serial.println("current pedal cycle is done");
         Serial.println("");
      } while ((numpresses < maxnumpresses) && (returnerrorcode == 80) && (upreached == false));

      if ((pedalselect == 4) && (upreached == false)) {
            returnerrorcode = 82; // table did not reach up sensor within 21 presses
            Serial.println("returnerrorcode = 82 table did not reach up sensor within 21 presses");
      }

      readp1 = P1.readDiscrete(1,9);  //Read value slot 1 channel 9 mechdock
      if (readp1 == 0)
         mechdock = false;
      else
         mechdock = true;
      if ((pedalselect == 1) && (mechdock == false) && (returnerrorcode == 80)) {
         returnerrorcode = 85; // table not docked
         Serial.println("returnerrorcode = 85 table not docked");
      }
      if ((pedalselect == 1) && (mechdock == true))
         Serial.println("table is docked");

      readp1 = P1.readDiscrete(1,9);  //Read value slot 1 channel 9 mechdock
      if (readp1 == 0)
         mechdock = false;
      else
         mechdock = true;
      if ((pedalselect == 2) && (mechdock == true) && (returnerrorcode == 80)) {
         returnerrorcode = 86; // table still docked
         Serial.println("returnerrorcode = 86 table still docked");
      }
      if ((pedalselect == 2) && (mechdock == false))
         Serial.println("table undocked");

      readp1 = P1.readDiscrete(1,10);  //Read value slot 1 channel 10 elecdock
      if (readp1 == 0)
         elecdock = false;
      else
         elecdock = true;
      if ((pedalselect == 3) && (elecdock == false) && (returnerrorcode == 80)) {
         returnerrorcode = 87; // table not electrically docked
         Serial.println("returnerrorcode = 87 table not electrically docked");
      }
      if ((pedalselect == 3) && (elecdock == true))
         Serial.println("table is electrically docked");

      if (returnerrorcode == 80)
         returnerrorcode = 0;

      modbusTCPServer.holdingRegisterWrite(0x0D, numpresses);
      modbusTCPServer.poll();

      modbusTCPServer.holdingRegisterWrite(0x0E, returnerrorcode);
      modbusTCPServer.poll();

      modbusTCPServer.holdingRegisterWrite(0x0F, timetotabledown);
      modbusTCPServer.poll();
      modbusTCPServer.coilWrite(1, 1); // set Press Data Ready
// start loop to wait until python script resets plc_data_ready
      do {
         modbusTCPServer.poll();
      } while (modbusTCPServer.coilRead(1) == 1);

// clear registers 0-f
// clear registers 0x10-24
// clear registers 0x25-39

//      while (1)
//         modbusTCPServer.poll();
   }
}
