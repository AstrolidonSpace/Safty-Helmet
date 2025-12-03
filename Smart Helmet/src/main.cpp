#include <TinyGPS++.h>
#include <HardwareSerial.h>

#define accPin 15
#define BUTTON_PIN 23 // GPIO pin for the button

int temp = 0;
bool collition = 0;
bool accidentReported = false;
bool buttonPressed=false;
float latitude;
float longitude;
bool cancel=false;
char Csms[73];
bool collisionDetected = false;
unsigned long collisionTime = 0;

unsigned long lastAnalogReadTime = 0;
unsigned long WaitTime = 0;

// Store the mobile number in a constant variable
const char* emergencyNumber = "+918170839799";

// GPS setup
TinyGPSPlus gps;
// Use hardware serial for GPS on RX2 (GPIO 16) and TX2 (GPIO 17)
HardwareSerial SerialGPS(1); 
HardwareSerial gsmSerial(2);  // Use UART2 for the SIMA7670 module

// Define RX and TX pins for the GPS module
static const int RXPin = 18;
static const int TXPin = 5;
static const uint32_t GPSBaud = 9600; // Common baud rate for GPS modules

// Function prototype
void sendAT(const char* cmd);
void sendSMS(const char* number, const char* message);
void waitForResponse(unsigned long timeout = 2000);
bool testConnection(int baud);
void displayInfo();
void sendsms();
void sendsos();


void setup() {
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  pinMode(2, OUTPUT);
  // Start serial communication for debugging (Serial Monitor)
  Serial.begin(115200); 
  Serial.println("ESP32 GPS Data Logger - Serial Output Only");
  Serial.println("Waiting for GPS data...");

  Serial.println("Initializing SIMA7670...");

   // Try to connect with the module at a few common baud rates
  if (!testConnection(115200)) {
    Serial.println("Failed to connect at 115200 baud. Trying 9600...");
    if (!testConnection(9600)) {
      Serial.println("Failed to connect at 9600 baud. Check wiring and power.");
      while(1); // Halt the program if no connection is established
    }
  }

  //testConnection(115200);
  //testConnection(9600);

  // Once a connection is established, set the module's baud rate to 9600 for reliable communication
  Serial.println("Connection successful. Setting fixed baud rate to 9600...");
  sendAT("AT+IPR=9600"); // Set the baud rate on the module
  sendAT("AT&W");        // Save the setting to non-volatile memory

  // Re-initialize the ESP32's serial port at 9600 baud to match the module
  gsmSerial.end();
  gsmSerial.begin(9600, SERIAL_8N1, 16, 17);
  delay(100); // Small delay to let the port stabilize

  // Send the remaining AT commands to configure the module for SMS
  sendAT("AT");              // Test communication at the new baud rate
  sendAT("AT+COPS?"); 
  sendAT("AT+CMGF=1");        // Set SMS to text mode
  sendAT("AT+CSCS=\"GSM\"");  // Set character set to GSM

  //testConnection(115200);
  //testConnection(9600);

  // Once a connection is established, set the module's baud rate to 9600 for reliable communication
  
  delay(1000); // Small delay to let the port stabilize

  // Send the remaining AT commands to configure the module for SMS
  sendAT("AT");              // Test communication at the new baud rate
  sendAT("AT+COPS?");
  sendAT("AT+CSQ");
  sendAT("AT+CMGF=1");        // Set SMS to text mode
  sendAT("AT+CSCS=\"GSM\"");  // Set character set to GSM

  // Send the SMS
  //sendSMS(emergencyNumber, "Hello from ESP32 with SIMA7670!");

  // Start serial communication with the GPS module
  // SERIAL_8N1: 8 data bits, no parity, 1 stop bit
  SerialGPS.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin); 
  while (1)
  {
    if(SerialGPS.available() > 0){
      char c = SerialGPS.read();
    // Encode the character with TinyGPS++
    if (gps.encode(c)) {
      // If a complete NMEA sentence was processed and new location data is available,
      // then display the information.
      if (gps.location.isUpdated()) {
        Serial.println("GPS Ready");
        digitalWrite(2, HIGH);
        break;
      }
    }
    }
  }
  
}


void loop() {
  //Serial.println("Dipanjan");

  if (gsmSerial.available()) {
    Serial.write(gsmSerial.read());
  }

  if (millis() - lastAnalogReadTime >= 100) {
    lastAnalogReadTime = millis();

   int analogValue = analogRead(accPin);
   analogValue = analogValue - 1870;
   analogValue = (analogValue >= 0 ? analogValue : -1 * analogValue);
   /*Serial.print("Raw Analog Value: ");
   Serial.print(analogValue);*/
   /*float voltage = (float)analogValue * (3.27 / 4095.0); // Assuming 3.3V reference and 12-bit resolution
   int g = voltage / 0.2;
   g = (g >= 0 ? g : g * -1);*/
   collition = (analogValue > 202 ? 1 : 0);
   // Serial.print("G value: ");
   //Serial.println(analogValue);
   // Serial.print("g");
   // Serial.println(collition);
   if (analogValue < 800)
   {
     if (collition && !accidentReported)
     {
       accidentReported = true;
       Serial.println("Accident occurs");
       collisionDetected = true;
       cancel = false;
       collisionTime = millis();
    }
    if(cancel){
      collisionTime = millis();
    }
    if (collisionDetected && accidentReported && (millis() - collisionTime >= 8000)) {
    accidentReported = false;
    //Serial.println("ok");
    // This loop continuously reads data from the GPS module
    // and feeds it to the TinyGPS++ library for parsing.
    while (SerialGPS.available() > 0)
    {
      // Read a character from the GPS serial port
      char c = SerialGPS.read();
      // Encode the character with TinyGPS++
      if (gps.encode(c))
      {
        // If a complete NMEA sentence was processed and new location data is available,
        // then display the information.
        if (gps.location.isUpdated())
        {
          latitude = gps.location.lat();
          longitude = gps.location.lng();
          Serial.println("According to Your gps coordinates : ");
          Serial.println(latitude);Serial.println(longitude);
          Serial.println("Nearest Police station found Kalyani Police Station");
          Serial.println("Sending SMS");
          sendsms();
          // displayInfo()
        }
      }
       }

       // Optional: Add a small delay if you want to reduce CPU usage
       // when no GPS data is available, but generally not needed for serial reading.
       delay(100);
       Serial.print(F("Latitude: "));
       // Print latitude with 6 decimal places for precision
       Serial.print(latitude, 6);
       Serial.print(F(", Longitude: "));
       // Print longitude with 6 decimal places for precision
       Serial.println(longitude, 6);
     }
  }
}
 if(!collisionDetected && cancel){
  while (SerialGPS.available() > 0)
    {
      // Read a character from the GPS serial port
      char c = SerialGPS.read();
      // Encode the character with TinyGPS++
      if (gps.encode(c))
      {
        // If a complete NMEA sentence was processed and new location data is available,
        // then display the information.
        if (gps.location.isUpdated())
        {
          latitude = gps.location.lat();
          longitude = gps.location.lng();
          Serial.println("According to Your gps coordinates : ");
          Serial.println(latitude);Serial.println(longitude);
          Serial.println("Nearest Police station found Kalyani Police Station");
          Serial.println("Sending SMS");
          sendsos();
          // displayInfo()
        }
      }
    }
   cancel = false;
 }
 // Serial.println("ok");
 if (digitalRead(BUTTON_PIN) == LOW && !buttonPressed)
 {
   // Button has just been pressed, start the timer
   long pressStartTime = millis();
   Serial.println("Button pressed. Holding for 2 seconds ");

   // Check if the button is held for 2 seconds
   while (digitalRead(BUTTON_PIN) == LOW && (millis() - pressStartTime) < 2000)
   {
     delay(50); // Small delay to debounce and avoid a tight loop
   }

   // If the button was held for 2 seconds and is now released
   if ((millis() - pressStartTime) >= 2000)
   {
     Serial.println("2-second button press detected!");
     buttonPressed = true; // Set flag to prevent re-triggering
     cancel = true;
   }
  } else if (digitalRead(BUTTON_PIN) == HIGH) {
    // Reset the flag when the button is released
    buttonPressed = false;
  }
  //Serial.print(" , Voltage: ");
  //Serial.print(voltage);
  //Serial.println(" V");
  //delay(100); 
}

// Function to send an AT command and wait for a response
void sendAT(const char* cmd) {
  gsmSerial.println(cmd);
  waitForResponse();
}

// Function to send an SMS and handle the '>' prompt
void sendSMS(const char* number, const char* message) {
  // Send the command to send an SMS
  gsmSerial.print("AT+CMGS=\"");
  gsmSerial.print(number);
  gsmSerial.println("\"");

  // Wait for the '>' prompt from the module, with a 5-second timeout
  unsigned long start = millis();
  bool promptReceived = false;
  while (millis() - start < 5000) {
    if (gsmSerial.available()) {
      char c = gsmSerial.read();
      Serial.write(c); // Echo the character to the debug serial monitor
      if (c == '>') {
        promptReceived = true;
        break; // Exit the loop once the '>' prompt is found
      }
    }
  }
  
  if (promptReceived) {
    delay(100); // Small delay to ensure the module is ready
    gsmSerial.print(message);
    gsmSerial.write(26); // Send Ctrl+Z to indicate the end of the message
    Serial.println("\nSMS content sent. Waiting for final response...");
    waitForResponse(10000); // Wait for the "OK" or "ERROR" response
  } else {
    Serial.println("Error: Timed out waiting for '>' prompt.");
  }
}

// Function to read and print all available data from the GSM module
void waitForResponse(unsigned long timeout) {
  unsigned long start = millis();
  while (millis() - start < timeout) {
    while (gsmSerial.available()) {
      Serial.write(gsmSerial.read());
    }
  }
}

// Function to test if the module responds to 'AT' at a specific baud rate
bool testConnection(int baud) {
  gsmSerial.end(); // Stop the serial port
  gsmSerial.begin(baud, SERIAL_8N1, 16, 17); // Start at the new baud rate
  Serial.print("\nTesting baud rate: ");
  Serial.println(baud);
  
  // Clear any old data from the buffer
  while(gsmSerial.available()) {
    gsmSerial.read();
  }

  for (int i = 0; i < 5; i++) { // Try sending AT multiple times
    gsmSerial.println("AT");
    unsigned long start = millis();
    while (millis() - start < 1500) { // Wait up to 1.5 seconds for a response
      if (gsmSerial.available()) {
        String response = gsmSerial.readStringUntil('\n');
        response.trim(); // Remove leading/trailing whitespace
        
        Serial.print("Received: '");
        Serial.print(response);
        Serial.println("'");
        
        // Check for "OK" anywhere in the response
        if (response.indexOf("OK") != -1) {
          Serial.println("Connection successful!");
          return true; // Connection successful
        }
      }
    }
    delay(500); // Wait between retries
  }
  
  Serial.println("Connection failed.");
  return false; // Connection failed
}

void sendsms(){
    // Use a single String variable to build the message
  String smsMessage = "Accident Detected.\n";
  smsMessage += "Latitude: " + String(latitude, 6) + "\n";
  smsMessage += "Longitude: " + String(longitude, 6);

  // The char array must be large enough to hold the complete message
  // including the null terminator ('\0'). Let's use a safe size.
  char Csms[100]; 
  
  // Convert the String to a character array
  smsMessage.toCharArray(Csms, sizeof(Csms));

  Serial.println("Preparing to send the following message:");
  Serial.println(Csms);
  sendSMS(emergencyNumber, Csms);
}

void sendsos(){
    // Use a single String variable to build the message
  String smsMessage = "SOS! Passenger is in Danger\n";
  smsMessage += "Latitude: " + String(latitude, 6) + "\n";
  smsMessage += "Longitude: " + String(longitude, 6);

  // The char array must be large enough to hold the complete message
  // including the null terminator ('\0'). Let's use a safe size.
  char Csms[100]; 
  
  // Convert the String to a character array
  smsMessage.toCharArray(Csms, sizeof(Csms));

  Serial.println("Preparing to send the following message:");
  Serial.println(Csms);
  sendSMS(emergencyNumber, Csms);
}

/**
 * @brief Displays the current GPS information (latitude, longitude, altitude, date, time)
 * to the Serial Monitor.
 */
/*void displayInfo() {
  Serial.print(F("Latitude: "));
  // Print latitude with 6 decimal places for precision
  Serial.print(gps.location.lat(), 6); 
  Serial.print(F(", Longitude: "));
  // Print longitude with 6 decimal places for precision
  Serial.println(gps.location.lng(), 6); 
  /*Serial.print(F(", Altitude: "));
  // Print altitude in meters
  Serial.print(gps.altitude.meters()); 
  Serial.print(F("m"));
  Serial.println();

  Serial.print(F("Date: "));
  // Print year, month, and day
  Serial.print(gps.date.year());
  Serial.print(F("/"));
  Serial.print(gps.date.month());
  Serial.print(F("/"));
  Serial.print(gps.date.day());

  Serial.print(F(", Time: "));
  // Print hour, minute, and second
  Serial.print(gps.time.hour());
  Serial.print(F(":"));
  Serial.print(gps.time.minute());
  Serial.print(F(":"));
  Serial.print(gps.time.second());
  Serial.println();

  Serial.print(F("Satellites: "));
  // Print number of satellites
  Serial.print(gps.satellites.value());
  Serial.print(F(", HDOP: "));
  // Print Horizontal Dilution of Precision
  Serial.print(gps.hdop.value());
  Serial.println();
  Serial.println("--------------------"); // Separator for readability
}*/