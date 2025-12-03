🚨 ESP32 Vehicle Accident & SOS Reporter
An IoT-based system designed using the ESP32 to automatically detect vehicle accidents and provide a manual SOS function, sending real-time GPS coordinates via SMS to a predefined emergency contact.
✨ Features
Automatic Collision Detection: Monitors an accelerometer for sudden, high-impact changes to detect accidents.

8-Second Grace Period: Allows the user to cancel the automatic accident report via the press of a button, preventing false alarms from hard bumps.

Manual SOS Signaling: A long press (2 seconds) on the button immediately triggers an urgent SOS message, regardless of collision status.

Real-time GPS Tracking: Integrates a GPS module to embed accurate location (latitude and longitude) in every alert message.

Robust GSM Communication: Uses an AT-command-compatible GSM module (like the SIMA7670) to reliably send text message alerts.
🛠️ Components & Wiring
This project requires an ESP32 and three key external modules.

Hardware
Component                              Description
ESP32 Dev Board                        Main processor.
Accelerometer                          "Detects sudden movement and impact (e.g., ADXL345, though code uses analog read)."
GPS Module                            "Provides location data (e.g., NEO-6M)."
GSM Module                            "Sends SMS alerts (e.g., SIMA7670, SIM800L)."
Momentary                              Push Button,For SOS signaling and cancellation.

Component	ESP32 Pin	Interface	Purpose
Accelerometer Out	accPin (GPIO 15)	Analog Input	Collision detection.
SOS/Cancel Button	BUTTON_PIN (GPIO 23)	Digital Input (PULLUP)	Manual trigger/cancellation.
GPS RX	GPIO 18 (Connected to GPS TX)	HardwareSerial 1	Receive GPS data (NMEA sentences).
GPS TX	GPIO 5 (Connected to GPS RX)	HardwareSerial 1	Transmit commands (optional).
GSM RX	GPIO 16 (Connected to GSM TX)	HardwareSerial 2	Receive GSM responses.
GSM TX	GPIO 17 (Connected to GSM RX)	HardwareSerial 2	Send AT commands.
Status LED	GPIO 2	Digital Output	HIGH when GPS lock is achieved.

🚀 Operation
Automatic Accident Alert
A strong impact is detected by the accelerometer (collition = 1).

The system announces "Accident occurs" and starts an 8-second countdown.

If the button is NOT pressed within 8 seconds:

The system acquires the latest GPS coordinates.

It calls sendsms() with the message:

Accident Detected.\nLatitude: [lat]\nLongitude: [lng]

If the button is pressed within 8 seconds, the alert is canceled, and the system resets.

Manual SOS Alert
The user presses and holds the button on BUTTON_PIN for 2 seconds.

The system immediately acquires the latest GPS coordinates.

It calls sendsos() with the urgent message:

SOS! Passenger is in Danger\nLatitude: [lat]\nLongitude: [lng]
