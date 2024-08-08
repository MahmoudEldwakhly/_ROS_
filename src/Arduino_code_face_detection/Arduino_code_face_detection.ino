// Pin for the built-in LED
const int ledPin = LED_BUILTIN;  // On most Arduinos, this is pin 13

// Variables to store the received data
int faceDetected = 0;

void setup() {
  // Initialize the LED pin as an output
  pinMode(ledPin, OUTPUT);

  // Initialize serial communication at 9600 baud
  Serial.begin(9600);
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    // Read the incoming byte
    faceDetected = Serial.read();

    // If faceDetected is 1, blink the LED
    if (faceDetected == '1') 
    {
      // Blink the LED
      digitalWrite(ledPin, HIGH);  // Turn the LED on
    }
    else 
    {
      digitalWrite(ledPin, LOW);   // Turn the LED off
     
    }
  }
}
