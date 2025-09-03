#include <Wire.h>

#define MY_ADDR   0x68       // I2C slave address
#define LED_PIN   2          // Onboard LED pin
#define RX_BUF_LEN 32        // Receive buffer length

char rx_buffer[RX_BUF_LEN];   // Buffer to store received data

void setup() {
  Serial.begin(9600);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // LED ON initially

  // Initialize I2C as slave
  Wire.begin(MY_ADDR);

  // Set function to call when data is received
  Wire.onReceive(receiveEvent);

  Serial.println("ESP32 I2C Slave ready");
}

void loop() {
  // Nothing here; all action happens in receiveEvent
}

// Function called when data is received from I2C master
void receiveEvent(int bytes) {
  // Blink LED to indicate data reception
  digitalWrite(LED_PIN, LOW);
  delay(50);
  digitalWrite(LED_PIN, HIGH);

  int i = 0;
  while(Wire.available() && i < RX_BUF_LEN - 1) {
    rx_buffer[i++] = Wire.read();
  }
  rx_buffer[i] = '\0';  // Null-terminate string

  Serial.print("Received: ");
  Serial.println(rx_buffer);
  
}
