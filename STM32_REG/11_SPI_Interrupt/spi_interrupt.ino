#include <ESP32SPISlave.h>

ESP32SPISlave slave;
#define BUFFER_SIZE 32
uint8_t spi_slave_rx_buf[BUFFER_SIZE];

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Initialize SPI slave (CPOL=0, CPHA=0, VSPI pins)
    slave.setDataMode(SPI_MODE0);
    slave.begin(VSPI, 18, 19, 23, 5);

    Serial.println("SPI Slave Ready...");
}

void loop() {
    memset(spi_slave_rx_buf, 0, BUFFER_SIZE);
   // Serial.println(spi_slave_rx_buf[0]); 

    // Wait for up to 32 bytes
    slave.wait(spi_slave_rx_buf, nullptr, BUFFER_SIZE);


      int bytesReceived = slave.available();

       for (int i = 0; i < bytesReceived ; i++) 
       {
            Serial.println("data received"); 
            
       }
        // Clear FIFO
        while (slave.available()) {
            slave.pop();
        }
  

    delay(500);
}
