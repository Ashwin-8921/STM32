#include <ESP32SPISlave.h>

ESP32SPISlave slave;
#define BUFFER_SIZE 32
uint8_t spi_slave_rx_buf[BUFFER_SIZE];

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Initialize SPI slave (CPOL=0, CPHA=0)
    slave.setDataMode(SPI_MODE0);
    slave.begin();

    Serial.println("SPI Slave Ready...");
}

void loop() {
    // Clear RX buffer before new transaction
    memset(spi_slave_rx_buf, 0, BUFFER_SIZE);

    // Wait for SPI master to send data
    slave.wait(spi_slave_rx_buf, nullptr, BUFFER_SIZE);

    // Check how many bytes were actually received
    int bytesReceived = slave.available();
    

    if (bytesReceived > 0) {
        // Read and print each received byte
        for (int i = 0; i < bytesReceived; i++) {
            uint8_t value = spi_slave_rx_buf[i];
            Serial.print("Master sent: ");
            Serial.println(value); // This will print 10 if STM32 sends 10

            // Remove byte from FIFO so next transaction is clean
            slave.pop();
        }
    }

   // delay(500); // optional delay to avoid flooding Serial
}
