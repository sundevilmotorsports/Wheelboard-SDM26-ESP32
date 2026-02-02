#include "mcp2518fd.h"
#include "esp_log.h"
#include <string.h>

#include "esp_err.h"

static const char *TAG = "MCP2518FD";

// Register addresses
#define REG_OSC         0xE00
#define REG_C1CON       0x000
#define REG_C1NBTCFG    0x004
#define REG_C1INT       0x01C
#define REG_C1RXIF      0x020
#define REG_C1TXQCON    0x050
#define REG_C1TXQSTA    0x054
#define REG_C1TXQUA     0x058
#define REG_C1FIFOCON1  0x05C
#define REG_C1FIFOSTA1  0x060
#define REG_C1FIFOUA1   0x064
#define REG_C1FLTCON0   0x1D0

// SPI command helper
static esp_err_t spi_write_read(spi_device_handle_t spi, uint8_t *tx, size_t tx_len, uint8_t *rx, size_t rx_len) {
    spi_transaction_t t = {
        .length = (tx_len + rx_len) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_transmit(spi, &t);
}

// Read 32-bit register
static esp_err_t read_reg(spi_device_handle_t spi, uint16_t addr, uint32_t *val) {
    uint8_t tx[6] = {(0x03 << 4) | ((addr >> 8) & 0x0F), addr & 0xFF};
    uint8_t rx[6];
    esp_err_t ret = spi_write_read(spi, tx, 2, rx, 4);
    if (ret == ESP_OK) {
        *val = rx[2] | (rx[3] << 8) | (rx[4] << 16) | (rx[5] << 24);
    }
    return ret;
}

// Write 32-bit register
static esp_err_t write_reg(spi_device_handle_t spi, uint16_t addr, uint32_t val) {
    uint8_t tx[6] = {
        (0x02 << 4) | ((addr >> 8) & 0x0F),
        addr & 0xFF,
        val & 0xFF,
        (val >> 8) & 0xFF,
        (val >> 16) & 0xFF,
        (val >> 24) & 0xFF
    };
    return spi_write_read(spi, tx, 6, NULL, 0);
}

// Reset MCP2518FD
static esp_err_t reset(spi_device_handle_t spi) {
    uint8_t cmd[2] = {0x00, 0x00};
    esp_err_t ret = spi_write_read(spi, cmd, 2, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    return ret;
}

// Wait for bit in register
static esp_err_t wait_bit(spi_device_handle_t spi, uint16_t addr, uint32_t mask, uint32_t val, uint32_t timeout_ms) {
    uint32_t reg;
    while (timeout_ms--) {
        if (read_reg(spi, addr, &reg) != ESP_OK) return ESP_FAIL;
        if ((reg & mask) == val) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_ERR_TIMEOUT;
}

// Initialize CAN controller
esp_err_t mcp_init(spi_device_handle_t spi) {
    ESP_LOGI(TAG, "Initializing...");

    // Reset
    if (reset(spi) != ESP_OK) return ESP_FAIL;

    // Wait for oscillator ready
    if (wait_bit(spi, REG_OSC, 0x400, 0x400, 100) != ESP_OK) {
        ESP_LOGE(TAG, "OSC not ready");
        return ESP_FAIL;
    }

    // Set configuration mode
    write_reg(spi, REG_C1CON, 0x04000000);
    if (wait_bit(spi, REG_C1CON, 0x00E00000, 0x00800000, 100) != ESP_OK) {
        ESP_LOGE(TAG, "Config mode failed");
        return ESP_FAIL;
    }

    // Configure 500kbps @ 40MHz (20 TQ, TSEG1=15, TSEG2=4, SJW=4)
    write_reg(spi, REG_C1NBTCFG, 0x00F13F03);

    // Configure TXQ (8 messages)
    write_reg(spi, REG_C1TXQCON, 0x07000080);

    // Configure RX FIFO1 (16 messages, enable interrupt)
    write_reg(spi, REG_C1FIFOCON1, 0x0F000001);

    // Configure filter 0: accept all, point to FIFO1
    write_reg(spi, REG_C1FLTCON0, 0x80000001);

    // Enable RX/TX interrupts
    write_reg(spi, REG_C1INT, 0x00030000);

    // Set normal mode
    write_reg(spi, REG_C1CON, 0x06000000); // CAN 2.0 mode
    if (wait_bit(spi, REG_C1CON, 0x00E00000, 0x00C00000, 100) != ESP_OK) {
        ESP_LOGE(TAG, "Normal mode failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Ready");
    return ESP_OK;
}

// Send CAN message
esp_err_t mcp_send(spi_device_handle_t spi, const twai_message_t *msg) {
    // Get TXQ address
    uint32_t addr;
    if (read_reg(spi, REG_C1TXQUA, &addr) != ESP_OK) return ESP_FAIL;

    // Build message (T0 + T1)
    uint32_t t0, t1;
    if (msg->extd) {
        // Extended ID (29-bit)
        t0 = ((msg->identifier & 0x1FFC0000) >> 18) |
             ((msg->identifier & 0x3F800) << 11) |
             (msg->identifier & 0x7FF) | 0x10;
    } else {
        // Standard ID (11-bit)
        t0 = msg->identifier & 0x7FF;
    }

    t1 = msg->data_length_code & 0x0F;
    if (msg->rtr) {
        t1 |= 0x20; // Set RTR bit
    }

    // Write message header
    uint8_t tx[14] = {
        (0x02 << 4) | ((addr >> 8) & 0x0F), addr & 0xFF,
        t0 & 0xFF, (t0 >> 8) & 0xFF, (t0 >> 16) & 0xFF, (t0 >> 24) & 0xFF,
        t1 & 0xFF, (t1 >> 8) & 0xFF, (t1 >> 16) & 0xFF, (t1 >> 24) & 0xFF,
        msg->data[0], msg->data[1], msg->data[2], msg->data[3]
    };

    if (spi_write_read(spi, tx, 14, NULL, 0) != ESP_OK) return ESP_FAIL;

    // Write remaining data
    if (msg->data_length_code > 4) {
        uint8_t tx2[6] = {
            (0x02 << 4) | (((addr + 8) >> 8) & 0x0F), (addr + 8) & 0xFF,
            msg->data[4], msg->data[5], msg->data[6], msg->data[7]
        };
        if (spi_write_read(spi, tx2, 6, NULL, 0) != ESP_OK) return ESP_FAIL;
    }

    // Trigger transmission (UINC + TXREQ)
    uint32_t con;
    read_reg(spi, REG_C1TXQCON, &con);
    write_reg(spi, REG_C1TXQCON, con | 0x00000300);

    return ESP_OK;
}

// Check if message available
bool mcp_available(spi_device_handle_t spi) {
    uint32_t rxif;
    if (read_reg(spi, REG_C1RXIF, &rxif) != ESP_OK) return false;
    return (rxif & 0x02) != 0;
}

// Receive CAN message
esp_err_t mcp_receive(spi_device_handle_t spi, twai_message_t *msg) {
    if (!mcp_available(spi)) return ESP_ERR_NOT_FOUND;

    // Clear message structure
    memset(msg, 0, sizeof(twai_message_t));

    // Get FIFO1 address
    uint32_t addr;
    if (read_reg(spi, REG_C1FIFOUA1, &addr) != ESP_OK) return ESP_FAIL;

    // Read message (skip timestamp, read T0, T1, data)
    uint8_t tx[18] = {(0x03 << 4) | ((addr >> 8) & 0x0F), addr & 0xFF};
    uint8_t rx[18];
    if (spi_write_read(spi, tx, 2, rx, 16) != ESP_OK) return ESP_FAIL;

    // Parse T0 (ID)
    uint32_t t0 = rx[2] | (rx[3] << 8) | (rx[4] << 16) | (rx[5] << 24);
    msg->extd = (t0 & 0x10) != 0;
    if (msg->extd) {
        // Extended ID (29-bit)
        msg->identifier = ((t0 & 0x1FFC0000) << 18) | ((t0 & 0xF800) >> 11) | (t0 & 0x7FF);
    } else {
        // Standard ID (11-bit)
        msg->identifier = t0 & 0x7FF;
    }

    // Parse T1 (DLC and flags)
    uint32_t t1 = rx[6] | (rx[7] << 8) | (rx[8] << 16) | (rx[9] << 24);
    msg->data_length_code = t1 & 0x0F;
    if (msg->data_length_code > 8) msg->data_length_code = 8;
    msg->rtr = (t1 & 0x20) != 0;

    // Parse data
    memcpy(msg->data, &rx[10], 8);

    // Increment FIFO
    uint32_t con;
    read_reg(spi, REG_C1FIFOCON1, &con);
    write_reg(spi, REG_C1FIFOCON1, con | 0x00000100);

    return ESP_OK;
}
