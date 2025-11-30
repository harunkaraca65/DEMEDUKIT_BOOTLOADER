/*
 * Project: STM32 OTA Update Manager (ESP32 Side)
 * Description: Hosts an HTTP client to fetch binary firmware and flashes a target STM32
 * microcontroller via UART using a custom bootloader protocol.
 * Includes Version Control, NVS Storage, and CRC32 Integrity Check.
 * Author: hrnkrc
 * Date: November 2025
 */

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_http_client.h"
#include "esp_rom_crc.h" // Hardware CRC for ESP32

/* ========================================================================== */
/* CONFIGURATION                               */
/* ========================================================================== */

// --- Network Configuration ---
#define WIFI_SSID           "FiberHGW_TP9928"
#define WIFI_PASS           "K.rc??66"

// --- Server Configuration ---
#define BASE_URL            "http://192.168.1.103:8000"
#define URL_VERSION_FILE    BASE_URL "/version.txt"
#define URL_FIRMWARE_BIN    BASE_URL "/app.bin"

// --- Hardware Pin Definitions ---
#define UART_PORT_NUM       UART_NUM_2
#define UART_TX_PIN         25  // Connected to STM32 RX
#define UART_RX_PIN         26  // Connected to STM32 TX
#define LED_TX_PIN          32  // Status LED for TX activity
#define LED_RX_PIN          33  // Status LED for RX activity
#define UART_RX_BUF_SIZE    256

/* ========================================================================== */
/* PROTOCOL DEFINITIONS                            */
/* ========================================================================== */

#define CMD_INIT            0xAB // Magic byte to enter bootloader mode
#define CMD_ERASE           0x50 // Command to erase application flash area
#define CMD_WRITE           0x51 // Command to write data chunk
#define CMD_JUMP            0x52 // Command to jump to user application
#define CMD_VERIFY          0x53 // Command to verify CRC32
#define ACK_BYTE            0xC1 // Acknowledge
#define NACK_BYTE           0x7F // Not Acknowledge

static const char *TAG = "OTA_MANAGER";
bool wifi_connected_flag = false;

/* ========================================================================== */
/* PERIPHERAL DRIVERS                               */
/* ========================================================================== */

/**
 * @brief Initialize UART peripheral for communication with STM32.
 * Baudrate: 115200, 8N1.
 */
void System_InitUART(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_PORT_NUM, UART_RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    // Initialize Status LEDs
    gpio_reset_pin(LED_TX_PIN);
    gpio_set_direction(LED_TX_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_RX_PIN);
    gpio_set_direction(LED_RX_PIN, GPIO_MODE_OUTPUT);
}

/**
 * @brief Wi-Fi Event Handler callback.
 */
static void WiFi_EventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Wi-Fi disconnected, retrying...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        wifi_connected_flag = true;
        ESP_LOGI(TAG, "Wi-Fi Connected!");
    }
}

/**
 * @brief Initialize Wi-Fi in Station Mode.
 */
void System_InitWiFi(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &WiFi_EventHandler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &WiFi_EventHandler, NULL, NULL);
    
    wifi_config_t wifi_config = { 
        .sta = { 
            .ssid = WIFI_SSID, 
            .password = WIFI_PASS, 
        } 
    };
    
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

/* ========================================================================== */
/* HELPER FUNCTIONS                               */
/* ========================================================================== */

/**
 * @brief  Waits for an ACK byte (0xC1) from the STM32.
 * @param  timeout_ms Maximum time to wait in milliseconds.
 * @return true if ACK received, false on timeout.
 */
bool UART_WaitForAck(int timeout_ms) {
    uint8_t data[10];
    int len = uart_read_bytes(UART_PORT_NUM, data, 1, timeout_ms / portTICK_PERIOD_MS);
    if (len > 0 && data[0] == ACK_BYTE) return true;
    return false;
}

/**
 * @brief  Retrieves the last installed firmware version from NVS.
 * @return Version number (int32), or 0 if not found.
 */
int32_t NVS_GetAppVersion(void) {
    nvs_handle_t my_handle;
    int32_t version = 0; // Default version
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        nvs_get_i32(my_handle, "fw_ver", &version);
        nvs_close(my_handle);
    }
    return version;
}

/**
 * @brief  Saves the new firmware version to NVS.
 * @param  version The version number to save.
 */
void NVS_SetAppVersion(int32_t version) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        nvs_set_i32(my_handle, "fw_ver", version);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
}

/**
 * @brief  Connects to the server and reads the 'version.txt' file.
 * @return Server version number, or -1 on error.
 */
int32_t HTTP_GetServerVersion(void) {
    char output_buffer[16] = {0};
    int32_t server_ver = -1;
    
    esp_http_client_config_t config = { .url = URL_VERSION_FILE, };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    
    if (esp_http_client_open(client, 0) == ESP_OK) {
        esp_http_client_fetch_headers(client); // Dummy call to clear headers
        
        int data_read = esp_http_client_read(client, output_buffer, sizeof(output_buffer)-1);
        if (data_read > 0) {
            output_buffer[data_read] = '\0';
            server_ver = atoi(output_buffer); // Parse String to Int
        }
    } else {
        ESP_LOGE(TAG, "Failed to fetch version file from server!");
    }
    esp_http_client_cleanup(client);
    return server_ver;
}

/* ========================================================================== */
/* MAIN OTA TASK                                */
/* ========================================================================== */

void OTA_ManagerTask(void *pvParameters) {
    uint8_t rx_buffer[128]; // Buffer for HTTP data
    
    // Command Bytes
    const char cmd_init = CMD_INIT;
    const char cmd_erase = CMD_ERASE;
    const char cmd_write = CMD_WRITE;
    const char cmd_jump = CMD_JUMP;
    const char cmd_verify = CMD_VERIFY;

    // 1. Wait for Wi-Fi Connection
    while (!wifi_connected_flag) vTaskDelay(500 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Network Connected. Checking for updates...");

    // 2. Version Check Strategy
    int32_t current_ver = NVS_GetAppVersion();
    int32_t server_ver = HTTP_GetServerVersion();
    
    ESP_LOGI(TAG, "Device Version: %d, Server Version: %d", (int)current_ver, (int)server_ver);

    if (server_ver > current_ver) 
    {
        ESP_LOGW(TAG, ">>> NEW FIRMWARE AVAILABLE <<<");
        ESP_LOGW(TAG, "Waiting for STM32 reset to initiate handshake...");
        
        // 3. Handshake Mode (Interception)
        bool target_connected = false;
        while (!target_connected) {
            // Continuously send INIT command to catch the bootloader
            uart_write_bytes(UART_PORT_NUM, &cmd_init, 1);
            
            // Blink TX LED
            gpio_set_level(LED_TX_PIN, 1); vTaskDelay(50/portTICK_PERIOD_MS); gpio_set_level(LED_TX_PIN, 0);
            
            if (UART_WaitForAck(100)) {
                ESP_LOGI(TAG, "STM32 BOOTLOADER CONNECTED! Handshake OK.");
                target_connected = true;
                gpio_set_level(LED_RX_PIN, 1); // Solid RX LED indicates connection
            }
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        // 4. Download and Flash Process
        esp_http_client_config_t config = { .url = URL_FIRMWARE_BIN, };
        esp_http_client_handle_t client = esp_http_client_init(&config);
        
        if (esp_http_client_open(client, 0) == ESP_OK) {
            int content_length = esp_http_client_fetch_headers(client);
            ESP_LOGI(TAG, "Firmware Found. Size: %d bytes", content_length);

            /* --- STEP A: Erase Application Area --- */
            ESP_LOGW(TAG, "Erasing STM32 Application Flash...");
            uart_write_bytes(UART_PORT_NUM, &cmd_erase, 1);
            
            // Erase takes time, wait up to 10 seconds
            if (!UART_WaitForAck(10000)) { 
                ESP_LOGE(TAG, "ERASE FAILED: Timeout!"); 
                vTaskDelete(NULL); 
            }
            ESP_LOGI(TAG, "Erase Complete.");
            
            /* --- STEP B: Write Loop with CRC Calculation --- */
            int total_read = 0;
            uint32_t calculated_crc = 0xFFFFFFFF; // CRC Init Value

            while (1) {
                int data_read = esp_http_client_read(client, (char*)rx_buffer, 64);
                if (data_read <= 0) break; // End of file

                // Calculate CRC32 on the fly (Standard Ethernet/ZIP Poly)
                for (int i = 0; i < data_read; i++) {
                    calculated_crc ^= rx_buffer[i];
                    for (int j = 0; j < 8; j++) {
                        if (calculated_crc & 1) calculated_crc = (calculated_crc >> 1) ^ 0xEDB88320;
                        else calculated_crc >>= 1;
                    }
                }

                // Send Packet: [CMD] + [LEN] + [DATA]
                uart_write_bytes(UART_PORT_NUM, &cmd_write, 1);
                uint8_t len_byte = (uint8_t)data_read;
                uart_write_bytes(UART_PORT_NUM, &len_byte, 1);
                uart_write_bytes(UART_PORT_NUM, rx_buffer, data_read);

                // Wait for ACK (5 seconds timeout for safety)
                if (!UART_WaitForAck(5000)) {
                    ESP_LOGE(TAG, "WRITE FAILED: Timeout!"); break;
                }
                
                total_read += data_read;
                if(total_read % 1024 == 0) ESP_LOGI(TAG, "Progress: %d bytes...", total_read);
                
                // Toggle LED for visual feedback
                gpio_set_level(LED_RX_PIN, !gpio_get_level(LED_RX_PIN));
            }
            
            // Finalize CRC
            calculated_crc = ~calculated_crc;

            ESP_LOGI(TAG, "Download Complete! Total: %d Bytes", total_read);
            ESP_LOGI(TAG, "Calculated Host CRC32: 0x%08X", (unsigned int)calculated_crc);

            /* --- STEP C: Verification --- */
            ESP_LOGW(TAG, "Verifying Integrity on Target...");
            
            uart_write_bytes(UART_PORT_NUM, &cmd_verify, 1);
            
            // Construct Verify Packet: [CRC32 (4)] + [LENGTH (4)] (Little Endian)
            uint8_t verify_packet[8];
            verify_packet[0] = calculated_crc & 0xFF;
            verify_packet[1] = (calculated_crc >> 8) & 0xFF;
            verify_packet[2] = (calculated_crc >> 16) & 0xFF;
            verify_packet[3] = (calculated_crc >> 24) & 0xFF;
            
            verify_packet[4] = total_read & 0xFF;
            verify_packet[5] = (total_read >> 8) & 0xFF;
            verify_packet[6] = (total_read >> 16) & 0xFF;
            verify_packet[7] = (total_read >> 24) & 0xFF;
            
            uart_write_bytes(UART_PORT_NUM, verify_packet, 8);

            // Wait for verification result
            if (UART_WaitForAck(2000)) {
                ESP_LOGW(TAG, "INTEGRITY CHECK PASSED! Firmware is valid.");
                
                // Only save version and jump if verification passes
                NVS_SetAppVersion(server_ver);
                ESP_LOGW(TAG, "New Version (%d) Saved to NVS.", (int)server_ver);

                /* --- STEP D: Jump to Application --- */
                uart_write_bytes(UART_PORT_NUM, &cmd_jump, 1);
                UART_WaitForAck(1000);
                ESP_LOGW(TAG, "MISSION SUCCESSFUL. TARGET REBOOTING.");
            } else {
                ESP_LOGE(TAG, "INTEGRITY CHECK FAILED! CRC Mismatch.");
                ESP_LOGE(TAG, "Version not updated. Manual intervention required.");
            }

        }
        esp_http_client_cleanup(client);
    } 
    else 
    {
        // NO UPDATE SCENARIO
        ESP_LOGI(TAG, "System is up to date (V%d). Releasing STM32.", (int)current_ver);
        ESP_LOGI(TAG, "Target will boot application in 3 seconds.");
    }
    
    // Task is done, delete self
    vTaskDelete(NULL);
}

/**
 * @brief Application Entry Point
 */
void app_main(void) {
    // Initialize NVS (Required for Wi-Fi and Version Storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Peripherals
    System_InitUART();
    System_InitWiFi();
    
    // Start OTA Manager Task
    xTaskCreate(OTA_ManagerTask, "OTA_Task", 8192, NULL, 5, NULL);
}