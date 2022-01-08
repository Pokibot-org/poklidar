#include "main.h"
#include "application/lidar.h"
#include "application/serial.h"

// Hardware handlers

CORDIC_HandleTypeDef hcordic;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

I2C_HandleTypeDef hi2c1;


// Statistics counter.
// Number of times DMA lost synchronisation with lidar data packets.
volatile uint16_t statLidarSyncLost = 0;

// UART DMA buffer.
// DMA transfers are 42 bytes long (to receive a complete data packet).
// But DMA alternates between 2 locations (for maximum performances),
// thus buffer is twice the size of transfers: 42*2 = 84.
uint8_t lidar_buffer[84];

// Flag to indicate when we are in recovery mode.
// We go to recovery mode when we could not detect a valid start of packet
// at the expected position on the uart link.
// When  recovery mode is enabled, data is no longer extracted because we
// are looking for a valid start of packet to synchronize DMA transfers
// with data packets (on uart link).
volatile uint8_t lidarRecoveryMode = 0;

// If you are too dumb, this is the lidar.
Lidar lidar;
// And this is the inertial measurement unit.
IMU imu;

// UART DMA interruption.
// An interruption is generated when DMA is at the middle of the buffer,
// or at the end.
// As the buffer size is twice a packet size, an interruption is genertated
// each time a full packet is received.
void receiveLidarPacket(uint8_t* buffer) {
    uint8_t i;
    // If not in recovery mode, just extract lidar data from packet
    if (!lidarRecoveryMode) {
        // Check if the DMA is still synchronized with lidar data
        if (buffer[0] == 0xFA) {
            if (buffer[1] == 160 || buffer[1] == 160+30) {
                insertIMUData(&imu, buffer);
            }
            Position p = getEstimatedPosition(&imu, 2);
            int16_t a = buffer[1];
            a = a + p.o/6;
            while (a < 160) {
                a = a + 60;
            }
            while (a >= 160+60) {
                a = a - 60;
            }
            buffer[1] = (uint8_t)a;
            // Forward data
            HAL_UART_Transmit_DMA(&huart2, buffer, 42);
            getHLSLFCD2Packet(&lidar, buffer);
        // Else, DMA is not synchronized
        } else {
            // Stop DMA
            HAL_UART_DMAStop(&huart1);
            statLidarSyncLost++;
            // Enable recovery mode
            lidarRecoveryMode = 1;
            // Search for a start of packet in the last data we got
            for(i = 0; i < 42 && buffer[i] != 0xFA; i++);
            // Get a specific amount of bytes to perfectly synchronize
            // on the next lidar packet.
            HAL_UART_Receive_DMA(&huart1, lidar_buffer, 2*i);
        }
    // Else, recovery mode enabled
    } else {
        // Stop DMA to reset transfer size
        HAL_UART_DMAStop(&huart1);
        // Disable recovery mode for next ITR
        lidarRecoveryMode = 0;
        // Restart DMA with a standard transfer size (doubler packet size)
        HAL_UART_Receive_DMA(&huart1, lidar_buffer, 84);
    }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        receiveLidarPacket(lidar_buffer);
    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        receiveLidarPacket(lidar_buffer+42);
    }
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
}
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart) {
}


void HAL_UART_ErrorCallback (UART_HandleTypeDef * huart) {
    if (huart->Instance == USART1) {
        HAL_UART_Receive_DMA(&huart1, lidar_buffer, 84);
        print("ERROR USART1\r\n");
    } else {
        print("ERROR USART2\r\n");
    }
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef * hi2c) {
    // print16(imu.rawData.EUL_X);
    // print("\r\n");
    updateIMUOrientation(&imu);
}

//=========================================================//
// Main function for application
//=========================================================//


void run_application() {
    print("Hello\r\n");

    uint16_t i;
    uint8_t command;

    LL_CORDIC_Config(
        CORDIC,
        LL_CORDIC_FUNCTION_PHASE,
        LL_CORDIC_PRECISION_4CYCLES,
        LL_CORDIC_SCALE_0,
        LL_CORDIC_NBWRITE_1,
        LL_CORDIC_NBREAD_1,
        LL_CORDIC_INSIZE_16BITS,
        LL_CORDIC_OUTSIZE_16BITS
    );
    // LL_CORDIC_WriteData(CORDIC, 0x7FFF4000);
    // int32_t cosOutput = (int32_t)LL_CORDIC_ReadData(CORDIC);

    setupBNO055(&hi2c1);

    initLidar(&lidar, &imu);
    // Start the fucking DMA
    // Because it will obviously not start on its own
    HAL_UART_Receive_DMA(&huart1, lidar_buffer, 84);

    // Start/stop Lidar
    command = 'b'; // 'b' or 'e'
    HAL_UART_Transmit(&huart1, &command, 1, 100);
    HAL_Delay(100);


    while(1) {
        getBNO055Data(&hi2c1, &imu);
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        // uint32_t timestamp = HAL_GetTick();
        // while(HAL_GetTick()-timestamp < 1000);
        HAL_Delay(100);
    }
}
