#include "main.h"                  

#define BOOT_FLAG_ADDRESS           0x08004000U
#define APP_ADDRESS                 0x08008000U
#define TIMEOUT_VALUE               SystemCoreClock/4

#define ACK     0x06U
#define NACK    0x16U

static UART_HandleTypeDef huart;    //Uart handle
static uint8_t RX_Buffer[32];       //Buffer for received messages
typedef enum
{
    ERASE = 0x43,
    WRITE = 0x31,
    CHECK = 0x51,
    JUMP  = 0xA1,
} COMMANDS;

static void Jump2App(void);
static void Boot_Init(void);
static void Transmit_ACK(UART_HandleTypeDef *huart);
static void Transmit_NACK(UART_HandleTypeDef *huart);
static uint8_t Check_Checksum(uint8_t *pBuffer, uint32_t len);
static void Erase(void);
static void Write(void);
static void Check(void);

int main(void)
{
    Clk_Update();  // Update system core clock
    Boot_Init();   // Inıt bootloader
    

    //Hookup Host and Target                           
    //First send an ACK. Host should reply with ACK   
    //If no valid ACK is received within TIMEOUT_VALUE then jump to main application          
    Transmit_ACK(&huart);
    if(HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT)
    {
        Transmit_NACK(&huart);
        Jump2App();
    }
    if(Check_Checksum(RX_Buffer, 2) != 1 || RX_Buffer[0] != ACK)
    {
        Transmit_NACK(&huart);
        Jump2App();
    }
    
	for(;;)      //  Wait for commands and execute accordingly      
	{
        while(HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT); // Wait for a command
        
        if(Check_Checksum(RX_Buffer, 2) != 1)
        {
            Transmit_NACK(&huart);
        }
        else
        {
            switch(RX_Buffer[0])
            {
                case ERASE:
                    Transmit_ACK(&huart);
                    Erase();
                    break;
                case WRITE:
                    Transmit_ACK(&huart);
                    Write();
                    break;
                case CHECK:
                    Transmit_ACK(&huart);
                    Check();
                    break;
                case JUMP:
                    Transmit_ACK(&huart);
                    Jump2App();                 //Jumps to the main application.
                    break;
                default: 
                    Transmit_NACK(&huart);
                    break;
            }
        }
	}
    
    for(;;);
	return 0;
}

static void Jump2App(void)
{
    if (((*(__IO uint32_t*)APP_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
    {
        __disable_irq();                                                 //First, disable all IRQs
        uint32_t jump_address = *(__IO uint32_t *)(APP_ADDRESS + 4);     // Get the main application start address 
        __set_MSP(*(__IO uint32_t *)APP_ADDRESS);                        //Set the main stack pointer to to the application start address
        void (*pmain_app)(void) = (void (*)(void))(jump_address);        // Create function pointer for the main application
        pmain_app();                                                     // Now jump to the main application
    }
    
}

//Initializes the bootloader for host communication. 
//Communication will be done through the UART peripheral.
static void Boot_Init(void)
{

    GPIO_InitTypeDef gpio_uart;                    //Define GPIO pins
    
    gpio_uart.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio_uart.Mode = GPIO_MODE_AF_PP;
    gpio_uart.Pull = GPIO_PULL_NONE;
    gpio_uart.Speed = GPIO_SPEED_LOW;
    gpio_uart.Alternate = GPIO_AF7_USART2;
    
    HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_Init(GPIOA, &gpio_uart);
    
    huart.Init.BaudRate = 115200;                 // Determine uart baudrate
    huart.Init.Mode = HAL_UART_MODE_TX_RX;
    huart.Init.OverSampling = HAL_UART_OVERSAMPLING_16;
    huart.Init.Parity = HAL_UART_PARITY_NONE;
    huart.Init.StopBits = HAL_UART_STOP_1;
    huart.Init.WordLength = HAL_UART_WORD8;
    huart.Instance = USART2;
    
    HAL_RCC_USART2_CLK_ENABLE();
    HAL_UART_Init(&huart);
}




static void Transmit_ACK(UART_HandleTypeDef *handle)       //Sends an ACKnowledge byte to the host.
{
    uint8_t msg[2] = {ACK, ACK};
    
    HAL_UART_Tx(handle, msg, 2);                            //UartHandle The UART handle

}


static void Transmit_NACK(UART_HandleTypeDef *handle)   //Sends an NACKnowledge byte to the host.
{
    uint8_t msg[2] = {NACK, NACK};
    
    HAL_UART_Tx(handle, msg, 2);                        //UartHandle The UART handle
}


//Validates the received message through a simple checksum.
static uint8_t Check_Checksum(uint8_t *pBuffer, uint32_t len)			//len -> The length of the message //*pBuffer -> The buffer where the message is stored.
{
    uint8_t initial = 0xFF;
    uint8_t result = 0x7F;                              // some random result value
    
    result = initial ^ *pBuffer++;
    len--;
    while(len--)
    {
        result ^= *pBuffer++;
    }
    
    result ^= 0xFF;
    
    if(result == 0x00)				//  The result of the validation. 1 = OK. 0 = FAIL
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
//  Erase flash function
static void Erase(void)
{
    Flash_EraseInitTypeDef flashEraseConfig;
    uint32_t sectorError;

        
    // Receive the number of pages to be erased (1 byte)
    // the initial sector to erase  (1 byte)
    // and the checksum             (1 byte)
    
    while(HAL_UART_Rx(&huart, RX_Buffer, 3, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
    // validate checksum
    if(Check_Checksum(RX_Buffer, 3) != 1)
    {
        Transmit_NACK(&huart);
        return;
    }
    
    if(RX_Buffer[0] == 0xFF)
    {
        // global erase: not supported
        Transmit_NACK(&huart);
    }
    else
    {
        flashEraseConfig.TypeErase = HAL_FLASH_TYPEERASE_SECTOR;            // Sector erase
        flashEraseConfig.NbSectors = RX_Buffer[0];                          // Set the number of sectors to erase
        flashEraseConfig.Sector = RX_Buffer[1];                             // Set the initial sector to erase

        HAL_Flash_Unlock();
        HAL_Flash_Erase(&flashEraseConfig, &sectorError);
        HAL_Flash_Lock();
        
        Transmit_ACK(&huart);
    }
}

// Write flash function
static void Write(void)
{
    uint8_t numBytes;
    uint32_t startingAddress = 0;
    uint8_t i;

    // Receive the starting address and checksum
    // Address = 4 bytes
    // Checksum = 1 byte

    while(HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);

    // Check checksum
    if(Check_Checksum(RX_Buffer, 5) != 1)
    {
        // invalid checksum
        Transmit_NACK(&huart);
        return;
    }
    else
    {
        Transmit_ACK(&huart);
    }
    
    startingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8) + (RX_Buffer[2] << 16) + (RX_Buffer[3] << 24);      // Set the starting address

    
    while(HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);            // Receive the number of bytes to be written
    numBytes = RX_Buffer[0];
    
    while(HAL_UART_Rx(&huart, RX_Buffer, numBytes+1, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);    // Receive the data
    
    if(Check_Checksum(RX_Buffer, 5) != 1)     // Check checksum of received data
    {
        Transmit_NACK(&huart);                        // invalid checksum
        return;
    }


    // valid checksum at this point
    // Program flash with the data
    i = 0;
    HAL_Flash_Unlock();
    while(numBytes--)
    {
        HAL_Flash_Program(FLASH_TYPEPROGRAM_BYTE, startingAddress, RX_Buffer[i]);
        startingAddress++;
        i++; 
    }
    HAL_Flash_Lock();

    
    Transmit_ACK(&huart);     // Send ACK
}

static void Check(void)     //Check flashed image
{
    uint32_t startingAddress = 0;
    uint32_t endingAddress = 0;
    uint32_t address;
    uint32_t *data;
    uint32_t crcResult;

    // Receive the starting address and checksum
    // Address = 4 bytes
    // Checksum = 1 byte
    
    while(HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
    
    
    if(Check_Checksum(RX_Buffer, 5) != 1)
    {
        Transmit_NACK(&huart);
        return;
    }
    else
    {
        Transmit_ACK(&huart);
    }
    
    startingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8) 
                    + (RX_Buffer[2] << 16) + (RX_Buffer[3] << 24);      // Set the starting address
    

    // Receive the ending address and checksum
    // Address = 4 bytes
    // Checksum = 1 byte

    while(HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);

    // Check checksum
    if(Check_Checksum(RX_Buffer, 5) != 1)
    {
        // invalid checksum
        Transmit_NACK(&huart);
        return;
    }
    else
    {
        Transmit_ACK(&huart);
    }
    
    // Set the starting address
    endingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8) 
                    + (RX_Buffer[2] << 16) + (RX_Buffer[3] << 24);
    
    HAL_RCC_CRC_CLK_ENABLE();
    data = (uint32_t *)((__IO uint32_t*) startingAddress);
    for(address = startingAddress; address < endingAddress; address += 4)
    {
        data = (uint32_t *)((__IO uint32_t*) address);
        crcResult = HAL_CRC_Accumulate(data, 1);
    }
    
    HAL_RCC_CRC_CLK_DISABLE();
    if(crcResult == 0x00)
    {
        Transmit_ACK(&huart);
    }
    else
    {
        Transmit_NACK(&huart);
    }
    
    Jump2App();
}
