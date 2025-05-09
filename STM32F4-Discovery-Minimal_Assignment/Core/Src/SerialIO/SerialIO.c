/*Header Files*/
#include "SerialIO.h"


/*Global Variables------------------------------------------------------*/
UART_HandleTypeDef serialIO;

/**
  * @brief Initialise UART6 (PC6/7 TX/RX) for RS232 on Expansion Board
  * @param None
  * @retval None
  */
void init_serial_io()
{
    /*Local Variables---------------------------------------------------*/
    GPIO_InitTypeDef gpioInit;


    /*Initialise Clocks-------------------------------------------------*/
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART6_CLK_ENABLE();


    /* Configure Pin Functionality--------------------------------------*/
    gpioInit.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FAST;
    gpioInit.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &gpioInit);


    /* Configure UART Functionality-------------------------------------*/
    serialIO.Instance = USART6;
    serialIO.Init.BaudRate = 115200;
    serialIO.Init.WordLength = UART_WORDLENGTH_8B;
    serialIO.Init.StopBits = UART_STOPBITS_1;
    serialIO.Init.Parity = UART_PARITY_NONE;
    serialIO.Init.Mode = UART_MODE_TX_RX;
    serialIO.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    serialIO.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&serialIO);
}


/**
  * @brief  Serial Character Input
  * @param  void
  * @retval int - unicode character
  */
int __io_getchar(void)
{
    /* Local Variable---------------------------------------------------*/
    int ch = 0;

    /* Clear the Overrun flag just before receiving the first character */
    __HAL_UART_CLEAR_OREFLAG(&serialIO);

    /* Wait for reception of a character on the USART RX line and echo this
    * character on console */
    HAL_UART_Receive(&serialIO, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

    if(ch == '\r')
    {
        HAL_UART_Transmit(&serialIO, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
        ch = '\n';
    }

    HAL_UART_Transmit(&serialIO, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

    return ch;
}

/**
  * @brief  Serial Character Output
  * @param  integer
  * @retval int - unicode character
  */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&serialIO, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

    return ch;
}
