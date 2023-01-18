/*  utilities.c for STM32 MIDI to M4000 controller.
 *  Copyright (C) 2019-2023  Daniel Jameson
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * Safe keeping:     if ((__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))) {
      ringBuffer.data[ringBuffer.writeIndex] = (uint8_t)huart2.Instance->DR;
      ringBuffer.writeIndex++;
      ringBuffer.writeIndex &= 255;
    }
*/

#include "main.h"
#define FALSE 0
#define TRUE 1

extern volatile uint8_t setChannel;
extern volatile uint8_t ignoreChannel;

void Button_Pushed(void)
{
    if (setChannel == FALSE)
    {
        /* One press to set the channel */
        setChannel = TRUE;
        /* Turn on the programming mode LED on pin 6 (active low) */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    }
    else {
        /* Two presses to ignore the channel */
        ignoreChannel = TRUE;
        setChannel = FALSE;
        /* Turn off the programming mode LED on pin 6 (active low) */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    }
}