/*
 * display.c
 *
 *  Created on: 25.08.2019
 *      Author: badi
 */
#include "stm32f4xx_hal.h"
#include "display.h"

#include "display_ssd1289.h"


disp_func_t* disp_ctor(disp_type type, disp_ctrl_t *ctrl)
{
	if (SSD1289 == type)
	{
		func_ssd1289.init(ctrl);
		return &func_ssd1289;
	}
	return NULL;
}
