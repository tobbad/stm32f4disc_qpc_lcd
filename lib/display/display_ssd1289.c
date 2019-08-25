/*
 * disp_ssd1289.c
 *
 *  Created on: 25.08.2019
 *      Author: badi
 */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_gpio.h"

#include "display.h"

static disp_ctrl_t ctrl;

static const uint32_t pinnr2mask[] =
{
	LL_GPIO_PIN_0  ,
	LL_GPIO_PIN_1  ,
	LL_GPIO_PIN_2  ,
	LL_GPIO_PIN_3  ,
	LL_GPIO_PIN_4  ,
	LL_GPIO_PIN_5  ,
	LL_GPIO_PIN_6  ,
	LL_GPIO_PIN_7  ,
	LL_GPIO_PIN_8  ,
	LL_GPIO_PIN_9  ,
	LL_GPIO_PIN_10 ,
	LL_GPIO_PIN_11 ,
	LL_GPIO_PIN_12 ,
	LL_GPIO_PIN_13 ,
	LL_GPIO_PIN_14 ,
	LL_GPIO_PIN_15 ,
};

void setUpPin(pin_dir_e dir, pin_t *pin)
{
	LL_GPIO_InitTypeDef init = {0};
	if (PIN_IN == dir)
	{
		init.Pin = pinnr2mask[pin->pin];
		init.Mode = LL_GPIO_MODE_INPUT;
		init.Pull = LL_GPIO_PULL_UP;
		LL_GPIO_Init(pin->gpio, &init);
	}
	else if (PIN_OUT == dir)
	{
		init.Pin = pinnr2mask[pin->pin];
		init.Mode = LL_GPIO_MODE_OUTPUT;
		init.Speed = LL_GPIO_SPEED_FREQ_LOW;
		init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		init.Pull = LL_GPIO_PULL_NO;
		LL_GPIO_Init(pin->gpio, &init);
	}
}

void setPinState(pin_t *pin, uint8_t value)
{
	if (1 == value)
	{
		LL_GPIO_SetOutputPin(pin->gpio, pinnr2mask[pin->pin]);
	}
	else if (0 == value)
	{
		LL_GPIO_ResetOutputPin(pin->gpio, pinnr2mask[pin->pin]);
	}
}

void init(disp_ctrl_t *ctrl_in)
{
	uint8_t i;
	memcpy(&ctrl, ctrl_in, sizeof(disp_ctrl_t));
	for (i=0; i<sizeof(ctrl.data);i++)
	{
		setUpPin(PIN_OUT, &ctrl.data[i]);
	}
	setUpPin(PIN_OUT, &ctrl.wr);
	setUpPin(PIN_OUT, &ctrl.rd);
	setUpPin(PIN_OUT, &ctrl.rs);
	setUpPin(PIN_OUT, &ctrl.cs);
	setUpPin(PIN_OUT, &ctrl.reset);
	setUpPin(PIN_OUT, &ctrl.led);
}

static void set_pixel(int32_t x, int32_t y, lv_color_t color)
{
	if (NULL != ctrl.cs.gpio)
	{

	}
}

static void on(void)
{
	setPinState(&ctrl.led, 0);
}

static void off(void)
{
	setPinState(&ctrl.led, 1);
}

disp_func_t func_ssd1289 =
{
		.init = init,
		.set_pixel = set_pixel,
		.on = on,
		.off = off
};
