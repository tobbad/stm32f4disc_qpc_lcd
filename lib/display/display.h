/*
 * display.h
 *
 *  Created on: 25.08.2019
 *      Author: badi
 */

#ifndef LIB_DISPLAY_DISPLAY_H_
#define LIB_DISPLAY_DISPLAY_H_
#include "lvgl.h"

typedef struct pin_t_
{
	GPIO_TypeDef *gpio;
	uint16_t pin;
} pin_t;

typedef enum
{
	PIN_IN,
	PIN_OUT,
} pin_dir_e;

typedef struct disp_ctrl_t_
{
	pin_t data[16];
	pin_t wr;
	pin_t rd;
	pin_t rs;
	pin_t cs;
	pin_t reset;
	pin_t led;
} disp_ctrl_t;

typedef enum {
	SSD1289,
} disp_type;



typedef struct disp_func_t_
{
	void (*init)(disp_ctrl_t * ctrl);
	void (*set_pixel)(int32_t x, int32_t y, lv_color_t color);
	void (*on)(void);
	void (*off)(void);
} disp_func_t;

disp_func_t* disp_ctor(disp_type type, disp_ctrl_t *ctrl);

#endif /* LIB_DISPLAY_DISPLAY_H_ */
