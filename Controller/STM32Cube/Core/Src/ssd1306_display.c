/*
 * ssd1306_display.c
 *
 *  Created on: 29 Dec 2020
 *      Author: 61416
 */

#include "ssd1306_display.h"
#include "ssd1306.h"

void display_template(void) {
	ssd1306_Fill(Black);

	ssd1306_DrawRectangle(2, 2, 10, 60, White);
	ssd1306_DrawRectangle(110, 2, 120, 60, White);

	ssd1306_SetCursor(25, 0);
	ssd1306_WriteString("Controller", Font_7x10, White);

	ssd1306_DrawCircle(40, 30, 15, White);
	ssd1306_DrawCircle(80, 30, 15, White);

	ssd1306_DrawCircle(20, 55, 5, White);
	ssd1306_DrawCircle(100, 55, 5, White);

	ssd1306_UpdateScreen();
}

void update_left_rect(void) {

}
void update_right_rect(void) {

}
void update_left_joystick(void) {

}
void update_right_joystick(void) {

}
void update_left_button(void) {

}
void update_right_button(void) {

}

