#ifndef HANDLERS_HPP
#define HANDLERS_HPP

int get_handler_index(char number);
void mesh_receive_temp_handler(char type, float target);
void mesh_receive_hum_handler(char type, float target);
void temp_norm_handler();
void temp_rising_handler();
void temp_falling_handler();
void hum_norm_handler();
void hum_rising_handler();
void hum_falling_handler();

extern void (*mesh_receive_handlers[6])(char type, float target);
extern void (*measurement_handlers[6])();

#endif /* HANDLERS_HPP */

// #define	TEMPERATURE_TYPE			'0'
// #define	HUMIDITY_TYPE				'1'
// #define	TEMPERATURE_NORM_TYPE		'2'
// #define	TEMPERATURE_RISING_TYPE		'3'
// #define	TEMPERATURE_FALLING_TYPE	'4'
// #define	HUMIDITY_NORM_TYPE			'5'
// #define	HUMIDITY_RISING_TYPE		'6'
// #define	HUMIDITY_FALLING_TYPE		'7'