#include "data.hpp"
#include "handlers.hpp"

#define ASCII_OFFSET    48
#define HANDLERS_OFFSET 2

extern float temp_target;
extern float hum_target;
extern float current_temp;
extern float current_hum;

extern char temp_mode;
extern char hum_mode;

extern bool respond;

int norm_temp_values_index = 0;
int norm_hum_values_index = 0;

bool norm_temp_rising = true;
bool norm_hum_rising = true;
bool not_norm_temp_mode = false;
bool not_norm_hum_mode = false;
bool temp_decimal_places_set = false;
bool hum_decimal_places_set = false;

void (*mesh_receive_handlers[6])(char type, float target) = {mesh_receive_temp_handler, mesh_receive_temp_handler, mesh_receive_temp_handler,
                                                                mesh_receive_hum_handler, mesh_receive_hum_handler, mesh_receive_hum_handler};

void (*measurement_handlers[6])() = {temp_norm_handler, temp_rising_handler, temp_falling_handler,
                                        hum_norm_handler, hum_rising_handler, hum_falling_handler};

namespace {
float set_decimal_places(float val) {
    int x = val * 100;
    x /= 10;
    return (float)x / 10;
}

int get_norm_hum_index() {
    float max_norm_hum = get_hum(VALUES_SIZE);
    float min_norm_hum = get_hum(0);

    if (current_hum == max_norm_hum)
        return VALUES_SIZE;

    if (current_hum == min_norm_hum)
        return 0;

    if (current_hum + 0.1 < min_norm_hum) {
        current_hum += 0.1;
        return -2;
    }

    if (current_hum - 0.1 > max_norm_hum) {
        current_hum -= 0.1;
        return -2;
    }

    return binary_search(HUMIDITY, current_hum);
}

int get_norm_temp_index() {
    float max_norm_temp = get_temp(VALUES_SIZE);
    float min_norm_temp = get_temp(0);

    if (current_temp == max_norm_temp)
        return VALUES_SIZE;

    if (current_temp == min_norm_temp)
        return 0;

    if (current_temp - 0.1 > max_norm_temp) {
        current_temp -= 0.1;
        return -2;
    }

    if (current_temp + 0.1 < min_norm_temp) {
        current_temp += 0.1;
        return -2;
    }

    return binary_search(TEMPERATURE, current_temp);    
}
}

int get_handler_index(char number) {
    return number - ASCII_OFFSET - HANDLERS_OFFSET;
}

void mesh_receive_temp_handler(char type, float target) {
    respond = true;
    temp_mode = type;
    temp_target = target;
}

void mesh_receive_hum_handler(char type, float target) {
    hum_mode = type;
    hum_target = target;
}

void temp_norm_handler() {
    if (not_norm_temp_mode) {
        if (!temp_decimal_places_set) {
            current_temp = set_decimal_places(current_temp);
            temp_decimal_places_set = true;
        }

        int index = get_norm_temp_index();
        if (index > -1)
            norm_temp_values_index = index;

        else
            return;
    }

    not_norm_temp_mode = false;
    if (norm_temp_values_index + 1 == VALUES_SIZE && norm_temp_rising)
        norm_temp_rising = false;

    else if (norm_temp_values_index == 0 && !norm_temp_rising)
        norm_temp_rising = true;

    norm_temp_values_index = norm_temp_rising ? norm_temp_values_index + 1 : norm_temp_values_index - 1;
    current_temp = get_temp(norm_temp_values_index);
}

void temp_rising_handler() {
    not_norm_temp_mode = true;
    temp_decimal_places_set = false;
    current_temp = current_temp + 0.1 < temp_target ? current_temp + 0.1 : current_temp;
}

void temp_falling_handler() {
    not_norm_temp_mode = true;
    temp_decimal_places_set = false;
    current_temp = current_temp - 0.1 > temp_target ? current_temp - 0.1 : current_temp;
}

void hum_norm_handler() {
    if (not_norm_hum_mode) {
        if (!hum_decimal_places_set) {
            current_hum = set_decimal_places(current_hum);
            hum_decimal_places_set = true;
        }

        int index = get_norm_hum_index();
        if (index > -1)        
            norm_hum_values_index = index;

        else
            return;
    }

    not_norm_hum_mode = false;
    if (norm_hum_values_index + 1 == VALUES_SIZE && norm_hum_rising)
        norm_hum_rising = false;

    else if (norm_hum_values_index == 0 && !norm_hum_rising)
        norm_hum_rising = true;

    norm_hum_values_index = norm_hum_rising ? norm_hum_values_index + 1 : norm_hum_values_index - 1;
    current_hum = get_hum(norm_hum_values_index);
}

void hum_rising_handler() {
    not_norm_hum_mode = true;
    hum_decimal_places_set = false;
    current_hum = current_hum + 0.1 < hum_target ? current_hum + 0.1 : current_hum;
}

void hum_falling_handler() {
    not_norm_hum_mode = true;
    hum_decimal_places_set = false;
    current_hum = current_hum - 0.1 > hum_target ? current_hum - 0.1 : current_hum;
}

