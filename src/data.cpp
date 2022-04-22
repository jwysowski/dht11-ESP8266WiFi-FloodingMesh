#include "data.hpp"
#include <pgmspace.h>

const char *ssid PROGMEM = "esp_floodingmesh";
const char *password PROGMEM = "123456789";

const char *temp_msg PROGMEM = "Temperature is: ";
const char *hum_msg PROGMEM = "Humidity is: ";

namespace {
float normal_temp[] = {21.00, 21.10, 21.20, 21.30, 21.40, 21.50, 21.60, 21.70, 21.80, 21.90,
						22.00, 22.10, 22.20, 22.30, 22.40, 22.50, 22.60, 22.70, 22.80, 22.90,
						23.00};

float normal_hum[] = {41.00, 41.10, 41.20, 41.30, 41.40, 41.50, 41.60, 41.70, 41.80, 41.90,
						42.00, 42.10, 42.20, 42.30, 42.40, 42.50, 42.60, 42.70, 42.80, 42.90,
						43.00};       

int binary_search_temp(float val) {
	int min = 0;
	int max = VALUES_SIZE;
	while (min <= max) {
		int mid = min + (max - min) / 2;
		if (normal_temp[mid] == val)
			return mid;

		if (normal_temp[mid] < val)
			min = mid + 1;

		else
			max = mid - 1;
	}

	return -1;
}

int binary_search_hum(float val) {
	int min = 0;
	int max = VALUES_SIZE;
	while (min <= max) {
		int mid = min + (max - min) / 2;
		if (normal_hum[mid] == val)
			return mid;

		if (normal_hum[mid] < val)
			min = mid + 1;

		else
			max = mid - 1;
	}

	return -1;
}
}

float get_temp(int index) {
    return normal_temp[index];
}

float get_hum(int index) {
    return normal_hum[index];
}

int binary_search(bool is_temp, float val) {
	if (is_temp)
		return binary_search_temp(val);
	else
		return binary_search_hum(val);
}
