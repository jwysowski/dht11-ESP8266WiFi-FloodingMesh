#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <FloodingMesh.h>
#include <TypeConversionFunctions.h>

#include "data.hpp"
#include "handlers.hpp"

void build_data_frame(data_frame &frame, bool is_temp, float val);
void get_message(char *msg, data_frame &frame);
uint16_t checksum(data_frame &frame);
void decode_msg(const char *msg, data_frame &frame);
bool validate(data_frame &frame);
bool received_callback(String &msg, FloodingMesh &meshInstance);

float temp_target = 0.0;
float hum_target = 0.0;
float current_temp = 21.0;
float current_hum = 41.0;
char temp_mode = TEMPERATURE_NORM_TYPE;
char hum_mode = HUMIDITY_NORM_TYPE;
char chip_id[NODE_ID_SIZE + 1];
bool start_test = false;
uint16_t message_counter = 0;
uint32_t previous_millis = 0;
uint32_t interval_millis = 100;


extern void (*mesh_receive_handlers[6])(char type, float target);
extern void (*measurement_handlers[6])();


// A custom encryption key is required when using encrypted ESP-NOW transmissions. There is always a default Kok set, but it can be replaced if desired.
// All ESP-NOW keys below must match in an encrypted connection pair for encrypted communication to be possible.
// Note that it is also possible to use Strings as key seeds instead of arrays.
uint8_t espnow_encrypted_connection_key[16] = { 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44,  // This is the key for encrypting transmissions of encrypted connections.
																						 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x32, 0x11 };
uint8_t espnow_hash_key[16] = { 0xEF, 0x44, 0x33, 0x0C, 0x33, 0x44, 0xFE, 0x44,  // This is the secret key used for HMAC during encrypted connection requests.
															0x33, 0x44, 0x33, 0xB0, 0x33, 0x44, 0x32, 0xAD };

FloodingMesh mesh = FloodingMesh(received_callback, FPSTR(password), espnow_encrypted_connection_key,
								espnow_hash_key, FPSTR(ssid), MeshTypeConversionFunctions::uint64ToString(ESP.getChipId()), true);;

void setup() {
	WiFi.persistent(false);
	Serial.begin(BAUDRATE);
	sprintf(chip_id, "%u", ESP.getChipId());
	chip_id[NODE_ID_SIZE - 2] = chip_id[0];
	chip_id[NODE_ID_SIZE - 1] = chip_id[NODE_ID_SIZE - 3];
	chip_id[NODE_ID_SIZE] = '\0';

	mesh.begin();
	mesh.activateAP();
}

void loop() {
	floodingMeshDelay(1);
	
	uint32_t current_millis = millis();
	if (start_test && current_millis - previous_millis >= interval_millis) {
		previous_millis = current_millis;

		char message[MESSAGE_SIZE] = {0};
		data_frame frame;
		build_data_frame(frame, TEMPERATURE, 10);
		get_message(message, frame);
		mesh.broadcast(String(message));

		message_counter++;
		if (message_counter == 2000)
			start_test = false;
	}
}

void get_message(char *msg, data_frame &frame) {
	msg[0] = START_SIGN;
	msg[1] = frame.data_type;
	msg[2] = '\0';
	
	if (frame.data_type == TEMPERATURE_TYPE)
		strcat(msg, frame.measurement.temperature);
	else
		strcat(msg, frame.measurement.humidity);

	strcat(msg, frame.node_id);
	
	sprintf(frame.checksum, "%02x", checksum(frame));
	strcat(msg, frame.checksum);
	msg[MESSAGE_SIZE - 2] = END_SIGN;
	msg[MESSAGE_SIZE - 1] = '\0';
}

uint16_t checksum(data_frame &frame) {
	uint16_t checksum = 0;

	checksum += START_SIGN;

	checksum += frame.data_type;
	if (frame.data_type == TEMPERATURE_TYPE)
		for (int i = 0; i < DATA_SIZE; i++)
			checksum += frame.measurement.temperature[i];

	if (frame.data_type == HUMIDITY_TYPE)
		for (int i = 0; i < DATA_SIZE; i++)
			checksum += frame.measurement.humidity[i];

	else
		for (int i = 0; i < DATA_SIZE; i++)
			checksum += frame.measurement.target[i];

	for (int i = 0; i < NODE_ID_SIZE; i++)
		checksum += frame.node_id[i];

	return checksum % CHECKSUM_MOD;
}

void build_data_frame(data_frame &frame, bool is_temp, float val) {
	if (is_temp) {
		frame.data_type = TEMPERATURE_TYPE;
		sprintf(frame.measurement.temperature, "%.2f", val);
	} else {
		frame.data_type = HUMIDITY_TYPE;
		sprintf(frame.measurement.humidity, "%.2f", val);
	}

	memcpy(frame.node_id, chip_id, NODE_ID_SIZE + 1);
}

void decode_msg(const char *msg, data_frame &frame) {
	int start = 0;
	while (start < MESSAGE_SIZE) {
		if (msg[start] == START_SIGN)
			break;

		start++;
	}

	frame.data_type = msg[start + 1];
	if (frame.data_type == TEMPERATURE_TYPE) {
		memcpy(frame.measurement.temperature, msg + start + 2, DATA_SIZE);
		frame.measurement.temperature[DATA_SIZE] = '\0';
	}
	
	if (frame.data_type == HUMIDITY_TYPE) {
		memcpy(frame.measurement.humidity, msg + start + 2, DATA_SIZE);
		frame.measurement.humidity[DATA_SIZE] = '\0';
	}

	else {
		memcpy(frame.measurement.target, msg + start + 2, DATA_SIZE);
		frame.measurement.target[DATA_SIZE] = '\0';
	}

	memcpy(frame.node_id, msg + start + 2 + DATA_SIZE, NODE_ID_SIZE);
	frame.node_id[NODE_ID_SIZE] = '\0';
	
	memcpy(frame.checksum, msg + start + 2 + DATA_SIZE + NODE_ID_SIZE, CHECKSUM_SIZE);
	frame.checksum[CHECKSUM_SIZE] = '\0';
}

bool validate(data_frame &frame) {
	uint16_t frame_checksum = strtoul(frame.checksum, nullptr, 16);
	return frame_checksum == checksum(frame);
}

bool received_callback(String &msg, FloodingMesh &meshInstance) {
	data_frame frame;
	const char *message = msg.c_str();

	decode_msg(message, frame);
	if (!validate(frame))
		return false;

	int handler_index = get_handler_index(frame.data_type);
	if (handler_index < 0)
		return true;

	if (strcmp(frame.node_id, chip_id) != 0 && strcmp(frame.node_id, "0000000000") != 0)
		return true;

	char *end_ptr = nullptr;
	float target = strtof(frame.measurement.target, &end_ptr);
	mesh_receive_handlers[handler_index](frame.data_type, target);
	
	return true;
}
