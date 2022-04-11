#ifndef TRENDS_HPP
#define TRENDS_HPP

#define BAUDRATE        9600

//DHT
#define DHTPIN          	2
#define DHTTYPE         	DHT11
#define OVERFLOW_LIMIT  	0x7FFFFF

//mesh
#define PORT            5555

extern const char *ssid;
extern const char *password;

extern const char *temp_msg;
extern const char *hum_msg;

#define	START_SIGN		':'
#define	END_SIGN		'\n'
#define	NODE_ID_SIZE	10
#define	DATA_SIZE		5
#define	CHECKSUM_SIZE	2
#define	MESSAGE_SIZE	21
#define ZERO			48

#define	TEMPERATURE_TYPE			'0'
#define	HUMIDITY_TYPE				'1'
#define	TEMPERATURE_NORM_TYPE		'2'
#define	TEMPERATURE_RISING_TYPE		'3'
#define	TEMPERATURE_FALLING_TYPE	'4'
#define	HUMIDITY_NORM_TYPE			'5'
#define	HUMIDITY_RISING_TYPE		'6'
#define	HUMIDITY_FALLING_TYPE		'7'

#define	CHECKSUM_MOD		0x100

#define TEMPERATURE		true
#define HUMIDITY		false

struct data_frame {
	char data_type;
	union data {
		char temperature[DATA_SIZE + 1];
		char humidity[DATA_SIZE + 1];
		char target[DATA_SIZE + 1];
	} measurement;
	char node_id[NODE_ID_SIZE + 1];
	char checksum[CHECKSUM_SIZE + 1];
};

#define VALUES_SIZE		21
        
float get_temp(int index);
float get_hum(int index);
int binary_search(bool is_temp, float val);

#endif /* TRENDS_HPP */

