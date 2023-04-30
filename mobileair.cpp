#include <WString.h>
#include <pgmspace.h>

#define SOFTWARE_VERSION_STR "MobileAir-Proto-V1-042823"
#define SOFTWARE_VERSION_STR_SHORT "V1-042823"
String SOFTWARE_VERSION(SOFTWARE_VERSION_STR);
String SOFTWARE_VERSION_SHORT(SOFTWARE_VERSION_STR_SHORT);

#include <Arduino.h>
#include "ccs811.h"
#include <FastLED.h>
#include <TinyGPSPlus.h>

#include "./Fonts/oledfont.h"
#include <SSD1306Wire.h>

// includes ESP32 libraries
#define FORMAT_SPIFFS_IF_FAILED true
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <HTTPClient.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <HardwareSerial.h>

#include <WebServer.h>
#include <ESPmDNS.h>
#include <MD5Builder.h>

#define ARDUINOJSON_ENABLE_ARDUINO_STREAM 0
#define ARDUINOJSON_ENABLE_ARDUINO_PRINT 0
#define ARDUINOJSON_DECODE_UNICODE 0
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <StreamString.h>
#include "./bmx280_i2c.h"
#include "./cairsens.h"

// includes files
#include "./intl.h"
#include "./utils.h"
#include "./defines.h"
#include "./ext_def.h"
#include "./html-content.h"

/*****************************************************************
 * CONFIGURATION                                          *
 *****************************************************************/
namespace cfg
{
	unsigned debug = DEBUG;

	unsigned time_for_wifi_config = TIME_FOR_WIFI_CONFIG;
	unsigned sending_intervall_ms = SENDING_INTERVALL_MS;

	char current_lang[3];

	// credentials of the sensor in access point mode
	char fs_ssid[LEN_FS_SSID] = FS_SSID;
	char fs_pwd[LEN_CFG_PASSWORD] = FS_PWD;

	bool wifi_permanent = WIFI_PERMANENT;

	// (in)active sensors
	bool npm_read = NPM_READ;
	bool bmx280_read = BMX280_READ;
	bool ccs811_read = CCS811_READ;
	bool enveano2_read = ENVEANO2_READ;

	// Location
	bool has_gps = HAS_GPS;

	// send to "APIs"
	bool send2csv = SEND2CSV;
	bool has_sdcard = HAS_SDCARD;

	// (in)active displays
	bool has_ssd1306 = HAS_SSD1306;
	bool has_led_value = HAS_LED_VALUE;
	unsigned brightness = BRIGHTNESS;
	unsigned value_displayed = VALUE_DISPLAYED;
	bool display_measure = DISPLAY_MEASURE;
	bool display_device_info = DISPLAY_DEVICE_INFO;

	// First load
	void initNonTrivials(const char *id)
	{
		if (!*fs_ssid)
		{
			strcpy(fs_ssid, SSID_BASENAME);
			strcat(fs_ssid, id);
		}
	}
}

// define size of the config JSON
#define JSON_BUFFER_SIZE 2300
// define size of the AtmoSud Forecast API JSON
#define JSON_BUFFER_SIZE2 200

// test variables
long int sample_count = 0;
unsigned long count_recorded = 0;
bool file_created;
bool bmx280_init_failed = false;
bool ccs811_init_failed = false;
bool mobileair_selftest_failed = false;

WebServer server(80);

// include JSON config reader
#include "./mobileair-cfg.h"

/*****************************************************************
 * Display definitions                                           *
 *****************************************************************/

SSD1306Wire *oled_ssd1306 = nullptr; // as pointer

//LED declarations

CRGB colorLED_value;

//For monoLED

CRGB colorLED_empty = CRGB(0, 0, 0);
CRGB colorLED_wifi = CRGB(0, 0, 255);
CRGB colorLED_lora = CRGB(255, 255, 0);
CRGB colorLED_nbiot = CRGB(0, 255, 0);
CRGB colorLED_start = CRGB(255, 255, 255);
CRGB colorLED_red = CRGB(255, 0, 0);
CRGB colorLED_orange = CRGB(255, 128, 0);
CRGB colorLED_yellow = CRGB(255, 255, 0);
CRGB colorLED_green = CRGB(0, 255, 0);

CRGB leds[LEDS_NB]; //DOIT ETRE UNE CONSTANTE => PAS CONFIGURABLE

struct RGB
{
	byte R;
	byte G;
	byte B;
};

struct RGB displayColor_value
{
	0, 0, 0
};

struct RGB displayColor_connect
{
	0, 0, 0
};

struct RGB displayColor_WiFi
{
	0, 0, 0
};

extern const uint8_t gamma8[]; //for gamma correction

bool gamma_correction = GAMMA;

// REVOIR LES INTERPOLATE

struct RGB interpolateint(float valueSensor, int step1, int step2, int step3, bool correction)
{

	struct RGB result;
	uint16_t rgb565;

	if (valueSensor == 0)
	{

		result.R = 0;
		result.G = 255; // VERT
		result.B = 0;
	}
	else if (valueSensor > 0 && valueSensor < step1)
	{

		result.R = 0;
		result.G = 255; // VERT
		result.B = 0;
	}
	else if (valueSensor >= step1 && valueSensor < step2)
	{
		result.R = 255;
		result.G = 255; // jaune
		result.B = 0;
	}
	else if (valueSensor >= step2 && valueSensor < step3)
	{
		result.R = 255;
		result.G = 140; // orange
		result.B = 0;
	}
	else if (valueSensor >= step3)
	{

		result.R = 255;
		result.G = 0; // ROUGE
		result.B = 0;
	}
	else
	{
		result.R = 0;
		result.G = 0;
		result.B = 0;
	}

	if (correction == true)
	{
		result.R = pgm_read_byte(&gamma8[result.R]);
		result.G = pgm_read_byte(&gamma8[result.G]);
		result.B = pgm_read_byte(&gamma8[result.B]);
	}

	rgb565 = ((result.R & 0b11111000) << 8) | ((result.G & 0b11111100) << 3) | (result.B >> 3);
	return result;
}

struct RGB interpolateindice(int valueIndice, bool correction)
{

	struct RGB result;
	uint16_t rgb565;

	switch (valueIndice)
	{
	case 1:
		result.R = 80;
		result.G = 240; //blue
		result.B = 230;
		break;
	case 2:
		result.R = 80;
		result.G = 204; //green
		result.B = 170;
		break;
	case 3:
		result.R = 237;
		result.G = 230; //yellow
		result.B = 97;
		break;
	case 4:
		result.R = 237;
		result.G = 94; //orange
		result.B = 88;
		break;
	case 5:
		result.R = 136;
		result.G = 26; //red
		result.B = 51;
		break;
	case 6:
		result.R = 115;
		result.G = 40; //violet
		result.B = 125;
		break;
	default:
		result.R = 0;
		result.G = 0;
		result.B = 0;
	}

	if (correction == true)
	{
		result.R = pgm_read_byte(&gamma8[result.R]);
		result.G = pgm_read_byte(&gamma8[result.G]);
		result.B = pgm_read_byte(&gamma8[result.B]);
	}

	rgb565 = ((result.R & 0b11111000) << 8) | ((result.G & 0b11111100) << 3) | (result.B >> 3);
	//Debug.println(rgb565); // to get list of color if drawGradient is acitvated
	return result;
}

struct RGB colorPM(int valueSensor, int step1, int step2, int step3, int step4, int step5, bool correction)
{
	struct RGB result;
	uint16_t rgb565;

	if (valueSensor == 0)
	{
		result.R = 80;
		result.G = 240; //blue
		result.B = 230;
	}
	else if (valueSensor > 0 && valueSensor <= step5)
	{
		if (valueSensor <= step1)
		{
			result.R = 80;
			result.G = 240; //blue
			result.B = 230;
		}
		else if (valueSensor > step1 && valueSensor <= step2)
		{
			result.R = 80;
			result.G = 204; //green
			result.B = 170;
		}
		else if (valueSensor > step2 && valueSensor <= step3)
		{
			result.R = 237;
			result.G = 230; //yellow
			result.B = 97;
		}
		else if (valueSensor > step3 && valueSensor <= step4)
		{
			result.R = 237;
			result.G = 94; //orange
			result.B = 88;
		}
		else if (valueSensor > step4 && valueSensor <= step5)
		{
			result.R = 136;
			result.G = 26; //red
			result.B = 51;
		}
	}
	else if (valueSensor > step5)
	{
		result.R = 115;
		result.G = 40; //violet
		result.B = 125;
	}
	else
	{
		result.R = 0;
		result.G = 0;
		result.B = 0;
	}

	//Gamma Correction

	if (correction == true)
	{
		result.R = pgm_read_byte(&gamma8[result.R]);
		result.G = pgm_read_byte(&gamma8[result.G]);
		result.B = pgm_read_byte(&gamma8[result.B]);
	}

	rgb565 = ((result.R & 0b11111000) << 8) | ((result.G & 0b11111100) << 3) | (result.B >> 3);
	//Debug.println(rgb565); // to get list of color if drawGradient is acitvated
	return result;
}

struct RGB interpolatePM(float valueSensor, int step1, int step2, int step3, int step4, int step5, bool correction)
{

	byte endColorValueR;
	byte startColorValueR;
	byte endColorValueG;
	byte startColorValueG;
	byte endColorValueB;
	byte startColorValueB;

	int valueLimitHigh;
	int valueLimitLow;
	struct RGB result;
	uint16_t rgb565;

	if (valueSensor == 0)
	{

		result.R = 80;
		result.G = 240; //blue
		result.B = 230;
	}
	else if (valueSensor > 0 && valueSensor <= step5)
	{
		if (valueSensor <= step1)
		{
			valueLimitHigh = step1;
			valueLimitLow = 0;
			endColorValueR = 80;
			startColorValueR = 80; //blue to green
			endColorValueG = 204;
			startColorValueG = 240;
			endColorValueB = 170;
			startColorValueB = 230;
		}
		else if (valueSensor > step1 && valueSensor <= step2)
		{
			valueLimitHigh = step2;
			valueLimitLow = step1;
			endColorValueR = 237;
			startColorValueR = 80;
			endColorValueG = 230; //green to yellow
			startColorValueG = 204;
			endColorValueB = 97;
			startColorValueB = 170;
		}
		else if (valueSensor > step2 && valueSensor <= step3)
		{
			valueLimitHigh = step3;
			valueLimitLow = step2;
			endColorValueR = 237;
			startColorValueR = 237;
			endColorValueG = 94; //yellow to orange
			startColorValueG = 230;
			endColorValueB = 88;
			startColorValueB = 97;
		}
		else if (valueSensor > step3 && valueSensor <= step4)
		{

			valueLimitHigh = step4;
			valueLimitLow = step3;
			endColorValueR = 136;
			startColorValueR = 237;
			endColorValueG = 26; // orange to red
			startColorValueG = 94;
			endColorValueB = 51;
			startColorValueB = 88;
		}
		else if (valueSensor > step4 && valueSensor <= step5)
		{
			valueLimitHigh = step5;
			valueLimitLow = step4;
			endColorValueR = 115;
			startColorValueR = 136;
			endColorValueG = 40; // red to violet
			startColorValueG = 26;
			endColorValueB = 125;
			startColorValueB = 51;
		}

		result.R = (byte)(((endColorValueR - startColorValueR) * ((valueSensor - valueLimitLow) / (valueLimitHigh - valueLimitLow))) + startColorValueR);
		result.G = (byte)(((endColorValueG - startColorValueG) * ((valueSensor - valueLimitLow) / (valueLimitHigh - valueLimitLow))) + startColorValueG);
		result.B = (byte)(((endColorValueB - startColorValueB) * ((valueSensor - valueLimitLow) / (valueLimitHigh - valueLimitLow))) + startColorValueB);
	}
	else if (valueSensor > step5)
	{
		result.R = 115;
		result.G = 40; //violet
		result.B = 125;
	}
	else
	{
		result.R = 0;
		result.G = 0;
		result.B = 0;
	}

	//Gamma Correction

	if (correction == true)
	{
		result.R = pgm_read_byte(&gamma8[result.R]);
		result.G = pgm_read_byte(&gamma8[result.G]);
		result.B = pgm_read_byte(&gamma8[result.B]);
	}

	rgb565 = ((result.R & 0b11111000) << 8) | ((result.G & 0b11111100) << 3) | (result.B >> 3);
	//Debug.println(rgb565); // to get list of color if drawGradient is acitvated
	return result;
}

struct RGB interpolateCOV(float valueSensor, int step1, int step2, bool correction)
{

	struct RGB result;
	uint16_t rgb565;

	if (valueSensor == 0)
	{
		result.R = 0;
		result.G = 255; // Green entre 0 et 800
		result.B = 0;
	}
	else if (valueSensor > 0 && valueSensor < step1)
	{

		result.R = 0;
		result.G = 255; // Green entre 0 et 800
		result.B = 0;
	}
	else if (valueSensor >= step1 && valueSensor < step2)
	{
		result.R = 255;
		result.G = 140; // Orange entre 800 et 1500
		result.B = 0;
	}
	else if (valueSensor >= step2)
	{
		result.R = 255;
		result.G = 0; // Rouge supérieur à 1500
		result.B = 0;
	}
	else
	{
		result.R = 0;
		result.G = 0;
		result.B = 0;
	}

	if (correction == true)
	{
		result.R = pgm_read_byte(&gamma8[result.R]);
		result.G = pgm_read_byte(&gamma8[result.G]);
		result.B = pgm_read_byte(&gamma8[result.B]);
	}

	rgb565 = ((result.R & 0b11111000) << 8) | ((result.G & 0b11111100) << 3) | (result.B >> 3);
	//Debug.println(rgb565); // to get list of color if drawGradient is acitvated
	return result;
}

struct RGB interpolateHumi(float valueSensor, int step1, int step2, bool correction) // Humi
{

	struct RGB result;
	uint16_t rgb565;

	if (valueSensor == 0)
	{
		result.R = 255;
		result.G = 0; // red
		result.B = 0;
	}
	else if (valueSensor > 0 && valueSensor < step1)
	{
		result.R = 255;
		result.G = 0; // red
		result.B = 0;
	}
	else if (valueSensor >= step1 && valueSensor < step2)
	{
		result.R = 0;
		result.G = 255; // green
		result.B = 0;
	}
	else if (valueSensor > step2)
	{
		result.R = 255;
		result.G = 0; // red
		result.B = 0;
	}
	else
	{
		result.R = 0;
		result.G = 0;
		result.B = 0;
	}

	if (correction == true)
	{
		result.R = pgm_read_byte(&gamma8[result.R]);
		result.G = pgm_read_byte(&gamma8[result.G]);
		result.B = pgm_read_byte(&gamma8[result.B]);
	}

	rgb565 = ((result.R & 0b11111000) << 8) | ((result.G & 0b11111100) << 3) | (result.B >> 3);
	//Debug.println(rgb565); // to get list of color if drawGradient is acitvated
	return result;
}

struct RGB interpolatePress(float valueSensor, int step1, int step2, bool correction) // Humi
{

	struct RGB result;
	uint16_t rgb565;

	if (valueSensor == 0)
	{
		result.R = 255;
		result.G = 0; // red
		result.B = 0;
	}
	else if (valueSensor > 0 && valueSensor < step1)
	{
		result.R = 255;
		result.G = 0; // red
		result.B = 0;
	}
	else if (valueSensor >= step1 && valueSensor < step2)
	{
		result.R = 0;
		result.G = 255; // green
		result.B = 0;
	}
	else if (valueSensor > step2)
	{
		result.R = 255;
		result.G = 0; // red
		result.B = 0;
	}
	else
	{
		result.R = 0;
		result.G = 0;
		result.B = 0;
	}

	if (correction == true)
	{
		result.R = pgm_read_byte(&gamma8[result.R]);
		result.G = pgm_read_byte(&gamma8[result.G]);
		result.B = pgm_read_byte(&gamma8[result.B]);
	}

	rgb565 = ((result.R & 0b11111000) << 8) | ((result.G & 0b11111100) << 3) | (result.B >> 3);
	//Debug.println(rgb565); // to get list of color if drawGradient is acitvated
	return result;
}

struct RGB interpolateTemp(float valueSensor, int step1, int step2, bool correction) // temp
{

	struct RGB result;
	uint16_t rgb565;

	if (valueSensor >= -128 && valueSensor < step1)
	{
		result.R = 0;
		result.G = 0; // Bleu / Trop froid inférieur à 19 (step1)
		result.B = 255;
	}
	else if (valueSensor >= step1 && valueSensor < step2)
	{
		result.R = 0;
		result.G = 255; // Green ok
		result.B = 0;
	}
	else if (valueSensor >= step2)
	{
		result.R = 255;
		result.G = 0; // RED / trop chaud supérieur à 28
		result.B = 0;
	}
	else
	{
		result.R = 0;
		result.G = 0;
		result.B = 0;
	}

	if (correction == true)
	{
		result.R = pgm_read_byte(&gamma8[result.R]);
		result.G = pgm_read_byte(&gamma8[result.G]);
		result.B = pgm_read_byte(&gamma8[result.B]);
	}

	rgb565 = ((result.R & 0b11111000) << 8) | ((result.G & 0b11111100) << 3) | (result.B >> 3);
	//Debug.println(rgb565); // to get list of color if drawGradient is acitvated
	return result;
}

struct RGB interpolateNO2(float valueSensor, int step1, int step2, int step3, int step4, int step5, bool correction)
{

	byte endColorValueR;
	byte startColorValueR;
	byte endColorValueG;
	byte startColorValueG;
	byte endColorValueB;
	byte startColorValueB;

	int valueLimitHigh;
	int valueLimitLow;
	struct RGB result;
	uint16_t rgb565;

	if (valueSensor == 0)
	{

		result.R = 80;
		result.G = 240; //blue
		result.B = 230;
	}
	else if (valueSensor > 0 && valueSensor <= step5)
	{
		if (valueSensor <= step1)
		{
			valueLimitHigh = step1;
			valueLimitLow = 0;
			endColorValueR = 80;
			startColorValueR = 80; //blue to green
			endColorValueG = 204;
			startColorValueG = 240;
			endColorValueB = 170;
			startColorValueB = 230;
		}
		else if (valueSensor > step1 && valueSensor <= step2)
		{
			valueLimitHigh = step2;
			valueLimitLow = step1;
			endColorValueR = 237;
			startColorValueR = 80;
			endColorValueG = 230; //green to yellow
			startColorValueG = 204;
			endColorValueB = 97;
			startColorValueB = 170;
		}
		else if (valueSensor > step2 && valueSensor <= step3)
		{
			valueLimitHigh = step3;
			valueLimitLow = step2;
			endColorValueR = 237;
			startColorValueR = 237;
			endColorValueG = 94; //yellow to orange
			startColorValueG = 230;
			endColorValueB = 88;
			startColorValueB = 97;
		}
		else if (valueSensor > step3 && valueSensor <= step4)
		{

			valueLimitHigh = step4;
			valueLimitLow = step3;
			endColorValueR = 136;
			startColorValueR = 237;
			endColorValueG = 26; // orange to red
			startColorValueG = 94;
			endColorValueB = 51;
			startColorValueB = 88;
		}
		else if (valueSensor > step4 && valueSensor <= step5)
		{
			valueLimitHigh = step5;
			valueLimitLow = step4;
			endColorValueR = 115;
			startColorValueR = 136;
			endColorValueG = 40; // red to violet
			startColorValueG = 26;
			endColorValueB = 125;
			startColorValueB = 51;
		}

		result.R = (byte)(((endColorValueR - startColorValueR) * ((valueSensor - valueLimitLow) / (valueLimitHigh - valueLimitLow))) + startColorValueR);
		result.G = (byte)(((endColorValueG - startColorValueG) * ((valueSensor - valueLimitLow) / (valueLimitHigh - valueLimitLow))) + startColorValueG);
		result.B = (byte)(((endColorValueB - startColorValueB) * ((valueSensor - valueLimitLow) / (valueLimitHigh - valueLimitLow))) + startColorValueB);
	}
	else if (valueSensor > step5)
	{
		result.R = 115;
		result.G = 40; //violet
		result.B = 125;
	}
	else
	{
		result.R = 0;
		result.G = 0;
		result.B = 0;
	}

	//Gamma Correction

	if (correction == true)
	{
		result.R = pgm_read_byte(&gamma8[result.R]);
		result.G = pgm_read_byte(&gamma8[result.G]);
		result.B = pgm_read_byte(&gamma8[result.B]);
	}

	rgb565 = ((result.R & 0b11111000) << 8) | ((result.G & 0b11111100) << 3) | (result.B >> 3);
	//Debug.println(rgb565); // to get list of color if drawGradient is acitvated
	return result;
}

static void drawpicture(uint8_t img[][3])
{
	for (unsigned int i = 0; i < LEDS_NB; ++i)
	{
		leds[i].r = img[i][0];
		leds[i].g = img[i][1];
		leds[i].b = img[i][2];
	}
}

static void drawtime1()
{
	for (unsigned int i = 0; i < (LEDS_NB / 2); ++i)
	{
		if (i < LEDS_NB / 4)
		{
			leds[i] = colorLED_red;
		}

		if (i > (LEDS_NB / 4) - 1)
		{
			leds[i] = colorLED_orange;
		}

		FastLED.show();
		delay(160);
	}
}

static void drawtimemono1()
{
	leds[0] = colorLED_red;
	FastLED.show();
	delay(7500);
	leds[0] = colorLED_orange;
	FastLED.show();
	delay(7500);
}

static void drawtimeline1()
{
	for (unsigned int i = 0; i < LEDS_NB; ++i)
	{
		leds[i] = colorLED_red;
		FastLED.show();
		delay(470);
	}
	for (unsigned int i = 0; i < LEDS_NB; ++i)
	{
		leds[i] = colorLED_orange;
		FastLED.show();
		delay(470);
	}
}

static void drawtime2()
{
	for (unsigned int i = LEDS_NB / 2; i < LEDS_NB; ++i)
	{
		if (i < (LEDS_NB / 4) * 3)
		{
			leds[i] = colorLED_yellow;
		}

		if (i > ((LEDS_NB / 4) * 3) - 1)
		{
			leds[i] = colorLED_green;
		}

		FastLED.show();
		delay(160);
	}
}

static void drawtimemono2()
{
	leds[0] = colorLED_orange;
	FastLED.show();
	delay(7500);
	leds[0] = colorLED_green;
	FastLED.show();
	delay(7500);
}

static void drawtimeline2()
{
	for (unsigned int i = 0; i < LEDS_NB; ++i)
	{
		leds[i] = colorLED_yellow;
		FastLED.show();
		delay(470);
	}
	for (unsigned int i = 0; i < LEDS_NB; ++i)
	{
		leds[i] = colorLED_green;
		FastLED.show();
		delay(470);
	}
}

unsigned int multiplier = 1;
bool LEDwait = false;
unsigned long starttime_waiter;

/*****************************************************************
 * Serial declarations                                           *
 *****************************************************************/

#define serialNPM (Serial1)
#define serialGPS (Serial2)
EspSoftwareSerial::UART serialNO2; //Serial3

/*****************************************************************
 * BMP/BME280 declaration                                        *
 *****************************************************************/
BMX280 bmx280;

/*****************************************************************
 * CCS811 declaration                                        *
 *****************************************************************/
CCS811 ccs811(-1);

/*****************************************************************
 * Envea Cairsens declaration                                        *
 *****************************************************************/
CairsensUART cairsens(&serialNO2);

/*****************************************************************
 * GPS declaration                                        *
 *****************************************************************/

TinyGPSPlus gps;

/*****************************************************************
 * Time                                       *
 *****************************************************************/

// time management varialbles
bool send_now = false;
unsigned long starttime;
unsigned long time_point_device_start_ms;
unsigned long starttime_NPM;
unsigned long starttime_CCS811;
unsigned long starttime_Cairsens;
unsigned long act_micro;
unsigned long act_milli;
unsigned long last_micro = 0;
unsigned long min_micro = 1000000000;
unsigned long max_micro = 0;

unsigned long sending_time = 0;
unsigned long last_update_attempt;
int last_update_returncode;
int last_sendData_returncode;

/*****************************************************************
 * NPM variables and enums                                       *
 *****************************************************************/

bool is_NPM_running = false;
bool nextpmconnected; //important to test nextpm and avoid endless loops

// To read NPM responses
enum
{
	NPM_REPLY_HEADER_16 = 16,
	NPM_REPLY_STATE_16 = 14,
	NPM_REPLY_BODY_16 = 13,
	NPM_REPLY_CHECKSUM_16 = 1
} NPM_waiting_for_16; // for concentration

enum
{
	NPM_REPLY_HEADER_4 = 4,
	NPM_REPLY_STATE_4 = 2,
	NPM_REPLY_CHECKSUM_4 = 1
} NPM_waiting_for_4; // for change

enum
{
	NPM_REPLY_HEADER_5 = 5,
	NPM_REPLY_STATE_5 = 3,
	NPM_REPLY_DATA_5 = 2,
	NPM_REPLY_CHECKSUM_5 = 1
} NPM_waiting_for_5; // for fan speed

enum
{
	NPM_REPLY_HEADER_6 = 6,
	NPM_REPLY_STATE_6 = 4,
	NPM_REPLY_DATA_6 = 3,
	NPM_REPLY_CHECKSUM_6 = 1
} NPM_waiting_for_6; // for version

enum
{
	NPM_REPLY_HEADER_8 = 8,
	NPM_REPLY_STATE_8 = 6,
	NPM_REPLY_BODY_8 = 5,
	NPM_REPLY_CHECKSUM_8 = 1
} NPM_waiting_for_8; // for temperature/humidity

String current_state_npm;
String current_th_npm;

/*****************************************************************
 * Data variables                                      *
 *****************************************************************/
float last_value_BMX280_T = -128.0;
float last_value_BMX280_P = -1.0;
float last_value_BME280_H = -1.0;

float last_value_no2 = -1.0;
uint32_t no2_sum = 0;
uint16_t no2_val_count = 0;

uint32_t npm_pm1_sum = 0;
uint32_t npm_pm10_sum = 0;
uint32_t npm_pm25_sum = 0;
uint32_t npm_pm1_sum_pcs = 0;
uint32_t npm_pm10_sum_pcs = 0;
uint32_t npm_pm25_sum_pcs = 0;
uint16_t npm_val_count = 0;

float last_value_NPM_P0 = -1.0;
float last_value_NPM_P1 = -1.0;
float last_value_NPM_P2 = -1.0;
float last_value_NPM_N1 = -1.0;
float last_value_NPM_N10 = -1.0;
float last_value_NPM_N25 = -1.0;

float last_value_CCS811 = -1.0;
uint32_t ccs811_sum = 0;
uint16_t ccs811_val_count = 0;

float last_value_latitude = 0.0;
float last_value_longitude = 0.0;
uint16_t last_value_altitude = 0;

String last_datajson_string;
String last_datacsv_string;
int last_signal_strength_wifi;
int last_signal_strength_nbiot;
int last_signal_strength_lorawan;
int last_disconnect_reason;

String esp_chipid;

String last_value_NPM_version;

unsigned long NPM_error_count;
unsigned long CCS811_error_count;
unsigned long Cairsens_error_count;
unsigned long WiFi_error_count;

unsigned long last_page_load = millis();

unsigned long count_sends = 0;
uint8_t next_display_count = 0;

unsigned long last_display_millis_oled = 0;

struct struct_wifiInfo
{
	char ssid[LEN_WLANSSID];
	uint8_t encryptionType;
	int32_t RSSI;
	int32_t channel;
};

struct struct_wifiInfo *wifiInfo;
uint8_t count_wifiInfo;

#define msSince(timestamp_before) (act_milli - (timestamp_before))

const char data_first_part[] PROGMEM = "{\"software_version\": \"" SOFTWARE_VERSION_STR "\", \"sensordatavalues\":[";
const char JSON_SENSOR_DATA_VALUES[] PROGMEM = "sensordatavalues";

static String displayGenerateFooter(unsigned int screen_count)
{
	String display_footer;
	for (unsigned int i = 0; i < screen_count; ++i)
	{
		display_footer += (i != (next_display_count % screen_count)) ? " . " : " o ";
	}
	return display_footer;
}

/*****************************************************************
 * NPM functions     *
 *****************************************************************/

static int8_t NPM_get_state()
{
	int8_t result = -1;
	NPM_waiting_for_4 = NPM_REPLY_HEADER_4;
	debug_outln_info(F("State NPM..."));
	NPM_cmd(PmSensorCmd2::State);

	unsigned long timeout = millis();

	do
	{
		debug_outln("Wait for Serial...", DEBUG_MAX_INFO);
	} while (!serialNPM.available() && millis() - timeout < 3000);

	while (serialNPM.available() >= NPM_waiting_for_4)
	{
		const uint8_t constexpr header[2] = {0x81, 0x16};
		uint8_t state[1];
		uint8_t checksum[1];
		uint8_t test[4];

		switch (NPM_waiting_for_4)
		{
		case NPM_REPLY_HEADER_4:
			if (serialNPM.find(header, sizeof(header)))
				NPM_waiting_for_4 = NPM_REPLY_STATE_4;
			break;
		case NPM_REPLY_STATE_4:
			serialNPM.readBytes(state, sizeof(state));
			NPM_state(state[0]);
			result = state[0];
			NPM_waiting_for_4 = NPM_REPLY_CHECKSUM_4;
			break;
		case NPM_REPLY_CHECKSUM_4:
			serialNPM.readBytes(checksum, sizeof(checksum));
			memcpy(test, header, sizeof(header));
			memcpy(&test[sizeof(header)], state, sizeof(state));
			memcpy(&test[sizeof(header) + sizeof(state)], checksum, sizeof(checksum));
			NPM_data_reader(test, 4);
			NPM_waiting_for_4 = NPM_REPLY_HEADER_4;
			if (NPM_checksum_valid_4(test))
			{
				debug_outln_info(F("Checksum OK..."));
			}
			break;
		}
	}
	return result;
}

static bool NPM_start_stop()
{
	bool result;
	NPM_waiting_for_4 = NPM_REPLY_HEADER_4;
	debug_outln_info(F("Switch start/stop NPM..."));
	NPM_cmd(PmSensorCmd2::Change);

	unsigned long timeout = millis();

	do
	{
		debug_outln("Wait for Serial...", DEBUG_MAX_INFO);
	} while (!serialNPM.available() && millis() - timeout < 3000);

	while (serialNPM.available() >= NPM_waiting_for_4)
	{
		const uint8_t constexpr header[2] = {0x81, 0x15};
		uint8_t state[1];
		uint8_t checksum[1];
		uint8_t test[4];

		switch (NPM_waiting_for_4)
		{
		case NPM_REPLY_HEADER_4:
			if (serialNPM.find(header, sizeof(header)))
				NPM_waiting_for_4 = NPM_REPLY_STATE_4;
			break;
		case NPM_REPLY_STATE_4:
			serialNPM.readBytes(state, sizeof(state));
			NPM_state(state[0]);

			if (bitRead(state[0], 0) == 0)
			{
				debug_outln_info(F("NPM start..."));
				result = true;
			}
			else if (bitRead(state[0], 0) == 1)
			{
				debug_outln_info(F("NPM stop..."));
				result = false;
			}
			else
			{
				result = !is_NPM_running; //DANGER BECAUSE NON INITIALISED
			}

			NPM_waiting_for_4 = NPM_REPLY_CHECKSUM_4;
			break;
		case NPM_REPLY_CHECKSUM_4:
			serialNPM.readBytes(checksum, sizeof(checksum));
			memcpy(test, header, sizeof(header));
			memcpy(&test[sizeof(header)], state, sizeof(state));
			memcpy(&test[sizeof(header) + sizeof(state)], checksum, sizeof(checksum));
			NPM_data_reader(test, 4);
			NPM_waiting_for_4 = NPM_REPLY_HEADER_4;
			if (NPM_checksum_valid_4(test))
			{
				debug_outln_info(F("Checksum OK..."));
			}
			break;
		}
	}
	return result;
}

static String NPM_version_date()
{
	// debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(DBG_TXT_NPM_VERSION_DATE));
	delay(250);
	NPM_waiting_for_6 = NPM_REPLY_HEADER_6;
	debug_outln_info(F("Version NPM..."));
	NPM_cmd(PmSensorCmd2::Version);

	unsigned long timeout = millis();

	do
	{
		debug_outln("Wait for Serial...", DEBUG_MAX_INFO);
	} while (!serialNPM.available() && millis() - timeout < 3000);

	while (serialNPM.available() >= NPM_waiting_for_6)
	{
		const uint8_t constexpr header[2] = {0x81, 0x17};
		uint8_t state[1];
		uint8_t data[2];
		uint8_t checksum[1];
		uint8_t test[6];

		switch (NPM_waiting_for_6)
		{
		case NPM_REPLY_HEADER_6:
			if (serialNPM.find(header, sizeof(header)))
				NPM_waiting_for_6 = NPM_REPLY_STATE_6;
			break;
		case NPM_REPLY_STATE_6:
			serialNPM.readBytes(state, sizeof(state));
			NPM_state(state[0]);
			NPM_waiting_for_6 = NPM_REPLY_DATA_6;
			break;
		case NPM_REPLY_DATA_6:
			if (serialNPM.readBytes(data, sizeof(data)) == sizeof(data))
			{
				NPM_data_reader(data, 2);
				uint16_t NPMversion = word(data[0], data[1]);
				last_value_NPM_version = String(NPMversion);
				// debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(DBG_TXT_NPM_VERSION_DATE));
				debug_outln_info(F("Next PM Firmware: "), last_value_NPM_version);
			}
			NPM_waiting_for_6 = NPM_REPLY_CHECKSUM_6;
			break;
		case NPM_REPLY_CHECKSUM_6:
			serialNPM.readBytes(checksum, sizeof(checksum));
			memcpy(test, header, sizeof(header));
			memcpy(&test[sizeof(header)], state, sizeof(state));
			memcpy(&test[sizeof(header) + sizeof(state)], data, sizeof(data));
			memcpy(&test[sizeof(header) + sizeof(state) + sizeof(data)], checksum, sizeof(checksum));
			NPM_data_reader(test, 6);
			NPM_waiting_for_6 = NPM_REPLY_HEADER_6;
			if (NPM_checksum_valid_6(test))
			{
				debug_outln_info(F("Checksum OK..."));
			}
			break;
		}
	}
	return last_value_NPM_version;
}

static void NPM_fan_speed()
{

	NPM_waiting_for_5 = NPM_REPLY_HEADER_5;
	debug_outln_info(F("Set fan speed to 50 %..."));
	NPM_cmd(PmSensorCmd2::Speed);

	unsigned long timeout = millis();

	do
	{
		debug_outln("Wait for Serial...", DEBUG_MAX_INFO);
	} while (!serialNPM.available() && millis() - timeout < 3000);

	while (serialNPM.available() >= NPM_waiting_for_5)
	{
		const uint8_t constexpr header[2] = {0x81, 0x21};
		uint8_t state[1];
		uint8_t data[1];
		uint8_t checksum[1];
		uint8_t test[5];

		switch (NPM_waiting_for_5)
		{
		case NPM_REPLY_HEADER_5:
			if (serialNPM.find(header, sizeof(header)))
				NPM_waiting_for_5 = NPM_REPLY_STATE_5;
			break;
		case NPM_REPLY_STATE_5:
			serialNPM.readBytes(state, sizeof(state));
			NPM_state(state[0]);
			NPM_waiting_for_5 = NPM_REPLY_DATA_5;
			break;
		case NPM_REPLY_DATA_5:
			if (serialNPM.readBytes(data, sizeof(data)) == sizeof(data))
			{
				NPM_data_reader(data, 1);
			}
			NPM_waiting_for_5 = NPM_REPLY_CHECKSUM_5;
			break;
		case NPM_REPLY_CHECKSUM_5:
			serialNPM.readBytes(checksum, sizeof(checksum));
			memcpy(test, header, sizeof(header));
			memcpy(&test[sizeof(header)], state, sizeof(state));
			memcpy(&test[sizeof(header) + sizeof(state)], data, sizeof(data));
			memcpy(&test[sizeof(header) + sizeof(state) + sizeof(data)], checksum, sizeof(checksum));
			NPM_data_reader(test, 5);
			NPM_waiting_for_5 = NPM_REPLY_HEADER_5;
			if (NPM_checksum_valid_5(test))
			{
				debug_outln_info(F("Checksum OK..."));
			}
			break;
		}
	}
}

static String NPM_temp_humi()
{
	uint16_t NPM_temp;
	uint16_t NPM_humi;
	NPM_waiting_for_8 = NPM_REPLY_HEADER_8;
	debug_outln_info(F("Temperature/Humidity in Next PM..."));
	NPM_cmd(PmSensorCmd2::Temphumi);

	unsigned long timeout = millis();

	do
	{
		debug_outln("Wait for Serial...", DEBUG_MAX_INFO);
	} while (!serialNPM.available() && millis() - timeout < 3000);

	while (serialNPM.available() >= NPM_waiting_for_8)
	{
		const uint8_t constexpr header[2] = {0x81, 0x14};
		uint8_t state[1];
		uint8_t data[4];
		uint8_t checksum[1];
		uint8_t test[8];

		switch (NPM_waiting_for_8)
		{
		case NPM_REPLY_HEADER_8:
			if (serialNPM.find(header, sizeof(header)))
				NPM_waiting_for_8 = NPM_REPLY_STATE_8;
			break;
		case NPM_REPLY_STATE_8:
			serialNPM.readBytes(state, sizeof(state));
			NPM_state(state[0]);
			NPM_waiting_for_8 = NPM_REPLY_BODY_8;
			break;
		case NPM_REPLY_BODY_8:
			if (serialNPM.readBytes(data, sizeof(data)) == sizeof(data))
			{
				NPM_data_reader(data, 4);
				NPM_temp = word(data[0], data[1]);
				NPM_humi = word(data[2], data[3]);
				debug_outln_verbose(F("Temperature (°C): "), String(NPM_temp / 100.0f));
				debug_outln_verbose(F("Relative humidity (%): "), String(NPM_humi / 100.0f));
			}
			NPM_waiting_for_8 = NPM_REPLY_CHECKSUM_8;
			break;
		case NPM_REPLY_CHECKSUM_16:
			serialNPM.readBytes(checksum, sizeof(checksum));
			memcpy(test, header, sizeof(header));
			memcpy(&test[sizeof(header)], state, sizeof(state));
			memcpy(&test[sizeof(header) + sizeof(state)], data, sizeof(data));
			memcpy(&test[sizeof(header) + sizeof(state) + sizeof(data)], checksum, sizeof(checksum));
			NPM_data_reader(test, 8);
			if (NPM_checksum_valid_8(test))
				debug_outln_info(F("Checksum OK..."));
			NPM_waiting_for_8 = NPM_REPLY_HEADER_8;
			break;
		}
	}
	return String(NPM_temp / 100.0f) + " / " + String(NPM_humi / 100.0f);
}

/*****************************************************************
 * write config to spiffs                                        *
 *****************************************************************/
static bool writeConfig()
{

	DynamicJsonDocument json(JSON_BUFFER_SIZE);
	debug_outln_info(F("Saving config..."));
	json["SOFTWARE_VERSION"] = SOFTWARE_VERSION;

	for (unsigned e = 0; e < sizeof(configShape) / sizeof(configShape[0]); ++e)
	{
		ConfigShapeEntry c;
		memcpy_P(&c, &configShape[e], sizeof(ConfigShapeEntry));
		switch (c.cfg_type)
		{
		case Config_Type_Bool:
			json[c.cfg_key()].set(*c.cfg_val.as_bool);
			break;
		case Config_Type_UInt:
		case Config_Type_Time:
			json[c.cfg_key()].set(*c.cfg_val.as_uint);
			break;
		case Config_Type_Password:
		case Config_Type_Hex:
		case Config_Type_String:
			json[c.cfg_key()].set(c.cfg_val.as_str);
			break;
		};
	}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

	SPIFFS.remove(F("/config.json.old"));
	SPIFFS.rename(F("/config.json"), F("/config.json.old"));

	File configFile = SPIFFS.open(F("/config.json"), "w");
	if (configFile)
	{
		serializeJsonPretty(json, Debug);
		serializeJson(json, configFile);
		configFile.close();
		debug_outln_info(F("Config written successfully."));
	}
	else
	{
		debug_outln_error(F("failed to open config file for writing"));
		return false;
	}

#pragma GCC diagnostic pop

	return true;
}

/*****************************************************************
 * read config from spiffs                                       *
 *****************************************************************/

/* backward compatibility for the times when we stored booleans as strings */
static bool boolFromJSON(const DynamicJsonDocument &json, const __FlashStringHelper *key)
{
	if (json[key].is<const char *>())
	{
		return !strcmp_P(json[key].as<const char *>(), PSTR("true"));
	}
	return json[key].as<bool>();
}

static void readConfig(bool oldconfig = false)
{
	bool rewriteConfig = false;

	String cfgName(F("/config.json"));
	if (oldconfig)
	{
		cfgName += F(".old");
	}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
	File configFile = SPIFFS.open(cfgName, "r");
	if (!configFile)
	{
		if (!oldconfig)
		{
			return readConfig(true /* oldconfig */);
		}

		debug_outln_error(F("failed to open config file."));
		return;
	}

	debug_outln_info(F("opened config file..."));
	DynamicJsonDocument json(JSON_BUFFER_SIZE);
	DeserializationError err = deserializeJson(json, configFile.readString());
	configFile.close();
#pragma GCC diagnostic pop

	if (!err)
	{
		serializeJsonPretty(json, Debug);
		debug_outln_info(F("parsed json..."));
		for (unsigned e = 0; e < sizeof(configShape) / sizeof(configShape[0]); ++e)
		{
			ConfigShapeEntry c;
			memcpy_P(&c, &configShape[e], sizeof(ConfigShapeEntry));
			if (json[c.cfg_key()].isNull())
			{
				continue;
			}
			switch (c.cfg_type)
			{
			case Config_Type_Bool:
				*(c.cfg_val.as_bool) = boolFromJSON(json, c.cfg_key());
				break;
			case Config_Type_UInt:
			case Config_Type_Time:
				*(c.cfg_val.as_uint) = json[c.cfg_key()].as<unsigned int>();
				break;
			case Config_Type_String:
			case Config_Type_Hex:
			case Config_Type_Password:
				strncpy(c.cfg_val.as_str, json[c.cfg_key()].as<const char *>(), c.cfg_len);
				c.cfg_val.as_str[c.cfg_len] = '\0';
				break;
			};
		}
		String writtenVersion(json["SOFTWARE_VERSION"].as<const char *>());
		if (writtenVersion.length() && writtenVersion[0] == 'N' && SOFTWARE_VERSION != writtenVersion)
		{
			debug_outln_info(F("Rewriting old config from: "), writtenVersion);
			// would like to do that, but this would wipe firmware.old which the two stage loader
			// might still need
			// SPIFFS.format();
			rewriteConfig = true;
		}

		if (boolFromJSON(json, F("bmp280_read")) || boolFromJSON(json, F("bme280_read")))
		{
			cfg::bmx280_read = true;
			rewriteConfig = true;
		}
	}
	else
	{
		debug_outln_error(F("failed to load json config"));

		if (!oldconfig)
		{
			return readConfig(true /* oldconfig */);
		}
	}

	if (rewriteConfig)
	{
		writeConfig();
	}
}

static void init_config()
{

	debug_outln_info(F("mounting FS..."));

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

	bool spiffs_begin_ok = SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED);

#pragma GCC diagnostic pop

	if (!spiffs_begin_ok)
	{
		debug_outln_error(F("failed to mount FS"));
		return;
	}
	readConfig();
}

/*****************************************************************
 * dew point helper function                                     *
 *****************************************************************/
static float dew_point(const float temperature, const float humidity)
{
	float dew_temp;
	const float k2 = 17.62;
	const float k3 = 243.12;

	dew_temp = k3 * (((k2 * temperature) / (k3 + temperature)) + log(humidity / 100.0f)) / (((k2 * k3) / (k3 + temperature)) - log(humidity / 100.0f));

	return dew_temp;
}

/*****************************************************************
 * GPS                                                     *
 *****************************************************************/

bool coordinates;

bool newGPSdata = false;

struct GPS
{
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	double latitude;
	double longitude;
	double altitude;
	bool checked;
};

struct GPS GPSdata
{
	0, 0, 0, 0, 0, 0, -1.0, -1.0, -1.0, false
};

static struct GPS getGPSdata()
{

	struct GPS result;
	Debug.print(F("Location: "));
	if (gps.location.isValid())
	{
		Debug.print(gps.location.lat(), 6);
		Debug.print(F(","));
		Debug.print(gps.location.lng(), 6);
		Debug.print(F(" | "));

		result.latitude = gps.location.lat();
		result.longitude = gps.location.lng();
	}
	else
	{
		Debug.print(F("INVALID"));
		Debug.println();
		result.checked = false;
		return result;
	}

	Debug.print(F("Altitude: "));
	if (gps.altitude.isValid())
	{
		Debug.print(gps.altitude.meters());
		Debug.print(F(" | "));
		result.altitude = gps.altitude.meters();
	}
	else
	{
		Debug.print(F("INVALID"));
		result.checked = false;
		Debug.println();
		return result;
	}

	Debug.print(F("Date/Time: "));
	if (gps.date.isValid())
	{
		Debug.print(gps.date.month());
		Debug.print(F("/"));
		Debug.print(gps.date.day());
		Debug.print(F("/"));
		Debug.print(gps.date.year());

		result.month = gps.date.month();
		result.day = gps.date.day();
		result.year = gps.date.year();
	}
	else
	{
		Debug.print(F("INVALID"));
		Debug.println();
		result.checked = false;
		return result;
	}

	Debug.print(F(" "));
	if (gps.time.isValid())
	{
		if (gps.time.hour() < 10)
			Debug.print(F("0"));
		Debug.print(gps.time.hour());
		Debug.print(F(":"));
		if (gps.time.minute() < 10)
			Debug.print(F("0"));
		Debug.print(gps.time.minute());
		Debug.print(F(":"));
		if (gps.time.second() < 10)
			Debug.print(F("0"));
		Debug.print(gps.time.second());

		result.hour = gps.time.hour();
		result.minute = gps.time.minute();
		result.second = gps.time.second();
	}
	else
	{
		Debug.print(F("INVALID"));
		Debug.println();
		result.checked = false;
		return result;
	}

	Debug.println();
	result.checked = true;
	return result;
}

/*****************************************************************
 * Pressure at sea level function                                     *
 *****************************************************************/
static float pressure_at_sealevel(const float temperature, const float pressure)
{
	float pressure_at_sealevel;

	if(cfg::has_gps && GPSdata.checked){
	pressure_at_sealevel = pressure * pow(((temperature + 273.15f) / (temperature + 273.15f + (0.0065f * readCorrectionOffset(String(GPSdata.altitude).c_str())))), -5.255f);
	}else
	{
	pressure_at_sealevel = pressure * pow(((temperature + 273.15f) / (temperature + 273.15f + (0.0065f * readCorrectionOffset("0")))), -5.255f);
	}


	return pressure_at_sealevel;
}

/*****************************************************************
 * html helper functions                                         *
 *****************************************************************/
static void start_html_page(String &page_content, const String &title)
{
	last_page_load = millis();

	RESERVE_STRING(s, LARGE_STR);
	s = FPSTR(WEB_PAGE_HEADER);
	s.replace("{t}", title);
	server.setContentLength(CONTENT_LENGTH_UNKNOWN);
	server.send(200, FPSTR(TXT_CONTENT_TYPE_TEXT_HTML), s);

	server.sendContent_P(WEB_PAGE_HEADER_HEAD);

	s = FPSTR(WEB_PAGE_HEADER_BODY);
	s.replace("{t}", title);
	if (title != " ")
	{
		s.replace("{n}", F("&raquo;"));
	}
	else
	{
		s.replace("{n}", emptyString);
	}
	s.replace("{id}", esp_chipid);
	page_content += s;
}

static void end_html_page(String &page_content)
{
	if (page_content.length())
	{
		server.sendContent(page_content);
	}
	server.sendContent_P(WEB_PAGE_FOOTER);
}

static void add_form_input(String &page_content, const ConfigShapeId cfgid, const __FlashStringHelper *info, const int length)
{
	RESERVE_STRING(s, MED_STR);
	s = F("<tr>"
		  "<td title='[&lt;= {l}]'>{i}:&nbsp;</td>"
		  "<td style='width:{l}em'>"
		  "<input form='main' type='{t}' name='{n}' id='{n}' placeholder='{i}' value='{v}' maxlength='{l}'/>"
		  "</td></tr>");
	String t_value;
	ConfigShapeEntry c;
	memcpy_P(&c, &configShape[cfgid], sizeof(ConfigShapeEntry));
	switch (c.cfg_type)
	{
	case Config_Type_UInt:
		t_value = String(*c.cfg_val.as_uint);
		s.replace("{t}", F("number"));
		break;
	case Config_Type_Time:
		t_value = String((*c.cfg_val.as_uint) / 1000);
		s.replace("{t}", F("number"));
		break;
	case Config_Type_Password:
		s.replace("{t}", F("password"));
		info = FPSTR(INTL_PASSWORD);
	case Config_Type_Hex:
		s.replace("{t}", F("hex"));
	default:
		t_value = c.cfg_val.as_str;
		t_value.replace("'", "&#39;");
		s.replace("{t}", F("text"));
	}
	s.replace("{i}", info);
	s.replace("{n}", String(c.cfg_key()));
	s.replace("{v}", t_value);
	s.replace("{l}", String(length));
	page_content += s;
}

static void add_form_input_nbiot(String &page_content, const String result, const String lteid, const int length)
{
	RESERVE_STRING(s, MED_STR);
	s = F("<tr>"
		  "<td title='[&lt;= {l}]'>{i}&nbsp;</td>"
		  "<td style='width:{l}em'>"
		  "<input form='none' type='text' value='{v}' maxlength='{l}'/>"
		  "</td></tr>");
	s.replace("{i}", result);
	s.replace("{v}", lteid);
	s.replace("{l}", String(length));
	page_content += s;
}

//  "<input form='secondar' type='text' name='lteid' id='lteid' value='{v}' maxlength='{l}'/>"

static void add_radio_input(String &page_content, const ConfigShapeId cfgid, const __FlashStringHelper *info)
{
	RESERVE_STRING(s, MED_STR);
	String t_value;
	ConfigShapeEntry c;
	memcpy_P(&c, &configShape[cfgid], sizeof(ConfigShapeEntry));
	t_value = String(*c.cfg_val.as_uint);

	if (cfgid == Config_value_displayed)
	{

		s = F("<b>{i}</b>"
			  "<div>"
			  "<input form='main' type='radio' id='pm1' name='{n}' value='0' {a}>"
			  "<label for='pm1'>PM1</label>"
			  "</div>"
			  "<div>"
			  "<input form='main' type='radio' id='pm10' name='{n}' value='1' {b}>"
			  "<label for='pm10'>PM10</label>"
			  "</div>"
			  "<div>"
			  "<input form='main' type='radio' id='pm25' name='{n}' value='2' {c}>"
			  "<label for='pm25'>PM2.5</label>"
			  "</div>"
			  "<div>"
			  "<input form='main' type='radio' id='temp' name='{n}' value='3' {d}>"
			  "<label for='temp'>Température</label>"
			  "</div>"
			  "<div>"
			  "<input form='main' type='radio' id='humi' name='{n}' value='4' {e}>"
			  "<label for='humi'>Humidité</label>"
			  "</div>"
			  "<div>"
			  "<input form='main' type='radio' id='press' name='{n}' value='5' {f}>"
			  "<label for='press'>Pression</label>"
			  "</div>"
			  "<div>"
			  "<input form='main' type='radio' id='cov' name='{n}' value='6' {g}>"
			  "<label for='cov'>COV</label>"
			  "</div>"
			  "<input form='main' type='radio' id='no2' name='{n}' value='7' {h}>"
			  "<label for='no2'>NO2</label>");

		switch (t_value.toInt())
		{
		case 0:
			s.replace("{a}", "checked");
			s.replace("{b}", "");
			s.replace("{c}", "");
			s.replace("{d}", "");
			s.replace("{e}", "");
			s.replace("{f}", "");
			s.replace("{g}", "");
			s.replace("{h}", "");
			break;
		case 1:
			s.replace("{a}", "");
			s.replace("{b}", "checked");
			s.replace("{c}", "");
			s.replace("{d}", "");
			s.replace("{e}", "");
			s.replace("{f}", "");
			s.replace("{g}", "");
			s.replace("{h}", "");
			break;
		case 2:
			s.replace("{a}", "");
			s.replace("{b}", "");
			s.replace("{c}", "checked");
			s.replace("{d}", "");
			s.replace("{e}", "");
			s.replace("{f}", "");
			s.replace("{g}", "");
			s.replace("{h}", "");
			break;
		case 3:
			s.replace("{a}", "");
			s.replace("{b}", "");
			s.replace("{c}", "");
			s.replace("{d}", "checked");
			s.replace("{e}", "");
			s.replace("{f}", "");
			s.replace("{g}", "");
			s.replace("{h}", "");
			break;
		case 4:
			s.replace("{a}", "");
			s.replace("{b}", "");
			s.replace("{c}", "");
			s.replace("{d}", "");
			s.replace("{e}", "checked");
			s.replace("{f}", "");
			s.replace("{g}", "");
			s.replace("{h}", "");
			break;
		case 5:
			s.replace("{a}", "");
			s.replace("{b}", "");
			s.replace("{c}", "");
			s.replace("{d}", "");
			s.replace("{e}", "");
			s.replace("{f}", "checked");
			s.replace("{g}", "");
			s.replace("{h}", "");
			break;
		case 6:
			s.replace("{a}", "");
			s.replace("{b}", "");
			s.replace("{c}", "");
			s.replace("{d}", "");
			s.replace("{e}", "");
			s.replace("{f}", "");
			s.replace("{g}", "checked");
			s.replace("{h}", "");
			break;
		case 7:
			s.replace("{a}", "");
			s.replace("{b}", "");
			s.replace("{c}", "");
			s.replace("{d}", "");
			s.replace("{e}", "");
			s.replace("{f}", "");
			s.replace("{g}", "");
			s.replace("{h}", "checked");
			break;
		}
	}

	s.replace("{i}", info);
	s.replace("{n}", String(c.cfg_key()));
	page_content += s;
}

static String form_checkbox(const ConfigShapeId cfgid, const String &info, const bool linebreak)
{
	RESERVE_STRING(s, MED_STR);
	s = F("<label for='{n}'>"
		  "<input form='main' type='checkbox' name='{n}' value='1' id='{n}' {c}/>"
		  "<input form='main' type='hidden' name='{n}' value='0'/>"
		  "{i}</label><br/>");

	if (*configShape[cfgid].cfg_val.as_bool)
	{
		s.replace("{c}", F(" checked='checked'"));
	}
	else
	{
		s.replace("{c}", emptyString);
	};
	s.replace("{i}", info);
	s.replace("{n}", String(configShape[cfgid].cfg_key()));
	if (!linebreak)
	{
		s.replace("<br/>", emptyString);
	}
	return s;
}

static String form_submit(const String &value)
{
	String s = F("<tr>"
				 "<td>&nbsp;</td>"
				 "<td>"
				 "<input form='main' type='submit' name='submit' value='{v}' />"
				 "</td>"
				 "</tr>");
	s.replace("{v}", value);
	return s;
}

static String form_select_lang()
{
	String s_select = F(" selected='selected'");
	String s = F("<tr>"
				 "<td>" INTL_LANGUAGE ":&nbsp;</td>"
				 "<td>"
				 "<select form='main' id='current_lang' name='current_lang'>"
				 "<option value='FR'>Français (FR)</option>"
				 "<option value='EN'>English (EN)</option>"
				 "</select>"
				 "</td>"
				 "</tr>");

	s.replace("'" + String(cfg::current_lang) + "'>", "'" + String(cfg::current_lang) + "'" + s_select + ">");
	return s;
}

static void add_warning_first_cycle(String &page_content)
{
	String s = FPSTR(INTL_TIME_TO_FIRST_MEASUREMENT);
	unsigned int time_to_first = cfg::sending_intervall_ms - msSince(starttime);
	if (time_to_first > cfg::sending_intervall_ms)
	{
		time_to_first = 0;
	}
	s.replace("{v}", String(((time_to_first + 500) / 1000)));
	page_content += s;
}

static void add_age_last_values(String &s)
{
	s += "<b>";
	unsigned int time_since_last = msSince(starttime);
	if (time_since_last > cfg::sending_intervall_ms)
	{
		time_since_last = 0;
	}
	s += String((time_since_last + 500) / 1000);
	s += FPSTR(INTL_TIME_SINCE_LAST_MEASUREMENT);
	s += FPSTR(WEB_B_BR_BR);
}

static void sendHttpRedirect()
{
	server.sendHeader(F("Location"), F("http://192.168.4.1/config"));
	server.send(302, FPSTR(TXT_CONTENT_TYPE_TEXT_HTML), emptyString);
}

/*****************************************************************
 * Webserver root: show all options                              *
 *****************************************************************/
static void webserver_root()
{

	if (WiFi.status() != WL_CONNECTED)
	{
		sendHttpRedirect();
	}
	else
	{
		RESERVE_STRING(page_content, XLARGE_STR);
		start_html_page(page_content, emptyString);
		debug_outln_info(F("ws: root ..."));

		// Enable Pagination
		page_content += FPSTR(WEB_ROOT_PAGE_CONTENT);
		page_content.replace(F("{t}"), FPSTR(INTL_CURRENT_DATA));
		page_content.replace(F("{s}"), FPSTR(INTL_DEVICE_STATUS));
		page_content.replace(F("{conf}"), FPSTR(INTL_CONFIGURATION));
		page_content.replace(F("{restart}"), FPSTR(INTL_RESTART_SENSOR));
		page_content.replace(F("{debug}"), FPSTR(INTL_DEBUG_LEVEL));
		end_html_page(page_content);
	}
}

/*****************************************************************
 * Webserver config: show config page                            *
 *****************************************************************/
static void webserver_config_send_body_get(String &page_content)
{
	auto add_form_checkbox = [&page_content](const ConfigShapeId cfgid, const String &info)
	{
		page_content += form_checkbox(cfgid, info, true);
	};

	auto add_form_checkbox_sensor = [&add_form_checkbox](const ConfigShapeId cfgid, __const __FlashStringHelper *info)
	{
		add_form_checkbox(cfgid, add_sensor_type(info));
	};

	debug_outln_info(F("begin webserver_config_body_get ..."));

	page_content += F("<form id='main' method='POST' action='/config' style='width:100%;'></form>\n"
					  "<input form='main' class='radio' id='r1' name='group' type='radio' checked>"
					  "<input form='main' class='radio' id='r2' name='group' type='radio'>"
					  "<input form='main' class='radio' id='r3' name='group' type='radio'>"
					  "<input form='main' class='radio' id='r4' name='group' type='radio'>"
					  "<input form='main' class='radio' id='r5' name='group' type='radio'>"
					  "<div class='tabs'>"
					  "<label class='tab' id='tab1' for='r1'>");
	page_content += FPSTR(INTL_WIFI_SETTINGS);
	page_content += F("</label>"
					  "<label class='tab' id='tab2' for='r2'>");
	page_content += FPSTR(INTL_MORE_SETTINGS);
	page_content += F("</label>"
					  "<label class='tab' id='tab3' for='r3'>");
	page_content += FPSTR(INTL_SENSORS);
	page_content += F("</label>"
					  "<label class='tab' id='tab4' for='r4'>");
	page_content += FPSTR(INTL_LEDS);
	page_content += F("</label>"
					  "<label class='tab' id='tab5' for='r5'>");
	page_content += FPSTR(INTL_APIS);
	page_content += F("</label></div>"
					  "<div class='panels'>"
					  "<div class='panel' id='panel1'>");

	add_form_checkbox(Config_wifi_permanent, FPSTR(INTL_WIFI_PERMANENT));
	page_content += FPSTR(BR_TAG);
	//page_content = FPSTR(INTL_FS_WIFI_DESCRIPTION);
	// page_content += FPSTR(BR_TAG);
	page_content += FPSTR(TABLE_TAG_OPEN);
	add_form_input(page_content, Config_fs_ssid, FPSTR(INTL_FS_WIFI_NAME), LEN_FS_SSID - 1);
	add_form_input(page_content, Config_fs_pwd, FPSTR(INTL_PASSWORD), LEN_CFG_PASSWORD - 1);
	page_content += FPSTR(TABLE_TAG_CLOSE_BR);

	// Paginate page after ~ 1500 Bytes
	server.sendContent(page_content);

	page_content = tmpl(FPSTR(WEB_DIV_PANEL), String(2));

	page_content += F("<b>" INTL_LOCATION "</b>&nbsp;");
	page_content += FPSTR("<br/>");
	add_form_checkbox(Config_has_gps, FPSTR(INTL_GPS));

	// Paginate page after ~ 1500 Bytes

	server.sendContent(page_content);
	page_content = emptyString;

	page_content = FPSTR(WEB_BR_LF_B);
	page_content += F(INTL_FIRMWARE "</b>&nbsp;");

	page_content += FPSTR(TABLE_TAG_OPEN);
	page_content += form_select_lang();
	page_content += FPSTR(TABLE_TAG_CLOSE_BR);

	page_content += FPSTR(TABLE_TAG_OPEN);
	add_form_input(page_content, Config_debug, FPSTR(INTL_DEBUG_LEVEL), 1);
	add_form_input(page_content, Config_time_for_wifi_config, FPSTR(INTL_DURATION_ROUTER_MODE), 5);
	page_content += FPSTR(TABLE_TAG_CLOSE_BR);

	server.sendContent(page_content);

	page_content = tmpl(FPSTR(WEB_DIV_PANEL), String(3));

	page_content += FPSTR("<b>");
	page_content += FPSTR(INTL_PM_SENSORS);
	page_content += FPSTR(WEB_B_BR);
	add_form_checkbox_sensor(Config_npm_read, FPSTR(INTL_NPM));
	// Paginate page after ~ 1500 Bytes  //ATTENTION RYTHME PAGINATION !
	server.sendContent(page_content);
	page_content = emptyString;

	page_content += FPSTR(WEB_BR_LF_B);
	page_content += FPSTR(INTL_THP_SENSORS);
	page_content += FPSTR(WEB_B_BR);

	add_form_checkbox_sensor(Config_bmx280_read, FPSTR(INTL_BMX280));

	// // Paginate page after ~ 1500 Bytes
	server.sendContent(page_content);
	page_content = emptyString;

	page_content += FPSTR(WEB_BR_LF_B);
	page_content += FPSTR(INTL_VOC_SENSORS);
	page_content += FPSTR(WEB_B_BR);

	add_form_checkbox_sensor(Config_ccs811_read, FPSTR(INTL_CCS811));

	// Paginate page after ~ 1500 Bytes
	server.sendContent(page_content);
	page_content = emptyString;

	page_content += FPSTR(WEB_BR_LF_B);
	page_content += FPSTR(INTL_NO2_SENSORS);
	page_content += FPSTR(WEB_B_BR);

	add_form_checkbox_sensor(Config_enveano2_read, FPSTR(INTL_ENVEANO2));

	// Paginate page after ~ 1500 Bytes
	server.sendContent(page_content);
	//page_content = emptyString;

	page_content = tmpl(FPSTR(WEB_DIV_PANEL), String(4));

	page_content += FPSTR("<b>");
	page_content += FPSTR(INTL_LED_CONFIG);
	page_content += FPSTR(WEB_B_BR);
	add_form_checkbox(Config_has_led_value, FPSTR(INTL_LED_VALUE));
	add_form_input(page_content, Config_brightness, FPSTR(INTL_BRIGHTNESS), 3);
	page_content += FPSTR("<br/><br/>");
	add_radio_input(page_content, Config_value_displayed, FPSTR(INTL_VALUE_DISPLAYED));

	server.sendContent(page_content);

	page_content = tmpl(FPSTR(WEB_DIV_PANEL), String(5));

	add_form_checkbox(Config_send2csv, FPSTR(WEB_CSV));
	page_content += FPSTR("<br/>");
	add_form_checkbox(Config_has_sdcard, FPSTR(SDCARD));

	server.sendContent(page_content);
	page_content = emptyString;

	//page_content += FPSTR(TABLE_TAG_CLOSE_BR);
	page_content += F("</div></div>");
	page_content += form_submit(FPSTR(INTL_SAVE_AND_RESTART));
	page_content += FPSTR(BR_TAG);
	page_content += FPSTR("<br/>");
	// page_content += FPSTR(WEB_BR_FORM);

	server.sendContent(page_content);
	page_content = emptyString;
}

static void webserver_config_send_body_post(String &page_content)
{
	String masked_pwd;

	for (unsigned e = 0; e < sizeof(configShape) / sizeof(configShape[0]); ++e)
	{
		ConfigShapeEntry c;
		memcpy_P(&c, &configShape[e], sizeof(ConfigShapeEntry));
		const String s_param(c.cfg_key());
		if (!server.hasArg(s_param))
		{
			continue;
		}
		const String server_arg(server.arg(s_param));

		switch (c.cfg_type)
		{
		case Config_Type_UInt:
			*(c.cfg_val.as_uint) = server_arg.toInt();
			break;
		case Config_Type_Time:
			*(c.cfg_val.as_uint) = server_arg.toInt() * 1000;
			break;
		case Config_Type_Bool:
			*(c.cfg_val.as_bool) = (server_arg == "1");
			break;
		case Config_Type_String:
			strncpy(c.cfg_val.as_str, server_arg.c_str(), c.cfg_len);
			c.cfg_val.as_str[c.cfg_len] = '\0';
			break;
		case Config_Type_Password:
			if (server_arg.length())
			{
				server_arg.toCharArray(c.cfg_val.as_str, LEN_CFG_PASSWORD);
			}
			break;
		case Config_Type_Hex:
			strncpy(c.cfg_val.as_str, server_arg.c_str(), c.cfg_len);
			c.cfg_val.as_str[c.cfg_len] = '\0';
			break;
		}
	}

	page_content += FPSTR(INTL_SENSOR_IS_REBOOTING);

	server.sendContent(page_content);
	page_content = emptyString;
}

static void sensor_restart()
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

	SPIFFS.end();

#pragma GCC diagnostic pop

	if (cfg::npm_read)
	{
		serialNPM.end();
	}

	if (cfg::has_led_value)
	{
		if (LEDS_NB == 1)
		{
			leds[0] = colorLED_empty;
			FastLED.show();
		}
		else
		{
			if (LEDS_MATRIX)
			{
				drawpicture(empty);
				FastLED.show();
			}
			else
			{
			}
		}
	}

	debug_outln_info(F("Restart."));
	delay(500);
	ESP.restart();
	// should not be reached
	while (true)
	{
		yield();
	}
}

static void webserver_config()
{

	if (WiFi.getMode() == WIFI_MODE_AP)
	{
		debug_outln_info(F("AP"));
	}

	debug_outln_info(F("ws: config page ..."));

	server.sendHeader(F("Cache-Control"), F("no-cache, no-store, must-revalidate"));
	server.sendHeader(F("Pragma"), F("no-cache"));
	server.sendHeader(F("Expires"), F("0"));
	// Enable Pagination (Chunked Transfer)
	server.setContentLength(CONTENT_LENGTH_UNKNOWN);

	RESERVE_STRING(page_content, XLARGE_STR);

	start_html_page(page_content, FPSTR(INTL_CONFIGURATION));

	if (server.method() == HTTP_GET)
	{
		webserver_config_send_body_get(page_content);
	}
	else
	{
		webserver_config_send_body_post(page_content);
	}
	end_html_page(page_content);

	if (server.method() == HTTP_POST)
	{
		if (writeConfig())
		{
			sensor_restart();
		}
	}
}

/*****************************************************************
 * Webserver root: show latest values                            *
 *****************************************************************/
static void webserver_values()
{

	if (WiFi.status() != WL_CONNECTED)
	{
		sendHttpRedirect();
		return;
	}

	RESERVE_STRING(page_content, XLARGE_STR);
	start_html_page(page_content, FPSTR(INTL_CURRENT_DATA));
	const String unit_Deg("°");
	const String unit_P("hPa");
	const String unit_T("°C");
	const String unit_CO2("ppm");
	const String unit_COV("ppb");
	const String unit_NC();
	const String unit_LA(F("dB(A)"));
	float dew_point_temp;

	const int signal_quality_wifi = calcWiFiSignalQuality(last_signal_strength_wifi);
	const int signal_quality_nbiot = calcNBIoTSignalQuality(last_signal_strength_nbiot);
	const int signal_quality_lorawan = calcLoRaWANSignalQuality(last_signal_strength_lorawan);
	debug_outln_info(F("ws: values ..."));
	if (!count_sends)
	{
		page_content += F("<b style='color:red'>");
		add_warning_first_cycle(page_content);
		page_content += FPSTR(WEB_B_BR_BR);
	}
	else
	{
		add_age_last_values(page_content);
	}

	auto add_table_pm_value = [&page_content](const __FlashStringHelper *sensor, const __FlashStringHelper *param, const float &value)
	{
		add_table_row_from_value(page_content, sensor, param, check_display_value(value, -1, 1, 0), F("µg/m³"));
	};

	auto add_table_nc_value = [&page_content](const __FlashStringHelper *sensor, const __FlashStringHelper *param, const float value)
	{
		add_table_row_from_value(page_content, sensor, param, check_display_value(value, -1, 1, 0), F("#/L"));
	};

	auto add_table_t_value = [&page_content](const __FlashStringHelper *sensor, const __FlashStringHelper *param, const float value)
	{
		add_table_row_from_value(page_content, sensor, param, check_display_value(value, -128, 1, 0), "°C");
	};

	auto add_table_h_value = [&page_content](const __FlashStringHelper *sensor, const __FlashStringHelper *param, const float value)
	{
		add_table_row_from_value(page_content, sensor, param, check_display_value(value, -1, 1, 0), "%");
	};

	auto add_table_voc_value = [&page_content](const __FlashStringHelper *sensor, const __FlashStringHelper *param, const float &value)
	{
		add_table_row_from_value(page_content, sensor, param, check_display_value(value, -1, 1, 0).substring(0, check_display_value(value, -1, 1, 0).indexOf(".")), "ppb"); //remove after .
	};

	auto add_table_no2_value = [&page_content](const __FlashStringHelper *sensor, const __FlashStringHelper *param, const float &value)
	{
		add_table_row_from_value(page_content, sensor, param, check_display_value(value, -1, 1, 0).substring(0, check_display_value(value, -1, 1, 0).indexOf(".")), "μg/m3");
	};

	auto add_table_value = [&page_content](const __FlashStringHelper *sensor, const __FlashStringHelper *param, const String &value, const String &unit)
	{
		add_table_row_from_value(page_content, sensor, param, value, unit);
	};

	server.sendContent(page_content);
	page_content = F("<table cellspacing='0' cellpadding='5' class='v'>\n"
					 "<thead><tr><th>" INTL_SENSOR "</th><th> " INTL_PARAMETER "</th><th>" INTL_VALUE "</th></tr></thead>");

	if (cfg::npm_read)
	{
		add_table_pm_value(FPSTR(SENSORS_NPM), FPSTR(WEB_PM1), last_value_NPM_P0);
		add_table_pm_value(FPSTR(SENSORS_NPM), FPSTR(WEB_PM25), last_value_NPM_P2);
		add_table_pm_value(FPSTR(SENSORS_NPM), FPSTR(WEB_PM10), last_value_NPM_P1);
		add_table_nc_value(FPSTR(SENSORS_NPM), FPSTR(WEB_NC1k0), last_value_NPM_N1);
		add_table_nc_value(FPSTR(SENSORS_NPM), FPSTR(WEB_NC2k5), last_value_NPM_N25);
		add_table_nc_value(FPSTR(SENSORS_NPM), FPSTR(WEB_NC10), last_value_NPM_N10);
		page_content += FPSTR(EMPTY_ROW);
	}

	if (cfg::bmx280_read)
	{
		const char *const sensor_name = (bmx280.sensorID() == BME280_SENSOR_ID) ? SENSORS_BME280 : SENSORS_BMP280;
		add_table_t_value(FPSTR(sensor_name), FPSTR(INTL_TEMPERATURE), last_value_BMX280_T);
		add_table_value(FPSTR(sensor_name), FPSTR(INTL_PRESSURE), check_display_value(last_value_BMX280_P / 100.0f, (-1 / 100.0f), 2, 0), unit_P);
		add_table_value(FPSTR(sensor_name), FPSTR(INTL_PRESSURE_AT_SEALEVEL), last_value_BMX280_P != -1.0f ? String(pressure_at_sealevel(last_value_BMX280_T, last_value_BMX280_P / 100.0f), 2) : "-", unit_P);
		if (bmx280.sensorID() == BME280_SENSOR_ID)
		{
			add_table_h_value(FPSTR(sensor_name), FPSTR(INTL_HUMIDITY), last_value_BME280_H);
			dew_point_temp = dew_point(last_value_BMX280_T, last_value_BME280_H);
			add_table_value(FPSTR(sensor_name), FPSTR(INTL_DEW_POINT), isnan(dew_point_temp) ? "-" : String(dew_point_temp, 1), unit_T);
		}
		page_content += FPSTR(EMPTY_ROW);
	}

	if (cfg::ccs811_read)
	{
		const char *const sensor_name = SENSORS_CCS811;
		add_table_voc_value(FPSTR(sensor_name), FPSTR(INTL_VOC), last_value_CCS811);
		page_content += FPSTR(EMPTY_ROW);
	}

	if (cfg::enveano2_read)
	{
		const char *const sensor_name = SENSORS_ENVEANO2;
		add_table_no2_value(FPSTR(sensor_name), FPSTR(INTL_NO2), last_value_no2);
		page_content += FPSTR(EMPTY_ROW);
	}

	server.sendContent(page_content);
	page_content = emptyString;

	page_content += FPSTR(TABLE_TAG_CLOSE_BR);
	page_content += FPSTR(BR_TAG);
	end_html_page(page_content);
}

/*****************************************************************
 * Webserver root: show device status
 *****************************************************************/
static void webserver_status()
{
	if (WiFi.status() != WL_CONNECTED)
	{
		sendHttpRedirect();
		return;
	}

	RESERVE_STRING(page_content, XLARGE_STR);
	start_html_page(page_content, FPSTR(INTL_DEVICE_STATUS));

	debug_outln_info(F("ws: status ..."));
	server.sendContent(page_content);
	page_content = F("<table cellspacing='0' cellpadding='5' class='v'>\n"
					 "<thead><tr><th> " INTL_PARAMETER "</th><th>" INTL_VALUE "</th></tr></thead>");
	String versionHtml(SOFTWARE_VERSION);
	versionHtml += F("/ST:");
	versionHtml += String(!mobileair_selftest_failed);
	versionHtml += '/';
	versionHtml.replace("/", FPSTR(BR_TAG));
	add_table_row_from_value(page_content, FPSTR(INTL_FIRMWARE), versionHtml);
	add_table_row_from_value(page_content, F("Free Memory"), String(ESP.getFreeHeap()));
	time_t now = time(nullptr);
	add_table_row_from_value(page_content, FPSTR(INTL_TIME_UTC), ctime(&now));
	add_table_row_from_value(page_content, F("Uptime"), delayToString(millis() - time_point_device_start_ms));

	if (cfg::npm_read)
	{
		page_content += FPSTR(EMPTY_ROW);
		add_table_row_from_value(page_content, FPSTR(SENSORS_NPM), last_value_NPM_version);
	}
	page_content += FPSTR(EMPTY_ROW);

	if (cfg::npm_read)
	{
		add_table_row_from_value(page_content, FPSTR(SENSORS_NPM), String(NPM_error_count));
	}
	if (cfg::ccs811_read)
	{
		add_table_row_from_value(page_content, FPSTR(SENSORS_CCS811), String(CCS811_error_count));
	}
	if (cfg::enveano2_read)
	{
		add_table_row_from_value(page_content, FPSTR(SENSORS_ENVEANO2), String(Cairsens_error_count));
	}
	server.sendContent(page_content);
	page_content = emptyString;

	if (count_sends > 0)
	{
		page_content += FPSTR(EMPTY_ROW);
		add_table_row_from_value(page_content, F(INTL_NUMBER_OF_MEASUREMENTS), String(count_sends));
		if (sending_time > 0)
		{
			add_table_row_from_value(page_content, F(INTL_TIME_SENDING_MS), String(sending_time), "ms");
		}
	}

	page_content += FPSTR(TABLE_TAG_CLOSE_BR);
	end_html_page(page_content);
}

/*****************************************************************
 * Webserver read serial ring buffer                             *
 *****************************************************************/
static void webserver_serial()
{
	String s(Debug.popLines());

	server.send(s.length() ? 200 : 204, FPSTR(TXT_CONTENT_TYPE_TEXT_PLAIN), s);
}

/*****************************************************************
 * Webserver set debug level                                     *
 *****************************************************************/
static void webserver_debug_level()
{

	RESERVE_STRING(page_content, LARGE_STR);
	start_html_page(page_content, FPSTR(INTL_DEBUG_LEVEL));

	if (server.hasArg("lvl"))
	{
		debug_outln_info(F("ws: debug level ..."));

		const int lvl = server.arg("lvl").toInt();
		if (lvl >= 0 && lvl <= 5)
		{
			cfg::debug = lvl;
			page_content += F("<h3>");
			page_content += FPSTR(INTL_DEBUG_SETTING_TO);
			page_content += ' ';

			const __FlashStringHelper *lvlText;
			switch (lvl)
			{
			case DEBUG_ERROR:
				lvlText = F(INTL_ERROR);
				break;
			case DEBUG_WARNING:
				lvlText = F(INTL_WARNING);
				break;
			case DEBUG_MIN_INFO:
				lvlText = F(INTL_MIN_INFO);
				break;
			case DEBUG_MED_INFO:
				lvlText = F(INTL_MED_INFO);
				break;
			case DEBUG_MAX_INFO:
				lvlText = F(INTL_MAX_INFO);
				break;
			default:
				lvlText = F(INTL_NONE);
			}

			page_content += lvlText;
			page_content += F(".</h3>");
		}
	}

	page_content += F("<br/><pre id='slog' class='panels'>");
	page_content += Debug.popLines();
	page_content += F("</pre>");
	page_content += F("<script>"
					  "function slog_update() {"
					  "fetch('/serial').then(r => r.text()).then((r) => {"
					  "document.getElementById('slog').innerText += r;}).catch(err => console.log(err));};"
					  "setInterval(slog_update, 3000);"
					  "</script>");
	page_content += F("<h4>");
	page_content += FPSTR(INTL_DEBUG_SETTING_TO);
	page_content += F("</h4>"
					  "<table style='width:100%;'>"
					  "<tr><td style='width:25%;'><a class='b' href='/debug?lvl=0'>" INTL_NONE "</a></td>"
					  "<td style='width:25%;'><a class='b' href='/debug?lvl=1'>" INTL_ERROR "</a></td>"
					  "<td style='width:25%;'><a class='b' href='/debug?lvl=3'>" INTL_MIN_INFO "</a></td>"
					  "<td style='width:25%;'><a class='b' href='/debug?lvl=5'>" INTL_MAX_INFO "</a></td>"
					  "</tr><tr>"
					  "</tr>"
					  "</table>");

	end_html_page(page_content);
}

/*****************************************************************
 * Webserver remove config                                       *
 *****************************************************************/
static void webserver_removeConfig()
{
	RESERVE_STRING(page_content, LARGE_STR);
	start_html_page(page_content, FPSTR(INTL_DELETE_CONFIG));
	debug_outln_info(F("ws: removeConfig ..."));

	if (server.method() == HTTP_GET)
	{
		page_content += FPSTR(WEB_REMOVE_CONFIG_CONTENT);
	}
	else
	{

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
		// Silently remove the desaster backup
		SPIFFS.remove(F("/config.json.old"));
		if (SPIFFS.exists(F("/config.json")))
		{ // file exists
			debug_outln_info(F("removing config.json..."));
			if (SPIFFS.remove(F("/config.json")))
			{
				page_content += F("<h3>" INTL_CONFIG_DELETED ".</h3>");
			}
			else
			{
				page_content += F("<h3>" INTL_CONFIG_CAN_NOT_BE_DELETED ".</h3>");
			}
		}
		else
		{
			page_content += F("<h3>" INTL_CONFIG_NOT_FOUND ".</h3>");
		}
#pragma GCC diagnostic pop
	}
	end_html_page(page_content);
}

/*****************************************************************
 * Webserver reset NodeMCU                                       *
 *****************************************************************/
static void webserver_reset()
{
	String page_content;
	page_content.reserve(512);

	start_html_page(page_content, FPSTR(INTL_RESTART_SENSOR));
	debug_outln_info(F("ws: reset ..."));

	if (server.method() == HTTP_GET)
	{
		page_content += FPSTR(WEB_RESET_CONTENT);
	}
	else
	{

		sensor_restart();
	}
	end_html_page(page_content);
}

/*****************************************************************
 * Webserver data.json                                           *
 *****************************************************************/
static void webserver_data_json()
{
	String s1;
	unsigned long age = 0;

	debug_outln_info(F("ws: data json..."));
	if (!count_sends)
	{
		s1 = FPSTR(data_first_part);
		s1 += "]}";
		age = cfg::sending_intervall_ms - msSince(starttime);
		if (age > cfg::sending_intervall_ms)
		{
			age = 0;
		}
		age = 0 - age;
	}
	else
	{
		s1 = last_datajson_string;
		age = msSince(starttime);
		if (age > cfg::sending_intervall_ms)
		{
			age = 0;
		}
	}
	String s2 = F(", \"age\":\"");
	s2 += String((long)((age + 500) / 1000));
	s2 += F("\", \"sensordatavalues\"");
	s1.replace(F(", \"sensordatavalues\""), s2);
	server.send(200, FPSTR(TXT_CONTENT_TYPE_JSON), s1);
}

/*****************************************************************
 * Webserver metrics endpoint                                    *
 *****************************************************************/
static void webserver_metrics_endpoint()
{
	debug_outln_info(F("ws: /metrics"));
	RESERVE_STRING(page_content, XLARGE_STR);
	page_content = F("software_version{version=\"" SOFTWARE_VERSION_STR "\",$i} 1\nuptime_ms{$i} $u\nsending_intervall_ms{$i} $s\nnumber_of_measurements{$i} $c\n");
	String id(F("node=\"" SENSOR_BASENAME));
	id += esp_chipid;
	id += '\"';
	page_content.replace("$i", id);
	page_content.replace("$u", String(msSince(time_point_device_start_ms)));
	page_content.replace("$s", String(cfg::sending_intervall_ms));
	page_content.replace("$c", String(count_sends));
	DynamicJsonDocument json2data(JSON_BUFFER_SIZE);
	DeserializationError err = deserializeJson(json2data, last_datajson_string);
	if (!err)
	{
		for (JsonObject measurement : json2data[FPSTR(JSON_SENSOR_DATA_VALUES)].as<JsonArray>())
		{
			page_content += measurement["value_type"].as<const char *>();
			page_content += '{';
			page_content += id;
			page_content += "} ";
			page_content += measurement["value"].as<const char *>();
			page_content += '\n';
		}
		page_content += F("last_sample_age_ms{");
		page_content += id;
		page_content += "} ";
		page_content += String(msSince(starttime));
		page_content += '\n';
	}
	else
	{
		debug_outln_error(FPSTR(DBG_TXT_DATA_READ_FAILED));
	}
	page_content += F("# EOF\n");
	debug_outln(page_content, DEBUG_MED_INFO);
	server.send(200, FPSTR(TXT_CONTENT_TYPE_TEXT_PLAIN), page_content);
}

/*****************************************************************
 * Webserver Images                                              *
 *****************************************************************/

static void webserver_favicon()
{
	server.sendHeader(F("Cache-Control"), F("max-age=2592000, public"));

	server.send_P(200, TXT_CONTENT_TYPE_IMAGE_PNG,
				  AIRCARTO_INFO_LOGO_PNG, AIRCARTO_INFO_LOGO_PNG_SIZE);
}

/*****************************************************************
 * Webserver page not found                                      *
 *****************************************************************/
static void webserver_not_found()
{
	last_page_load = millis();
	debug_outln_info(F("ws: not found ..."));

	if (WiFi.status() != WL_CONNECTED)
	{
		if ((server.uri().indexOf(F("success.html")) != -1) || (server.uri().indexOf(F("detect.html")) != -1))
		{
			server.send(200, FPSTR(TXT_CONTENT_TYPE_TEXT_HTML), FPSTR(WEB_IOS_REDIRECT));
		}
		else
		{
			sendHttpRedirect();
		}
	}
	else
	{
		server.send(404, FPSTR(TXT_CONTENT_TYPE_TEXT_PLAIN), F("Not found."));
	}
}

static void webserver_static()
{
	server.sendHeader(F("Cache-Control"), F("max-age=2592000, public"));

	if (server.arg(String('r')) == F("logo"))
	{

		server.send_P(200, TXT_CONTENT_TYPE_IMAGE_PNG,
					  AIRCARTO_INFO_LOGO_PNG, AIRCARTO_INFO_LOGO_PNG_SIZE);
	}
	else if (server.arg(String('r')) == F("css"))
	{
		server.send_P(200, TXT_CONTENT_TYPE_TEXT_CSS,
					  WEB_PAGE_STATIC_CSS, sizeof(WEB_PAGE_STATIC_CSS) - 1);
	}
	else
	{
		webserver_not_found();
	}
}

/*****************************************************************
 * Webserver setup                                               *
 *****************************************************************/
static void setup_webserver()
{
	server.on("/", webserver_root);
	server.on(F("/config"), webserver_config);
	server.on(F("/values"), webserver_values);
	server.on(F("/status"), webserver_status);
	server.on(F("/generate_204"), webserver_config);
	server.on(F("/fwlink"), webserver_config);
	server.on(F("/debug"), webserver_debug_level);
	server.on(F("/serial"), webserver_serial);
	server.on(F("/removeConfig"), webserver_removeConfig);
	server.on(F("/reset"), webserver_reset);
	server.on(F("/data.json"), webserver_data_json);
	server.on(F("/metrics"), webserver_metrics_endpoint);
	server.on(F("/favicon.ico"), webserver_favicon);
	server.on(F(STATIC_PREFIX), webserver_static);
	server.onNotFound(webserver_not_found);
	debug_outln_info(F("Starting Webserver... "));
	server.begin();
}

static int selectChannelForAp()
{
	std::array<int, 14> channels_rssi;
	std::fill(channels_rssi.begin(), channels_rssi.end(), -100);

	for (unsigned i = 0; i < std::min((uint8_t)14, count_wifiInfo); i++)
	{
		if (wifiInfo[i].RSSI > channels_rssi[wifiInfo[i].channel])
		{
			channels_rssi[wifiInfo[i].channel] = wifiInfo[i].RSSI;
		}
	}

	if ((channels_rssi[1] < channels_rssi[6]) && (channels_rssi[1] < channels_rssi[11]))
	{
		return 1;
	}
	else if ((channels_rssi[6] < channels_rssi[1]) && (channels_rssi[6] < channels_rssi[11]))
	{
		return 6;
	}
	else
	{
		return 11;
	}
}

/*****************************************************************
 * WifiConfig                                                    *
 *****************************************************************/

static void wifiConfig()
{
	if (cfg::has_led_value)
	{
		if (LEDS_NB == 1)
		{
			leds[0] = colorLED_empty;
			FastLED.show();
			leds[0] = colorLED_wifi;
			FastLED.show();
		}
		else
		{
			if (LEDS_MATRIX)
			{
				drawpicture(wifi);
				FastLED.show();
			}
			else
			{
				fill_solid(leds, LEDS_NB, colorLED_empty);
				FastLED.show();
				fill_solid(leds, LEDS_NB, colorLED_wifi);
				FastLED.show();
			}
		}
	}

	debug_outln_info(F("Starting WiFiManager"));
	debug_outln_info(F("AP ID: "), String(cfg::fs_ssid));
	debug_outln_info(F("Password: "), String(cfg::fs_pwd));

	WiFi.disconnect(true, true);

	debug_outln_info(F("scan for wifi networks..."));
	int8_t scanReturnCode = WiFi.scanNetworks(false /* scan async */, true /* show hidden networks */);
	if (scanReturnCode < 0)
	{
		debug_outln_error(F("WiFi scan failed. Treating as empty. "));
		count_wifiInfo = 0;
	}
	else
	{
		count_wifiInfo = (uint8_t)scanReturnCode;
	}

	delete[] wifiInfo;
	wifiInfo = new struct_wifiInfo[std::max(count_wifiInfo, (uint8_t)1)];

	for (unsigned i = 0; i < count_wifiInfo; i++)
	{
		String SSID;
		uint8_t *BSSID;

		memset(&wifiInfo[i], 0, sizeof(struct_wifiInfo));
		WiFi.getNetworkInfo(i, SSID, wifiInfo[i].encryptionType, wifiInfo[i].RSSI, BSSID, wifiInfo[i].channel);
		SSID.toCharArray(wifiInfo[i].ssid, sizeof(wifiInfo[0].ssid));
	}

	// Use 13 channels if locale is not "EN"
	wifi_country_t wifi;
	wifi.policy = WIFI_COUNTRY_POLICY_MANUAL;
	strcpy(wifi.cc, INTL_LANG);
	wifi.nchan = (INTL_LANG[0] == 'E' && INTL_LANG[1] == 'N') ? 11 : 13;
	wifi.schan = 1;

	WiFi.mode(WIFI_AP);
	const IPAddress apIP(192, 168, 4, 1);
	WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
	WiFi.softAP(cfg::fs_ssid, cfg::fs_pwd, selectChannelForAp());
	// In case we create a unique password at first start
	debug_outln_info(F("AP Password is: "), cfg::fs_pwd);

	DNSServer dnsServer;
	dnsServer.setTTL(0);
	dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
	dnsServer.start(53, "*", apIP); // 53 is port for DNS server

	setup_webserver();

	// X minutes timeout for wifi config
	last_page_load = millis();

	while ((millis() - last_page_load) < cfg::time_for_wifi_config + 500)
	{
		dnsServer.processNextRequest();
		server.handleClient();
		yield();
	}

	//restart in loop

	if (cfg::has_led_value)
	{
		if (LEDS_NB == 1)
		{
			leds[0] = colorLED_empty;
			FastLED.show();
		}
		else
		{
			if (LEDS_MATRIX)
			{
				drawpicture(empty);
				FastLED.show();
			}
			else
			{
			}
		}
	}

	WiFi.softAPdisconnect(true);
	dnsServer.stop(); // A VOIR
	delay(100);
	// WiFi.disconnect(true, true);
	WiFi.mode(WIFI_OFF); //A tenter

	debug_outln_info(F("---- Result Webconfig ----"));
	debug_outln_info(FPSTR(DBG_TXT_SEP));
	debug_outln_info_bool(F("NPM: "), cfg::npm_read);
	debug_outln_info_bool(F("BMX: "), cfg::bmx280_read);
	debug_outln_info_bool(F("CCS811: "), cfg::ccs811_read);
	debug_outln_info(FPSTR(DBG_TXT_SEP));
	debug_outln_info_bool(F("CSV: "), cfg::send2csv);
	debug_outln_info(FPSTR(DBG_TXT_SEP));
	debug_outln_info_bool(F("LED value: "), cfg::has_led_value);
	debug_outln_info(F("Debug: "), String(cfg::debug));

	//ENVEA + SD
}

/*****************************************************************
 * send data as csv to serial out                                *
 *****************************************************************/
static void send_csv(const String &data)
{
	DynamicJsonDocument json2data(JSON_BUFFER_SIZE);
	DeserializationError err = deserializeJson(json2data, data);
	debug_outln_info(F("CSV Output: "), data);
	if (!err)
	{
		String headline = F("Timestamp_ms;");
		String valueline(act_milli);
		valueline += ';';
		for (JsonObject measurement : json2data[FPSTR(JSON_SENSOR_DATA_VALUES)].as<JsonArray>())
		{
			headline += measurement["value_type"].as<const char *>();
			headline += ';';
			valueline += measurement["value"].as<const char *>();
			valueline += ';';
		}
		static bool first_csv_line = true;
		if (first_csv_line)
		{
			if (headline.length() > 0)
			{
				headline.remove(headline.length() - 1);
			}
			Debug.println(headline);
			first_csv_line = false;
		}
		if (valueline.length() > 0)
		{
			valueline.remove(valueline.length() - 1);
		}
		Debug.println(valueline);
	}
	else
	{
		debug_outln_error(FPSTR(DBG_TXT_DATA_READ_FAILED));
	}
}

/*****************************************************************
 * read BMP280/BME280 sensor values                              *
 *****************************************************************/
static void fetchSensorBMX280(String &s)
{
	const char *const sensor_name = (bmx280.sensorID() == BME280_SENSOR_ID) ? SENSORS_BME280 : SENSORS_BMP280;
	debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(sensor_name));

	bmx280.takeForcedMeasurement();
	const auto t = bmx280.readTemperature();
	const auto p = bmx280.readPressure();
	const auto h = bmx280.readHumidity();
	if (isnan(t) || isnan(p))
	{
		last_value_BMX280_T = -128.0;
		last_value_BMX280_P = -1.0;
		last_value_BME280_H = -1.0;
		debug_outln_error(F("BMP/BME280 read failed"));
	}
	else
	{
		last_value_BMX280_T = t;
		last_value_BMX280_P = p;
		if (bmx280.sensorID() == BME280_SENSOR_ID)
		{
			add_Value2Json(s, F("BME280_temperature"), FPSTR(DBG_TXT_TEMPERATURE), last_value_BMX280_T);
			add_Value2Json(s, F("BME280_pressure"), FPSTR(DBG_TXT_PRESSURE), last_value_BMX280_P);
			last_value_BME280_H = h;
			add_Value2Json(s, F("BME280_humidity"), FPSTR(DBG_TXT_HUMIDITY), last_value_BME280_H);
		}
		else
		{
			add_Value2Json(s, F("BMP280_pressure"), FPSTR(DBG_TXT_PRESSURE), last_value_BMX280_P);
			add_Value2Json(s, F("BMP280_temperature"), FPSTR(DBG_TXT_TEMPERATURE), last_value_BMX280_T);
		}
	}
	debug_outln_info(FPSTR(DBG_TXT_SEP));
	debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(sensor_name));
}

/*****************************************************************
 * read CCS811 sensor values                              *
 *****************************************************************/
static void fetchSensorCCS811(String &s)
{
	const char *const sensor_name = SENSORS_CCS811;
	debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(sensor_name));

	uint16_t etvoc, errstat;
	ccs811.read(NULL, &etvoc, &errstat, NULL);

	if (errstat == CCS811_ERRSTAT_OK)
	{

		ccs811_sum += etvoc;
		ccs811_val_count++;
		debug_outln(String(ccs811_val_count), DEBUG_MAX_INFO);
	}
	else if (errstat == CCS811_ERRSTAT_OK_NODATA)
	{
		Debug.println("CCS811: waiting for (new) data");
	}
	else if (errstat & CCS811_ERRSTAT_I2CFAIL)
	{
		Debug.println("CCS811: I2C error");
	}
	else
	{
		Debug.print("CCS811: errstat=");
		Debug.print("errstat,HEX");
		Debug.print("=");
		Debug.println(ccs811.errstat_str(errstat));
	}

	if (send_now && cfg::sending_intervall_ms == 120000)
	{
		last_value_CCS811 = -1.0f;

		if (ccs811_val_count >= 12)
		{
			last_value_CCS811 = float(ccs811_sum / ccs811_val_count);
			add_Value2Json(s, F("CCS811_VOC"), FPSTR(DBG_TXT_VOCPPB), last_value_CCS811);
			debug_outln_info(FPSTR(DBG_TXT_SEP));
		}
		else
		{
			CCS811_error_count++;
		}

		ccs811_sum = 0;
		ccs811_val_count = 0;
	}

	debug_outln_info(FPSTR(DBG_TXT_SEP));
	debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(sensor_name));
}

/*****************************************************************
 * read Tera Sensor Next PM sensor sensor values                 *
 *****************************************************************/
static void fetchSensorNPM(String &s)
{
	NPM_waiting_for_16 = NPM_REPLY_HEADER_16;

	debug_outln_info(F("Concentration NPM..."));
	NPM_cmd(PmSensorCmd2::Concentration);

	unsigned long timeout = millis();

	do
	{
		debug_outln("Wait for Serial...", DEBUG_MAX_INFO);
	} while (!serialNPM.available() && millis() - timeout < 3000);

	while (serialNPM.available() >= NPM_waiting_for_16)
	{
		const uint8_t constexpr header[2] = {0x81, 0x12};
		uint8_t state[1];
		uint8_t data[12];
		uint8_t checksum[1];
		uint8_t test[16];
		uint16_t N1_serial;
		uint16_t N25_serial;
		uint16_t N10_serial;
		uint16_t pm1_serial;
		uint16_t pm25_serial;
		uint16_t pm10_serial;

		switch (NPM_waiting_for_16)
		{
		case NPM_REPLY_HEADER_16:
			if (serialNPM.find(header, sizeof(header)))
				NPM_waiting_for_16 = NPM_REPLY_STATE_16;
			break;
		case NPM_REPLY_STATE_16:
			serialNPM.readBytes(state, sizeof(state));
			current_state_npm = NPM_state(state[0]);
			NPM_waiting_for_16 = NPM_REPLY_BODY_16;
			break;
		case NPM_REPLY_BODY_16:
			if (serialNPM.readBytes(data, sizeof(data)) == sizeof(data))
			{
				NPM_data_reader(data, 12);
				N1_serial = word(data[0], data[1]);
				N25_serial = word(data[2], data[3]);
				N10_serial = word(data[4], data[5]);

				pm1_serial = word(data[6], data[7]);
				pm25_serial = word(data[8], data[9]);
				pm10_serial = word(data[10], data[11]);

				debug_outln_info(F("Next PM Measure..."));

				debug_outln_verbose(F("PM1 (μg/m3) : "), String(pm1_serial / 10.0f));
				debug_outln_verbose(F("PM2.5 (μg/m3): "), String(pm25_serial / 10.0f));
				debug_outln_verbose(F("PM10 (μg/m3) : "), String(pm10_serial / 10.0f));

				debug_outln_verbose(F("PM1 (pcs/L) : "), String(N1_serial));
				debug_outln_verbose(F("PM2.5 (pcs/L): "), String(N25_serial));
				debug_outln_verbose(F("PM10 (pcs/L) : "), String(N10_serial));
			}
			NPM_waiting_for_16 = NPM_REPLY_CHECKSUM_16;
			break;
		case NPM_REPLY_CHECKSUM_16:
			serialNPM.readBytes(checksum, sizeof(checksum));
			memcpy(test, header, sizeof(header));
			memcpy(&test[sizeof(header)], state, sizeof(state));
			memcpy(&test[sizeof(header) + sizeof(state)], data, sizeof(data));
			memcpy(&test[sizeof(header) + sizeof(state) + sizeof(data)], checksum, sizeof(checksum));
			NPM_data_reader(test, 16);
			if (NPM_checksum_valid_16(test))
			{
				debug_outln_info(F("Checksum OK..."));

				npm_pm1_sum += pm1_serial;
				npm_pm25_sum += pm25_serial;
				npm_pm10_sum += pm10_serial;

				npm_pm1_sum_pcs += N1_serial;
				npm_pm25_sum_pcs += N25_serial;
				npm_pm10_sum_pcs += N10_serial;
				npm_val_count++;
				debug_outln(String(npm_val_count), DEBUG_MAX_INFO);
			}
			NPM_waiting_for_16 = NPM_REPLY_HEADER_16;
			break;
		}
	}

	if (send_now && cfg::sending_intervall_ms >= 120000)
	{
		last_value_NPM_P0 = -1.0f;
		last_value_NPM_P1 = -1.0f;
		last_value_NPM_P2 = -1.0f;
		last_value_NPM_N1 = -1.0f;
		last_value_NPM_N10 = -1.0f;
		last_value_NPM_N25 = -1.0f;

		if (npm_val_count == 2)
		{
			last_value_NPM_P0 = float(npm_pm1_sum) / (npm_val_count * 10.0f);
			last_value_NPM_P1 = float(npm_pm10_sum) / (npm_val_count * 10.0f);
			last_value_NPM_P2 = float(npm_pm25_sum) / (npm_val_count * 10.0f);

			last_value_NPM_N1 = float(npm_pm1_sum_pcs) / (npm_val_count); //enlevé * 1000.0f pour Litre
			last_value_NPM_N10 = float(npm_pm10_sum_pcs) / (npm_val_count);
			last_value_NPM_N25 = float(npm_pm25_sum_pcs) / (npm_val_count);

			add_Value2Json(s, F("NPM_P0"), F("PM1: "), last_value_NPM_P0);
			add_Value2Json(s, F("NPM_P1"), F("PM10:  "), last_value_NPM_P1);
			add_Value2Json(s, F("NPM_P2"), F("PM2.5: "), last_value_NPM_P2);

			add_Value2Json(s, F("NPM_N1"), F("NC1.0: "), last_value_NPM_N1);
			add_Value2Json(s, F("NPM_N10"), F("NC10:  "), last_value_NPM_N10);
			add_Value2Json(s, F("NPM_N25"), F("NC2.5: "), last_value_NPM_N25);

			debug_outln_info(FPSTR(DBG_TXT_SEP));
		}
		else
		{
			NPM_error_count++;
		}

		npm_pm1_sum = 0;
		npm_pm10_sum = 0;
		npm_pm25_sum = 0;

		npm_val_count = 0;

		npm_pm1_sum_pcs = 0;
		npm_pm10_sum_pcs = 0;
		npm_pm25_sum_pcs = 0;

		debug_outln_info(F("Temperature and humidity in NPM after measure..."));
		current_th_npm = NPM_temp_humi();
	}
}

/*****************************************************************
 * read Cairsens sensor values                              *
 *****************************************************************/
static void fetchSensorCairsens(String &s)
{
	const char *const sensor_name = SENSORS_ENVEANO2;
	debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(sensor_name));

	uint8_t no2_val = 0;

	if (cairsens.getNO2InstantVal(no2_val) == CairsensUART::NO_ERROR)
	{
		no2_sum += no2_val;
		no2_val_count++;
		debug_outln(String(no2_val_count), DEBUG_MAX_INFO);
	}
	else
	{
		Debug.println("Could not get Cairsens NOX value");
	}

	if (send_now && cfg::sending_intervall_ms == 120000)
	{
		last_value_no2 = -1.0f;

		if (no2_val_count >= 12)
		{
			last_value_no2 = CairsensUART::ppbToPpm(CairsensUART::NO2, float(no2_sum / no2_val_count));
			add_Value2Json(s, F("Cairsens_NO2"), FPSTR(DBG_TXT_NO2PPB), last_value_no2);
			debug_outln_info(FPSTR(DBG_TXT_SEP));
		}
		else
		{
			Cairsens_error_count++;
		}

		no2_sum = 0;
		no2_val_count = 0;
	}

	debug_outln_info(FPSTR(DBG_TXT_SEP));
	debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(sensor_name));
}

/*****************************************************************
 * display values                                                *
 *****************************************************************/
static void display_values_oled() //COMPLETER LES ECRANS
{
	float t_value = -128.0;
	float h_value = -1.0;
	float p_value = -1.0;

	String t_sensor, h_sensor, p_sensor;

	float pm01_value = -1.0;
	float pm25_value = -1.0;
	float pm10_value = -1.0;

	String pm01_sensor, pm10_sensor, pm25_sensor;

	float nc010_value = -1.0;
	float nc025_value = -1.0;
	float nc100_value = -1.0;

	String cov_sensor, no2_sensor;

	float no2_value = -1.0;
	float cov_value = -1.0;

	double lat_value = -200.0;
	double lon_value = -200.0;
	double alt_value = -1000.0;

	String display_header;
	String display_lines[3] = {"", "", ""};

	uint8_t screen_count = 0;
	uint8_t screens[7];
	int line_count = 0;
	debug_outln_info(F("output values to display..."));

	if (cfg::npm_read)
	{
		pm01_value = last_value_NPM_P0;
		pm10_value = last_value_NPM_P1;
		pm25_value = last_value_NPM_P2;
		pm01_sensor = FPSTR(SENSORS_NPM);
		pm10_sensor = FPSTR(SENSORS_NPM);
		pm25_sensor = FPSTR(SENSORS_NPM);
		nc010_value = last_value_NPM_N1;
		nc100_value = last_value_NPM_N10;
		nc025_value = last_value_NPM_N25;
	}

	if (cfg::bmx280_read)
	{
		t_sensor = p_sensor = FPSTR(SENSORS_BMP280);
		t_value = last_value_BMX280_T;
		p_value = last_value_BMX280_P;
		if (bmx280.sensorID() == BME280_SENSOR_ID)
		{
			h_sensor = t_sensor = FPSTR(SENSORS_BME280);
			h_value = last_value_BME280_H;
		}
	}

	if (cfg::ccs811_read)
	{
		cov_value = last_value_CCS811;
		cov_sensor = FPSTR(SENSORS_CCS811);
	}

	if (cfg::enveano2_read)
	{
		cov_value = last_value_no2;
		cov_sensor = FPSTR(SENSORS_ENVEANO2);
	}

	if (cfg::has_gps)
	{
	double lat_value = GPSdata.latitude;
	double lon_value = GPSdata.longitude;
	}

	if (cfg::npm_read && cfg::display_measure)
	{
		screens[screen_count++] = 0;
	}
	if (cfg::bmx280_read && cfg::display_measure)
	{
		screens[screen_count++] = 1;
	}

	if (cfg::ccs811_read && cfg::display_measure)
	{
		screens[screen_count++] = 2;
	}

	if (cfg::enveano2_read && cfg::display_measure)
	{
		screens[screen_count++] = 3;
	}

	if (cfg::has_gps && coordinates)
	{
		screens[screen_count++] = 4;
	}

	if (cfg::display_device_info)
	{
		screens[screen_count++] = 5; // chipID, firmware and count of measurements
		if (cfg::npm_read && cfg::display_measure)
		{
			screens[screen_count++] = 6; // info NPM
		}
	}

	if (file_created)
	{
		screens[screen_count++] = 7;
	}

	switch (screens[next_display_count % screen_count])
	{
	case 0:
		display_header = FPSTR(SENSORS_NPM);
		display_lines[0] = std::move(tmpl(F("PM1: {v} µg/m³"), check_display_value(pm01_value, -1, 1, 6)));
		display_lines[1] = std::move(tmpl(F("PM2.5: {v} µg/m³"), check_display_value(pm25_value, -1, 1, 6)));
		display_lines[2] = std::move(tmpl(F("PM10: {v} µg/m³"), check_display_value(pm10_value, -1, 1, 6)));
		break;
	case 1:
		display_header = t_sensor;
		if (t_sensor != "")
		{
			display_lines[line_count] = "Temp.: ";
			display_lines[line_count] += check_display_value(t_value, -128, 1, 6);
			display_lines[line_count++] += " °C";
		}
		if (h_sensor != "")
		{
			display_lines[line_count] = "Hum.:  ";
			display_lines[line_count] += check_display_value(h_value, -1, 1, 6);
			display_lines[line_count++] += " %";
		}
		if (p_sensor != "")
		{
			display_lines[line_count] = "Pres.: ";
			display_lines[line_count] += check_display_value(p_value / 100, (-1 / 100.0), 1, 6);
			display_lines[line_count++] += " hPa";
		}
		while (line_count < 3)
		{
			display_lines[line_count++] = emptyString;
		}
		break;
	case 2:
		display_header = FPSTR(SENSORS_CCS811);
		display_lines[0] = std::move(tmpl(F("COV: {v} ppb"), check_display_value(cov_value, -1, 1, 6)));
		break;
	case 4:
		display_header = FPSTR(SENSORS_ENVEANO2);
		display_lines[0] = std::move(tmpl(F("NO2: {v} µg/m³"), check_display_value(no2_value, -1, 1, 6)));
		break;
	case 5:
		display_header = F("Device Info");
		display_lines[0] = "ID: ";
		display_lines[0] += esp_chipid;
		display_lines[1] = "FW: ";
		display_lines[1] += SOFTWARE_VERSION;
		display_lines[2] = F("Measurements: ");
		display_lines[2] += String(count_sends);
		break;
	case 6:
		display_header = FPSTR(SENSORS_NPM);
		display_lines[0] = current_state_npm;
		display_lines[1] = F("T_NPM / RH_NPM");
		display_lines[2] = current_th_npm;
		break;
	case 7:
		display_header = FPSTR("SD card");
		display_lines[0] = F("Recorded:");
		display_lines[1] = String(count_recorded);
		break;
	}

	oled_ssd1306->clear();
	oled_ssd1306->displayOn();
	oled_ssd1306->setTextAlignment(TEXT_ALIGN_CENTER);
	oled_ssd1306->drawString(64, 1, display_header);
	oled_ssd1306->setTextAlignment(TEXT_ALIGN_LEFT);
	oled_ssd1306->drawString(0, 16, display_lines[0]);
	oled_ssd1306->drawString(0, 28, display_lines[1]);
	oled_ssd1306->drawString(0, 40, display_lines[2]);
	oled_ssd1306->setTextAlignment(TEXT_ALIGN_CENTER);
	oled_ssd1306->drawString(64, 52, displayGenerateFooter(screen_count));
	oled_ssd1306->display();

	yield();
	next_display_count++;
}

/*****************************************************************
 * Init LCD/OLED display                                         *
 *****************************************************************/
static void init_display()
{
	if (cfg::has_ssd1306)

	{
		oled_ssd1306 = new SSD1306Wire(0x3c, I2C_PIN_SDA, I2C_PIN_SCL);
		oled_ssd1306->init();
		oled_ssd1306->flipScreenVertically(); // ENLEVER ???
		oled_ssd1306->clear();
		oled_ssd1306->displayOn();
		oled_ssd1306->setTextAlignment(TEXT_ALIGN_CENTER);
		oled_ssd1306->drawString(64, 1, "START");
		oled_ssd1306->display();

		// reset back to 100k as the OLEDDisplay initialization is
		// modifying the I2C speed to 400k, which overwhelms some of the
		// sensors.
		Wire.setClock(100000);
		// Wire.setClockStretchLimit(150000);
	}
}

/*****************************************************************
 * Init BMP280/BME280                                            *
 *****************************************************************/
static bool initBMX280(char addr)
{
	debug_out(String(F("Trying BMx280 sensor on ")) + String(addr, HEX), DEBUG_MIN_INFO);

	if (bmx280.begin(addr))
	{
		debug_outln_info(FPSTR(DBG_TXT_FOUND));
		bmx280.setSampling(
			BMX280::MODE_FORCED,
			BMX280::SAMPLING_X1,
			BMX280::SAMPLING_X1,
			BMX280::SAMPLING_X1);
		return true;
	}
	else
	{
		debug_outln_info(FPSTR(DBG_TXT_NOT_FOUND));
		return false;
	}
}

/*****************************************************************
 * Init CCS811                                            *
 *****************************************************************/
static bool initCCS811()
{

	debug_out(String(F("Trying CCS811 sensor: ")), DEBUG_MIN_INFO);

	if (!ccs811.begin())
	{
		debug_out(String(F("CCS811 begin FAILED")), DEBUG_MIN_INFO);
		return false;
	}
	else
	{
		// Print CCS811 versions
		debug_outln_info(F("hardware version: "), ccs811.hardware_version());
		debug_outln_info(F("bootloader version: "), ccs811.bootloader_version());
		debug_outln_info(F("application version: "), ccs811.application_version());

		if (!ccs811.start(CCS811_MODE_1SEC))
		{
			debug_out(String(F("CCS811 start FAILED")), DEBUG_MIN_INFO);
			return false;
		}
		else
		{
			debug_out(String(F("CCS811 OK")), DEBUG_MIN_INFO);
			Debug.printf("\n");
			return true;
		}
	}
}

/*****************************************************************
   Functions
 *****************************************************************/

static void powerOnTestSensors()
{

	if (cfg::npm_read)
	{
		int8_t test_state;

		//AJOUTER TEST SI PAS LED

		if (cfg::has_led_value)
		{
			if (LEDS_NB == 1)
			{
				drawtimemono1();
			}
			else
			{
				if (LEDS_MATRIX)
				{
					drawtime1();
				}
				else
				{
					drawtimeline1();
				}
			}
		}
		else
		{
			delay(15000); // wait a bit to be sure Next PM is ready to receive instructions.
		}

		test_state = NPM_get_state();
		if (test_state == -1)
		{
			debug_outln_info(F("NPM not connected"));
			nextpmconnected = false;
		}
		else
		{
			nextpmconnected = true;
			if (test_state == 0x00)
			{
				debug_outln_info(F("NPM already started..."));
				nextpmconnected = true;
			}
			else if (test_state == 0x01)
			{
				debug_outln_info(F("Force start NPM...")); // to read the firmware version
				is_NPM_running = NPM_start_stop();
			}
			else
			{
				if (bitRead(test_state, 1) == 1)
				{
					debug_outln_info(F("Degraded state"));
				}
				else
				{
					debug_outln_info(F("Default state"));
				}
				if (bitRead(test_state, 2) == 1)
				{
					debug_outln_info(F("Not ready"));
				}
				if (bitRead(test_state, 3) == 1)
				{
					debug_outln_info(F("Heat error"));
				}
				if (bitRead(test_state, 4) == 1)
				{
					debug_outln_info(F("T/RH error"));
				}
				if (bitRead(test_state, 5) == 1)
				{
					debug_outln_info(F("Fan error"));

					// if (bitRead(test_state, 0) == 1){
					// 	debug_outln_info(F("Force start NPM..."));
					// 	is_NPM_running = NPM_start_stop();
					// 	delay(5000);
					// }
					// NPM_fan_speed();
					// delay(5000);
				}
				if (bitRead(test_state, 6) == 1)
				{
					debug_outln_info(F("Memory error"));
				}
				if (bitRead(test_state, 7) == 1)
				{
					debug_outln_info(F("Laser error"));
				}
				if (bitRead(test_state, 0) == 0)
				{
					debug_outln_info(F("NPM already started..."));
					is_NPM_running = true;
				}
				else
				{
					debug_outln_info(F("Force start NPM..."));
					is_NPM_running = NPM_start_stop();
				}
			}
		}

		if (nextpmconnected)
		{

			//AJOUTER TEST SI PAS LED

			if (cfg::has_led_value)
			{
				if (LEDS_NB == 1)
				{
					drawtimemono2();
				}
				else
				{
					if (LEDS_MATRIX)
					{
						drawtime2();
					}
					else
					{
						drawtimeline2();
					}
				}
			}
			else
			{
				delay(15000); // wait a bit to be sure Next PM is ready to receive instructions.
			}

			NPM_version_date();
			delay(3000);
			NPM_temp_humi();
			delay(2000);
		}
	}

	if (cfg::bmx280_read)
	{
		debug_outln_info(F("Read BMxE280..."));
		if (!initBMX280(0x76) && !initBMX280(0x77))
		{
			debug_outln_error(F("Check BMx280 wiring"));
			bmx280_init_failed = true;
		}
	}

	if (cfg::ccs811_read)
	{
		debug_outln_info(F("Read CCS811..."));
		if (!initCCS811())
		{
			debug_outln_error(F("Check CCS811 wiring"));
			ccs811_init_failed = true;
		}
	}


if (cfg::has_sdcard)
	{
		if (!SD.begin(5))
		{
			Debug.println("Card Mount Failed");
			return;
		}
		uint8_t cardType = SD.cardType();

		if (cardType == CARD_NONE)
		{
			Debug.println("No SD card attached");
			return;
		}

		Debug.print("SD Card Type: ");
		if (cardType == CARD_MMC)
		{
			Debug.println("MMC");
		}
		else if (cardType == CARD_SD)
		{
			Debug.println("SDSC");
		}
		else if (cardType == CARD_SDHC)
		{
			Debug.println("SDHC");
		}
		else
		{
			Debug.println("UNKNOWN");
		}

		uint64_t cardSize = SD.cardSize() / (1024 * 1024);
		Debug.printf("SD Card Size: %lluMB\n", cardSize);
	}







}

static void logEnabledAPIs()
{

	if (cfg::send2csv)
	{
		debug_outln_info(F("Serial as CSV"));
	}
}

/*****************************************************************
 * Check stack                                                    *
 *****************************************************************/
void *StackPtrAtStart;
void *StackPtrEnd;
UBaseType_t watermarkStart;

/*****************************************************************
 * SD card                                                  *
 *****************************************************************/

// char data_file[24];

String file_name;

static void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
	Debug.printf("Listing directory: %s\n", dirname);

	File root = fs.open(dirname);
	if (!root)
	{
		Debug.println("Failed to open directory");
		return;
	}
	if (!root.isDirectory())
	{
		Debug.println("Not a directory");
		return;
	}

	File file = root.openNextFile();
	while (file)
	{
		if (file.isDirectory())
		{
			Debug.print("  DIR : ");
			Debug.println(file.name());
			if (levels)
			{
				listDir(fs, file.name(), levels - 1);
			}
		}
		else
		{
			Debug.print("  FILE: ");
			Debug.print(file.name());
			Debug.print("  SIZE: ");
			Debug.println(file.size());
		}
		file = root.openNextFile();
	}
}

static void createDir(fs::FS &fs, const char *path)
{
	Debug.printf("Creating Dir: %s\n", path);
	if (fs.mkdir(path))
	{
		Debug.println("Dir created");
	}
	else
	{
		Debug.println("mkdir failed");
	}
}

static void removeDir(fs::FS &fs, const char *path)
{
	Debug.printf("Removing Dir: %s\n", path);
	if (fs.rmdir(path))
	{
		Debug.println("Dir removed");
	}
	else
	{
		Debug.println("rmdir failed");
	}
}

static void readFile(fs::FS &fs, const char *path)
{
	Debug.printf("Reading file: %s\n", path);

	File file = fs.open(path);
	if (!file)
	{
		Debug.println("Failed to open file for reading");
		return;
	}

	Debug.print("Read from file: ");
	while (file.available())
	{
		Serial.write(file.read());
	}
	file.close();
}

static void writeFile(fs::FS &fs, const char *path, const char *message)
{
	Debug.printf("Writing file: %s\n", path);

	File file = fs.open(path, FILE_WRITE);
	if (!file)
	{
		Debug.println("Failed to open file for writing");
		return;
	}
	if (file.print(message))
	{
		Debug.println("File written");
	}
	else
	{
		Debug.println("Write failed");
	}
	file.close();
}

static void appendFile(fs::FS &fs, const char *path, const char *message)
{
	Debug.printf("Appending to file: %s\n", path);

	File file = fs.open(path, FILE_APPEND);
	if (!file)
	{
		Debug.println("Failed to open file for appending");
		return;
	}
	if (file.print(message))
	{
		Debug.println("Message appended");
	}
	else
	{
		Debug.println("Append failed");
	}
	file.close();
}

static void renameFile(fs::FS &fs, const char *path1, const char *path2)
{
	Debug.printf("Renaming file %s to %s\n", path1, path2);
	if (fs.rename(path1, path2))
	{
		Debug.println("File renamed");
	}
	else
	{
		Debug.println("Rename failed");
	}
}

static void deleteFile(fs::FS &fs, const char *path)
{
	Debug.printf("Deleting file: %s\n", path);
	if (fs.remove(path))
	{
		Debug.println("File deleted");
	}
	else
	{
		Debug.println("Delete failed");
	}
}

static void testFileIO(fs::FS &fs, const char *path)
{
	File file = fs.open(path);
	static uint8_t buf[512];
	size_t len = 0;
	uint32_t start = millis();
	uint32_t end = start;
	if (file)
	{
		len = file.size();
		size_t flen = len;
		start = millis();
		while (len)
		{
			size_t toRead = len;
			if (toRead > 512)
			{
				toRead = 512;
			}
			file.read(buf, toRead);
			len -= toRead;
		}
		end = millis() - start;
		Debug.printf("%u bytes read for %u ms\n", flen, end);
		file.close();
	}
	else
	{
		Debug.println("Failed to open file for reading");
	}

	file = fs.open(path, FILE_WRITE);
	if (!file)
	{
		Debug.println("Failed to open file for writing");
		return;
	}

	size_t i;
	start = millis();
	for (i = 0; i < 2048; i++)
	{
		file.write(buf, 512);
	}
	end = millis() - start;
	Debug.printf("%u bytes written for %u ms\n", 2048 * 512, end);
	file.close();
}

/*****************************************************************
 * The Setup                                                     *
 *****************************************************************/

void setup()
{
	void *SpStart = NULL;
	StackPtrAtStart = (void *)&SpStart;
	watermarkStart = uxTaskGetStackHighWaterMark(NULL);
	StackPtrEnd = StackPtrAtStart - watermarkStart;

	Debug.begin(115200); // Output to Serial at 115200 baud
	Debug.println(F("Starting"));

	  Debug.print("MOSI: ");
  Debug.println(MOSI);
  Debug.print("MISO: ");
  Debug.println(MISO);
  Debug.print("SCK: ");
  Debug.println(SCK);
  Debug.print("SS: ");
  Debug.println(SS);  

	Debug.printf("\r\n\r\nAddress of Stackpointer near start is:  %p \r\n", (void *)StackPtrAtStart);
	Debug.printf("End of Stack is near: %p \r\n", (void *)StackPtrEnd);
	Debug.printf("Free Stack at setup is:  %d \r\n", (uint32_t)StackPtrAtStart - (uint32_t)StackPtrEnd);

	esp_chipid = String((uint16_t)(ESP.getEfuseMac() >> 32), HEX); // for esp32
	esp_chipid += String((uint32_t)ESP.getEfuseMac(), HEX);
	esp_chipid.toUpperCase();
	cfg::initNonTrivials(esp_chipid.c_str());
	WiFi.persistent(false);

	debug_outln_info(F("MobileAir: " SOFTWARE_VERSION_STR "/"), String(CURRENT_LANG));

	init_config();

	Wire.begin(I2C_PIN_SDA, I2C_PIN_SCL);

	if (cfg::npm_read)
	{
		serialNPM.begin(115200, SERIAL_8E1, PM_SERIAL_RX, PM_SERIAL_TX);
		Debug.println("Read Next PM... serialNPM 115200 8E1");
		serialNPM.setTimeout(400);
	}

	if (cfg::has_gps)
	{
		serialGPS.begin(9600, SERIAL_8N1, GPS_SERIAL_RX, GPS_SERIAL_TX);
		Debug.println("Read GPS... serialGPS 9600 8N1");
		serialNPM.setTimeout(400);
	}

	if (cfg::enveano2_read)
	{
		serialNO2.begin(9600, EspSoftwareSerial::SWSERIAL_8N1, NO2_SERIAL_RX, NO2_SERIAL_TX); //OK
		Debug.println("Envea Cairsens NO2... serialN02 9600 8N1");
		serialNO2.setTimeout((4 * 12 * 1000) / 9600);
	}

	if (cfg::has_ssd1306)
	{
		init_display();
	}

	if (cfg::has_led_value)
	{
		debug_outln_info(F("init FastLED"));
		FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, LEDS_NB); //swap R and G !  //ATTENTION AU TYPE DE LED
		FastLED.setBrightness(cfg::brightness);				   //max=255

		if (LEDS_NB == 1)
		{
			for (int i = 0; i < 13; i++)
			{
				leds[0] = colorLED_empty;
				FastLED.show();
				delay(200);
				leds[0] = colorLED_start;
				FastLED.show();
				delay(200);
			}
			leds[0] = colorLED_empty;
			FastLED.show();
		}
		else
		{
			if (LEDS_MATRIX)
			{
				for (int i = 0; i < 5; i++)
				{
					drawpicture(empty);
					FastLED.show();
					delay(200);
					drawpicture(connect1);
					FastLED.show();
					delay(200);
					drawpicture(connect2);
					FastLED.show();
					delay(200);
					drawpicture(connect3);
					FastLED.show();
					delay(200);
					drawpicture(connect4);
					FastLED.show();
					delay(200);
				}
				drawpicture(empty);
				FastLED.show();
				//5 secondes
			}
			else
			{

				for (unsigned int i = 0; i < 3; ++i)
				{
					for (unsigned int i = 0; i < LEDS_NB; ++i)
					{
						leds[i] = colorLED_start;
						FastLED.show();
						delay(200);
					}
					for (unsigned int i = 0; i < LEDS_NB; ++i)
					{
						leds[i] = colorLED_empty;
					}
					FastLED.show();
				}
			}
		}
	}

	debug_outln_info(F("\nChipId: "), esp_chipid);

	wifiConfig();
	logEnabledAPIs();
	powerOnTestSensors();

	delay(50);

	starttime = millis(); // store the start time
	last_update_attempt = time_point_device_start_ms = starttime;

	if (cfg::npm_read)
	{
		last_display_millis_oled = starttime_NPM = starttime;
	}

	if (cfg::ccs811_read)
	{
		last_display_millis_oled = starttime_CCS811 = starttime;
	}

	if (cfg::enveano2_read)
	{
		last_display_millis_oled = starttime_Cairsens = starttime;
	}

	if (cfg::has_gps)
	{

		while (!GPSdata.checked)
		{
			while (serialGPS.available() > 0 && (!newGPSdata || !GPSdata.checked))
			{
				if (gps.encode(serialGPS.read()))
				{
					Debug.println(F("First GPS coordinates"));
					GPSdata = getGPSdata();
					newGPSdata = true;
				}
			}
			if (millis() > 5000 && gps.charsProcessed() < 10)
			{
				Debug.println(F("No GPS detected: check wiring."));
				GPSdata = {0, 0, 0, 0, 0, 0, -1.0, -1.0, -1.0, false};
			}
		}

		if (GPSdata.checked)
		{
			coordinates = true;

			Debug.println(F("GPS coordinates found!"));

			listDir(SD, "/", 0);

			file_name = String("/") + String(GPSdata.year) + String("_") + String(GPSdata.month) + String("_") + String(GPSdata.day) + String("_") + String(GPSdata.hour) + String("_") + String(GPSdata.minute) + String("_") + String(GPSdata.second) + String(".csv");

			// const char data_file[30] = file_name.c_str();

			// char year[4];
			// String(GPSdata.year).toCharArray(year,4);
			// char month[2];
			// String(GPSdata.year+48).toCharArray(month,2);
			// char day[2];
			// String(GPSdata.day+48).toCharArray(day,2);
			// char hour[2];
			// String(GPSdata.hour+48).toCharArray(hour,2);
			// char minute[2];
			// String(GPSdata.minute+48).toCharArray(minute,2);
			// char second[2];
			// String(GPSdata.second+48).toCharArray(second,2);

			// strcat(data_file,year);
			// strcat(data_file,"_");
			// strcat(data_file,month);
			// strcat(data_file,"_");
			// strcat(data_file,day);
			// strcat(data_file,"_");
			// strcat(data_file,hour);
			// strcat(data_file,"_");
			// strcat(data_file,minute);
			// strcat(data_file,"_");
			// strcat(data_file,second);
			// strcat(data_file,".csv");

			writeFile(SD, file_name.c_str(), "");
			appendFile(SD, file_name.c_str(), "Date;NextPM_PM1;NextPM_PM2_5;NextPM_PM10;NextPM_NC1;NextPM_NC2_5;NextPM_NC10;CCS811_COV;Cairsens_NO2;BME280_T;BME280_H;BME280_P;Latitude;Longitude;Altitude\n");
			Debug.println("Date;NextPM_PM1;NextPM_PM2_5;NextPM_PM10;NextPM_NC1;NextPM_NC2_5;NextPM_NC10;CCS811_COV;Cairsens_NO2;BME280_T;BME280_H;BME280_P;Latitude;Longitude;Altitude\n");
			file_created = true;
		}
	}

	newGPSdata = false;
	Debug.printf("End of void setup()\n");
	starttime_waiter = millis();

	if (cfg::wifi_permanent)
	{
		WiFi.mode(WIFI_AP);
		const IPAddress apIP(192, 168, 4, 1);
		WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
		WiFi.softAP(cfg::fs_ssid, cfg::fs_pwd, selectChannelForAp());

		DNSServer dnsServer;
		dnsServer.setTTL(0);
		dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
		dnsServer.start(53, "*", apIP); // 53 is port for DNS server

		setup_webserver();

		dnsServer.processNextRequest();
		server.handleClient();
		yield();
	}
}

void loop()
{
	String result_NPM, result_CCS811, result_Cairsens;

	unsigned sum_send_time = 0;

	act_micro = micros();
	act_milli = millis();
	send_now = msSince(starttime) > cfg::sending_intervall_ms;

	//first run

	if (count_sends == 0 && !send_now)
	{
		LEDwait = msSince(starttime_waiter) > (1000 * multiplier);

		// Debug.println(starttime_waiter);

		if (LEDwait)
		{

			if (multiplier & 1) //impair
			{

				if (LEDS_NB == 1)
				{
					leds[0] = colorLED_start;
					FastLED.show();
				}
				else
				{
					if (LEDS_MATRIX)
					{
						drawpicture(damier1);
						FastLED.show();
					}
					else
					{
						leds[0] = colorLED_empty;
						leds[1] = colorLED_start;
						leds[2] = colorLED_empty;
						leds[3] = colorLED_start;
						leds[4] = colorLED_empty;
						leds[5] = colorLED_start;
						leds[6] = colorLED_empty;
						leds[7] = colorLED_start;
						FastLED.show();
					}
				}
			}
			else //pair
			{

				if (LEDS_NB == 1)
				{
					leds[0] = colorLED_empty;
					FastLED.show();
				}
				else
				{
					if (LEDS_MATRIX)
					{
						drawpicture(damier2);
						FastLED.show();
					}
					else
					{
						leds[0] = colorLED_start;
						leds[1] = colorLED_empty;
						leds[2] = colorLED_start;
						leds[3] = colorLED_empty;
						leds[4] = colorLED_start;
						leds[5] = colorLED_empty;
						leds[6] = colorLED_start;
						leds[7] = colorLED_empty;
						FastLED.show();
					}
				}
			}
			multiplier += 1;
		}
	}

	sample_count++;
	if (last_micro != 0)
	{
		unsigned long diff_micro = act_micro - last_micro;
		UPDATE_MIN_MAX(min_micro, max_micro, diff_micro);
	}
	last_micro = act_micro;

	if (cfg::npm_read)
	{
		if ((msSince(starttime_NPM) > SAMPLETIME_NPM_MS && npm_val_count == 0) || send_now)
		{
			starttime_NPM = act_milli;
			fetchSensorNPM(result_NPM);
		}
	}

	if (cfg::ccs811_read && (!ccs811_init_failed))
	{
		if ((msSince(starttime_CCS811) > SAMPLETIME_CCS811_MS && ccs811_val_count < 11) || send_now)
		{
			starttime_CCS811 = act_milli;
			fetchSensorCCS811(result_CCS811);
		}
	}

	if (cfg::enveano2_read)
	{
		if ((msSince(starttime_Cairsens) > SAMPLETIME_Cairsens_MS && no2_val_count < 11) || send_now)
		{
			starttime_Cairsens = act_milli;
			fetchSensorCairsens(result_Cairsens);
		}
	}

	if ((msSince(last_display_millis_oled) > DISPLAY_UPDATE_INTERVAL_MS) && (cfg::has_ssd1306))
	{
		display_values_oled();
		last_display_millis_oled = act_milli;
	}

	if (cfg::wifi_permanent)
	{
		server.handleClient();
		yield();
	}

	if (send_now && cfg::sending_intervall_ms >= 120000)
	{

	if (cfg::has_gps)
	{

		while (serialGPS.available() > 0 && !newGPSdata)
		{
			if (gps.encode(serialGPS.read()))
			{
				GPSdata = getGPSdata();
				newGPSdata = true;
			}
		}

		if (millis() > 5000 && gps.charsProcessed() < 10)
		{
			Debug.println(F("No GPS"));
			GPSdata = {0, 0, 0, 0, 0, 0, -1.0, -1.0, -1.0, false};
		}
	}

		void *SpActual = NULL;
		Debug.printf("Free Stack at send_now is: %d \r\n", (uint32_t)&SpActual - (uint32_t)StackPtrEnd);

		RESERVE_STRING(datajson, LARGE_STR);
		datajson = FPSTR(data_first_part);
		//data_custom
		RESERVE_STRING(result, MED_STR);

		if (cfg::npm_read)
		{
			datajson += result_NPM;
		}

		if (cfg::bmx280_read && (!bmx280_init_failed))
		{
			fetchSensorBMX280(result);
			datajson += result;
			result = emptyString;
		}

		//These values are not sent because not configured in the SC API:

		if (cfg::ccs811_read && (!ccs811_init_failed))
		{
			datajson += result_CCS811;
		}

		if (cfg::enveano2_read)
		{
			datajson += result_Cairsens;
		}

		add_Value2Json(datajson, F("samples"), String(sample_count));
		add_Value2Json(datajson, F("min_micro"), String(min_micro));
		add_Value2Json(datajson, F("max_micro"), String(max_micro));
		add_Value2Json(datajson, F("interval"), String(cfg::sending_intervall_ms));

		if ((unsigned)(datajson.lastIndexOf(',') + 1) == datajson.length())
		{
			datajson.remove(datajson.length() - 1);
		}
		datajson += "]}";

		Debug.println(datajson);

		if (GPSdata.checked)
		{
			RESERVE_STRING(datacsv, LARGE_STR);

			datacsv += String(GPSdata.year);
			datacsv += "-";

			if (GPSdata.month < 10)
			{
				datacsv += "0";
			}
			datacsv += String(GPSdata.month);
			datacsv += "-";

			if (GPSdata.day < 10)
			{
				datacsv += "0";
			}
			datacsv += String(GPSdata.day);

			datacsv += "T";
			if (GPSdata.hour < 10)
			{
				datacsv += "0";
			}
			datacsv += String(GPSdata.hour);
			datacsv += ":";

			if (GPSdata.minute < 10)
			{
				datacsv += "0";
			}
			datacsv += String(GPSdata.minute);
			datacsv += ":";

			if (GPSdata.second < 10)
			{
				datacsv += "0";
			}
			datacsv += String(GPSdata.second);
			datacsv += "Z";
			datacsv += ";";
			datacsv += String(last_value_NPM_P0);
			datacsv += ";";
			datacsv += String(last_value_NPM_P2);
			datacsv += ";";
			datacsv += String(last_value_NPM_P1);
			datacsv += ";";
			datacsv += String(last_value_NPM_N1);
			datacsv += ";";
			datacsv += String(last_value_NPM_N25);
			datacsv += ";";
			datacsv += String(last_value_NPM_N10);
			datacsv += ";";
			datacsv += String(last_value_CCS811);
			datacsv += ";";
			datacsv += String(last_value_no2);
			datacsv += ";";
			datacsv += String(last_value_BMX280_T);
			datacsv += ";";
			datacsv += String(last_value_BME280_H);
			datacsv += ";";
			datacsv += String(last_value_BMX280_P);
			datacsv += ";";
			datacsv += String(GPSdata.latitude,8);
			datacsv += ";";
			datacsv += String(GPSdata.longitude,8);
			datacsv += ";";
			datacsv += String(GPSdata.altitude);
			datacsv += "\n";

			appendFile(SD, file_name.c_str(), datacsv.c_str());

			Debug.println(datacsv);
			count_recorded += 1;
			last_datacsv_string = std::move(datacsv);
		}

		//"Date;NextPM_PM1;NextPM_PM2_5;NextPM_PM10;NextPM_NC1;NextPM_NC2_5;NextPM_NC10;CCS811_COV;Cairsens_NO2;BME280_T;BME280_H;BME280_P;Latitude;Longitude;Altitude\n"

		if (cfg::has_led_value)
		{

			switch (cfg::value_displayed)
			{
			case 0:
				if (cfg::npm_read && last_value_NPM_P0 != -1.0)
				{
					displayColor_value = colorPM(last_value_NPM_P0, 10, 20, 25, 50, 75, gamma_correction);
				}
				else
				{
					displayColor_value = {0, 0, 0}; //BLACK
				}
				break;
			case 1:
				if (cfg::npm_read && last_value_NPM_P1 != -1.0)
				{
					displayColor_value = colorPM(last_value_NPM_P1, 20, 40, 50, 100, 150, gamma_correction);
				}
				else
				{
					displayColor_value = {0, 0, 0}; //BLACK
				}
				break;
			case 2:
				if (cfg::npm_read && last_value_NPM_P2 != -1.0)
				{
					displayColor_value = colorPM(last_value_NPM_P2, 10, 20, 25, 50, 75, gamma_correction);
				}
				else
				{
					displayColor_value = {0, 0, 0}; //BLACK
				}
				break;
			case 3:
				if (cfg::bmx280_read && last_value_BMX280_T != -128.0)
				{
					displayColor_value = interpolateTemp(last_value_BMX280_T, 19, 28, gamma_correction);
				}
				else
				{
					displayColor_value = {0, 0, 0}; //BLACK
				}
				break;
			case 4:
				if (cfg::bmx280_read && last_value_BME280_H != -1.0)
				{
					displayColor_value = interpolateHumi(last_value_BME280_H, 40, 60, gamma_correction);
				}
				else
				{
					displayColor_value = {0, 0, 0}; //BLACK
				}
				break;
			case 5:
				if (cfg::bmx280_read && last_value_BMX280_P != -1.0)
				{
					displayColor_value = interpolatePress(last_value_BMX280_P, 40, 60, gamma_correction);
				}
				else
				{
					displayColor_value = {0, 0, 0}; //BLACK
				}
				break;
			case 6:
				if (cfg::ccs811_read && last_value_CCS811 != -1.0)
				{
					displayColor_value = interpolateCOV(last_value_CCS811, 800, 1500, gamma_correction); //REVOIR GRadient
				}
				else
				{
					displayColor_value = {0, 0, 0}; //BLACK
				}
				break;
			}

			colorLED_value = CRGB(displayColor_value.R, displayColor_value.G, displayColor_value.B);

			if (LEDS_NB == 1)
			{
				leds[0] = colorLED_value;
				FastLED.show();
			}
			else
			{
				if (LEDS_MATRIX)
				{
					fill_solid(leds, LEDS_NB, colorLED_value);
					FastLED.show();
				}
				else
				{
					fill_solid(leds, LEDS_NB, colorLED_value);
					FastLED.show();
				}
			}
		}

		yield();

		// only do a restart after finishing sending (Wifi). Befor Lora to avoid conflicts with the LMIC
		if (msSince(time_point_device_start_ms) > DURATION_BEFORE_FORCED_RESTART_MS)
		{
			sensor_restart();
		}

		// Resetting for next sampling
		last_datajson_string = std::move(datajson);
		sample_count = 0;
		last_micro = 0;
		min_micro = 1000000000;
		max_micro = 0;
		sum_send_time = 0;
		newGPSdata = false;

		// if (cfg::has_led_value)
		// {
		// 	if (LEDS_NB == 1)
		// 	{
		// 		leds[0] = colorLED_value;
		// 		FastLED.show();
		// 	}
		// 	else
		// 	{
		// 		if (LEDS_MATRIX)
		// 		{
		// 			fill_solid(leds, LEDS_NB, colorLED_value);
		// 			FastLED.show();
		// 		}
		// 		else
		// 		{
		// 			fill_solid(leds, LEDS_NB, colorLED_value);
		// 			FastLED.show();
		// 		}
		// 	}
		// }

		starttime = millis(); // store the start time
		count_sends++;
	}

	if (sample_count % 500 == 0)
	{
		//		Debug.println(ESP.getFreeHeap(),DEC);
	}
}

const uint8_t PROGMEM gamma8[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2,
	2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5,
	5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10,
	10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
	17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
	25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
	37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
	51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
	69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
	90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
	115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142,
	144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175,
	177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213,
	215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255};
