// Language config
#define CURRENT_LANG INTL_LANG

#define TIME_FOR_WIFI_CONFIG 120000;
#define SENDING_INTERVALL_MS 120000;

// Sensor Wifi config (config mode)
#define FS_SSID ""
#define FS_PWD "mobileaircfg"

#define WIFI_PERMANENT 1

#define HAS_GPS 0
#define HAS_SDCARD 0

#define SEND2CSV 0

#define PM_SERIAL_RX D39
#define PM_SERIAL_TX D32 
#define GPS_SERIAL_RX D36
#define GPS_SERIAL_TX D27
#define NO2_SERIAL_RX D16
#define NO2_SERIAL_TX D17 
#define I2C_PIN_SCL D22
#define I2C_PIN_SDA D21
#define LED_PIN D33

// Tera Sensor Next PM sensor
#define NPM_READ 0
#define NPM_API_PIN 1

// BMP280/BME280, temperature, pressure (humidity on BME280)
#define BMX280_READ 0
#define BMP280_API_PIN 3
#define BME280_API_PIN 11

// CCS811, COV Sensor

#define CCS811_READ 0
// #define CCS811_API_PIN X

// Envea Cairsens NO2

#define ENVEANO2_READ 0
// #define ENVEANO2_API_PIN X

#define HAS_SSD1306 0
#define HAS_LED_VALUE 1
//#define LEDS_NB 64
//#define LEDS_NB 1 //for monoLED
#define LEDS_NB 8 //for LEDline
//#define LEDS_MATRIX true 
#define LEDS_MATRIX false // for lEDline  
#define BRIGHTNESS 50
#define RGPD 0
#define GAMMA true
#define VALUE_DISPLAYED 2

#define DISPLAY_MEASURE 1
#define DISPLAY_DEVICE_INFO 1

// Set debug level for serial output?
#define DEBUG 5
