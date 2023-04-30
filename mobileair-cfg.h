

// This file is generated, please do not edit.
// Change mobileair-cfg.h.py instead.

enum ConfigEntryType : unsigned short {
	Config_Type_Bool,
	Config_Type_UInt,
	Config_Type_Time,
	Config_Type_String,
	Config_Type_Password,
    Config_Type_Hex
};

struct ConfigShapeEntry {
	enum ConfigEntryType cfg_type;
	unsigned short cfg_len;
	const char* _cfg_key;
	union {
		void* as_void;
		bool* as_bool;
		unsigned int* as_uint;
		char* as_str;
	} cfg_val;
	const __FlashStringHelper* cfg_key() const { return FPSTR(_cfg_key); }
};

enum ConfigShapeId {
	Config_current_lang,
	Config_fs_ssid,
	Config_fs_pwd,
	Config_wifi_permanent,
	Config_npm_read,
	Config_bmx280_read,
	Config_has_gps,
	Config_ccs811_read,
	Config_enveano2_read,
	Config_send2csv,
	Config_has_ssd1306,
	Config_has_led_value,
	Config_brightness,
	Config_value_displayed,
	Config_debug,
	Config_time_for_wifi_config,
	Config_has_sdcard,
	Config_display_measure,
	Config_display_device_info,
};
static constexpr char CFG_KEY_CURRENT_LANG[] PROGMEM = "current_lang";
static constexpr char CFG_KEY_FS_SSID[] PROGMEM = "fs_ssid";
static constexpr char CFG_KEY_FS_PWD[] PROGMEM = "fs_pwd";
static constexpr char CFG_KEY_WIFI_PERMANENT[] PROGMEM = "wifi_permanent";
static constexpr char CFG_KEY_NPM_READ[] PROGMEM = "npm_read";
static constexpr char CFG_KEY_BMX280_READ[] PROGMEM = "bmx280_read";
static constexpr char CFG_KEY_HAS_GPS[] PROGMEM = "has_gps";
static constexpr char CFG_KEY_CCS811_READ[] PROGMEM = "ccs811_read";
static constexpr char CFG_KEY_ENVEANO2_READ[] PROGMEM = "enveano2_read";
static constexpr char CFG_KEY_SEND2CSV[] PROGMEM = "send2csv";
static constexpr char CFG_KEY_HAS_SSD1306[] PROGMEM = "has_ssd1306";
static constexpr char CFG_KEY_HAS_LED_VALUE[] PROGMEM = "has_led_value";
static constexpr char CFG_KEY_BRIGHTNESS[] PROGMEM = "brightness";
static constexpr char CFG_KEY_VALUE_DISPLAYED[] PROGMEM = "value_displayed";
static constexpr char CFG_KEY_DEBUG[] PROGMEM = "debug";
static constexpr char CFG_KEY_TIME_FOR_WIFI_CONFIG[] PROGMEM = "time_for_wifi_config";
static constexpr char CFG_KEY_HAS_SDCARD[] PROGMEM = "has_sdcard";
static constexpr char CFG_KEY_DISPLAY_MEASURE[] PROGMEM = "display_measure";
static constexpr char CFG_KEY_DISPLAY_DEVICE_INFO[] PROGMEM = "display_device_info";
static constexpr ConfigShapeEntry configShape[] PROGMEM = {
	{ Config_Type_String, sizeof(cfg::current_lang)-1, CFG_KEY_CURRENT_LANG, cfg::current_lang },
	{ Config_Type_String, sizeof(cfg::fs_ssid)-1, CFG_KEY_FS_SSID, cfg::fs_ssid },
	{ Config_Type_Password, sizeof(cfg::fs_pwd)-1, CFG_KEY_FS_PWD, cfg::fs_pwd },
	{ Config_Type_Bool, 0, CFG_KEY_WIFI_PERMANENT, &cfg::wifi_permanent },
	{ Config_Type_Bool, 0, CFG_KEY_NPM_READ, &cfg::npm_read },
	{ Config_Type_Bool, 0, CFG_KEY_BMX280_READ, &cfg::bmx280_read },
	{ Config_Type_Bool, 0, CFG_KEY_HAS_GPS, &cfg::has_gps },
	{ Config_Type_Bool, 0, CFG_KEY_CCS811_READ, &cfg::ccs811_read },
	{ Config_Type_Bool, 0, CFG_KEY_ENVEANO2_READ, &cfg::enveano2_read },
	{ Config_Type_Bool, 0, CFG_KEY_SEND2CSV, &cfg::send2csv },
	{ Config_Type_Bool, 0, CFG_KEY_HAS_SSD1306, &cfg::has_ssd1306 },
	{ Config_Type_Bool, 0, CFG_KEY_HAS_LED_VALUE, &cfg::has_led_value },
	{ Config_Type_UInt, 0, CFG_KEY_BRIGHTNESS, &cfg::brightness },
	{ Config_Type_UInt, 0, CFG_KEY_VALUE_DISPLAYED, &cfg::value_displayed },
	{ Config_Type_UInt, 0, CFG_KEY_DEBUG, &cfg::debug },
	{ Config_Type_Time, 0, CFG_KEY_TIME_FOR_WIFI_CONFIG, &cfg::time_for_wifi_config },
	{ Config_Type_Bool, 0, CFG_KEY_HAS_SDCARD, &cfg::has_sdcard },
	{ Config_Type_Bool, 0, CFG_KEY_DISPLAY_MEASURE, &cfg::display_measure },
	{ Config_Type_Bool, 0, CFG_KEY_DISPLAY_DEVICE_INFO, &cfg::display_device_info },
};
