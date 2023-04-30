#!/usr/bin/env python3

configshape_in = """
String		current_lang
String		fs_ssid
Password	fs_pwd
Bool		wifi_permanent
Bool		npm_read
Bool		bmx280_read
Bool        has_gps
Bool		ccs811_read
Bool		enveano2_read
Bool		send2csv
Bool        has_ssd1306
Bool        has_led_value
UInt		brightness
UInt        value_displayed
UInt		debug
Time		time_for_wifi_config
Bool        has_sdcard
Bool		display_measure
Bool		display_device_info
"""

with open("mobileair-cfg.h", "w") as h:
    print("""

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

enum ConfigShapeId {""", file=h)

    for cfgentry in configshape_in.strip().split('\n'):
        print("\tConfig_", cfgentry.split()[1], ",", sep='', file=h)
    print("};", file=h)

    for cfgentry in configshape_in.strip().split('\n'):
        _, cfgkey = cfgentry.split()
        print("static constexpr char CFG_KEY_", cfgkey.upper(),
              "[] PROGMEM = \"", cfgkey, "\";", sep='', file=h)

    print("static constexpr ConfigShapeEntry configShape[] PROGMEM = {",
          file=h)
    for cfgentry in configshape_in.strip().split('\n'):
        cfgtype, cfgkey = cfgentry.split()
        print("\t{ Config_Type_", cfgtype,
              ", sizeof(cfg::" + cfgkey + ")-1" if cfgtype in ('String', 'Password', 'Hex') else ", 0",
              ", CFG_KEY_", cfgkey.upper(),
              ", ", "" if cfgtype in ('String', 'Password', 'Hex') else "&",
              "cfg::", cfgkey, " },", sep='', file=h)
    print("};", file=h)
