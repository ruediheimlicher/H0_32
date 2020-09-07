#
# embedXcode
# ----------------------------------
# Embedded Computing on Xcode
#
# Copyright © Rei VILO, 2010-2019
# All rights reserved
# https://embedXcode.weebly.com
#
# Last update: 19 Dec 2017 release 8.4.2


# Libraries
# ----------------------------------
# Declare application Arduino/chipKIT/Digispark/Energia/Maple/Microduino/Teensy/Wiring
# and user's libraries used 
# Short-listing libraries speeds-up building
# Typing = 0 takes none (default for application and user)
# Otherwise, leaving empty considers all (default for local)

# List application Arduino/chipKIT/Digispark/Energia/Maple/Microduino/Teensy/Wiring libraries here
# !!! Help: http://bit.ly/2hYLBDC
# default = 0 = none
#
# TEENSY : common = Adafruit_Circuit_Playground Bridge Esplora 
#	Ethernet Firmata GSM Keyboard LiquidCrystal Mouse RobotIRremote 
#	Robot_Control Robot_Motor SD Servo SpacebrewYun Stepper TFT Temboo 
#	WiFi ADC AccelStepper Adafruit_CC3000 Adafruit_GFX 
#	Adafruit_ILI9340 Adafruit_ILI9341 Adafruit_NeoPixel Adafruit_RA8875 
#	Adafruit_SSD1306 Adafruit_ST7735 Adafruit_STMPE610 Adafruit_VS1053 
#	Adafruit_nRF8001 AltSoftSerial Artnet Audio Bounce Bounce2 
#	CapacitiveSensor CryptoAccel DS1307RTC DmxSimple DogLcd EEPROM 
#	EasyTransfer EasyTransferI2C Encoder Entropy Ethernet FastCRC 
#	FastLED FlexCAN FlexiTimer2 FreqCount FreqMeasure 
#	FreqMeasureMulti FrequencyTimer2 ILI9341_t3 IRremote Keypad LedControl 
#	LedDisplay LiquidCrystal LiquidCrystalFast LowPower MFRC522 MIDI 
#	Metro MsTimer2 NXPMotionSense OSC OctoWS2811 OneWire 
#	PS2Keyboard PWMServo Ping PulsePosition RA8875 RadioHead 
#	ResponsiveAnalogRead SD SPI SPIFlash ST7565 ST7735_t3 SerialFlash Servo 
#	ShiftPWM Snooze SoftPWM SoftwareSerial TFT_ILI9163C Talkie 
#	TeensyThreads Time TimeAlarms TimerOne TimerThree TinyGPS Tlc5940 
#	TouchScreen USBHost_t36 UTFT VirtualWire WS2812Serial Wire XBee 
#	XPT2046_Touchscreen i2c_t3 ks0108 openGLCD ssd1351 x10 
#
APP_LIBS_LIST = 0

# List user's libraries here
# !!! Help: http://bit.ly/2AEscDx
# default = 0 = none
#
#
USER_LIBS_LIST = 0

# List local libraries here
# !!! Help: http://bit.ly/2juEP91
# default = empty = all
#
LOCAL_LIBS_LIST =

# List core, application and user's libraries to be excluded
# !!! Help: http://bit.ly/2AijUOI
# For example, WiFi may crash on Arduino 1.0.2, Esplora on Arduino 1.0.3, Firmata on Teensy 3.0,
#   OneWire on MPIDE 0023, HardwareSerial may conflict with MarlinSerial
#
#EXCLUDE_LIBS = Firmata WiFi Esplora OneWire Robot_Control Robot_Control/utility Robot_Motor


# Warning options
# ----------------------------------
# Contrary to errors, warnings don't stop compilation but they point at possible cause of errors.
# !!! Help: http://bit.ly/2hXiCQL
# For example, unused variables with unused-variable.
#
# If 0, no warnings
# If empty, all warnings, same as WARNING_OPTIONS = all
# WARNING_OPTIONS = all no-missing-braces no-conversion-null no-pointer-arith
# WARNING_OPTIONS = unused-variable unused-function unused-label unused-value no-conversion-null no-pointer-arith
#
WARNING_OPTIONS = 0


# Compiler options
# ----------------------------------
#
# If 0 or empty, no additional options
# !!! Help: http://bit.ly/2Af7rxq
#
COMPILER_OPTIONS =


# Applications path folder
# ----------------------------------
# Path to the folder where the IDEs are located
# !!! Help: http://bit.ly/2BkdoWV
# By default, = /Applications
#
#APPLICATIONS_PATH = /Applications


# Serial console for Fast target
# ----------------------------------
# For Fast target, open serial console, false or true
# !!! Help: http://bit.ly/2AfyA37
#
NO_SERIAL_CONSOLE = true


# Teensy USB options (default)
# ----------------------------------
# !!! Help: http://bit.ly/2BmH31o
# TEENSY_USB = (USB_SERIAL) USB_HID USB_SERIAL_HID USB_DISK USB_DISK_SDFLASH USB_MIDI
#   USB_RAWHID USB_FLIGHTSIM
# TEENSY_LAYOUT = (LAYOUT_US_ENGLISH) LAYOUT_CANADIAN_FRENCH LAYOUT_CANADIAN_MULTILINGUAL
#   LAYOUT_DANISH LAYOUT_FINNISH LAYOUT_FRENCH LAYOUT_FRENCH_BELGIAN LAYOUT_FRENCH_SWISS
#   LAYOUT_GERMAN LAYOUT_GERMAN_MAC
#   LAYOUT_GERMAN_SWISS LAYOUT_ICELANDIC LAYOUT_IRISH LAYOUT_ITALIAN LAYOUT_NORWEGIAN
#   LAYOUT_PORTUGUESE LAYOUT_PORTUGUESE_BRAZILIAN LAYOUT_SPANISH LAYOUT_SPANISH_LATIN_AMERICA
#   LAYOUT_SWEDISH LAYOUT_TURKISH LAYOUT_UNITED_KINGDOM LAYOUT_US_INTERNATIONAL
#
#TEENSY_USB = USB_SERIAL
TEENSY_USB = USB_RAWHID
TEENSY_LAYOUT = LAYOUT_US_ENGLISH


# Teensy CPU options (default)
# ----------------------------------
# !!! Help: http://bit.ly/2Bj3c0G
# TEENSY_F_CPU = (96000000) 72000000 48000000 24000000 for Teensy 3.1
# TEENSY_F_CPU = (96000000) 48000000 24000000 for Teensy 3.0
# TEENSY_F_CPU = (48000000) 24000000 for Teensy LC
# TEENSY_F_CPU = (16000000) 8000000 4000000 for Teensy 2.0
# TEENSY_OPTIMISATION = (-Os)
#
#TEENSY_F_CPU = 48000000
#TEENSY_OPTIMISATION = -Os


# Parameters
# ----------------------------------
# Xcode takes BOARD_TAG and BOARD_PORT from the .xcconfig file
# For other IDEs than Xcode,

# BOARD_TAG is required
# If not defined, error BOARD_TAG not defined
#
#BOARD_TAG  = uno

# BOARD_PORT is optional
# !!! Help: http://bit.ly/2yUvq0h
# If not defined, BOARD_PORT = /dev/tty.usb* (default)
#
#BOARD_PORT = /dev/tty.usbmodem*

# Xcode takes SKETCHBOOK_DIR from preferences.txt
# If not defined, each IDE has its own SKETCHBOOK_DIR (default)
# To share the same SKETCHBOOK_DIR along all IDEs, define
#
#SKETCHBOOK_DIR =

# SERIAL_BAUDRATE for the serial console, 9600 by default
# !!! Help: http://bit.ly/2AUEcgp
# Uncomment and specify another speed
#
#SERIAL_BAUDRATE = 9600

# OPTIMISATION for the optimisation and debugger levels
# !!! Help: http://bit.ly/2kheSLz
# Uncomment and specify another options
#
#OPTIMISATION    = -Os -g3


# Miscellaneous
# ----------------------------------
# DO NOT EDIT THIS FILE BELOW THIS LINE.
# ----------------------------------
#
EMBEDXCODE_EDITION = embedXcode+ Teensy

# Manage path with space in the name
CURRENT_DIR_SPACE := $(shell pwd)
CURRENT_DIR       := $(shell echo '$(CURRENT_DIR_SPACE)' | sed 's/ /\\\ /g')

# C-compliant project name and extension
PROJECT_NAME_AS_IDENTIFIER = H0_32
SKETCH_EXTENSION   = ino

# Include only current variant of selected platform, false or true
SELECTED_RESOURCES = false

# For distribution, keep main cpp, false or true
KEEP_MAIN_CPP = false

# For building, show details, false or true
HIDE_NUMBER = false
HIDE_COMMAND = false

# For building, keep main and tasks unchanged, false or true
KEEP_MAIN = false
KEEP_TASKS = false

# Local makefiles
#MAKEFILE_PATH     := $(CURRENT_DIR)/Makefiles
#UTILITIES_PATH    := $(CURRENT_DIR)/Utilities

# Remote makefiles
EMBEDXCODE_TOOLS   = $(HOME)/Library/embedXcode/Tools
MAKEFILE_PATH      = $(EMBEDXCODE_TOOLS)/Makefiles
UTILITIES_PATH     = $(EMBEDXCODE_TOOLS)/Utilities

# Local builds
#BUILDS_PATH_SPACE := $(CURRENT_DIR_SPACE)/Builds
#BUILDS_PATH       := $(CURRENT_DIR)/Builds

# Remote builds
BUILDS_PATH       := $(BUILT_PRODUCTS_DIR)
BUILDS_PATH_SPACE := $(BUILDS_PATH)

include $(MAKEFILE_PATH)/Step1.mk

