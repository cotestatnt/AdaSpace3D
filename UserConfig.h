#ifndef USER_CONFIG_H
#define USER_CONFIG_H


#define I2C_SDA 14
#define I2C_SCL 15

// ===================================================================================
//   ADA SPACE 3D - USER CONFIGURATION
// ===================================================================================

// --- DEBUG MODE ---
#define DEBUG_MODE          1      // 1 = Print HID values to Serial instead of sending them
                                   // 0 = Normal mode (send HID reports)

// --- LED CONFIGURATION ---
// The firmware drives BOTH outputs simultaneously. Connect whichever you like.
// - Addressable Strip: GPIO 4
// - Simple LED:        GPIO 3

#define LED_MODE            2      // 0 = Static (Solid Color)
                                   // 1 = Breathing (Gently Fades)
                                   // 2 = Debug / Reactive (Status Colors + White Flash on Move)

// Settings for Addressable Strip (GPIO 4)
#define NUM_ADDRESSABLE_LEDS 3      // How many LEDs are in your strip?
#define LED_BRIGHTNESS       128    // 0 to 255 (Global brightness)

// Settings for Static/Breathing Modes (Ignored in Debug Mode)
// Set your preferred color (0-255)
#define LED_COLOR_R         0      // Red
#define LED_COLOR_G         255    // Green
#define LED_COLOR_B         0    // Blue (Cyan)

// --- PIN DEFINITIONS ---
#define MAG_POWER_PIN       15     

// Button Pins
#define BUTTON1_PIN         29
#define BUTTON2_PIN         28
#define BUTTON3_PIN         27
#define BUTTON4_PIN         26

// --- SENSOR SETTINGS ---
// SENSITIVITY: 150.0 is the "Golden" value for this sensor
#define CONFIG_TRANS_SCALE     100 
#define CONFIG_ZOOM_SCALE      50  
#define CONFIG_ROT_SCALE       40 

// DEADZONES: Keep small (1.0) because raw values are tiny
#define CONFIG_DEADZONE        1.0    
#define CONFIG_ZOOM_DEADZONE   2.5  

// --- USB IDENTIFICATION ---
// 0x046d / 0xc626 = SpaceNavigator (Best for DIY compatibility)
#define USB_VID             0x256f
#define USB_PID             0xc631

#endif // USER_CONFIG_H
