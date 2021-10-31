/*******************************************************************************
* Project name:         Enclosure temperature controller
* Project description:  Enclosure used for 3d printer has built-in controller to
*                       sustain constant environment temperature. PID controlled
*                       12V enclosure fan and 220V AC heater as a heat source
*                       is used to regulate enclosure temperature and DS18B20
*                       temperature sensor to measure enclosure (env.)
*                       temperature. Three buttons, LCD 16x2 and Bluetooth
*                       module will provide an option to set desired env. temp.
*                       Temperature measurement unit is in Celsius degrees.
*                       Desired minimum and maximum target temperature which is
*                       supported accordingly in a range: [20...59]
* Author:               Rolands Zuters
* Date:                 27.09.2021.
*
* Main components used:
*   - fan               Noctua NF-P14s (12V);
*   - relay             JQC-3FF-S-Z 5V (220V heater connected to "Normally
*                       Open");
*   - temp. sensor      DS18B20
*   - LCD               1602A (16x2) with I2C module
*   - Bluetooth module  HC-06 Bluetooth Module
*******************************************************************************/
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <mjson.h>
#include <SoftwareSerial.h>
#include <TimerOne.h>
#include <PID_v1.h>
//------------------------------------------------------------------------------
//! 3 buttons (from left on breadboard - up, down, execute) for user to setup
//! desired target temperaure settings
//! Button "Up"
#define BUTTON_UP_PIN                                                         12
//! Button "Down"
#define BUTTON_DOWN_PIN                                                        5
//! Button "Set"
#define BUTTON_APPLY_PIN                                                       8
//! Bluetooth module (HC-06) pins for remote user configuration setup option
#define BT_RX_PIN                                                              3
#define BT_TX_PIN                                                             11
//! Relay pin to switch on/off heater of enclosure
#define RELAY_MNG_PIN                                                          4
//! Enclosure temperature sensor DS18B20 data pin
#define ENCL_T_PIN                                                             7
//! Buzzer pin
#define BUZZER_PIN                                                             6
//! Button debounce delay
#define BTN_DEB_DELAY                                                         40
//! Enclosure fan control pin (Timer1 pin)
#define FAN_PWM_PIN                                                            9
//! TimerOne library specific values used in calculations
//! 100% - full duty cycle
#define FULL_DUTY                                                            100
//! value @FULL_DUTY cycle
#define MAX_DUTY_VALUE                                                      1023
//! 16x2 LCD I2C address (detected by address scanner)
#define LCD_I2C_ADDRESS                                                     0x27
//! String literals for LCD
#define STRING_ACTUAL_TEMP                                         "Act.temp.: "
#define STRING_DESIRED_TEMP                                        "Dst.temp.: "
//! Actual temperature request interval in seconds
#define TEMP_POLL_INTV                                                         2
//! Temperature digits count (target temperature range 20 - 59; two digits)
#define LENGTH_OF_TENTHS                                                       4
#define LENGTH_OF_ONES                                                        10
//------------------------------------------------------------------------------
//! Button control usage:
//!     Using buttons user can set desired target temperature. Button
//!     functionalities:
//!         Button "Set" -  press once to manage tenth position for the
//!                         temperature. Press twice to manage ones position.
//!                         Pressing one more time while on ones position will
//!                         apply configured target temperature.
//!         Button "Up" -   Change temperature digit in ascending order.
//!         Button "Down" - Change temperature digit in descending order.
//------------------------------------------------------------------------------
//! API description for temperature setup via Bluetooth:
//! Set target temperature (JSON request)
//!     method: "setdestemp" - call function to set the desired target
//!             temperature;
//!     params: array of parameter. Funtion takes first value of the array and 
//!             tries to set it as a target temperature;
//!     example: {"method":"setdestemp","params":[29]}
//! Error on incorrect (out of range) temperature value:
//!     {
//!         "method": "setdestemp",
//!         "destTemp": 76,
//!         "error": "Invalid target temperature. Valid range: 20-59"
//!     }
//! Warning on attempt to set new target temperature if controller is
//! already in configuration mode
//!     {
//!         "method": "setdestemp",
//!         "destTemp": 30,
//!         "warning": "Cannot set temperature as control unit is in
//!                   configuration mode!"
//!     }
//------------------------------------------------------------------------------
//! Class to handle pressed button debounce effect and register pressed button
//! event
class btnDebounce{
private:
    byte inputPin;
    unsigned long lastDebounceTime;
    //! Current input state
    byte inputState;
    //! Previous input state
    byte lastInputState;
public:
    btnDebounce(byte ip);
    bool updateRead(bool);
};
//------------------------------------------------------------------------------
btnDebounce::btnDebounce(byte ip){
    lastDebounceTime    = 0;
    inputPin            = ip;
    lastInputState      = LOW;
}
//------------------------------------------------------------------------------
bool btnDebounce::updateRead(bool playBuzzer){
    bool buttonAction = false;
    byte lclUp = digitalRead(inputPin);
    if(lclUp != lastInputState){
        //! Reset debounce timer for the Up button toggle event
        lastDebounceTime = millis();
    }   
    if((millis() - lastDebounceTime) > BTN_DEB_DELAY){
        if(inputState != lclUp){
            inputState = lclUp;
            //! Toggle output pin only if input pin was HIGH
            if(inputState == HIGH){
                //! Register pressed button event
                buttonAction = true;

                if(playBuzzer){
                    //! Play buzzer sound on pushed button
                    tone(BUZZER_PIN, 5000, 500);
                    delay(1);
                    noTone(BUZZER_PIN);
                }
            }
        }
    }   
    //! Assign local input pin read state to previous input pin state
    lastInputState = lclUp;
    //! Return if user pressed button action has been registered
    return buttonAction;
}
//------------------------------------------------------------------------------
//! Actual enclosure temperature variable
double actualTemp;
//! Desired enclosure temperature variable (PID setpoint)
int desiredTemp;
//! Temporary temperature value displyed on setup
int tmpTemp;
//! PID input value will be calculated according to actualTemp value
double pidInput;
//! PID output (fan expects duty cycle value 0 - 100)
double pidOutput;
//! PID setpoint which is equal to desired temperature
double pidSetpoint;
//! PID parameters
double Kp, Ki, Kd;
//! Time registry variable on the last temperature request
unsigned long lastTempReqTime;
//! Use enum for identification of used array of number collections (tenths or
//! ones). It also determines in which position to place the LCD cursor
enum setupMode{tenths = 1, ones, off};
//! Flag having one of setupMode values assigned (tenths comes always first)
setupMode inMode = setupMode::off;
//! Static text length on LCD
size_t phraseLen = strlen(STRING_ACTUAL_TEMP);
//! Digits array for temperature tenths (2 - 5)
byte destTempDigitsTenths[LENGTH_OF_TENTHS] = {2, 3, 4, 5};
//! Digits array for temperature ones (0 - 9)
byte destTempDigitsOnes[LENGTH_OF_ONES] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
//! Index pointing to the member in one of both digit collections
byte digitIndex;
//------------------------------------------------------------------------------
//! Instantiate button action detectors
//! Button UP press
btnDebounce UpLedButtn((byte)BUTTON_UP_PIN);
//! Button DOWN press
btnDebounce DownLedButtn((byte)BUTTON_DOWN_PIN);
//! Button APPLY press
btnDebounce ApplyLedButtn((byte)BUTTON_APPLY_PIN);
//! OneWire instance to communicate with temperature sensor
OneWire oneWire(ENCL_T_PIN);
//! Instantiate temperature sensor using OneWire instance
DallasTemperature dsSensors(&oneWire);
//! Instantiate 16x2 LCD using pre-detected I2C device address
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 16, 2);
//! Instantiate Bluetooth object
SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN);
//! Create PID instance
PID pwmFanPID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, REVERSE);
//------------------------------------------------------------------------------
//! Print supported temperature range
void printSupportedTempRange(String& range){
    int minTenths   = destTempDigitsTenths[0];
    int minOnes     = destTempDigitsOnes[0];
    int maxTenths   = destTempDigitsTenths[LENGTH_OF_TENTHS-1];
    int maxOnes     = destTempDigitsOnes[LENGTH_OF_ONES-1];
    range           = String(minTenths) + String(minOnes) + "-"
                                          + String(maxTenths) + String(maxOnes);
}
//------------------------------------------------------------------------------
//! Format response JOSN string
void sendResponse(const String& dataFmt, const String& message,
                                                            const char* method){
    String response = "{\"method\":\""+ String(method) + "\",\"destTemp\":" +
                                                  dataFmt + "," + message + "}";
    btSerial.println(response);
}
//------------------------------------------------------------------------------
//! Format warning message informing user that someone is already configuring
//! the target temperature using buttons
void reportJSONRPCWarning(int tmpDesiredTemp, const char* method){
    String message = "\"warning\":\"Cannot set temperature as control unit is"
                                                    " in configuration mode!\"";
    sendResponse(String(tmpDesiredTemp), message, method);
}
//------------------------------------------------------------------------------
//! Format error message with invalid target temperature
void reportJSONRPCError(int tmpDesiredTemp, const char* method){
    String range;
    printSupportedTempRange(range);
    String message = "\"error\":\"Invalid target temperature."
                                                 "Valid range: " + range + "\"";
    sendResponse(String(tmpDesiredTemp), message, method);
}
//------------------------------------------------------------------------------
//! Format successful temperature setup result message
void reportJSONRPCSucc(int tmpDesiredTemp, const char* method){
    String message = "\"success\":\"Target temperature set to " +
                                                  String(tmpDesiredTemp) + "\"";
    sendResponse(String(tmpDesiredTemp), message, method);
}
//------------------------------------------------------------------------------
//! JSON RPC method in order to set target temperature via Bluetooth
static void setdestemp(struct jsonrpc_request* r){
    double tmpDesiredTempDbl;
    mjson_get_number(r->params, r->params_len, "$[0]", &tmpDesiredTempDbl);
    int tmpDesiredTemp = (int)tmpDesiredTempDbl;
    //! If temperature is being configured already using buttons, ignore this
    //! request (report warning message)
    if(inMode != setupMode::off){
        reportJSONRPCWarning(tmpDesiredTemp, __func__);
        return;
    }
    //! Validate value:
    //! Is it non-negative?
    if(tmpDesiredTemp < 0){
        reportJSONRPCError(tmpDesiredTemp, __func__);
        return;
    }
    //! Is in temperature range?
    //! Get desired target temperature tenths
    int tenths = tmpDesiredTemp / 10;
    //! ...and ones
    int ones = tmpDesiredTemp - tenths * 10;
    //! Find in array of tenths
    bool found = false;
    for(byte i=0;i<LENGTH_OF_TENTHS;i++){
        if(destTempDigitsTenths[i] == tenths){
            found = true;
            break;
        }
    }
    //! Report error if did not found
    //! Tenths digit unsupported
    if(!found){
        reportJSONRPCError(tmpDesiredTemp, __func__);
        return;
    }
    //! Find in array of ones
    found = false;
    for(byte i=0;i<LENGTH_OF_ONES;i++){
        if(destTempDigitsOnes[i] == ones){
            found = true;
            break;
        }
    }
    //! Report error if did not found
    //! Ones digit unsupported
    if(!found){
        reportJSONRPCError(tmpDesiredTemp, __func__);
        return;
    }
    //! If desired target temperature is the same as one already set than
    //! do not set anything. Report about successful result
    if(desiredTemp == tmpDesiredTemp){
        reportJSONRPCSucc(tmpDesiredTemp, __func__);
        return;
    }

    //! If range is valid and temperature values are not equal set it as new
    //! target temperature and display it alos on LCD
    desiredTemp = tmpDesiredTemp;
    lcd.setCursor(phraseLen,1);
    lcd.print(desiredTemp);
    //! Report about successful result and new target temperature
    reportJSONRPCSucc(tmpDesiredTemp, __func__);
}
//------------------------------------------------------------------------------
//! Gets called by the RPC engine to send a reply frame
static int wfn(const char *frame, int frame_len, void *privdata) {
    btSerial.println(frame);
    return frame_len;
}
//------------------------------------------------------------------------------
void setup() {
    //! Setup button pin modes
    pinMode(BUTTON_UP_PIN,          INPUT);
    pinMode(BUTTON_DOWN_PIN,        INPUT);
    pinMode(BUTTON_APPLY_PIN,       INPUT);
    //! Setup relay pin mode
    pinMode(RELAY_MNG_PIN,          OUTPUT);
    //! Setup fan pin mode
    pinMode(FAN_PWM_PIN,            OUTPUT);
    //! This breaks pins 9&10 and sets timer to 25000 Hz frequency
    Timer1.initialize(40);
    //! Start up temperature sensors
    dsSensors.begin();
    
    //! Setup LCD
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print(STRING_ACTUAL_TEMP);
    lcd.setCursor(0,1);
    lcd.print(STRING_DESIRED_TEMP);

    //! Set initial OUTPUT states
    digitalWrite(BUTTON_UP_PIN,     LOW);
    digitalWrite(BUTTON_DOWN_PIN,   LOW);
    digitalWrite(BUTTON_APPLY_PIN,  LOW);
    digitalWrite(RELAY_MNG_PIN,     LOW);
    digitalWrite(FAN_PWM_PIN,       LOW);

    //! Initialize all three temperature values the same as actual ambient
    //! temperature
    dsSensors.requestTemperatures();
    desiredTemp = tmpTemp = actualTemp = dsSensors.getTempCByIndex(0);
    lastTempReqTime = millis();
    //! Display temperature values on LCD
    lcd.setCursor(phraseLen,0);
    lcd.print(actualTemp);
    lcd.setCursor(phraseLen,1);
    lcd.print(desiredTemp);

    Serial.begin(9600);
    btSerial.begin(9600);

    //! JSON RPC initialization
    jsonrpc_init(NULL, NULL);
    //! Set handler for JSON RPC request to configure target temperature
    jsonrpc_export("setdestemp", setdestemp);

    //! Set PID limitations (0 - 100)
    pwmFanPID.SetOutputLimits(0, FULL_DUTY);
    pwmFanPID.SetOutputLimits(0, 100);
    //! Turn the PID on
    pwmFanPID.SetMode(AUTOMATIC);
    //! Setup PID tuning for fan control
    Kp = 2;
    Ki = 5;
    Kd = 1;
    pwmFanPID.SetTunings(Kp, Ki, Kd);
    //! Set initial fan duty cycle to 0 (OFF)
    Timer1.pwm(FAN_PWM_PIN, 0);
}
//------------------------------------------------------------------------------
void computePID(){
    //! Set input value (actual temperature)
    pidInput    = actualTemp;
    //! Update setpoint value (which is user defined target temperature)
    pidSetpoint = (double)desiredTemp;
    //! Calculate PID
    pwmFanPID.Compute();
    Serial.println("-------------------------");
    Serial.print("Input: ");
    Serial.println(pidInput);
    Serial.print("Output: ");
    Serial.println(pidOutput);
    Serial.print("Setpoint: ");  
    Serial.println(pidSetpoint);
}
//------------------------------------------------------------------------------
void updateActualTemperature(){
    //! If in configuration mode than do not update
    if(inMode != setupMode::off)
        return;
    //! If more than 5 seconds between requests has passed do a temperature req.
    unsigned long now = millis();
    if(now - lastTempReqTime > 1000 * TEMP_POLL_INTV){
        dsSensors.requestTemperatures();
        lastTempReqTime = millis();
        int prevTemp = actualTemp;
        //! Read new temperature value
        actualTemp = dsSensors.getTempCByIndex(0);
        //! Compare temperature values. Update if differ
        if(prevTemp != actualTemp){
            //! Update temperature in LCD
            //! Clean previous values displayed
            for(size_t i = phraseLen; i < 16; i++){
                lcd.setCursor(i,0);
                lcd.print(" ");
            }
            lcd.setCursor(phraseLen,0);
            lcd.print(actualTemp);
            //! Update PID calculations
            computePID();
        }
    }
}
//------------------------------------------------------------------------------
void loop(){
    //! Register pressed buttons
    bool playBuzzer = inMode != setupMode::off ? true : false;
    bool btnUpPressed = UpLedButtn.updateRead(playBuzzer);
    bool btnDwPressed = DownLedButtn.updateRead(playBuzzer);
    bool btnExPressed = ApplyLedButtn.updateRead(true);
    //! Handle pressed buttons - button UP
    if(btnUpPressed){
        //! Desired target temperature incremental setup based on actual
        //! inMode value
        switch(inMode){
            case setupMode::tenths:{
                //! Increase tenths by one
                digitIndex += 1;
                //! If index value reached LENGTH_OF_TENTHS, than reset it to 0
                if(digitIndex == LENGTH_OF_TENTHS)
                    digitIndex = 0;
                //! Get digit value from the array
                byte tenthsValue = destTempDigitsTenths[digitIndex];
                //! Get ones from temporary temperature
                tmpTemp = tmpTemp - (tmpTemp / 10) * 10;
                //! Set new tenths to the temporary temperature value
                tmpTemp = tmpTemp + (tenthsValue * 10);
                //! Display digit in the LCD
                lcd.print(tenthsValue);
                lcd.setCursor(phraseLen,1);
                break;
            }
            case setupMode::ones:{
                //! Increase ones by one
                digitIndex += 1;
                //! If index value reached LENGTH_OF_ONES, than reset it to 0
                if(digitIndex == LENGTH_OF_ONES)
                    digitIndex = 0;
                //! Get digit value from the array
                byte onesValue = destTempDigitsOnes[digitIndex];
                //! Get tenths from temporary temperature
                 tmpTemp = (tmpTemp / 10) * 10;
                 //! Set new ones to the temporary temperature value
                 tmpTemp = tmpTemp + onesValue;
                 //! Display digit in the LCD
                 lcd.print(onesValue);
                 lcd.setCursor(phraseLen+1,1);
                break;
            }
            case setupMode::off:{
                //! do nothing
                break;
            }
        }
    }
    //! Handle pressed buttons - button DOWN
    if(btnDwPressed){
        //! Desired target temperature decremental setup based on actual
        //! inMode value
        switch(inMode){
            case setupMode::tenths:{
                //! Decrease tenths by one
                digitIndex -= 1;
                //! If index value is greater than LENGTH_OF_TENTHS than reset 
                //! it to LENGTH_OF_TENTHS-1
                if(digitIndex > LENGTH_OF_TENTHS)
                    digitIndex = LENGTH_OF_TENTHS - 1;
                //! Get digit value from the array
                byte tenthsValue = destTempDigitsTenths[digitIndex];
                //! Get ones from temporary temperature
                tmpTemp = tmpTemp - (tmpTemp / 10) * 10;
                //! Set new tenths to the temporary temperature value
                tmpTemp = tmpTemp + (tenthsValue * 10);
                //! Display digit in the LCD
                lcd.print(tenthsValue);
                lcd.setCursor(phraseLen,1);
                break;
            }
            case setupMode::ones:{
                //! Decrease ones by one
                digitIndex -= 1;
                //! If index value is greater than LENGTH_OF_ONES than reset it
                //! to LENGTH_OF_ONES-1
                if(digitIndex > LENGTH_OF_ONES)
                    digitIndex = LENGTH_OF_ONES - 1;
                //! Get digit value from the array
                byte onesValue = destTempDigitsOnes[digitIndex];
                //! Get tenths from temporary temperature
                 tmpTemp = (tmpTemp / 10) * 10;
                 //! Set new ones to the temporary temperature value
                 tmpTemp = tmpTemp + onesValue;
                 //! Display digit in the LCD
                 lcd.print(onesValue);
                 lcd.setCursor(phraseLen+1,1);
                break;
            }
            case setupMode::off:{
                //! do nothing
                break;
            }
        }
    }
    //! Handle pressed buttons - button SET
    //! Exec (or switch between modes) button pressed - determines which cursor
    //! position to blink
    if(btnExPressed){
        //! If in setup mode already than switch between modes
        switch(inMode){
            case setupMode::tenths:{
                //! Switch to ones
                inMode = setupMode::ones;
                lcd.setCursor(phraseLen+1,1);
                lcd.blink_on();
                //! Determine ones from the temporary temperature as it is being
                //! altered by user (now in ones mode)
                byte onesDigit = tmpTemp - (tmpTemp / 10) * 10;
                //! Get index by iterating through ones
                for(byte i=0;i<LENGTH_OF_ONES;i++){
                    if(destTempDigitsOnes[i] == onesDigit){
                        digitIndex = i;
                        break;
                    }
                }
                break;
            }
            case setupMode::ones:{
                //! Switch to off
                inMode = setupMode::off;
                lcd.blink_off();
                //! Actually settings are being applied here
                //! Assign temporary temperature to the desired target
                //! temperature
                desiredTemp = tmpTemp;
                break;
            }
            case setupMode::off:{
                //! Switch to tenths
                inMode = setupMode::tenths;
                lcd.setCursor(phraseLen,1);
                lcd.blink_on();
                //! Also determine actual target temperature and find index
                //! in collection of tenths digits
                byte actualTenthDigit = desiredTemp / 10;
                //! Also assign desired temperature to temporary value which
                //! will probably be altered by user setting tenths and ones
                tmpTemp = desiredTemp;
                //! Get index by iterating through tenths
                for(byte i=0;i<LENGTH_OF_TENTHS;i++){
                    if(destTempDigitsTenths[i] == actualTenthDigit){
                        digitIndex = i;
                        break;
                    }
                }
                break;
            }
        }
    }

    //! Temperature setup via Bluetooth - listen for the requests
    if(btSerial.available()){
        String rxString = btSerial.readString();
        jsonrpc_process(rxString.c_str(), rxString.length(), wfn, NULL, NULL);
    }

    //! Read actual temperature value
    updateActualTemperature();

    //! If temperature above desired target temperature value
    //! use fan to suck out warm air from enclosure in order to sustain ambient
    //! temperature below the below desired target temperature value
    //! Control fan PWM (here is the place for PID controlled PWM input for fan)
    if(actualTemp > desiredTemp)
        //! Assign PID-calculated output value as duty cycle for the fan
        Timer1.pwm(FAN_PWM_PIN, ((double)pidOutput / (double)FULL_DUTY)
                                                              * MAX_DUTY_VALUE);
    else
        //! Turn of fan by setting duty cycle to 0
        Timer1.pwm(FAN_PWM_PIN, 0);

    //! If temperature below desired target temperature value use heater to
    //! heat up ambient temperature until it reaches desired target
    //! temperature value (relay is used hear to turn ON and OFF the AC 220V
    //! encloder heater)
    if(actualTemp <= desiredTemp)
        //! Turn heater ON
        digitalWrite(RELAY_MNG_PIN, HIGH);
    else
        //! Turn heater OFF
        digitalWrite(RELAY_MNG_PIN, LOW);
}
//------------------------------------------------------------------------------