// Import Library
  #include <AntaresESP32HTTP.h>
  #include <Arduino.h>
  #include "HLW8012.h"
  
// Configuration
  // Debug
    #define is_debug    true
  // Baudrate Config
    #define SERIAL_BAUDRATE 115200
  // GPIO Config
    #define RELAY_PIN       22
    #define SEL_PIN         16
    #define CF1_PIN         17
    #define CF_PIN          21
  // Wifi Configuration
    #define WIFISSID    "DXB"
    #define PASSWORD    "telkom2021"
  // Apps Configuration
    #define ACCESSKEY       "ec181439c172f36b:322dbade46fce459"
    #define applicationName "Indobot"
    #define dat_device    "Database"
    #define rel_device    "Relay"

  // Set SEL_PIN to HIGH to sample current
    // This is the case for Itead's Sonoff POW, where a
    // the SEL_PIN drives a transistor that pulls down
    // the SEL pin in the HLW8012 when closed
    #define CURRENT_MODE    HIGH
  // These are the nominal values for the resistors in the circuit
    #define CURRENT_RESISTOR                0.001
    #define VOLTAGE_RESISTOR_UPSTREAM       ( 5 * 470000 ) // Real: 2280k
    #define VOLTAGE_RESISTOR_DOWNSTREAM     ( 1000 ) // Real 1.009k
  //Sensor variables
    float power;
    float current;
    float voltage;
    int rel_stat;
  //Delay variables
    static unsigned long last;
    
// Initialization
  AntaresESP32HTTP dat_dev(ACCESSKEY);
  AntaresESP32HTTP rel_dev(ACCESSKEY);
  HLW8012 hlw8012;
  
// Procedure
    void unblockingDelay(unsigned long mseconds) {
        unsigned long timeout = millis();
        while ((millis() - timeout) < mseconds) delay(1);
    }
    void calibrate() {
        // Let's first read power, current and voltage
        // with an interval in between to allow the signal to stabilise:
            hlw8012.getActivePower(); //beri nilai power_pulse_width dari pulseIN cf_pin
                                      //_power = (_power_pulse_width > 0) ? _power_multiplier / _power_pulse_width / 2 : 0;
            hlw8012.setMode(MODE_CURRENT);  //set SEL pin jadi current
            unblockingDelay(2000);  //delay timeout
            hlw8012.getCurrent(); //beri nilai current_pulse_width dari pulseIN cf1_pin
                                  // _current = (_current_pulse_width > 0) ? _current_multiplier / _current_pulse_width / 2 : 0;
            hlw8012.setMode(MODE_VOLTAGE); //set SEL pin jadi 1 - current
            unblockingDelay(2000);  //delay timeout
            hlw8012.getVoltage(); //beri nilai voltage_pulse_width dari pulseIN cf1_pin
                                  //_voltage = (_voltage_pulse_width > 0) ? _voltage_multiplier / _voltage_pulse_width / 2 : 0;
            
        // Calibrate using a 60W bulb (pure resistive) on a 230V line
            hlw8012.expectedActivePower(40.0);  //_power_multiplier *= ((double) value / _power);
            hlw8012.expectedVoltage(222.0); //_voltage_multiplier *= ((double) value / _voltage);
            hlw8012.expectedCurrent(40.0 / 222.0);  //_current_multiplier *= (value / _current);
        // Show corrected factors
            Serial.print("[HLW] New current multiplier : "); Serial.println(hlw8012.getCurrentMultiplier());
            Serial.print("[HLW] New voltage multiplier : "); Serial.println(hlw8012.getVoltageMultiplier());
            Serial.print("[HLW] New power multiplier   : "); Serial.println(hlw8012.getPowerMultiplier());
            Serial.println();
    }
    void sendData(){
        dat_dev.add("power", power);
        dat_dev.add("voltage_rms", voltage);
        dat_dev.add("current_rms", current);
        // Send from buffer to Antares
        dat_dev.send(applicationName, dat_device);
    }
    void getRelayData(){
        rel_dev.get(applicationName, rel_device);
        rel_stat = rel_dev.getInt("relay");
    }
    void defineDelay(){
        last = millis();
        if ((millis() - last) > 2000) {
        last = millis();
        }
    }
    void printData(){
      Serial.print("[HLW] Voltage (V)         : "); Serial.println(voltage);
      Serial.print("[HLW] Current (A)         : "); Serial.println(current);
      Serial.print("[HLW] Power (VA)          : "); Serial.println(power);
      Serial.println();
    }

    void getHLWData(){
    if(rel_stat == 0){
      digitalWrite(RELAY_PIN,HIGH);
      }
    else if(rel_stat == 1){
      digitalWrite(RELAY_PIN,LOW);
      power = hlw8012.getApparentPower();
      voltage        = hlw8012.getVoltage();
      current        = hlw8012.getCurrent()*1000;
      }
    }
void setup() {
  // General
    // Setup Serial
      Serial.begin(SERIAL_BAUDRATE);
    // Setup Pin
      pinMode     (RELAY_PIN,OUTPUT);
      digitalWrite(RELAY_PIN,HIGH);
  // Antares
    // Setup Power Database
      dat_dev.setDebug(is_debug);
      dat_dev.wifiConnection(WIFISSID,PASSWORD);
    // Setup Relay Database
      rel_dev.setDebug(is_debug);
      rel_dev.wifiConnection(WIFISSID,PASSWORD); 
  // HLW8012
    // Close the relay to switch on the load
      pinMode(RELAY_PIN, OUTPUT);
      digitalWrite(RELAY_PIN, LOW);
    // Initialize HLW8012
      // void begin(unsigned char cf_pin, unsigned char cf1_pin, unsigned char sel_pin, unsigned char currentWhen = HIGH, bool use_interrupts = false, unsigned long pulse_timeout = PULSE_TIMEOUT);
      // * cf_pin, cf1_pin and sel_pin are GPIOs to the HLW8012 IC
      // * currentWhen is the value in sel_pin to select current sampling
      // * set use_interrupts to false, we will have to call handle() in the main loop to do the sampling
      // * set pulse_timeout to 500ms for a fast response but losing precision (that's ~24W precision :( )
      hlw8012.begin(CF_PIN, CF1_PIN, SEL_PIN, CURRENT_MODE, false, 500000);
    // These values are used to calculate current, voltage and power factors as per datasheet formula
      // These are the nominal values for the Sonoff POW resistors:
      // * The CURRENT_RESISTOR is the 1milliOhm copper-manganese resistor in series with the main line
      // * The VOLTAGE_RESISTOR_UPSTREAM are the 5 470kOhm resistors in the voltage divider that feeds the V2P pin in the HLW8012
      // * The VOLTAGE_RESISTOR_DOWNSTREAM is the 1kOhm resistor in the voltage divider that feeds the V2P pin in the HLW8012
      hlw8012.setResistors(CURRENT_RESISTOR, VOLTAGE_RESISTOR_UPSTREAM, VOLTAGE_RESISTOR_DOWNSTREAM);
    // Show default (as per datasheet) multipliers
      Serial.print("[HLW] Default current multiplier : "); Serial.println(hlw8012.getCurrentMultiplier());
        //_current_multiplier = ( 1000000.0 * 512 * V_REF / _current_resistor / 24.0 / F_OSC );
      Serial.print("[HLW] Default voltage multiplier : "); Serial.println(hlw8012.getVoltageMultiplier());
        //_voltage_multiplier = ( 1000000.0 * 512 * V_REF * _voltage_resistor / 2.0 / F_OSC );
      Serial.print("[HLW] Default power multiplier   : "); Serial.println(hlw8012.getPowerMultiplier());
        //_power_multiplier = ( 1000000.0 * 128 * V_REF * V_REF * _voltage_resistor / _current_resistor / 48.0 / F_OSC );
      Serial.println();
    //calibrate();
}

void loop() {
    defineDelay();
    getRelayData();

    Serial.print("Relay Status: "); Serial.println(rel_stat);

    // When not using interrupts we have to manually switch to current or voltage monitor
    // This means that every time we get into the conditional we only update one of them
    // while the other will return the cached value.
    hlw8012.toggleMode();
}
