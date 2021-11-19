/* Import Library */
#include <AntaresESP32HTTP.h>
#include <Arduino.h>
#include "HLW8012.h"

/* Configuration */

/* Debug */
#define is_debug    true

/* Baudrate Config */
#define SERIAL_BAUDRATE 115200

/* GPIO Config */
#define RELAY_PIN       22
#define SEL_PIN         16
#define CF1_PIN         17
#define CF_PIN          21

/* Wifi Configuration */
#define WIFISSID    "DXB"
#define PASSWORD    "telkom2021"

/* Apps Configuration */
#define ACCESSKEY       "ec181439c172f36b:322dbade46fce459"
#define applicationName "Indobot"
#define dat_device    "Database"
#define rel_device    "Relay"

/* Set SEL_PIN to HIGH to sample current */
/* This is the case for Itead's Sonoff POW, where a
   the SEL_PIN drives a transistor that pulls down
   the SEL pin in the HLW8012 when closed */
#define CURRENT_MODE    HIGH

/* These are the nominal values for the resistors in the circuit */
#define CURRENT_RESISTOR                0.001
#define VOLTAGE_RESISTOR_UPSTREAM       ( 5 * 470000 ) // Real: 2280k
#define VOLTAGE_RESISTOR_DOWNSTREAM     ( 1000 ) // Real 1.009k

/* Sensor variables */
float power;
float active_power;
float power_factor;
float current;
float voltage;
int rel_stat;

/* Delay variables */
unsigned long rel_timeout;
bool timer_done;

/* Initialization */
AntaresESP32HTTP dat_dev(ACCESSKEY);
AntaresESP32HTTP rel_dev(ACCESSKEY);
HLW8012 hlw8012;

/* Procedure */
void defineDelay(){
  static unsigned long last = millis();
  if ((millis() - last) > rel_timeout) {
    last = millis();
    timer_done = true;
  }
}  

void getRelayData(){
  rel_dev.get(applicationName, rel_device);
  rel_stat = rel_dev.getInt("relay");
  rel_timeout = rel_dev.getInt("timeout");
  if(rel_timeout < 2000)
    rel_timeout = 2000;
}

void getHLWData(){
  if(rel_stat == 0){
    digitalWrite(RELAY_PIN,HIGH);
  }
  else if(rel_stat == 1){
    digitalWrite(RELAY_PIN,LOW);
    active_power   = hlw8012.getActivePower();
    power          = hlw8012.getApparentPower();
    power_factor   = 100 * hlw8012.getPowerFactor();
    voltage        = hlw8012.getVoltage();
    current        = hlw8012.getCurrent();
  }
}

void printData(){
  Serial.print("[HLW] Power (VA)          : "); Serial.println(power);
  Serial.print("[HLW] Voltage (V)         : "); Serial.println(voltage);
  Serial.print("[HLW] Current (A)         : "); Serial.println(current);
  Serial.println();
}
void sendData(){
  dat_dev.add("power", power);
  dat_dev.add("voltage_rms", voltage);
  dat_dev.add("current_rms", current);
  
  /* When not using interrupts we have to manually switch to current or voltage monitor
    This means that every time we get into the conditional we only update one of them
    while the other will return the cached value. */
  hlw8012.toggleMode();        
  
  /* Send from buffer to Antares */
  dat_dev.send(applicationName, dat_device);
}
    
void setup() {
/* Setup Serial */
  Serial.begin(SERIAL_BAUDRATE);
  
/* Setup Pin */
  pinMode     (RELAY_PIN,OUTPUT);
  digitalWrite(RELAY_PIN,HIGH);
  
/* Antares */
/* Setup Power Database */
  dat_dev.setDebug(is_debug);
  dat_dev.wifiConnection(WIFISSID,PASSWORD);
  
/* Setup Relay Database */
  rel_dev.setDebug(is_debug);
  rel_dev.wifiConnection(WIFISSID,PASSWORD); 
  
/* HLW8012 */
  /* Close the relay to switch on the load */
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  
/* Initialize HLW8012 */
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
/* Show default (as per datasheet) multipliers */
  Serial.print("[HLW] Default current multiplier : "); Serial.println(hlw8012.getCurrentMultiplier());
  //_current_multiplier = ( 1000000.0 * 512 * V_REF / _current_resistor / 24.0 / F_OSC );
  Serial.print("[HLW] Default voltage multiplier : "); Serial.println(hlw8012.getVoltageMultiplier());
  //_voltage_multiplier = ( 1000000.0 * 512 * V_REF * _voltage_resistor / 2.0 / F_OSC );
  Serial.print("[HLW] Default power multiplier   : "); Serial.println(hlw8012.getPowerMultiplier());
  //_power_multiplier = ( 1000000.0 * 128 * V_REF * V_REF * _voltage_resistor / _current_resistor / 48.0 / F_OSC );
  
  Serial.println();
  timer_done = false;
  rel_timeout = 2000;    

}

void loop() {
  getRelayData();
  defineDelay();
  if(timer_done){
    getHLWData();
    printData();
    sendData();
    timer_done = false;
  }
}
