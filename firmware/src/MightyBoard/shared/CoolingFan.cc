#include "CoolingFan.hh"
//#include "ExtruderMotor.hh"
#include "Eeprom.hh"
#include "EepromMap.hh"
#ifdef IS_EXTRUDER_BOARD
#include "ExtruderBoard.hh"
#endif

#define FAN_ENABLED 1
#define FAN_DISABLED 0

static char en;

// TODO: Come up with a unified strategy for these.
// EEPROM map


CoolingFan::CoolingFan(uint8_t id, Heater& heater_in, uint16_t eeprom_base_in, const Pin &fan, const Pin &fan2) :
	slave_id(id),
        heater(heater_in),
        eeprom_base(eeprom_base_in),
        Fan_Pin(fan),
	Fan_salve(fan2)
{
	en = 0;
	reset();
}

void CoolingFan::reset() {
	uint16_t offset = eeprom_base + cooler_eeprom_offsets::SETPOINT_C_OFFSET;
	setSetpoint(eeprom::getEeprom8(offset, DEFAULT_COOLING_FAN_SETPOINT_C));

	Fan_Pin.setValue(false);
	Fan_Pin.setDirection(true);

	offset = eeprom_base + cooler_eeprom_offsets::ENABLE_OFFSET;
	if (eeprom::getEeprom8(offset ,DEFAULT_COOLING_FAN_ENABLE) == FAN_ENABLED) {
		enable();
	}
	else {
		disable();
	}
	fan_on = false;
}

void CoolingFan::setSetpoint(int temperature) {
	setPoint = temperature;
	midSetPoint = temperature;
	lowSetPoint = temperature - 10;
	highSetPoint = temperature + 10;
}

void CoolingFan::enable() {
	enabled = true;
}

void CoolingFan::disable() {
	enabled = false;
	disableFan();
}

void CoolingFan::manageCoolingFan() {
	// TODO: only change the state if necessary
	if (enabled) {
		int temp = heater.get_current_temperature();
		
		if ((temp > setPoint) && (temp != DEFAULT_THERMOCOUPLE_VAL)){
			enableFan();
			// hysteresis in fan on/off behavior
			if(!fan_on && temp < highSetPoint){
				setPoint = lowSetPoint;
			}
			else{
				fan_on = true;
				setPoint = midSetPoint;
			}
			
		}
		else {
			disableFan();
			// hysteresis in fan on/off behavior
			if(fan_on && temp > lowSetPoint){
				setPoint = highSetPoint;
			}
			else{
				fan_on = false;
				setPoint = midSetPoint;
			}
		}
	}
}

void CoolingFan::enableFan() {
	if (slave_id==1) en|=1;
	else en|=2;
	Fan_salve.setValue(true);
//#ifdef IS_EXTRUDER_BOARD
	Fan_Pin.setValue(true);
//#else
//	#warning cooling fan feature disabled
//#endif
}

void CoolingFan::disableFan() {
	if (slave_id==1) en&=2;
	else en&=1;
//#ifdef IS_EXTRUDER_BOARD
//#warning cooling fan feature disabled
	if (en==0)
	{
		Fan_Pin.setValue(false);
		Fan_salve.setValue(false);
	}

//#else
//	#warning cooling fan feature disabled
//#endif
}
