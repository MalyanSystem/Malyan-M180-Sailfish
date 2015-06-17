#include "LcdBoard.hh"
#include "Configuration.hh"
#include "Host.hh"
#include "Timeout.hh"
#include "Command.hh"
#include "Motherboard.hh"
#include "Steppers.hh"
#include "EepromMap.hh"
#include "Eeprom.hh"
#include <avr/eeprom.h>
#include <avr/wdt.h>

#include <stdlib.h>

int16_t jog_speed=250;

void writeString(char message[]) {
    char* letter = message;
    while (*letter != 0) {
        put(*letter);
        letter++;
    }
}

void LcdBoard::writeInt(uint16_t value, uint8_t digits) {
    uint32_t currentDigit, nextDigit, uvalue;
    
    switch (digits) {
        case 1:  currentDigit = 10;	     break;
        case 2:  currentDigit = 100;     break;
        case 3:  currentDigit = 1000;    break;
        case 4:  currentDigit = 10000;   break;
        case 5:  currentDigit = 100000;  break;
        case 6:  currentDigit = 1000000;  break;
        default: return;
    }
    
    uvalue = (uint32_t)value;
    for (uint8_t i = 0; i < digits; i++) {
        nextDigit = currentDigit / 10;
        put((uvalue % currentDigit) / nextDigit + '0');
        currentDigit = nextDigit;
    }
}

void LcdBoard::writeInt32(uint32_t value, uint8_t digits) {
    uint32_t currentDigit, nextDigit, uvalue;
    
    switch (digits) {
        case 1:  currentDigit = 10;      break;
        case 2:  currentDigit = 100;     break;
        case 3:  currentDigit = 1000;    break;
        case 4:  currentDigit = 10000;   break;
        case 5:  currentDigit = 100000;  break;
        case 6:  currentDigit = 1000000;  break;
        case 7:  currentDigit = 10000000;  break;
        default: return;
    }
    
    uvalue = (uint32_t)value;
    for (uint8_t i = 0; i < digits; i++) {
        nextDigit = currentDigit / 10;
        put((uvalue % currentDigit) / nextDigit + '0');
        currentDigit = nextDigit;
    }
}


LcdBoard::LcdBoard() :
	waitingMask(0)
{
}

void LcdBoard::init() {
    building = false;

    screenIndex = -1;
	waitingMask = 0;
    pushScreen(0);
    screen_locked = false;
    buttonRepetitions = 0;
    lockoutButtonRepetitionsClear = false;
    
    buff_state=0;
    writeString((char *)"{SYS:STARTED}");
}

void LcdBoard::resetLCD() {
}

extern uint8_t file_from_wifi;//,last_file_from_wifi;
extern uint8_t countFiles(bool update);
extern bool getFilename(uint8_t index, char buffer[], uint8_t buffer_size, uint8_t *buflen, bool *isdir);

void LcdBoard::ListFile(uint8_t index)
{
    uint8_t idx, filenameLength;
	uint8_t longFilenameOffset = 0;
	uint8_t displayWidth = LCD_SCREEN_WIDTH - 1;
	uint8_t offset = 1;
	char fnbuf[64]; // extra +1 since we may precede the name with a folder indicator
	bool isdir;
    
    getFilename(index, fnbuf, sizeof(fnbuf), &filenameLength, &isdir);
    
	if ( isdir ) writeString((char *)"{DIR:");
    else writeString((char *)"{FILE:");
    
    writeString(fnbuf);
    put('}');
}

void BuildScreen(uint8_t i)
{
    if (i==1) writeString((char *)"{SYS:BUILD}");
    else writeString((char *)"{SYS:ENDOFBUILD}");
}

void LcdBoard::PrintingStatus()
{
    int16_t t;
    int32_t t32;
    
    Motherboard& board = Motherboard::getBoard();
    writeString((char *)"{T0:");
    t=board.getExtruderBoard(0).getExtruderHeater().get_current_temperature();
    if (t>999) t=999;
    writeInt(t,3);
    put('/');
    t=board.getExtruderBoard(0).getExtruderHeater().get_set_temperature();
    writeInt(t,3);
    put('}');
    
    writeString((char *)"{T1:");
    t=board.getExtruderBoard(1).getExtruderHeater().get_current_temperature();
    if (t>999) t=999;
    writeInt(t,3);
    put('/');
    t=board.getExtruderBoard(1).getExtruderHeater().get_set_temperature();
    writeInt(t,3);
    put('}');
    
    writeString((char *)"{TP:");
    t=board.getPlatformHeater().get_current_temperature();
    if (t>999) t=999;
    writeInt(t,3);
    put('/');
    t=board.getPlatformHeater().get_set_temperature();
    writeInt(t,3);
    put('}');
    
    writeString((char *)"{TQ:");
    t=command::getBuildPercentage();
    writeInt(t,3);
    switch(host::getHostState())
    {
        case host::HOST_STATE_BUILDING_ONBOARD:
        case host::HOST_STATE_BUILDING:
        case host::HOST_STATE_BUILDING_FROM_SD:
            put('P');
            break;
            
        case host::HOST_STATE_HEAT_SHUTDOWN:
            put('S');
            break;
            
        default:
            put('C');
            break;
    }
    put('}');
    
    writeString((char *)"{TT:");
    t32=host::getPrintSeconds();
    writeInt32(t32,6);
    put('}');
    
    writeString((char *)"{TR:");
    t32=command::estimatedTimeLeftInSeconds();
    writeInt32(t32,6);
    put('}');
    
    writeString((char *)"{TF:");
    t32=command::filamentUsed();
    writeInt32(t32,6);
    put('}');
}

void LcdBoard::process()
{
    switch (buff_obj[0]) {
        case 'V':
            writeString((char *)"{VER:008}");
            return;
        case 'S':
            if (buff_value[0]=='E') writeString((char *)"{SYS:echo}");
            else if (buff_value[0]=='H')
            {
                uint8_t i,itemCount;
                
                itemCount=countFiles(false);
                //if (file_from_wifi!=0)
                {
                    writeString((char *)"{WIFI:");
                    writeInt(file_from_wifi,3);
                    put('/');
                    writeInt(itemCount,3);
                    put('}');
                }
            }
            else if (buff_value[0]=='L')
            {
                uint8_t i;
                uint8_t itemCount;
                
                if ( host::getHostState() != host::HOST_STATE_READY ) {
                    writeString((char *)"{SYS:BUSY}");
                    return;
                }

                itemCount = countFiles(true);
                
                for (i=0;i<itemCount;i++)
                {
                    ListFile(i);
                }

                if (itemCount==0)
                {
                    if ( (sdcard::sdAvailable == sdcard::SD_ERR_DEGRADED) ||
                        (sdcard::sdErrno & 0x80/*SDR_ERR_COMMS*/) ) i = 100;
                    else if ( sdcard::sdAvailable == sdcard::SD_SUCCESS ) i = 101;
                    else if ( sdcard::sdAvailable == sdcard::SD_ERR_NO_CARD_PRESENT ) i = 102;
                    else if ( sdcard::sdAvailable == sdcard::SD_ERR_OPEN_FILESYSTEM ) i = 103;
                    else if ( sdcard::sdAvailable == sdcard::SD_ERR_VOLUME_TOO_BIG ) i = 104;
                    else if ( sdcard::sdAvailable == sdcard::SD_ERR_CRC ) i = 105;
                    else i = 106;
                    
                    writeString((char *)"{ERR:");
                    writeInt(i,3);
                    put('}');
                }
                else writeString((char *)"{SYS:OK}");
            }
            else if (buff_value[0]=='I')
            {
                int16_t t;
                
                Motherboard& board = Motherboard::getBoard();
                
                writeString((char *)"{T0:");
                t=board.getExtruderBoard(0).getExtruderHeater().get_current_temperature();
                if (t>999) t=999;
                writeInt(t,3);
                put('/');
                t=board.getExtruderBoard(0).getExtruderHeater().get_set_temperature();
                writeInt(t,3);
                put('}');
                
                writeString((char *)"{T1:");
                t=board.getExtruderBoard(1).getExtruderHeater().get_current_temperature();
                if (t>999) t=999;
                writeInt(t,3);
                put('/');
                t=board.getExtruderBoard(1).getExtruderHeater().get_set_temperature();
                writeInt(t,3);
                put('}');
                
                writeString((char *)"{TP:");
                t=board.getPlatformHeater().get_current_temperature();
                if (t>999) t=999;
                writeInt(t,3);
                put('/');
                t=board.getPlatformHeater().get_set_temperature();
                writeInt(t,3);
                put('}');
            }
            else if (buff_value[0]=='F')
            {
                if (buff_value[1]=='X')
                {
                    eeprom::setEepromInt64(eeprom_offsets::FILAMENT_TRIP, eeprom::getEepromInt64(eeprom_offsets::FILAMENT_LIFETIME, 0));
                    eeprom::setEepromInt64(eeprom_offsets::FILAMENT_TRIP + sizeof(int64_t), eeprom::getEepromInt64(eeprom_offsets::FILAMENT_LIFETIME + sizeof(int64_t), 0));
                }
                
                writeString((char *)"{TU:");
                
                uint16_t total_hours;
                uint8_t total_minutes;
                eeprom::getBuildTime(&total_hours, &total_minutes);
                writeInt(total_hours,5);
                put('.');
                writeInt(total_minutes,2);
                put('/');
                
                uint8_t build_hours;
                uint8_t build_minutes;
                host::getPrintTime(build_hours, build_minutes);
                writeInt(build_hours,3);
                put('.');
                writeInt(build_minutes,2);
                put('/');
                
                uint32_t filamentUsedA,filamentUsedB,filamentUsed;
                char str[11];
                filamentUsedA=stepperAxisStepsToMM(eeprom::getEepromInt64(eeprom_offsets::FILAMENT_LIFETIME, 0),                  A_AXIS);
                filamentUsedB=stepperAxisStepsToMM(eeprom::getEepromInt64(eeprom_offsets::FILAMENT_LIFETIME + sizeof(int64_t),0), B_AXIS);
                filamentUsed=filamentUsedA+filamentUsedB;
                itoa(filamentUsed,str,10);
                writeString((char *)str);
                put('/');
                
                filamentUsedA -= stepperAxisStepsToMM(eeprom::getEepromInt64(eeprom_offsets::FILAMENT_TRIP, 0),                  A_AXIS);
                filamentUsedB -= stepperAxisStepsToMM(eeprom::getEepromInt64(eeprom_offsets::FILAMENT_TRIP + sizeof(int64_t),0), B_AXIS);
                filamentUsed=filamentUsedA+filamentUsedB;
                itoa(filamentUsed,str,10);
                writeString((char *)str);
                put('}');
            }
            else if (buff_value[0]=='R' && 
                buff_value[1]=='E' &&
                buff_value[2]=='S' &&
                buff_value[3]=='E' &&
                buff_value[4]=='T')
            {
                Motherboard::getBoard().reset(true);
            }
            else if (buff_value[0]=='S')
            {
                //char str[4];

                writeString((char *)"{SYS:P");
                uint8_t i = command::pauseState();
                //itoa(i,str,3);
                //writeString((char *)str);
                writeInt(i,3);
                put('/');

                //if (i==0 && eeprom::getEeprom8(eeprom_offsets::HEAT_DURING_PAUSE, DEFAULT_HEAT_DURING_PAUSE) == 0) Motherboard::heatersOff(false);

                put('H');
                i=host::getHostState();
                //itoa(i,str,3);
                //writeString((char *)str);
                writeInt(i,3);
                put('}');
            }
            break;
            
        case 'C':
            if (buff_value[0]=='P')
            {
                uint16_t t;
                
                t=atoi((const char*)buff_value+1);
                
                if (t<0 || t>150) return;
                
                Motherboard& board = Motherboard::getBoard();
                board.getPlatformHeater().set_target_temperature(t);
                if (t!=0) eeprom_write_word((uint16_t*)(eeprom_offsets::PREHEAT_SETTINGS + preheat_eeprom_offsets::PREHEAT_PLATFORM_OFFSET), t);
                //board.setUsingPlatform(true);
                
                /*writeString((char *)"{C:P");
                writeInt(t,3);
                put('}');*/
            }
            else if (buff_value[0]=='T')
            {
                int16_t t;
                
                t=atoi((const char*)buff_value+2);
                if (t<0 || t>280) return;
                
                Motherboard& board = Motherboard::getBoard();
                if (buff_value[1] == '0')
                {
                    command::altTemp[0] = t;
                    board.getExtruderBoard(0).getExtruderHeater().set_target_temperature(t);
                    if (t!=0) eeprom_write_word((uint16_t*)(eeprom_offsets::PREHEAT_SETTINGS + preheat_eeprom_offsets::PREHEAT_RIGHT_OFFSET), t);
                    /*writeString((char *)"{C:T0");
                    writeInt(t,3);
                    put('}');*/
                }
                else
                {
                    command::altTemp[1] = t;
                    board.getExtruderBoard(1).getExtruderHeater().set_target_temperature(t);
                    if (t!=0) eeprom_write_word((uint16_t*)(eeprom_offsets::PREHEAT_SETTINGS + preheat_eeprom_offsets::PREHEAT_LEFT_OFFSET), t);
                    /*writeString((char *)"{C:T1");
                    writeInt(t,3);
                    put('}');*/
                }
            }
            else if (buff_value[0]=='S')
            {
                int16_t t;
                uint8_t i;
                
                t=atoi((const char*)buff_value+1);
                
                FPTYPE sf;
                
                if (t<1) t=1;
                else if (t>50) t=50;
                
                sf = KCONSTANT_0_1;
                for (i=1;i<t;i++) sf+=KCONSTANT_0_1;
                
                /*if ( sf >= KCONSTANT_5 ) sf = KCONSTANT_5;
                else if ( sf <= KCONSTANT_0_1 ) sf = KCONSTANT_0_1;*/
                steppers::alterSpeed  = (sf == KCONSTANT_1) ? 0x00 : 0x80;
                steppers::speedFactor = sf;
                
                /*writeString((char *)"{C:S");
                writeInt(t,3);
                put('}');*/
            }
            break;
            
        case 'P':
            uint8_t i;
#define SD_MAXFILELENGTH 64
            char fname[SD_MAXFILELENGTH + 1];
            uint8_t flen;
            bool isdir;
            
            if (buff_value[0]=='H')
            {
            	host::startOnboardBuild(utility::HOME_AXES);
            }
            else if (buff_value[0]=='C')
            {
                host::startOnboardBuild(utility::TOOLHEAD_CALIBRATE);
            }
            else if (buff_value[0]=='X')
            {
                writeString((char *)"{SYS:CANCELING}");
                Motherboard::heatersOff(true);
                command::addFilamentUsed();
                host::stopBuild();
            }
            else if (buff_value[0]=='P')
            {
                writeString((char *)"{SYS:PAUSE}");
                host::pauseBuild(true, false);
                writeString((char *)"{SYS:PAUSED}");
            }
            else if (buff_value[0]=='R')
            {
                writeString((char *)"{SYS:RESUME}");
                host::pauseBuild(false, false);
                writeString((char *)"{SYS:RESUMED}");
            }
            else if (buff_value[0]=='Z')
            {
                //int32_t currentPause = command::getPauseAtZPos();
                i=(buff_value[1]-'0')*100 + (buff_value[2]-'0')*10 + (buff_value[3]-'0');
                float pauseAtZPos = i;
                command::pauseAtZPos(stepperAxisMMToSteps(pauseAtZPos, Z_AXIS));
            }
            else
            {
                i=(buff_value[0]-'0')*100 + (buff_value[1]-'0')*10 + (buff_value[2]-'0');
                if ( !getFilename(i, fname, sizeof(fname), &flen, &isdir)) writeString((char *)"{ERR:031}");
                else
                {
                	if (isdir)
                	{
                		if (!sdcard::changeDirectory(fname) ) writeString((char *)"{ERR:033}");
                		else
                		{
                			writeString((char *)"{SYS:DIR}");
                		}
                	}
                	else 
                    {
                        writeString((char *)"{PRINTFILE:");
                        writeString(fname);
                        put('}');
                        if ( host::startBuildFromSD(fname, flen) != sdcard::SD_SUCCESS ) writeString((char *)"{ERR:032}");
                    }
                }
            }
            break;
            
        case 'B':
            PrintingStatus();
            break;

        case 'J':
            switch (buff_value[0])
            {
                case 'S':
                    BOARD_STATUS_SET(Motherboard::STATUS_MANUAL_MODE);
                    jog_speed=atoi((const char*)buff_value+1);
                    break;
                    
                case 'E':
                    steppers::enableAxes(0xff, false);
                    BOARD_STATUS_CLEAR(Motherboard::STATUS_MANUAL_MODE);
                    break;
                    
                case 'X':
                case 'Y':
                case 'Z':
                case 'A':
                case 'B':
                    steppers::abort();
                    uint8_t dummy;
                    Point position = steppers::getStepperPosition(&dummy);
                    
                    int32_t t;
                    t=atoi((const char*)buff_value+1);

                    if (buff_value[0]<='B') position[buff_value[0]-'A'+3] += (t<<4);
                    else position[buff_value[0]-'X'] += (t<<4);
                    
                    steppers::setTargetNew(position, jog_speed, 0, 0);
                    break;
            }
            break;
        case 'H':
        	if (buff_value[0]=='R')
        	{
        		extern uint32_t homePosition[PROFILES_HOME_POSITIONS_STORED];

        		writeString((char *)"{H:R");
        		eeprom_read_block(homePosition, (void *)eeprom_offsets::AXIS_HOME_POSITIONS_STEPS, PROFILES_HOME_POSITIONS_STORED * sizeof(uint32_t));
        		writeInt(homePosition[0],5);
        		put('/');
        		writeInt(homePosition[1],5);
        		put('/');
        		writeInt(homePosition[2],5);
        		put('/');
        		writeInt((int32_t)(eeprom::getEeprom32(eeprom_offsets::TOOLHEAD_OFFSET_SETTINGS, 0)),5);
        		put('/');
        		writeInt((int32_t)(eeprom::getEeprom32(eeprom_offsets::TOOLHEAD_OFFSET_SETTINGS + sizeof(int32_t), 0)),5);
        		put('}');
        	}
        	else if (buff_value[0]=='W')
        	{
        		extern uint32_t homePosition[PROFILES_HOME_POSITIONS_STORED];
        		int32_t offset[2],t;
        		uint8_t axis;
        		axis=buff_value[1]-'X';
        		if (axis>=0 && axis<=2)
        		{
        			homePosition[axis]=atoi((const char*)buff_value+2);
        			cli();
					eeprom_write_block((void *)&homePosition[axis],
					   (void*)(eeprom_offsets::AXIS_HOME_POSITIONS_STEPS + sizeof(uint32_t) * axis) ,
					   sizeof(uint32_t));
					sei();
        		}

        		axis=buff_value[1]-'x';
        		if (axis>=0 && axis<=1)
        		{
        			t=atoi((const char*)buff_value+2);

                    int32_t offset[2];
                    bool    smallOffsets;

                    offset[0]  = (int32_t)(eeprom::getEeprom32(eeprom_offsets::TOOLHEAD_OFFSET_SETTINGS, 0));
                    offset[1]  = (int32_t)(eeprom::getEeprom32(eeprom_offsets::TOOLHEAD_OFFSET_SETTINGS + sizeof(int32_t), 0));
                    smallOffsets = abs(offset[0]) < ((int32_t)stepperAxisStepsPerMM(0) << 2);

                    int32_t delta = stepperAxisMMToSteps((float)(t - 7) * 0.1f, axis);
                    if ( !smallOffsets ) delta = -delta;

                    int32_t new_offset = offset[axis] + delta;
                    eeprom_write_block((uint8_t *)&new_offset,
                           (uint8_t *)eeprom_offsets::TOOLHEAD_OFFSET_SETTINGS + axis * sizeof(int32_t),
                           sizeof(int32_t));
        		}
        	}
        	break;
        case 'U':
            if (buff_value[0]=='R')
            {
            	int temp;

                writeString((char *)"{U:RG");
                if (eeprom::getEeprom8(eeprom_offsets::OVERRIDE_GCODE_TEMP, 0) != 0) put('1');
                else put('0');

                put('R');
                temp=eeprom::getEeprom16(eeprom_offsets::PREHEAT_SETTINGS + preheat_eeprom_offsets::PREHEAT_LEFT_OFFSET, DEFAULT_PREHEAT_TEMP);
                writeInt(temp,3);
                //put('/');
                temp=eeprom::getEeprom16(eeprom_offsets::PREHEAT_SETTINGS + preheat_eeprom_offsets::PREHEAT_RIGHT_OFFSET, DEFAULT_PREHEAT_TEMP);
                writeInt(temp,3);
                //put('/');
                temp=eeprom::getEeprom16(eeprom_offsets::PREHEAT_SETTINGS + preheat_eeprom_offsets::PREHEAT_PLATFORM_OFFSET, DEFAULT_PREHEAT_TEMP);
                writeInt(temp,3);

                put('P');
                if (eeprom::hasHBP() != 0) put('1');
                else put('0');
                
                put('L');
                if (eeprom::getEeprom8(eeprom_offsets::ACCELERATION_SETTINGS + acceleration_eeprom_offsets::ACCELERATION_ACTIVE, 0x01) != 0) put('1');
                else put('0');
                
                put('S');
                if (eeprom::getEeprom8(eeprom_offsets::COOL_PLAT, 0) != 0) put('1');
                else put('0');
                
                put('D');
                if (eeprom::getEeprom8(eeprom_offsets::DITTO_PRINT_ENABLED, 0) != 0) put('1');
                else put('0');
                
                put('O');
                if (eeprom::getEeprom8(eeprom_offsets::TOOLHEAD_OFFSET_SYSTEM,
                                       DEFAULT_TOOLHEAD_OFFSET_SYSTEM) != 0) put('1');
                else put('0');
                
                put('E');
                if (eeprom::getEeprom8(eeprom_offsets::EXTRUDER_HOLD,
                                       DEFAULT_EXTRUDER_HOLD) != 0) put('1');
                else put('0');
                
                put('H');
                if (eeprom::getEeprom8(eeprom_offsets::HEAT_DURING_PAUSE, DEFAULT_HEAT_DURING_PAUSE) != 0) put('1');
                else put('0');
                
                put('C');
                if (eeprom::getEeprom8(eeprom_offsets::SD_USE_CRC, DEFAULT_SD_USE_CRC) != 0) put('1');
                else put('0');
                
                /*put('T');
                if (eeprom::getEeprom8(eeprom_offsets::PSTOP_ENABLE, 0) != 0) put('1');
                else put('0');*/
                
                put('X');
                put ('0' + eeprom::getEeprom8(eeprom_offsets::STEPPER_X_CURRENT, 0));
                
                put('Y');
                put ('0' + eeprom::getEeprom8(eeprom_offsets::STEPPER_Y_CURRENT, 0));
                
                put('Z');
                put ('0' + eeprom::getEeprom8(eeprom_offsets::STEPPER_Z_CURRENT, 0));
                
                put('A');
                put ('0' + eeprom::getEeprom8(eeprom_offsets::STEPPER_A_CURRENT, 0));
                
                put('B');
                put ('0' + eeprom::getEeprom8(eeprom_offsets::STEPPER_B_CURRENT, 0));

                put('N');
                put ('0' + eeprom::getEeprom8(eeprom_offsets::LANGUAGE, 0));

                put('M');
                put ('0' + eeprom::getEeprom8(eeprom_offsets::WIFI_SD, 0));
                
                put('}');
            }
            else if (buff_value[0]=='W')
            {
                uint8_t *c;
                uint8_t cmd=0;
                
                c=buff_value;
                while (*++c!=0)
                {
                    if (*c<='9' && *c>='0')
                    {
                        uint8_t value;
                        value = *c - '0';
                        
                        switch (cmd)
                        {
                            case 'G':
                                eeprom_write_byte((uint8_t *)eeprom_offsets::OVERRIDE_GCODE_TEMP,value);
                                break;
                                
                            case 'P':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::HBP_PRESENT, value);
                                break;
                                
                            case 'L':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::ACCELERATION_SETTINGS +
                                                  acceleration_eeprom_offsets::ACCELERATION_ACTIVE,
                                                  value);
                                break;
                                
                            case 'S':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::COOL_PLAT, value);
                                break;
                                
                            case 'D':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::DITTO_PRINT_ENABLED, value);
                                break;
                                
                            case 'O':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::TOOLHEAD_OFFSET_SYSTEM, value);
                                break;
                                
                            case 'E':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::EXTRUDER_HOLD, value);
                                break;
                                
                            case 'H':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::HEAT_DURING_PAUSE, value);
                                break;
                                
                            case 'C':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::SD_USE_CRC, value);
                                break;
                                
                            case 'T':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::PSTOP_ENABLE, value);
                                break;
                                
                            case 'X':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::STEPPER_X_CURRENT, value);
                                break;
                                
                            case 'Y':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::STEPPER_Y_CURRENT, value);
                                break;
                                
                            case 'Z':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::STEPPER_Z_CURRENT, value);
                                break;
                                
                            case 'A':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::STEPPER_A_CURRENT, value);
                                break;
                                
                            case 'B':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::STEPPER_B_CURRENT, value);
                                break;

                            case 'N':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::LANGUAGE, value);
                                break;

                            case 'M':
                                eeprom_write_byte((uint8_t*)eeprom_offsets::WIFI_SD, value);
                                break;
                                
                            default:
                                break;
                        }
                    }
                    else if (*c<='Z' && *c>='A') cmd = *c;
                }
            }
            else if (buff_value[0] == 'U' &&
                    buff_value[1] == 'P' &&
                    buff_value[2] == 'D' &&
                    buff_value[3] == 'A' &&
                    buff_value[4] == 'T' &&
                    buff_value[5] == 'E')
            {
                char r;
                cli();
                wdt_disable();

                while (1)
                {
                    if (UCSR0A & (1<<RXC0))
                    {
                        r = UDR0;
                        UDR3 = r;
                    }

                    if (UCSR3A & (1<<RXC3))
                    {
                        r = UDR3;
                        UDR0 = r;
                    }
                }
            }
            else {
                if (buff_value[0]=='E' && buff_value[1]=='R' && buff_value[2]=='A' && buff_value[3]=='S' && buff_value[4]=='E')
                {
                    eeprom::factoryResetEEPROM();
                    Motherboard::getBoard().reset(true);
                }
                
                else if (buff_value[0]=='F' && buff_value[1]=='U' && buff_value[2]=='L' && buff_value[3]=='L' && buff_value[4]=='E' && buff_value[5]=='R' && buff_value[6]=='A' && buff_value[7]=='S' && buff_value[8]=='E')
                {
                    eeprom::erase();
                    host::stopBuildNow();
                }
            }
            break;
        default:
            break;
    }
}

/*
 {FN:LIST},{}
 */
void LcdBoard::doInterrupt() {
	//buttons.scanButtons();
    uint8_t c;
    while (UCSR3A & (1<<RXC3))
	{
        c = UDR3;
        put(c);
		switch (c)
		{
			case '{': if (buff_state==0)
                      {
                          buff_state=1;
                          buff_ptr=0;
                      }
                      break;
                
			case ':': if (buff_state==1)
                      {
                          buff_obj[buff_ptr++]=0;
                          buff_state=2;
                          buff_ptr=0;
                      }
                      break;
                
			case ',': if (buff_state==2)
                      {
                          buff_value[buff_ptr++]=0;
                          process();
                          buff_state=1;
                          buff_ptr=0;
                      }
                      else if (buff_state==1)
                      {
                          put('?');
                          buff_state=0;
                          buff_ptr=0;
                      }
                      break;
			
            case '}': if (buff_state==2)
                      {
                          buff_value[buff_ptr++]=0;
                          process();
                          buff_state=0;
                          buff_ptr=0;
                      }
                      else if (buff_state==1)
                      {
                          put('?');
                          buff_state=0;
                          buff_ptr=0;
                      }
                      break;

            default:  if (c>=0x21 && c<=0x7E)
                      {
                          if (buff_state==1) buff_obj[buff_ptr++]=c;
                          else if (buff_state==2) buff_value[buff_ptr++]=c;
                      }
                      else
                      {
                          put('?');
                          buff_state=0;
                          buff_ptr=0;
                      }
		}
	}
}

bool LcdBoard::isButtonPressed(uint8_t button) {
        //bool buttonPressed = buttons.isButtonPressed(button);

        //return buttonPressed;
    return false;
}

micros_t LcdBoard::getUpdateRate() {
    return 500L * 1000L;
	//return screenStack[screenIndex]->getUpdateRate();
}

/// push Error Message Screen
void LcdBoard::errorMessage(uint8_t errid, bool incomplete) {
	//errorMessage(buf, 0, incomplete);
}

bool lcd_onboard_build = false;

void LcdBoard::doUpdate() {
/*
	// If we are building, make sure we show a build menu; otherwise,
	// turn it off.
	switch(host::getHostState()) {
    case host::HOST_STATE_BUILDING_ONBOARD:
            lcd_onboard_build = true;
	case host::HOST_STATE_BUILDING:
	case host::HOST_STATE_BUILDING_FROM_SD:
            //PrintingStatus();
		if (!building ){
					}
		break;
	case host::HOST_STATE_HEAT_SHUTDOWN:
		break;
	default:
		if ( building ) {
		}
		
		break;
	}
*/
}


// add a screen to the stack but don't refresh the screen
void LcdBoard::pushNoUpdate(uint8_t newScreen){
	if (screenIndex < SCREEN_STACK_DEPTH - 1) {
		screenIndex++;
		screenStack[screenIndex] = newScreen;
	}
	//screenStack[screenIndex]->reset();
}

// push screen to stack and call update
void LcdBoard::pushScreen(uint8_t newScreen) {
	if (screenIndex < SCREEN_STACK_DEPTH - 1) {
		screenIndex++;
		screenStack[screenIndex] = newScreen;
	}
	//screenStack[screenIndex]->reset();
	//screenStack[screenIndex]->update(lcd, true);
}

void LcdBoard::popScreen() {
	
	// Don't allow the root menu to be removed.
	if (screenIndex > 0) {
		screenIndex--;
	}
	//screenStack[screenIndex]->update(lcd, true);
}


/// Tell the interface board that the system is waiting for a button push
/// corresponding to one of the bits in the button mask. The interface board
/// will not process button pushes directly until one of the buttons in the
/// mask is pushed.
void LcdBoard::waitForButton(uint8_t button_mask) {
  waitingMask = button_mask;
}

/// Check if the expected button push has been made. If waitForButton was
/// never called, always return true.
bool LcdBoard::buttonPushed() {
  return waitingMask == 0;
}

/// Returns the number of times a button has been held down
/// Only applicable to continuous buttons
uint16_t LcdBoard::getButtonRepetitions(void) {
	return buttonRepetitions;
}

