/*
 * Copyright 2013 by Yongzong Lin <yongzong@malyansys.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */


#ifndef LCD_BOARD_HH_
#define LCD_BOARD_HH_

#include "Configuration.hh"
#include "Menu.hh"

#define SCREEN_STACK_DEPTH      7

/// The LcdBoard module provides support for the Malyan System LCD Interface.
/// \ingroup HardwareLibraries
class LcdBoard {
public:
private:
    
        uint8_t buff_obj[8];
        uint8_t buff_value[12];
        uint8_t buff_ptr,buff_state;
    
        /// Stack of screens to display; the topmost one will actually
        /// be drawn to the screen, while the other will remain resident
        /// but not active.
        uint8_t screenStack[SCREEN_STACK_DEPTH];
        int8_t screenIndex;             ///< Stack index of the current screen.

        /// TODO: Delete this.
        bool building;                  ///< True if the bot is building
        
        uint8_t waitingMask;            ///< Mask of buttons the interface is
                                        ///< waiting on.
    
        bool screen_locked;             /// set to true in case of catastrophic failure (ie heater cuttoff triggered)

	uint16_t buttonRepetitions;	/// Number of times a button has been repeated whilst held down
					/// used for continuous buttons and speed scaling of incrementers/decrementers

	bool lockoutButtonRepetitionsClear; /// Used to lockout the clearing of buttonRepetitions
    
    void ListFile(uint8_t index);

public:
        LcdBoard();

    void writeInt(uint16_t value, uint8_t digits);
    void writeInt32(uint32_t value, uint8_t digits);
	void process();
	void PrintingStatus();

        /// Initialze the interface board. This needs to be called once
        /// at system startup (or reset).
	void init();

        /// This should be called periodically by a high-speed interrupt to
        /// service the button input pad.
	void doInterrupt();

        /// This is called for a specific button and returns true if the
        /// button is currently depressed
        bool isButtonPressed(uint8_t button);

        /// Add a new screen to the stack. This automatically calls reset()
        /// and then update() on the screen, to ensure that it displays
        /// properly. If there are more than SCREEN_STACK_DEPTH screens already
        /// in the stack, than this function does nothing.
        /// \param[in] newScreen Screen to display.
	void pushScreen(uint8_t newScreen);
	
    /// Remove the current screen from the stack. If there is only one screen
    /// being displayed, then this function does nothing.
	void popScreen();
    
	/// Return a pointer to the currently displayed screen.
	uint8_t getCurrentScreen() { return screenStack[screenIndex]; }

	micros_t getUpdateRate();

	void doUpdate();

	void showMonitorMode();
	
	void setLED(uint8_t id, bool on);

	/// Tell the interface board that the system is waiting for a button push
	/// corresponding to one of the bits in the button mask. The interface board
	/// will not process button pushes directly until one of the buttons in the
	/// mask is pushed.
	void waitForButton(uint8_t button_mask);
	
	/// Check if the expected button push has been made. If waitForButton was
	/// never called, always return true.
	bool buttonPushed();
	
	/// push Error Message Screen
	void errorMessage(uint8_t errid, bool incomplete = false);
    
    /// lock screen so that no pushes/pops can occur
    /// used in the case of heater failure to force restart
    void lock(){ screen_locked = true;}
    
    /// push screen onto the stack but don't update - this is used to create
    /// screen queue
    void pushNoUpdate(uint8_t newScreen);
    
    /// re-initialize LCD
    void resetLCD();

    /// Returns the number of times a button has been held down
    /// Only applicable to continuous buttons
    uint16_t getButtonRepetitions(void);
};

#endif
