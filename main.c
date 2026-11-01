/* Current setup
 Master PCB Output Triggers (Sent to Motion MCU)
?	MC1 Line (Next Round): Send a brief falling edge (High \(\rightarrow \) Low pulse) to strip player control and auto-home the motors to the bottom before returning them to their start coordinates \((X, Y)\).
?	MC2 Line (Game Over): Send a brief falling edge (High \(\rightarrow \) Low pulse) to terminate player control, home the motors to the bottom, and lock the system down in power-save mode.
?	MC3 Line (New Game): Send a brief falling edge (High \(\rightarrow \) Low pulse) while the system is idle to wake up the stepper drivers, lift the motors up to their start positions, and enable the players.
Master PCB Input Status Monitors (Received from Motion MCU)
MC5 Line (Player Control Enabled):
o	High (\(1\)): The game is actively in a round. The master board can safely display game timers, read score sensors, or light up player-facing LEDs.
o	Low (\(0\)): Motors are currently calibrating, homing, or the machine is completely uncooperative/idle. [1]
MC6 Line (Waiting for Start):
o	High (\(1\)): The motion system is safely parked on the bottom stops with drivers turned off. The master board should show a "Press Start / Insert Coin" animation.
o	Low (\(0\)): The machine is doing an active task or a match is underway.

 */



#define F_CPU 4000000UL
#include "mcc_generated_files/system/system.h"
#include <util/delay.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdint.h>

// --- CONFIGURABLE CONFIGURATION ---
#define START_STEPS_M1  500  // 'X' steps for Left Motor
#define START_STEPS_M2  750  // 'Y' steps for Right Motor

// Define operating states
typedef enum {
    STATE_STARTUP_CALIBRATE_UP,
    STATE_STARTUP_CALIBRATE_DN,
    STATE_GOTO_START_POSITION,    
    STATE_PLAYER_CONTROL,
    STATE_NEXT_ROUND_RESET,
    STATE_GAME_OVER_RESET,
    STATE_GAME_OVER_IDLE
} GameState_t;

volatile GameState_t current_state = STATE_STARTUP_CALIBRATE_UP;

// Step tracking counters for positioning
uint16_t steps_moved_m1 = 0;
uint16_t steps_moved_m2 = 0;

// --- INTERRUPT HANDLERS ---

void MC1_Custom_ISR(void) 
{
    if (current_state == STATE_PLAYER_CONTROL) {
        current_state = STATE_NEXT_ROUND_RESET;
    }
}

void MC2_Custom_ISR(void) 
{
    if (current_state != STATE_STARTUP_CALIBRATE_UP && 
        current_state != STATE_STARTUP_CALIBRATE_DN && 
        current_state != STATE_GOTO_START_POSITION && 
        current_state != STATE_GAME_OVER_IDLE) {
        current_state = STATE_GAME_OVER_RESET;
    }
}

void MC3_Custom_ISR(void) 
{
    if (current_state == STATE_GAME_OVER_IDLE) {
        MC6_SetLow(); 
        ENABLE_PC3_SetLow(); 
        
        steps_moved_m1 = 0;
        steps_moved_m2 = 0;
        current_state = STATE_GOTO_START_POSITION;
    }
}

// --- HELPER FUNCTIONS ---

void DriveMotorsToTop(void)
{
    bool m1_at_top = !LIMIT_UP_L_GetValue();
    bool m2_at_top = !LIMIT_UP_R_GetValue();
    
    if (!m1_at_top) {
        DIR_M1_SetHigh(); 
        STEP_M1_SetHigh();
    } else {
        STEP_M1_SetLow();
    }
    
    if (!m2_at_top) {
        DIR_M2_SetHigh(); 
        STEP_M2_SetHigh();
    } else {
        STEP_M2_SetLow();
    }
    
    if (m1_at_top && m2_at_top) {
        current_state = STATE_STARTUP_CALIBRATE_DN;
    }
}

void DriveMotorsToBottom(GameState_t next_target_state)
{
    bool m1_at_bottom = !LIMIT_DN_L_GetValue();
    bool m2_at_bottom = !LIMIT_DN_R_GetValue();
    
    if (!m1_at_bottom) {
        DIR_M1_SetLow(); 
        STEP_M1_SetHigh();
    } else {
        STEP_M1_SetLow();
    }
    
    if (!m2_at_bottom) {
        DIR_M2_SetLow(); 
        STEP_M2_SetHigh();
    } else {
        STEP_M2_SetLow();
    }
    
    if (m1_at_bottom && m2_at_bottom) {
        if (next_target_state == STATE_GOTO_START_POSITION) {
            steps_moved_m1 = 0;
            steps_moved_m2 = 0;
        }
        current_state = next_target_state;
    }
}

void MoveToStartPosition(void)
{
    bool m1_ready = (steps_moved_m1 >= START_STEPS_M1) || !LIMIT_UP_L_GetValue();
    bool m2_ready = (steps_moved_m2 >= START_STEPS_M2) || !LIMIT_UP_R_GetValue();
    
    if (!m1_ready) {
        DIR_M1_SetHigh();
        STEP_M1_SetHigh();
        steps_moved_m1++;
    } else {
        STEP_M1_SetLow();
    }
    
    if (!m2_ready) {
        DIR_M2_SetHigh();
        STEP_M2_SetHigh();
        steps_moved_m2++;
    } else {
        STEP_M2_SetLow();
    }
    
    if (m1_ready && m2_ready) {
        current_state = STATE_PLAYER_CONTROL;
    }
}

void HandlePlayerControls(void)
{
    if (!JOY_1_UP_GetValue() && LIMIT_UP_L_GetValue()) {
        DIR_M1_SetHigh();
        STEP_M1_SetHigh();
    } else if (!JOY_1_DN_GetValue() && LIMIT_DN_L_GetValue()) {
        DIR_M1_SetLow();
        STEP_M1_SetHigh();
    } else {
        STEP_M1_SetLow();
    }
    
    if (!JOY_2_UP_GetValue() && LIMIT_UP_R_GetValue()) {
        DIR_M2_SetHigh();
        STEP_M2_SetHigh();
    } else if (!JOY_2_DN_GetValue() && LIMIT_DN_R_GetValue()) {
        DIR_M2_SetLow();
        STEP_M2_SetHigh();
    } else {
        STEP_M2_SetLow();
    }
}

// --- MAIN APPLICATION ---

void main(void)
{
    SYSTEM_Initialize();
    
    MC1_SetInterruptHandler(MC1_Custom_ISR);
    MC2_SetInterruptHandler(MC2_Custom_ISR);
    MC3_SetInterruptHandler(MC3_Custom_ISR); 
    
    sei();
    
    ENABLE_PC3_SetLow(); 
    MC6_SetLow();
    MC5_SetLow(); 
    
    while(1)
    {
        switch(current_state)
        {
            case STATE_STARTUP_CALIBRATE_UP:
                MC5_SetLow();
                DriveMotorsToTop();
                break;
                
            case STATE_STARTUP_CALIBRATE_DN:
                MC5_SetLow();
                DriveMotorsToBottom(STATE_GAME_OVER_IDLE); 
                break;
                
            case STATE_GOTO_START_POSITION:
                MC6_SetLow(); 
                MC5_SetLow(); // Still automated; player does not have control yet
                MoveToStartPosition();
                break;
                
            case STATE_PLAYER_CONTROL:
                MC6_SetLow();
                MC5_SetHigh(); // Player officially has control now (even if sitting idle)
                HandlePlayerControls();
                break;
                
            case STATE_NEXT_ROUND_RESET:
                MC6_SetLow(); 
                MC5_SetLow(); // Control stripped away for auto-reset
                DriveMotorsToBottom(STATE_GOTO_START_POSITION); 
                break;
                
            case STATE_GAME_OVER_RESET:
                MC6_SetLow(); 
                MC5_SetLow(); // Control stripped away for game over homing
                DriveMotorsToBottom(STATE_GAME_OVER_IDLE); 
                break;
                
            case STATE_GAME_OVER_IDLE:
                MC6_SetHigh(); 
                MC5_SetLow(); // Permanently locked low between games
                ENABLE_PC3_SetHigh(); 
                STEP_M1_SetLow();
                STEP_M2_SetLow();
                break;
        }
        
        // --- UNIFIED TIMING PULSE ---
        _delay_us(100);
        STEP_M1_SetLow();
        STEP_M2_SetLow();
        _delay_us(100);
    }
}