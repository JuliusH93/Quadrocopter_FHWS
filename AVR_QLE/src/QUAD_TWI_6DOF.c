// This Quadcopter Software Framework was developed at
// University Wuerzburg
// Chair Computer Science 8
// Aerospace Information Technology
// Copyright (c) 2011-2014 Nils Gageik

// This Software uses the AVR Framework:
// Copyright (c) 2009 Atmel Corporation. All rights reserved.
#include "atmel.h"			//Mandantory

// ******************************************
// *		Quadcopter Main					*
// ******************************************

// !!! Quadrocopter !!!
#include "QUAD_TWI_6DOF.h"			// All Header Files

// --------------------   Variables		-------------------------------------------------------
// Timer Counter
volatile avr32_tc_t *tc = &AVR32_TC;

// Common Topics for interchange (Datenaustausch)
SignalProcessed SP_Data;
MotorControl Motorn;


// ***** my Vars *******************
#ifdef DISPLAY_ON
short display_modus = DIP_START_MODUS;
#endif

// Aktuelle Steuerdaten des Quadcopters wie EngineOn, HeightController, etc.
steuerDaten steuerWerte = { 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
sensorDaten sensorWerte; // SensorValues, die aktuell gültigen Daten

extern int last_IR_OD_Time;

#ifdef DMA_PDCA
extern pdca_Daten pdca_Werte;
#endif

unsigned int tc_ticks = 0; // Timer Counter in ms| 49days runtime
unsigned int last_SignalProcessing_time = 0;
unsigned int last_Remote_time = 0; // Last    "      "    "  Valid Remote Values
unsigned int last_DP_time = 0; // Last    "      "    "  Renew Display
unsigned int last_QuatCalc_time = 0;
unsigned int last_Sample_time = 0;
//unsigned int last_SEND_time;
unsigned int last_button_time;
unsigned int last_Yaw_Filtertime = 0;

// Debug and USART
char debug_line[DEBUG_BUFFER_SIZE];

volatile avr32_pm_t* pm = &AVR32_PM;

unsigned int debug_wait;

unsigned char engineSpeed = 0;

__attribute__((__interrupt__))
static void Joy_int_handler(void) {
	if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_UP)) {
		Motorn.stellwert_blctrl_engine1++;
		Motorn.stellwert_blctrl_engine2++;
		Motorn.stellwert_blctrl_engine3++;
		Motorn.stellwert_blctrl_engine4++;
		/* allow new interrupt : clear the IFR flag */
		gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_UP);
	}
	if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_DOWN)) {
		if (Motorn.stellwert_blctrl_engine1>0) {
			Motorn.stellwert_blctrl_engine1--;
			Motorn.stellwert_blctrl_engine2--;
			Motorn.stellwert_blctrl_engine3--;
			Motorn.stellwert_blctrl_engine4--;
		}
		/* allow new interrupt : clear the IFR flag */
		gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_DOWN);
	}
	if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_PUSH)) {
		Motorn.stellwert_blctrl_engine1 = 0;
		Motorn.stellwert_blctrl_engine2 = 0;
		Motorn.stellwert_blctrl_engine3 = 0;
		Motorn.stellwert_blctrl_engine4 = 0;
		/* allow new interrupt : clear the IFR flag */
		gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_PUSH);
	}
}

int main(void) {
	///////////////////////////////////////////////////////////////////////////////////////
	// Init //////////// //////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////

	gpio_set_gpio_pin(INIT_READY_LED); // First LED means Main is starting

	gpio_clr_gpio_pin(I2C_RESET_CLEAR_PIN); // Set GND Pin back to GND to start Sensor

	if (CPU_SPEED == 60000000)
		SetProcessorFrequency(pm); // Use 60MHz
	else
		pcl_switch_to_osc(PCL_OSC0, CPU_SPEED, OSC0_STARTUP); // Switch main clock to external oscillator 0 (crystal), 12 MHz

	AVR32_HMATRIX.mcfg[AVR32_HMATRIX_MASTER_CPU_INSN] = 0x1;

	// Disable interrupts
	Disable_global_interrupt();

	// init the interrupts
	INTC_init_interrupts();

	#ifdef DISPLAY_ON
	myDIP_init(); // Init DIP
	#endif

	init_button_irq();

	myTC_init(); // Init TC
	myUSART_init(); // Init USART
	myTWI_init(); // Init TWI

	INTC_register_interrupt(&Joy_int_handler, AVR32_GPIO_IRQ_0
			+ (GPIO_JOYSTICK_UP / 8), AVR32_INTC_INT1);

	INTC_register_interrupt(&Joy_int_handler, AVR32_GPIO_IRQ_0
			+ (GPIO_JOYSTICK_DOWN / 8), AVR32_INTC_INT1);

	INTC_register_interrupt(&Joy_int_handler, AVR32_GPIO_IRQ_0
			+ (GPIO_JOYSTICK_PUSH / 8), AVR32_INTC_INT1);

	wait_ms(500);

	//Enable all interrupts.
	Enable_global_interrupt();

	// Init TWI Slaves
	blctrl_init(); // Motor Controller

    //INIT IMU
    Init_IMU_Sensors();

	gpio_clr_gpio_pin(INIT_READY_LED);

	#ifdef DISPLAY_ON
	// ---- StartUp --------------------------------
	// *******	Display		**********************
	dip204_set_cursor_position(1,1);
	dip204_write_string(PROJEKT_NAME);
	dip204_hide_cursor();
	// ---------------------------------------------	 *
	#endif

	debug_wait = wait_ms(500);

	USART_schreiben("Init accomplished\n");

	///////////////////////////////////////////////////////////////////////////////////////
	// Init accomplished //////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////

	gpio_set_gpio_pin(INIT_READY_LED);

	// Main Loop
	while (1) {

		if (steuerWerte.calibrate_on && !steuerWerte.engine_on) {

			gpio_clr_gpio_pin(INIT_READY_LED);

			gpio_set_gpio_pin(INIT_READY_LED);

			steuerWerte.calibrate_on = 0;
			last_Sample_time = tc_ticks;
		}

		// Datenpfad:
		// 1. SignalProcessing liest und schreibt in SP_Data
		// 2. Wenn aktiv, Kalman Filter bzw. Complementary Quaternionen Filter ändern SP_Data
		// 3. Attitude Control hat SP_Data als Input und Stellwerte für Motorn als Output
		// 4. Motor Control, hier set_engine, stellt die Motoren: Input Motorn

		// Bleibende Abhängigkeiten:
		// Kalman Filter benötigt Daten aus Sensorwerten und Controllerwerte
		// Attitude Control liest SteuerWerte
		// Signal Processing manipuliert SteuerWerte

		if (tc_ticks >= last_Sample_time + SAMPLE_TIME) {
			last_Sample_time = tc_ticks;

			set_engine(&Motorn);

			#ifdef DISPLAY_ON
			  setDisplay();
			#endif
		}

	}
	return 0;
}
