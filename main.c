#include "F2833x_Device.h"     // F28335 Headerfile
#include "IQmathLib.h"         // Optimized math library
#include "f2833x_adc.h"        // ADC library
#include "f2833x_pwm.h"        // PWM library
#include "f2833x_gpio.h"       // GPIO library
#include "DSP2833x_Sci.h"      // SCI for LCD communication

// System Definitions
#define SAMPLE_RATE 10000      // 10kHz sampling
#define PWM_FREQ 50            // 50Hz output frequency
#define PWM_PERIOD 3750        // 150MHz/(50Hz*800)
#define V_IN_MAX 5.0           // Max input voltage (5V)
#define V_OUT_MAX 230.0        // Max output voltage (230V)
#define ZERO_CROSS_THRESH 0.1  // Zero-crossing detection threshold

// Global Variables
volatile _iq20 gAdcVoltage = 0;        // Scaled input voltage (0-5V) in IQ20
volatile _iq20 gOutputVoltage = 0;     // Scaled output voltage (0-230V)
volatile _iq20 gFrequency = 0;         // Measured frequency
volatile Uint32 gZeroCrossCount = 0;   // Zero crossing counter
volatile Uint32 gLastCrossTime = 0;    // Last zero crossing timestamp
volatile Uint32 gSampleCount = 0;      // Sample counter

// Function Prototypes
void InitSystem(void);
void InitADC(void);
void InitPWM(void);
void InitGPIO(void);
void InitLCD(void);
__interrupt void ADC_ISR(void);
void ZeroCrossingDetection(_iq20 sample);
void UpdatePWM(void);
void LCD_Display(void);
void LCD_WriteString(Uint16 row, Uint16 col, char *str);

void main(void)
{
    // Initialize system
    InitSystem();       // Clock, watchdog
    InitGPIO();         // Configure GPIO pins
    InitADC();          // Configure ADC
    InitPWM();          // Setup PWM for AC output
    InitLCD();          // Initialize LCD display

    // Enable interrupts
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();
    EINT;

    // Main loop
    while(1)
    {
        // Update display periodically (every 1000 samples)
        if(gSampleCount % 1000 == 0)
        {
            LCD_Display();
        }
        asm(" NOP");
    }
}

// System Initialization
void InitSystem(void)
{
    // Initialize system clock to 150MHz
    InitSysCtrl();

    // Disable watchdog
    DisableDog();
}

// GPIO Initialization
void InitGPIO(void)
{
    EALLOW;
    // Configure GPIO0 as ADC input
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // ADCINA0
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 0;    // Input

    // Configure GPIO1 as PWM output
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // EPWM1A
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;    // Output

    // Configure I2C pins for LCD (GPIO32/33)
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;  // SDA
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;  // SCL
    EDIS;
}

// ADC Initialization
void InitADC(void)
{
    EALLOW;
    // Power up ADC
    AdcRegs.ADCTRL3.bit.ADCBGRFDN = 0x3;  // Bandgap/reference power
    AdcRegs.ADCTRL3.bit.ADCPWDN = 1;      // Power up ADC
    DELAY_US(1000);                       // Wait 1ms

    // Configure ADC
    AdcRegs.ADCTRL1.bit.ACQ_PS = 0xF;     // Acquisition window
    AdcRegs.ADCTRL3.bit.ADCCLKPS = 0x3;   // Clock prescaler
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;     // Cascaded sequencer

    // Configure ADCINA0 as input
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0;  // ADCINA0

    // Enable interrupts
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1; // Enable SEQ1 interrupt
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1; // SOC from ePWM
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;    // Enable ADCINT in PIE
    IER |= M_INT1;                        // Enable INT1

    EDIS;
}

// PWM Initialization
void InitPWM(void)
{
    EALLOW;
    // Time Base Setup
    EPwm1Regs.TBPRD = PWM_PERIOD;         // Set period
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Up-down count

    // Channel A Setup
    EPwm1Regs.CMPA.bit.CMPA = PWM_PERIOD/2; // 50% initial duty
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;     // Set on zero
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;   // Clear on compare up

    EDIS;
}

// ADC Interrupt Service Routine
__interrupt void ADC_ISR(void)
{
    // Read and scale ADC result (IQ20 format)
    Uint16 adcResult = AdcRegs.ADCRESULT0 >> 4;
    gAdcVoltage = _IQ20mpy(_IQ20(adcResult), _IQ20(V_IN_MAX/4095.0));

    // Scale to output voltage
    gOutputVoltage = _IQ20mpy(gAdcVoltage, _IQ20(V_OUT_MAX/V_IN_MAX));

    // Zero crossing detection (remove DC offset)
    ZeroCrossingDetection(_IQ20sub(gAdcVoltage, _IQ20(V_IN_MAX/2.0)));

    // Update PWM output
    UpdatePWM();

    // Clear interrupt
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    gSampleCount++;
}

// Zero Crossing Detection (IQ20 format)
void ZeroCrossingDetection(_iq20 sample)
{
    static _iq20 prevSample = 0;
    Uint32 currentTime = ReadTimer0();

    // Detect positive crossing with hysteresis
    if((prevSample < _IQ20(ZERO_CROSS_THRESH)) &&
       (sample >= _IQ20(ZERO_CROSS_THRESH)))
    {
        gZeroCrossCount++;

        // Calculate frequency after 2 crossings
        if(gZeroCrossCount >= 2)
        {
            Uint32 period = currentTime - gLastCrossTime;
            gFrequency = _IQ20(150000000.0) / _IQ20(period); // 150MHz clock
        }
        gLastCrossTime = currentTime;
    }
    prevSample = sample;
}

// Update PWM Duty Cycle (IQ20 math)
void UpdatePWM(void)
{
    static _iq20 angle = 0;
    _iq20 sineValue;

    // Increment angle (2π*frequency*Δt)
    angle = _IQ20add(angle, _IQ20mpy(_IQ20(2*3.14159265),
              _IQ20mpy(gFrequency, _IQ20(1.0/SAMPLE_RATE))));

    // Wrap angle
    if(angle > _IQ20(2*3.14159265))
        angle = _IQ20sub(angle, _IQ20(2*3.14159265));

    // Generate sine wave
    sineValue = _IQ20sin(angle);

    // Scale to PWM duty (0-100%)
    Uint16 pwmDuty = _IQ20mpy(_IQ20(PWM_PERIOD/2),
                      _IQ20add(_IQ20(1.0), sineValue));

    // Apply voltage scaling
    pwmDuty = _IQ20mpy(pwmDuty, _IQ20div(gOutputVoltage, _IQ20(V_OUT_MAX)));

    // Update PWM
    EPwm1Regs.CMPA.bit.CMPA = pwmDuty;
}

// LCD Display Function
void LCD_Display(void)
{
    char lcdBuffer[17];
    float dispVoltage = _IQ20toF(gAdcVoltage);
    float dispFreq = _IQ20toF(gFrequency);

    // Line 1: Voltage
    sprintf(lcdBuffer, "In:%2.1fV Out:%3.0f", dispVoltage,
            _IQ20toF(gOutputVoltage));
    LCD_WriteString(0, 0, lcdBuffer);

    // Line 2: Frequency
    sprintf(lcdBuffer, "Freq:%3.1f Hz", dispFreq);
    LCD_WriteString(0, 1, lcdBuffer);
}

// Basic LCD Write Function (I2C)
void LCD_WriteString(Uint16 row, Uint16 col, char *str)
{
    // Placeholder - implement with SCI/I2C as needed
    // This would use SCI or bit-banged I2C from GPIO
}
