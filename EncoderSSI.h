#pragma once

// ##############################################################################################
// MCU Select:

#include "mcu_select.h"

/*
    If there is not exist mcu_select.h at beside of this header file, Create it and put this bellow following content. 
    Then select your desired MCU that want work with.
*/
// ----------------------------------------------------------------
// mcu_select.h file:

// Define the target MCU family here
// Uncomment the desired MCU family definition below:

// #define STM32F1
// #define STM32F4
// #define STM32H7

// ----------------------------------------------------------------

// ########################################################################
// Include libraries:

#if defined(STM32F1)
#include "stm32f1xx_hal.h"      // HAL library for STM32F1 series
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"      // HAL library for STM32F4 series
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"      // HAL library for STM32H7 series
#else
#error "Unsupported MCU family. Please define a valid target (e.g., STM32F1, STM32F4, STM32H7)."
#endif

#include <string>       // Include the standard string library for error handling and messages
#include <math.h>
#include "TimerControl.h"

// ######################################################################################
// Define global macros:

/**
 * @namespace EncoderSSI_Namespace
 * @brief The namespace for safety protection.
 *  */ 
namespace EncoderSSI_Namespace
{
    #define EncoderSSI_COM_Mode_SPI     0
    #define EncoderSSI_COM_Mode_GPIO    1
}

// #######################################################################################
// LPF class:

/**
 * @namespace EncoderSSI_Namespace
 * @brief The namespace for safety protection.
 *  */ 
namespace EncoderSSI_Namespace
{
    /**
     * @class LPF
     * @brief Low pass filter class
     */
    class LPF
    {
        public:
            
            /**
             * @brief Parameters structure.
             */
            struct ParametersStructure
            {
                /// @brief Cut off frequency parameter of low pass filter. [Hz]. The value of 0 means it is disabled. 
                double FRQ;                   
            }parameters;

            /**
             * @brief Default constructor. Init some variables.
             */
            LPF();

            /**
             * @brief Buffer memory for output of lowpass filter.
             */
            double output;              

            /**
             * @brief Low pass filter updatation. Calculate output signal relative to input signal and time step.
             * Store value of filtered output signal.
             * @param input: Value of input signal.
             * @param dt: Time step duration. [sec].
             * @return output of filter.
             */
            virtual double updateByTime(const double &input, const double &dt);

            /**
             * @brief Low pass filter updatation. Calculate output signal relative to input signal and frequency of update step.
             * Store value of filtered output signal.
             * @param input: Value of input signal.
             * @param frq: Frequency updatation step. [Hz].
             * @return output of filter
             */
            virtual double updateByFrequency(const double &input, const double &frq);

            /**
             * @brief Set Cut off frequency of low pass filter.
             * @param frq: Cut off frequency. [Hz]
             * @return true if successed.
             *  */  
            bool setFrequency(const double &frq);

            /**
             * @brief Set output directely without filter.
             * Use this function for init value of output signal.
             */
            void setOutputDirect(const double &data);

            /**
             * @brief Clear any buffer on variables.
             */
            void clear(void); 

        protected:

            /**
             * @brief Low Pass filter gain.
             * @note - Filter in continuse space: G(s) = output/input = 1/(T*s + 1)
             * @note - T: Time constant for lowpass filter.
             * @note -  Filter cut off frequency (frq_c) = 1/(2pi*T)
             * @note -  alpha gain = 1/( 1 + 1/(dt/T) ) = 1/( 1 + 1/(2*pi*frq_c*dt) )
             * @note -  dt: Time step duration. [sec]
             * @note -  output_(n) = (1 - alpha) * output_(n-1) + alpha * input_(n)
             *  */ 
            double _alpha;           

    };

}

// #####################################################################################
// LotusEncoderSSI class 

/**
 * @class EncoderSSI
 * @brief EncoderSSI class.
 * @warning - Set the MCU hardware SPI or GPIO communication from outside of this object before using the init method of this object.
 * @warning - Set The TimerControl object that want used for this object from outside of this object before using the init method of this object. 
 *  */ 
class EncoderSSI
{
    public:

        /// @brief  Last error occurred for object.
        std::string errorMessage;               

        /**
         * @struct ParametersStructure
         * @brief Parameters structure.
         */
        struct ParametersStructure
        {
            /**
             * @brief HAL SPI handle pointer.
             * @note - Only if the communication mode is SPI, this parameter need to be set; otherwise, leave it.
             * @warning - Set the MCU hardware SPI or GPIO communication from outside of this object before using the init method of this object.
            */
            SPI_HandleTypeDef* HSPI;

            /**
             * @brief TimerControl object pointer.
             * @warning - Set The TimerControl object that want used for this object from outside of this object before using the init method of this object. 
             *  */ 
            TimerControl* TIMER;

            /**
             * @brief Encoder clock GPIO port. 
             * @note - Only if the communication mode is GPIO, this parameter need to be set; otherwise, leave it.
             * @warning - Set the MCU hardware SPI or GPIO communication from outside of this object before using the init method of this object.
             */
            GPIO_TypeDef* CLK_GPIO_PORT;

            /**
             * @brief Encoder data GPIO port. 
             * @note - Only if the communication mode is GPIO, this parameter need to be set; otherwise, leave it.
             * @warning - Set the MCU hardware SPI or GPIO communication from outside of this object before using the init method of this object.
             */
            GPIO_TypeDef* DATA_GPIO_PORT;

            /**
             * @brief Encoder clock GPIO pin. It can be GPIO_PIN_0, GPIO_PIN_1, ... 
             * @note - Only if the communication mode is GPIO, this parameter need to be set; otherwise, leave it.
             * @warning - Set the MCU hardware SPI or GPIO communication from outside of this object before using the init method of this object.
             */
            uint16_t CLK_GPIO_PIN;

            /**
             * @brief Encoder data GPIO pin. It can be GPIO_PIN_0, GPIO_PIN_1, ... 
             * @note - Only if the communication mode is GPIO, this parameter need to be set; otherwise, leave it.
             * @warning - Set the MCU hardware SPI or GPIO communication from outside of this object before using the init method of this object.
             */
            uint16_t DATA_GPIO_PIN;

            /**
             * @brief Cut off frequency parameter of low pass filter for rate value. [Hz]
             * @note The value of 0 means it is disabled. 
             *  */ 
            double FLTR;   

            /**
             * @brief Cut off frequency parameter of low pass filter for angle value. [Hz].
             * @note The value of 0 means it is disabled.   
             *  */               
            double FLTA;     

            /**
             * @brief Mapping enable/disable of encoder data.
             */
            bool MAP_ENA;

            /**
             * @brief The value of Map for minimum angle calculations. [deg]
             */
            double MAP_MIN;

            /**
             * @brief The value of Map for maximum angle calculations. [deg]
             */
            double MAP_MAX;
            
            /**
             * @brief Encoder single turn resolution.
             * @note It must be lower than 23 bit single turn.
             *  */                  
            uint8_t RESOLUTION;         

            /**
             * @brief Offset value channel 1 and 2 for raw angle measurement. [deg]
             * @note Initial value is 0.
             *  */ 
            double POSRAW_OFFSET_DEG;            

            /**
             * @brief Gearbox ratio for angle measurement.
             * @note posDeg = posDeg * gearRatio.
             *  */  
            double GEAR_RATIO;

            /**
             * @brief The frequency of rate measurements derived from position data. [Hz]
             * @note The value of 0 means it is disabled.
             */
            double RATE_FRQ;   
        }parameters;

        /**
         * @struct ValuesStructure
         * @brief Values structure.
         */
        struct ValuesStructure
        {
            /**
             * @brief Raw encoder value for channel 1 and 2. [-]
             * @note Max value depend on resolution.
             *  */ 
            uint32_t posRawStep;                 

             /**
             * @brief Encoder shaft angle for channel 1 and 2. [deg].
             * @note posRawDeg = (posRawStep / max_posRawStep) * 360
             *  */ 
            double posRawDeg;     

            /**
             * @brief Position angle for channel 1 and 2. [deg]
             * @note posDeg = (posRawDeg - posRawOffsetDeg + _revolutionCounter * 360.0) * gearRatio.
             *  */ 
            double posDeg;                     

            /**
             * @brief Velocity for channel 1 and 2. [deg/s].
             * @note velDegSec = d(posDeg)/dt 
             *  */      
            double velDegSec;   

            ///! @brief Number of cross over from overflow for posRawStep since object created.                
            int32_t revolutionCounter;             
        }value;

        /** 
         * @brief EncoderSSI constructor. Set encoder resolution and communication mode. Not apply setting.
         * @param communication_mode: refers to how data is read.
         * 
         * - A value of 0 means the clock and data are provided by SPI communication. (Default value)
         * 
         * - A value of 1 means the clock and data are provided by raw GPIO digital input/output signal communication.
         * 
         * - Other value not acceptable and it set to default value of 0.
         * 
         * - This parameter can only be set when the object is created and cannot be changed during the object's lifetime.
         * @note init() method needs after this for apply setting on hardware.
         * @warning Set the MCU hardware SPI or GPIO communication from outside of this object before using the init method of this object.
        */      
        EncoderSSI(uint8_t communication_mode = 0);

        /// @brief Destructor.
        ~EncoderSSI();

        /** 
         * @brief Init/Setup then Apply setting on hardware. Start LotusEncoderSSI action.
         * @return true if succeeded.
        */
        bool init(void);

        /**
         * @brief Read and update values.
         */
        void update(void);

        /**
         * @brief Set the PresetValue object that contains the desired absolute preset value. Writing this object executes a preset.
         * The encoder internally calculates a preset offset value which is being stored in a non-volatile memory
         * (no store command via CoE object 0x1010 required).
         * @param value is desired step value for position at current position.  
         * @return true if successed.
         * @warning Use this function after setting direction rotation of encoder. Otherwise it maybe not correct set.
         *  */  
        bool setPresetValueDeg(double value);

        /**
         * @brief Clean setting on hardware. Stop LotusEncoderSSI action. 
         * Clear any buffer on output data.
        */
        void clean(void);

    private:

        /**
         * @brief Communication mode refers to how data is read.
         * @note - A value of 0 means the clock and data are provided by SPI communication.
         * @note - A value of 1 means the clock and data are provided by raw GPIO digital input/output signal communication.
         * @note - This parameter can only be set when the object is created and cannot be changed during the object's lifetime.
         */
        uint8_t _communicationMode;

        double _virtualOffset;

        /**
         * @brief Buffer for angleInput
         *  */ 
        double _posRawDegPast;                   

        /**
         * @brief [us]. Time at update method. used for derivative and low pass filter calculations.
         */
        uint64_t _T;                                

        uint64_t _TRate;

        double _posForRate;

        /**
         * @brief Low pass filter for rate value.
         *  */ 
        EncoderSSI_Namespace::LPF _LPFR;   

        /**
         * @brief Low pass filter for angle value.
         *  */ 
        EncoderSSI_Namespace::LPF _LPFA;   

        /** 
         * @brief Read raw value in SPI mode. Calculate and update values of posRawStep and posRawDeg.
        */
        void _readAngle_spi(void);

        /** 
         * @brief Read raw value in GPIO mode. Calculate and update values of posRawStep and posRawDeg.
        */
        void _readAngle_gpio(void);

        /**
         * @brief Check parameters validation.
         * @return true if succeeded.
         */
        bool _checkParameters(void);

        /**
         * @brief Map angle of encoder to certain range.
         */
        double _mapAngleToCustomRange(double angle, double minRange, double maxRange);

};

