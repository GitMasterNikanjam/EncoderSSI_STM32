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
    #define EncoderSSI_COM_Mode_SPI         0
    #define EncoderSSI_COM_Mode_GPIO        1

    #define EncoderSSI_DATA_FORMAT_BINARY   0
    #define EncoderSSI_DATA_FORMAT_GRAY     1
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

// ##############################################################
// MedianFilter class (small, no dynamic allocation)
class MedianFilter
{
public:
    MedianFilter(): _n(0), _idx(0), _count(0) { clear(); }

    // window: 0=disabled, otherwise must be odd 3–15
    bool setWindow(uint8_t window)
    {
        if(window == 0) { _n = 0; clear(); return true; }
        if( (window < 3) || (window > 15) || ((window % 2) == 0) )
            return false;
        _n = window;
        clear();
        return true;
    }

    // push new value and return median
    double push(double x)
    {
        if(_n == 0) return x; // disabled
        _buf[_idx] = x;
        _idx = (_idx + 1) % _n;
        if(_count < _n) _count++;

        // simple selection sort (since n <= 9)
        for(uint8_t i=0; i<_count; ++i) _tmp[i] = _buf[i];
        for(uint8_t i=0; i<_count; ++i) {
            uint8_t min_i = i;
            for(uint8_t j=i+1; j<_count; ++j)
                if(_tmp[j] < _tmp[min_i]) min_i = j;
            double t = _tmp[i]; _tmp[i] = _tmp[min_i]; _tmp[min_i] = t;
        }
        return _tmp[_count/2];
    }

    void clear() {
        for(uint8_t i=0;i<9;++i){ _buf[i]=0; _tmp[i]=0; }
        _idx=0; _count=0;
    }

    bool enabled() const { return _n != 0; }

private:
    double _buf[15];
    double _tmp[15];
    uint8_t _n;
    uint8_t _idx;
    uint8_t _count;
};

// ##############################################################################################
// LotusEncoderSSI class 

/**
 * @class EncoderSSI
 * @brief EncoderSSI class.
 * @warning - Set the MCU hardware SPI or GPIO communication from outside of this object before using the init method of this object.
 * @warning - Set The TimerControl object that want used for this object from outside of this object before using the init method of this object. 
 * @note The default SSI data output format is binary.
 *  */ 
class EncoderSSI
{
    public:

        /**
         * @brief  Last error occurred for object.
         * @warning The message length is limited.
         *  */ 
        char errorMessage[100];               

        /**
         * @struct ParametersStructure
         * @brief Parameters structure.
         */
        struct ParametersStructure
        {
            /**
             * @brief Communication mode refers to how data is read.
             * @note - A value of 0 means the clock and data are provided by SPI communication.
             * @note - A value of 1 means the clock and data are provided by raw GPIO digital input/output signal communication.
             * @note - This parameter can only be set when the object is created and cannot be changed during the object's lifetime.
             */
            uint8_t READ_MODE;

            /**
             * @brief HAL SPI handle pointer.
             * @note - Only if the communication mode is SPI, this parameter need to be set; otherwise, leave it.
             * @warning - Set the MCU hardware SPI or GPIO communication from outside of this object before using the init method of this object.
            */
            SPI_HandleTypeDef* HSPI;

            /**
             * @brief TimerControl object pointer.
             * @note - Only if the RATE_ENA be true, this parameter need to be set; otherwise, leave it.
             * @warning - Set The TimerControl object that want used for this object from outside of this object before using the init method of this object. 
             *  */ 
            TimerControl* TIMER;

            /**
             * @brief SSI output data format. It can be binary or gray format.
             * @note - Its value can be -> 0:EncoderSSI_DATA_FORMAT_BINARY, 1:EncoderSSI_DATA_FORMAT_GRAY
             */
            uint8_t DATA_FORAMT;
            
            /**
             * @brief Is data has start bit or not.
             * @note A value of true means data has start bit.
             */
            bool START_BIT;

            /**
             * @brief Encoder clock GPIO port. 
             * @note - Only if the communication mode is GPIO, this parameter need to be set; otherwise, leave it.
             * @warning - Set the MCU hardware SPI or GPIO communication from outside of this object before using the init method of this object.
             */
            GPIO_TypeDef* CLK_GPIO_PORT;

            /**
             * @brief Encoder clock GPIO pin. It can be GPIO_PIN_0, GPIO_PIN_1, ... 
             * @note - Only if the communication mode is GPIO, this parameter need to be set; otherwise, leave it.
             * @warning - Set the MCU hardware SPI or GPIO communication from outside of this object before using the init method of this object.
             */
            uint16_t CLK_GPIO_PIN;

            /**
             * @brief Encoder data GPIO port. 
             * @note - Only if the communication mode is GPIO, this parameter need to be set; otherwise, leave it.
             * @warning - Set the MCU hardware SPI or GPIO communication from outside of this object before using the init method of this object.
             */
            GPIO_TypeDef* DATA_GPIO_PORT;

            /**
             * @brief Encoder data GPIO pin. It can be GPIO_PIN_0, GPIO_PIN_1, ... 
             * @note - Only if the communication mode is GPIO, this parameter need to be set; otherwise, leave it.
             * @warning - Set the MCU hardware SPI or GPIO communication from outside of this object before using the init method of this object.
             */
            uint16_t DATA_GPIO_PIN;

            /**
             * @brief Standard SPI mode for encoder data. usually it is mode3: CPOL=1, PHASE=1   
             * @note The spiMode value must be one of these: 0, 1, 2, or 3.
             * 
             * @note SPI mode 0: CPOL=0, CPHA=0  
             * 
             * @note SPI mode 1: CPOL=0, CPHA=1  
             * 
             * @note SPI mode 2: CPOL=1, CPHA=0  
             * 
             * @note SPI mode 3: CPOL=1, CPHA=1  
             *  */       
            uint8_t SPI_MODE; 

            /**
             * @brief SPI baudrate prescaler. It can be SPI_BAUDRATEPRESCALER_2, SPI_BAUDRATEPRESCALER_4, ... .
             */
            uint32_t SPI_BAUDRATE_PRESCALER;
            
            /**
             * @brief The clock frequency for read data in GPIO mode.[Hz]
             */
            uint32_t GPIO_CLOCK_FRQ;

            /**
             * @brief Cut off frequency parameter of low pass filter for rate value. [Hz]
             * @note The value of 0 means it is disabled. 
             *  */ 
            double FLTR;    

            /**
             * @brief Cut off frequency parameter of low pass filter for angle value. [Hz]
             * @note The value of 0 means it is disabled. 
             *  */ 
            double FLTA;  
            
            /**
             * @brief Median filter window size. 0=disabled, valid: 3,5,7,9,11,15   
             * */
            uint8_t FLTM;   
            
            /**
             * @brief slewrate filter gain. 
             * @note The value of 0 means it is disabled. 
             */
            double FLTS;

            /**
             * @brief Mapping enable/disable of encoder data.
             * @note Mapping is an operation for mapping raw positions (in degrees) within a specific position domain.
             */
            bool MAP_ENA;

            /**
             * @brief The value of Map for minimum angle calculations. [deg]
             * @note Mapping is an operation for mapping raw positions (in degrees) within a specific position domain.
             */
            double MAP_MIN;

            /**
             * @brief The value of Map for maximum angle calculations. [deg]
             * @note Mapping is an operation for mapping raw positions (in degrees) within a specific position domain.
             */
            double MAP_MAX;
            
            /**
             * @brief Encoder single turn part resolution.
             * @note It must be lower than 23 bit single turn.
             *  */                  
            uint8_t RESOLUTION_SINGLE_TURN;     

            /**
             * @brief Encoder multi turn part resolution.
             * @note - It must be lower than 23 bit multi turn.
             * @note - For single turn encoders this parameter must be 0.
             *  */                  
            uint8_t RESOLUTION_MULTI_TURN;     
            
            /**
             * @brief Enable/Disbale ignore/clear multi turn bits from reading.
             */
            bool IGNORE_MULTI_TURN;

            /**
             * @brief Offset value for raw angle measurement. [deg]
             * @note Initial value is 0.
             *  */ 
            double POSRAW_OFFSET_DEG;            

            /**
             * @brief Gearbox ratio for angle measurement.
             * @note - output_posDeg = posDeg * gearRatio.
             * @note - The value of 0 means it is disabled.
             *  */  
            double GEAR_RATIO;

            /**
             * @brief Enable/disable rate calculations. The value of true means rate calculations enabled.
             */
            bool RATE_ENA;

            /**
             * @brief The sample rate measurements derived from position data. [Hz]
             * @note The value of 0 means it is disabled.
             */
            double RATE_SPS; 
            
            /**
             * @brief The max frequency for update() method. [Hz]
             * @note The value of 0 means it is disabled.
             */
            float UPDATE_FRQ;

        }parameters;

        /**
         * @struct ValuesStructure
         * @brief Values structure.
         */
        struct ValuesStructure
        {
            /**
             * @brief Raw encoder value. [Step]
             * @note Max value depend on resolution.
             *  */ 
            uint32_t posRawStep;                 

             /**
             * @brief Encoder shaft angle. [deg].
             * @note posRawDeg = (posRawStep / max_posRawStep_singleTurn) * 360
             *  */ 
            double posRawDeg;     

            /**
             * @brief Position angle. [deg]
             * @note posDeg = (posRawDeg - posRawOffsetDeg) * gearRatio.
             *  */ 
            double posDeg;                     

            /**
             * @brief Velocity. [deg/s].
             * @note velDegSec = d(posDeg)/dt 
             *  */      
            double velDegSec;             
        }value;

        /** 
         * @brief EncoderSSI constructor. Set encoder resolution and communication mode. Not apply setting.
         * @param communication_mode: refers to how data is read.
         * 
         * - A value of 0 means the clock and data are provided by SPI communication. (Default value)
         * 
         * - A value of 1 means the clock and data are provided by raw GPIO digital input/output signal communication.
         * 
         * - Other value not acceptable and it set to default value of SPI mode.
         * 
         * - This parameter can only be set when the object is created and cannot be changed during the object's lifetime.
         * @note - init() method needs after this for apply setting on hardware.
         * @note - The default SSI data output format is binary.
         * @warning Set the MCU hardware SPI or GPIO communication from outside of this object before using the init method of this object.
        */      
        EncoderSSI();

        /// @brief Destructor.
        ~EncoderSSI();

        /** 
         * @brief Init/Setup then Apply setting on hardware.
         * @return true if succeeded.
        */
        bool init(void);

        /**
         * @brief Read and update values.
         * @note The rate is updated if RATE_ENA is true. Otherwise, only the position value is updated.
         */
        void update(void);

        /**
         * @brief Set the PresetValue position [deg].
         * @param value is desired output position value [deg] for position at current position.  
         * @return true if succeeded.
         * @warning Use this function after setting direction rotation of encoder. Otherwise it maybe not correct set.
         *  */   
        bool setPresetValueDeg(double value);

        void filterEnable(bool state) {_filterEnable = state;};

        /**
         * @brief Clean setting on hardware.
         * @note Clear any buffer on output data.
        */
        void clean(void);

    private:

        /**
         * @brief Buffer for last position deg for rate calculation. [deg]
         *  */ 
        double _posDegPast;                   

        /**
         * @brief [us]. Time at update method. used for derivative and low pass filter calculations.
         */
        uint64_t _T;                                

        /**
         * @brief The time at rate calculation. [us]
         */
        uint64_t _TRate;

        /// @brief Total encoder resolution equal to single turn resolution + multi turn resolution. It must be lower than 23.
        uint8_t _totalResolution;

        /**
         * @brief The full range value for position. [deg]
         */
        double _fullPosDegRange;

        /**
         * @brief The half range value for position. [deg]
         */
        double _halfPosDegRange;

        /**
         * @brief Low pass filter for rate value.
         *  */ 
        EncoderSSI_Namespace::LPF _LPFR;   

        /**
         * @brief Low pass filter for rate value.
         *  */ 
        EncoderSSI_Namespace::LPF _LPFA;  
        
        /** @brief Median filter */
        MedianFilter _MED;  

        bool _filterEnable;

        /** 
         * @brief Read raw value in SPI mode. Calculate and update values of posRawStep and posRawDeg.
        */
        void _readRaw_spi(void);

        /** 
         * @brief Read raw value in GPIO mode. Calculate and update values of posRawStep and posRawDeg.
        */
        void _readRawgpio(void);

        /**
         * @brief Check parameters validation.
         * @return true if succeeded.
         */
        bool _checkParameters(void);

        /**
         * @brief Map angle of encoder to certain range.
         */
        double _mapAngleToCustomRange(double angle, double minRange, double maxRange);

        /**
         * @brief Enable RCC GPIO PORT for certain port.
         * @return true if successful.
         */
        bool RCC_GPIO_CLK_ENABLE(GPIO_TypeDef *GPIO_PORT);
};

