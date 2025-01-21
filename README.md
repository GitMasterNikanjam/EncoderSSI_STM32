# EncoderSSI Library for STM32

- This library can be used for encoder SSI sensor managements.
- The rate of position estimated by position derivative with lowpass filter for smoothing.

----------------------------------------------------------------------------------------------------------

## Public Member Variables For EncoderSSI Class

```cpp
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
     * @brief Cut off frequency parameter of low pass filter for rate value. [Hz]
     * @note The value of 0 means it is disabled. 
     *  */ 
    double FLTR;    

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
     * @brief Offset value for raw angle measurement. [deg]
     * @note Initial value is 0.
     *  */ 
    double POSRAW_OFFSET_DEG;            

    /**
     * @brief Gearbox ratio for angle measurement.
     * @note output_posDeg = posDeg * gearRatio.
     *  */  
    double GEAR_RATIO;

    /**
     * @brief Enable/disable rate calculations. The value of true means rate calculations enabled.
     */
    bool RATE_ENA;

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
     * @note posDeg = (posRawDeg - posRawOffsetDeg + overFlowCounter * _fullRange) * gearRatio.
     *  */ 
    double posDeg;                     

    /**
     * @brief Velocity. [deg/s].
     * @note velDegSec = d(posDeg)/dt 
     *  */      
    double velDegSec;   

    ///! @brief Number of cross over from overflow for posRawStep since object created.                
    int32_t overFlowCounter;             
}value;
```

## Public Member Functions For EncoderSSI Class

```cpp
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
EncoderSSI(uint8_t communication_mode = EncoderSSI_COM_Mode_SPI);

/// @brief Destructor.
~EncoderSSI();

/** 
 * @brief Init/Setup then Apply setting on hardware.
 * @return true if succeeded.
*/
bool init(void);

/**
 * @brief Read and update values.
 */
void update(void);

/**
 * @brief Set the PresetValue position [deg].
 * @param value is desired output position value [deg] for position at current position.  
 * @return true if succeeded.
 * @warning Use this function after setting direction rotation of encoder. Otherwise it maybe not correct set.
 *  */  
bool setPresetValueDeg(double value);

/**
 * @brief Clean setting on hardware.
 * @note Clear any buffer on output data.
*/
void clean(void);

```