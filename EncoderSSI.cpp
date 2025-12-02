
// #############################################################################################
// Include libraries:

#include "EncoderSSI.h"

using namespace EncoderSSI_Namespace;

// ###############################################################################################
// Include private macros.

#define _2PI         6.283185307            // 2*3.14159 = 2*pi

// ###############################################################################################

/**
 * @brief Make a bitmask with 'bits' ones (bits in [0..32]).
 */
static inline constexpr uint32_t makeMask(unsigned bits)
{
    return (bits >= 32) ? 0xFFFFFFFFu : ((1u << bits) - 1u);
}

// ###############################################################################################
// LPF and LPFLimit class:

LPF::LPF()
{
    parameters.FRQ = 0;
    _alpha = 1;

    clear();
}

double LPF::updateByTime(const double &input, const double &dt)
{
    if( (dt <= 0) || (parameters.FRQ < 0) )
    {
        // Not update output.;
        return output;
    }

    if(parameters.FRQ == 0)
    {
        output = input;
        return output;
    }

    _alpha = 1.0/(1.0 + 1.0 / (_2PI * parameters.FRQ * dt));
    output = (1.0 - _alpha) * output + _alpha * input;

    return output;
}

double LPF::updateByFrequency(const double &input, const double &frq)
{
    if( (frq <= 0) || (parameters.FRQ < 0) )
    {
        // Not update output.;
        return output;
    }

    if(parameters.FRQ == 0)
    {
        output = input;
        return output;
    }

    _alpha = 1.0/(1.0 + frq / (_2PI * parameters.FRQ));

    output = (1.0 - _alpha) * output + _alpha * input;

    return output;
}

bool LPF::setFrequency(const double &frq)
{
    if(frq < 0)
    {
        return false;
    }

    parameters.FRQ = frq;

    return true;
}

void LPF::setOutputDirect(const double &data)
{
    output = data;
}

void LPF::clear(void)
{
    output = 0;
}

// ###############################################################################################
// LotusEncoderSSI class

EncoderSSI::EncoderSSI()
{    
    parameters.READ_MODE = EncoderSSI_COM_Mode_SPI;
    parameters.CLK_GPIO_PORT = nullptr;
    parameters.DATA_GPIO_PORT = nullptr;
    parameters.CLK_GPIO_PIN = GPIO_PIN_0;
    parameters.DATA_GPIO_PIN = GPIO_PIN_0;
    parameters.TIMER = nullptr;
    parameters.HSPI = nullptr;
    parameters.FLTR = 0;
    parameters.FLTA = 0;
    parameters.FLTM = 0;  // median filter off by default
    parameters.FLTS = 0;
    parameters.RESOLUTION_SINGLE_TURN = 17;
    parameters.RESOLUTION_MULTI_TURN = 0;
    parameters.IGNORE_MULTI_TURN = false;
    parameters.DATA_FORAMT = EncoderSSI_DATA_FORMAT_BINARY;
    parameters.RATE_SPS = 0;
    parameters.UPDATE_FRQ = 0;
    parameters.GEAR_RATIO = 1;
    parameters.POSRAW_OFFSET_DEG = 0;
    parameters.MAP_ENA = false;
    parameters.MAP_MAX = 0;
    parameters.MAP_MIN = 0;
    parameters.RATE_ENA = false;
    parameters.SPI_MODE = 3;
    parameters.SPI_BAUDRATE_PRESCALER = SPI_BAUDRATEPRESCALER_256;
    parameters.GPIO_CLOCK_FRQ = 0;

    _filterEnable = true;

    clean();
}

EncoderSSI::~EncoderSSI()
{
    clean();
}

bool EncoderSSI::init(void)
{
    if(_checkParameters() == false)
    {
        return false;
    }

    _totalResolution = parameters.RESOLUTION_MULTI_TURN + parameters.RESOLUTION_SINGLE_TURN;

    if(parameters.RESOLUTION_MULTI_TURN > 0)
    {
        _fullPosDegRange = pow(2, (double)parameters.RESOLUTION_MULTI_TURN) * 360.0; 
    }
    else if(parameters.RESOLUTION_MULTI_TURN == 0)
    {
        _fullPosDegRange = 360;
    }

    _halfPosDegRange = _fullPosDegRange / 2.0;

    if(parameters.READ_MODE == EncoderSSI_COM_Mode_SPI)
    {
        switch(parameters.SPI_MODE)
        {
            case 0:
                parameters.HSPI->Init.CLKPolarity = SPI_POLARITY_LOW;
                parameters.HSPI->Init.CLKPhase = SPI_PHASE_1EDGE;
            break;
            case 1:
                parameters.HSPI->Init.CLKPolarity = SPI_POLARITY_LOW;
                parameters.HSPI->Init.CLKPhase = SPI_PHASE_2EDGE;
            break;
            case 2:
                parameters.HSPI->Init.CLKPolarity = SPI_POLARITY_HIGH;
                parameters.HSPI->Init.CLKPhase = SPI_PHASE_1EDGE;
            break;
            case 3:
                parameters.HSPI->Init.CLKPolarity = SPI_POLARITY_HIGH;
                parameters.HSPI->Init.CLKPhase = SPI_PHASE_2EDGE;
            break;
        }

        parameters.HSPI->Init.BaudRatePrescaler = parameters.SPI_BAUDRATE_PRESCALER;

        if (HAL_SPI_Init(parameters.HSPI) != HAL_OK) 
        {
            sprintf(errorMessage, "HAL_SPI_Init() is not succeeded.");
            return false;
        }
    }
    else if(parameters.READ_MODE == EncoderSSI_COM_Mode_GPIO)
    {
        if(RCC_GPIO_CLK_ENABLE(parameters.CLK_GPIO_PORT) == false)
        {
            sprintf(errorMessage, "RCC_GPIO_CLK_ENABLE() is not succeeded.");
            return false;
        }

        if(RCC_GPIO_CLK_ENABLE(parameters.DATA_GPIO_PORT) == false)
        {
            sprintf(errorMessage, "RCC_GPIO_CLK_ENABLE() is not succeeded.");
            return false;
        }

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = parameters.CLK_GPIO_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Normal push-pull output
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(parameters.CLK_GPIO_PORT, &GPIO_InitStruct);

        GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = parameters.DATA_GPIO_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(parameters.DATA_GPIO_PORT, &GPIO_InitStruct);

        switch(parameters.SPI_MODE)
        {
            case 0:
                HAL_GPIO_WritePin(parameters.CLK_GPIO_PORT, parameters.CLK_GPIO_PIN, GPIO_PIN_RESET);
            break;
            case 1:
                HAL_GPIO_WritePin(parameters.CLK_GPIO_PORT, parameters.CLK_GPIO_PIN, GPIO_PIN_RESET);
            break;
            case 2:
                HAL_GPIO_WritePin(parameters.CLK_GPIO_PORT, parameters.CLK_GPIO_PIN, GPIO_PIN_SET);
            break;
            case 3:
                HAL_GPIO_WritePin(parameters.CLK_GPIO_PORT, parameters.CLK_GPIO_PIN, GPIO_PIN_SET);
            break;
        }
    }
    
    if(parameters.RATE_ENA == true)
    {
        if(!_LPFR.setFrequency(parameters.FLTR) || !_LPFR.setFrequency(parameters.FLTR))
        {
            sprintf(errorMessage, "Filter frequency for rate is not correct value.");
            return false;
        }
    }
    
    _LPFA.setFrequency(parameters.FLTA);
    _T = parameters.TIMER->micros();

    // --- Median filter configuration:
    if(!_MED.setWindow(parameters.FLTM))
    {
        sprintf(errorMessage, "Median window invalid. Use 0 or {3,5,7,9,11,15}.");
        return false;
    }

    return true;
}

void EncoderSSI::_readRaw_spi(void)
{
    // Reset value of reading data
    uint8_t encoder_read_data[3] = {0, 0, 0};

    // Transfer Dummy data for generate clock 
    if(_totalResolution > 16)
    { 
        HAL_SPI_Receive(parameters.HSPI, (uint8_t*)encoder_read_data, 3, HAL_MAX_DELAY);    
    }
    else if(_totalResolution > 8)
    {
        HAL_SPI_Receive(parameters.HSPI, (uint8_t*)encoder_read_data, 2, HAL_MAX_DELAY);
    }
    else
    {
        HAL_SPI_Receive(parameters.HSPI, (uint8_t*)encoder_read_data, 1, HAL_MAX_DELAY);
    }

    encoder_read_data[0] = encoder_read_data[0] & 0b01111111;       // Clear first bit data for start communication.

    double rawDataDeg = 0;

    uint32_t _rawDataStep = (  (((uint32_t)encoder_read_data[0]) << 16) | ((uint32_t)encoder_read_data[1] << 8) | (((uint32_t)encoder_read_data[2])) );
    _rawDataStep = (_rawDataStep >> (24 - (_totalResolution + 1)));

    if(parameters.IGNORE_MULTI_TURN == true)
    {
        _rawDataStep = (_rawDataStep & makeMask(parameters.RESOLUTION_SINGLE_TURN));
    }

    rawDataDeg = (double)_rawDataStep / pow(2, (double)parameters.RESOLUTION_SINGLE_TURN) * 360.0;

    value.posRawStep = _rawDataStep;
    value.posRawDeg = rawDataDeg;
}

void EncoderSSI::_readRawgpio_new(void)
{
    if((parameters.CLK_GPIO_PORT == nullptr) || (parameters.DATA_GPIO_PORT == nullptr) ||
       (parameters.TIMER == nullptr))
    {
        return;
    }

    int ii=0;
    int i=0;
    int wait1 = 40;
    int wait2 = 90;
    uint8_t read_data_bit=0;
    float temp_Encoder_R =0;
    float Encoder_value=0;
    uint32_t Encoder_R=0;

    HAL_GPIO_WritePin(parameters.CLK_GPIO_PORT, parameters.CLK_GPIO_PIN, GPIO_PIN_RESET);
    for(ii=0;ii<=wait2;ii++);		
    Encoder_R=0; //131072
    
    for(i=1;i<=14;i++){
        //Encoder_R=Encoder_R<<1; 
        HAL_GPIO_WritePin(parameters.CLK_GPIO_PORT, parameters.CLK_GPIO_PIN, GPIO_PIN_SET);
        for(ii=0;ii<=wait1;ii++);		//wait
        HAL_GPIO_WritePin(parameters.CLK_GPIO_PORT, parameters.CLK_GPIO_PIN, GPIO_PIN_RESET);
        for(ii=0;ii<=wait2;ii++);		//wait
        read_data_bit=HAL_GPIO_ReadPin(parameters.DATA_GPIO_PORT, parameters.DATA_GPIO_PIN);
    
        //Encoder_R=Encoder_R|read_data_bit;  // write one bit data
    }
    for(i=1;i<=10;i++){
        Encoder_R=Encoder_R<<1; 
        HAL_GPIO_WritePin(parameters.CLK_GPIO_PORT, parameters.CLK_GPIO_PIN, GPIO_PIN_SET);
        for(ii=0;ii<=wait1;ii++);		//wait
        HAL_GPIO_WritePin(parameters.CLK_GPIO_PORT, parameters.CLK_GPIO_PIN, GPIO_PIN_RESET);
        for(ii=0;ii<=wait2;ii++);		//wait
        read_data_bit=HAL_GPIO_ReadPin(parameters.DATA_GPIO_PORT, parameters.DATA_GPIO_PIN);
    
        Encoder_R=Encoder_R|read_data_bit;  // write one bit data
    }
    HAL_GPIO_WritePin(parameters.CLK_GPIO_PORT, parameters.CLK_GPIO_PIN, GPIO_PIN_SET);
    temp_Encoder_R=Encoder_R;
    Encoder_value=(temp_Encoder_R*360.0/1024.0);

    value.posRawStep = (double)temp_Encoder_R;
    value.posRawDeg = (double)Encoder_value;
}

void EncoderSSI::_readRawgpio(void)
{   
    if((parameters.CLK_GPIO_PORT == nullptr) || (parameters.DATA_GPIO_PORT == nullptr) ||
       (parameters.TIMER == nullptr))
    {
        return;
    }

    // Reset value of reading data
    uint32_t encoder_read_data = 0;

    uint32_t half_period_us = (uint32_t)(500000.0 / (float)parameters.GPIO_CLOCK_FRQ);  // Half-period in µs

    for(int i = 0; i <= _totalResolution; i++)
    {
        switch(parameters.SPI_MODE)
        {
            case 0:
                // CLK HIGH
                HAL_GPIO_WritePin(parameters.CLK_GPIO_PORT, parameters.CLK_GPIO_PIN, GPIO_PIN_SET);
                parameters.TIMER->delayMicroseconds(half_period_us);

                // Read DATA on rising edge (check your encoder datasheet!)
                encoder_read_data <<= 1;
                if (HAL_GPIO_ReadPin(parameters.DATA_GPIO_PORT, parameters.DATA_GPIO_PIN) == GPIO_PIN_SET)
                {
                    encoder_read_data |= 1;
                }

                // CLK LOW
                HAL_GPIO_WritePin(parameters.CLK_GPIO_PORT, parameters.CLK_GPIO_PIN, GPIO_PIN_RESET);
                parameters.TIMER->delayMicroseconds(half_period_us);

                break;
            case 1:
                // CLK HIGH
                HAL_GPIO_WritePin(parameters.CLK_GPIO_PORT, parameters.CLK_GPIO_PIN, GPIO_PIN_SET);
                parameters.TIMER->delayMicroseconds(half_period_us);

                // CLK LOW
                HAL_GPIO_WritePin(parameters.CLK_GPIO_PORT, parameters.CLK_GPIO_PIN, GPIO_PIN_RESET);
                parameters.TIMER->delayMicroseconds(half_period_us);

                // Read DATA on rising edge (check your encoder datasheet!)
                encoder_read_data <<= 1;
                if (HAL_GPIO_ReadPin(parameters.DATA_GPIO_PORT, parameters.DATA_GPIO_PIN) == GPIO_PIN_SET)
                {
                    encoder_read_data |= 1;
                }

                break;
            case 2:
                // CLK LOW
                HAL_GPIO_WritePin(parameters.CLK_GPIO_PORT, parameters.CLK_GPIO_PIN, GPIO_PIN_RESET);
                parameters.TIMER->delayMicroseconds(half_period_us);

                // Read DATA on rising edge (check your encoder datasheet!)
                encoder_read_data <<= 1;
                if (HAL_GPIO_ReadPin(parameters.DATA_GPIO_PORT, parameters.DATA_GPIO_PIN) == GPIO_PIN_SET)
                {
                    encoder_read_data |= 1;
                }

                // CLK HIGH
                HAL_GPIO_WritePin(parameters.CLK_GPIO_PORT, parameters.CLK_GPIO_PIN, GPIO_PIN_SET);
                parameters.TIMER->delayMicroseconds(half_period_us);

                break;
            case 3:
                // CLK LOW
                HAL_GPIO_WritePin(parameters.CLK_GPIO_PORT, parameters.CLK_GPIO_PIN, GPIO_PIN_RESET);
                parameters.TIMER->delayMicroseconds(half_period_us);

                // CLK HIGH
                HAL_GPIO_WritePin(parameters.CLK_GPIO_PORT, parameters.CLK_GPIO_PIN, GPIO_PIN_SET);
                
                parameters.TIMER->delayMicroseconds(half_period_us);
                // Read DATA on rising edge (check your encoder datasheet!)
                encoder_read_data <<= 1;
                if (HAL_GPIO_ReadPin(parameters.DATA_GPIO_PORT, parameters.DATA_GPIO_PIN) == GPIO_PIN_SET)
                {
                    encoder_read_data |= 1;
                }

                break;
        }  
    }

    encoder_read_data = encoder_read_data & ~((uint32_t)0b1 << _totalResolution);       // Clear first bit data for start communication.

    double rawDataDeg = 0;

    uint32_t _rawDataStep = encoder_read_data;

    if(parameters.IGNORE_MULTI_TURN == true)
    {
        _rawDataStep = (_rawDataStep & makeMask(parameters.RESOLUTION_SINGLE_TURN));
    }

    rawDataDeg = (double)_rawDataStep / pow(2, (double)parameters.RESOLUTION_SINGLE_TURN) * 360.0;
    // rawDataDeg = ((double)_rawDataStep * 360.0) / (1 << parameters.RESOLUTION_SINGLE_TURN);

    value.posRawStep = _rawDataStep;
    value.posRawDeg = rawDataDeg;
}

double EncoderSSI::_mapAngleToCustomRange(double angle, double minRange, double maxRange) 
{ 
    double range = maxRange - minRange; 
    double temp = fmod(angle - minRange, range); // Shift angle to start at minRange 
    if (temp < 0) 
    { 
        temp += range; 
    } 
    return temp + minRange; 
}

bool EncoderSSI::RCC_GPIO_CLK_ENABLE(GPIO_TypeDef *GPIO_PORT)
{
    if(GPIO_PORT == nullptr)
    {
        return false;
    }

    #ifdef GPIOA
    if(GPIO_PORT == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOB
    else if (GPIO_PORT == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOC
    else if (GPIO_PORT == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOD
    else if (GPIO_PORT == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOE
    else if (GPIO_PORT == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOF
    else if (GPIO_PORT == GPIOF)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOG
    else if (GPIO_PORT == GPIOG)
    {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOH
    else if (GPIO_PORT == GPIOH)
    {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    }
    #endif
    #ifdef GPIOI
    else if (GPIO_PORT == GPIOI)
    {
        __HAL_RCC_GPIOI_CLK_ENABLE();
    }
    #endif
    else
    {
        return false;
    }

    return true;
}

void EncoderSSI::update(void)
{
    uint64_t T_now = parameters.TIMER->micros();
    
    uint64_t dt_u = (T_now >= _T) ? (T_now - _T) : (UINT64_MAX - _T + T_now + 1ULL);

    if(parameters.UPDATE_FRQ != 0)
    {
        if(dt_u < (1000000.0 / parameters.UPDATE_FRQ))
        {
            return;
        }
    }
    
    switch(parameters.READ_MODE)
    {
        case EncoderSSI_COM_Mode_SPI:
            _readRaw_spi();
        break;
        case EncoderSSI_COM_Mode_GPIO:
            _readRawgpio();
        break;
    }

    double temp = (value.posRawDeg - parameters.POSRAW_OFFSET_DEG); 

    if(parameters.MAP_ENA == true)
    {
        temp = _mapAngleToCustomRange(temp, parameters.MAP_MIN / parameters.GEAR_RATIO, parameters.MAP_MAX / parameters.GEAR_RATIO);
    }

    if(parameters.GEAR_RATIO !=0)
    {
        temp = temp * parameters.GEAR_RATIO;
    }

    if(_filterEnable == true)
    {
        if(parameters.FLTS > 0)
        {
            if(abs(temp - value.posDeg) > 1)
            {
                if(temp > value.posDeg)
                {
                    temp = value.posDeg + parameters.FLTS;
                }
                else
                {
                    temp = value.posDeg - parameters.FLTS;
                }
            }
        }

        // Apply median filter BEFORE low-pass if enabled
        if(_MED.enabled())
        {
            temp = _MED.push(temp);
        }
    }
    
    if(parameters.RATE_ENA == true)
    {
        if(T_now > _TRate)
        {
            uint64_t dt_uint = (T_now - _TRate);
            double dtRate = (double)dt_uint;
            double frqRate = 1000000 / dtRate;
            
            if(parameters.RATE_SPS > 0)
            {
                if( frqRate <= parameters.RATE_SPS )
                {
                    value.velDegSec =  _LPFR.updateByFrequency((temp - _posDegPast) * frqRate, frqRate);
                    _posDegPast = temp;
                    _TRate = T_now;
                }
            }
            else
            {
                value.velDegSec =  _LPFR.updateByFrequency((temp - value.posDeg) * frqRate, frqRate);
                _TRate = T_now;
                _posDegPast = temp;
            }
        }
        else
        {
            _TRate = T_now;
            _posDegPast = temp;
        }  
    }

    if((parameters.FLTA > 0))
    {
        if(T_now > _T)
        {
            uint64_t dt_uint = T_now - _T;
            double dt_double = ((double)dt_uint) / 1000000.0;

            if(_filterEnable == true)
            {
                value.posDeg = _LPFA.updateByTime(temp, dt_double); 
            }
            else
            {
                _LPFA.setOutputDirect(temp);
                value.posDeg = temp;
            } 
        }
    }
    else
    {
        value.posDeg = temp;
    }  

    _T = T_now;
}

bool EncoderSSI::setPresetValueDeg(double data)
{
    parameters.TIMER->delayMicroseconds(100);
    switch(parameters.READ_MODE)
    {
        case EncoderSSI_COM_Mode_SPI:
            _readRaw_spi();
        break;
        case EncoderSSI_COM_Mode_GPIO:
            _readRawgpio();
        break;
    }

    parameters.POSRAW_OFFSET_DEG = value.posRawDeg - data / parameters.GEAR_RATIO;

    return true;
}

void filterEnable(bool state)
{

}

void EncoderSSI::clean(void)
{
    value.posDeg = 0;
    value.posRawDeg = 0;
    value.posRawStep = 0;
    value.velDegSec = 0;

    _posDegPast = 0;
    _T = 0;
    _TRate = 0;
    _totalResolution = 0;

    _MED.clear();   // reset median filter buffer
}

bool EncoderSSI::_checkParameters(void)
{
    switch(parameters.READ_MODE)
    {
        case EncoderSSI_COM_Mode_SPI:
            if(parameters.HSPI == nullptr)
            {
                sprintf(errorMessage, "HAL SPI handle pointer can not be nullptr.");
                return false;
            }
        break;
        case EncoderSSI_COM_Mode_GPIO:
            if( (parameters.CLK_GPIO_PORT == nullptr) || (parameters.DATA_GPIO_PORT == nullptr))
            {
                sprintf(errorMessage, "GPIO port pointer for clock or data can not be nullptr.");
                return false;
            }
        break;
        default:
            return false;
    }
    
    bool state = true;

    state = state && (parameters.FLTR >= 0) && (parameters.FLTA >= 0) &&
                     (parameters.FLTS >= 0) &&
                     (parameters.SPI_MODE <= 3) &&
                     (parameters.DATA_FORAMT <= 1) && (parameters.RATE_SPS >= 0) && (parameters.UPDATE_FRQ >= 0) &&
                     (parameters.RESOLUTION_SINGLE_TURN > 0) && (parameters.RESOLUTION_MULTI_TURN >= 0) &&
                     ((parameters.RESOLUTION_SINGLE_TURN + parameters.RESOLUTION_MULTI_TURN) <= 23);

    // Median window valid?
    if( !(parameters.FLTM == 0 || parameters.FLTM == 3 ||
        parameters.FLTM == 5 || parameters.FLTM == 7 || parameters.FLTM == 9 || parameters.FLTM == 11 || parameters.FLTM == 15) )
    {
        state = false;
    }

    if(!state)
    {
        sprintf(errorMessage, "Parameter validation failed");
        return false;
    }

    if( (parameters.MAP_ENA == true) && (parameters.MAP_MAX <= parameters.MAP_MIN) )
    {
        sprintf(errorMessage, "Mapping parameters validation.");
        return false;
    }
    
    return true;
}



