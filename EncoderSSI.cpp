
// #############################################################################################
// Include libraries:

#include "EncoderSSI.h"

using namespace EncoderSSI_Namespace;

// ###############################################################################################
// Include private macros.

#define _2PI         6.283185307            // 2*3.14159 = 2*pi

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

    return true;
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

EncoderSSI::EncoderSSI(uint8_t communication_mode)
{    
    if(communication_mode <= 1)
    {
        _communicationMode = communication_mode;
    }
    else
    {
        _communicationMode = EncoderSSI_COM_Mode_SPI;
    }

    parameters.CLK_GPIO_PORT = nullptr;
    parameters.DATA_GPIO_PORT = nullptr;
    parameters.CLK_GPIO_PIN = GPIO_PIN_0;
    parameters.DATA_GPIO_PIN = GPIO_PIN_0;
    parameters.TIMER = nullptr;
    parameters.HSPI = nullptr;
    parameters.FLTR = 0;
    parameters.RESOLUTION_SINGLE_TURN = 17;
    parameters.RESOLUTION_MULTI_TURN = 0;
    parameters.DATA_FORAMT = EncoderSSI_DATA_FORMAT_BINARY;
    parameters.RATE_FRQ = 0;
    parameters.GEAR_RATIO = 1;
    parameters.POSRAW_OFFSET_DEG = 0;
    parameters.MAP_ENA = false;
    parameters.MAP_MAX = 0;
    parameters.MAP_MIN = 0;
    parameters.RATE_ENA = false;
    parameters.SPI_MODE = 3;
    parameters.SPI_BAUDRATE_PRESCALER = SPI_BAUDRATEPRESCALER_256;

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

    if(_communicationMode == EncoderSSI_COM_Mode_SPI)
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
    /*
    else if(_communicationMode = EncoderSSI_COM_Mode_GPIO)
    {

    }
    */
    if(parameters.RATE_ENA == true)
    {
        if(!_LPFR.setFrequency(parameters.FLTR) || !_LPFR.setFrequency(parameters.FLTR))
        {
            sprintf(errorMessage, "Filter frequency for rate is not correct value.");
            return false;
        }

        _T = parameters.TIMER->micros();
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
    rawDataDeg = (double)_rawDataStep / pow(2, (double)parameters.RESOLUTION_SINGLE_TURN) * 360.0;

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
    } return temp + minRange; 
}

void EncoderSSI::update(void)
{
    uint64_t T_now = parameters.TIMER->micros();
    double dt = (double)(T_now - _T) / 1000000.0;
    _T = T_now;

    if(dt <= 0)
    {
        return;
    }

    _readRaw_spi();

    double temp = ( (value.posRawDeg - parameters.POSRAW_OFFSET_DEG + value.overFlowCounter * _fullPosDegRange) * parameters.GEAR_RATIO );

    if(parameters.RATE_ENA == true)
    {
        if(parameters.RATE_FRQ > 0)
        {
            double dt_rate = ((double)(T_now - _TRate) / 1000000.0);

            if( dt_rate >= (1.0 / parameters.RATE_FRQ) )
            {
                value.velDegSec = (temp - value.posDeg) / dt_rate;
                value.velDegSec =  _LPFR.updateByTime(value.velDegSec, dt_rate);
                _TRate = T_now;
            }
        }
        else
        {
            value.velDegSec = (temp - value.posDeg) / dt;
            value.velDegSec =  _LPFR.updateByTime(value.velDegSec, dt);
        }
    }
    
    // if( (temp - value.posDeg) < -_halfPosDegRange ) 
    // {
    //     value.overFlowCounter++;
    // }
    // else if( ((temp - value.posDeg) > _halfPosDegRange) && (_posDegPast != 0))
    // {
    //     value.overFlowCounter--;
    // }  

    if(parameters.MAP_ENA == true)
    {
        temp = _mapAngleToCustomRange(temp, parameters.MAP_MIN, parameters.MAP_MAX);
    }

    value.posDeg = temp;
}

bool EncoderSSI::setPresetValueDeg(double data)
{
    parameters.TIMER->delayMicroseconds(100);
    switch(_communicationMode)
    {
        case EncoderSSI_COM_Mode_SPI:
            _readRaw_spi();
        break;
        case EncoderSSI_COM_Mode_GPIO:

        break;
    }

    parameters.POSRAW_OFFSET_DEG = value.posRawDeg - data / parameters.GEAR_RATIO;

    return true;
}

void EncoderSSI::clean(void)
{
    value.posDeg = 0;
    value.posRawDeg = 0;
    value.posRawStep = 0;
    value.velDegSec = 0;
    value.overFlowCounter = 0;

    _posDegPast = 0;
    _T = 0;
    _TRate = 0;
    _totalResolution = 0;
}

bool EncoderSSI::_checkParameters(void)
{
    switch(_communicationMode)
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
    
    bool state;

    state = state && (parameters.FLTR >= 0) && (parameters.FLTR >= 0) &&
                     (parameters.SPI_MODE <= 3) &&
                     (parameters.DATA_FORAMT <= 1) && (parameters.RATE_FRQ >= 0) &&
                     (parameters.RESOLUTION_SINGLE_TURN > 0) && (parameters.RESOLUTION_MULTI_TURN >= 0) &&
                     ((parameters.RESOLUTION_SINGLE_TURN + parameters.RESOLUTION_MULTI_TURN) <= 23);

    if(state == false)
    {
        sprintf(errorMessage, "Parameter validation");
    }

    if( (parameters.MAP_ENA == true) && (parameters.MAP_MAX <= parameters.MAP_MIN) )
    {
        sprintf(errorMessage, "Mapping parameters validation.");
        return false;
    }
    
    return true;
}



