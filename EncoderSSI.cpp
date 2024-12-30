
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
        _communicationMode = 0;
    }

    parameters.CLK_GPIO_PORT = nullptr;
    parameters.DATA_GPIO_PORT = nullptr;
    parameters.CLK_GPIO_PIN = GPIO_PIN_0;
    parameters.DATA_GPIO_PIN = GPIO_PIN_0;
    parameters.TIMER = nullptr;
    parameters.HSPI = nullptr;
    parameters.FLTA = 0;
    parameters.FLTR = 0;
    parameters.RESOLUTION = 17;
    parameters.RATE_FRQ = 0;
    parameters.GEAR_RATIO = 1;
    parameters.POSRAW_OFFSET_DEG = 0;
    parameters.MAP_ENA = false;
    parameters.MAP_MAX = 0;
    parameters.MAP_MIN = 0;

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

    if (HAL_SPI_Init(parameters.HSPI) != HAL_OK) 
    {
        errorMessage = "Error EncoderSSI: HAL_SPI_Init is not succeeded.";
        return false;
    }
        
    if(!_LPFR.setFrequency(parameters.FLTR) || !_LPFR.setFrequency(parameters.FLTR))
    {
        errorMessage = "Error LotusEncoderSSI: Filter frequency for rate is not correct value.";
        return false;
    }

    if(!_LPFA.setFrequency(parameters.FLTA) || !_LPFA.setFrequency(parameters.FLTA))
    {
        errorMessage = "Error LotusEncoderSSI: Filter frequency for angle is not correct value.";
        return false;
    }

    _T = parameters.TIMER->micros();
    
    return true;
}

void EncoderSSI::_readAngle_spi(void)
{
    // Reset value of reading data
    char encoder_read_data[3] = {0, 0, 0};

    // Transfer Dummy data for generate clock 
    if(parameters.RESOLUTION > 16)
    { 
        HAL_SPI_TransmitReceive(parameters.HSPI, (uint8_t*)encoder_read_data, (uint8_t*)encoder_read_data, 3, HAL_MAX_DELAY);    
    }
    else if(parameters.RESOLUTION > 8)
    {
        HAL_SPI_TransmitReceive(parameters.HSPI, (uint8_t*)encoder_read_data, (uint8_t*)encoder_read_data, 2, HAL_MAX_DELAY);
    }
    else
    {
        HAL_SPI_TransmitReceive(parameters.HSPI, (uint8_t*)encoder_read_data, (uint8_t*)encoder_read_data, 1, HAL_MAX_DELAY);
    }

    encoder_read_data[0] = encoder_read_data[0] & 0b01111111;       // Clear first bit data for start communication.

    uint32_t rawDataStep = 0;
    double rawDataDeg = 0;

    rawDataStep = (  (((uint32_t)encoder_read_data[0]) << 16) | ((uint32_t)encoder_read_data[1] << 8) | (((uint32_t)encoder_read_data[2])) );
    rawDataStep = (rawDataStep >> (24 - parameters.RESOLUTION - 1));   // Clear extera bits.

    rawDataDeg = (double)rawDataStep / pow(2, (double)parameters.RESOLUTION) * 360.0;

    value.posRawStep = rawDataStep;
    _posRawDegPast = value.posRawDeg;
    value.posRawDeg = rawDataDeg;

    if( (value.posRawDeg - _posRawDegPast) < -180 ) 
    {
        value.revolutionCounter++;
    }
    else if( ((value.posRawDeg - _posRawDegPast) > 180) && (_posRawDegPast != 0))
    {
        value.revolutionCounter--;
    }
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

    _readAngle_spi();

    double temp = ( (value.posRawDeg - parameters.POSRAW_OFFSET_DEG + value.revolutionCounter * 360.0) * parameters.GEAR_RATIO );
    _LPFA.updateByTime(temp, dt);
    if(parameters.RATE_FRQ > 0)
    {
        double dt_rate = ((double)(T_now - _TRate) / 1000000.0);

        if( dt_rate >= (1.0 / parameters.RATE_FRQ) )
        {
            value.velDegSec = (_LPFA.output - _posForRate) / dt_rate;
            value.velDegSec =  _LPFR.updateByTime(value.velDegSec, dt_rate);
            _TRate = T_now;
            _posForRate = _LPFA.output;
        }
    }
    else
    {
        value.velDegSec = (_LPFA.output - value.posDeg) / dt;
        value.velDegSec =  _LPFR.updateByTime(value.velDegSec, dt);
    }
    
    value.posDeg = _LPFA.output;  

    if(parameters.MAP_ENA == true)
    {
        value.posDeg = _mapAngleToCustomRange(value.posDeg, parameters.MAP_MIN, parameters.MAP_MAX);
    }
    
}

bool EncoderSSI::setPresetValueDeg(double data)
{
    parameters.TIMER->delayMicroseconds(100);
    switch(_communicationMode)
    {
        case EncoderSSI_COM_Mode_SPI:
            _readAngle_spi();
        break;
        case EncoderSSI_COM_Mode_GPIO:

        break;
    }

    parameters.POSRAW_OFFSET_DEG = value.posRawDeg - data;

    return true;
}

void EncoderSSI::clean(void)
{
    value.posDeg = 0;
    value.posRawDeg = 0;
    value.posRawStep = 0;
    value.velDegSec = 0;
    value.revolutionCounter = 0;

    _virtualOffset = 0;
    _posRawDegPast = 0;
    _T = 0;
    _TRate = 0;
    _posForRate = 0;
}

bool EncoderSSI::_checkParameters(void)
{
    switch(_communicationMode)
    {
        case EncoderSSI_COM_Mode_SPI:
            if(parameters.HSPI == nullptr)
            {
                errorMessage = "Error EncoderSSI: HAL SPI handle pointer can not be nullptr.";
                return false;
            }
        break;
        case EncoderSSI_COM_Mode_GPIO:
            if( (parameters.CLK_GPIO_PORT == nullptr) || (parameters.DATA_GPIO_PORT == nullptr))
            {
                errorMessage = "Error EncoderSSI: GPIO port pointer for clock or data can not be nullptr.";
                return false;
            }
        break;
        default:
            return false;
    }
    
    bool state;

    state = state && (parameters.FLTA >= 0) && (parameters.FLTA >= 0) &&
                     (parameters.FLTR >= 0) && (parameters.FLTR >= 0) &&
                     (parameters.RESOLUTION > 0) && (parameters.RESOLUTION <= 23) &&
                     (parameters.RESOLUTION > 0) && (parameters.RESOLUTION <= 23);

    if(state == false)
    {
        errorMessage = "Error EncoderSSI: One or some parameters are not correct.";
    }

    if( (parameters.MAP_ENA == true) && (parameters.MAP_MAX <= parameters.MAP_MIN) )
    {
        errorMessage = "Error EncoderSSI: mapMax can not be equal or less than mapMin when mapENA is true.";
        return false;
    }
    
    return true;
}



