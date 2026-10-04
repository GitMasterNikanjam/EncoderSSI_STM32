# EncoderSSI_STM32

`EncoderSSI_STM32` reads position from an SSI absolute encoder using either an STM32 HAL SPI peripheral or clock/data GPIO pins. It converts the encoder frame to a position in degrees and can calculate angular velocity and apply position filters.

The library is written in C++ and uses STM32 HAL. The implementation is in [`src/EncoderSSI.cpp`](src/EncoderSSI.cpp), with the public class and parameter definitions in [`src/EncoderSSI.h`](src/EncoderSSI.h). A CubeMX/Keil STM32F407 example is provided under [`examples/STM32F407VGT6/ex1`](examples/STM32F407VGT6/ex1).

## Features

- SSI data capture using HAL SPI or software-generated GPIO clock
- Binary or Gray-coded position data
- Optional start-bit handling and multi-turn bit masking
- Single-turn and multi-turn position conversion
- Position offset, gear ratio, and optional angle-range mapping
- Optional rate calculation in degrees per second
- Optional rate and angle low-pass filters, median filter, and per-update slew limiting
- SPI read timeout and `errorMessage` reporting

## Requirements and integration

- An STM32 project with the HAL for the selected MCU family
- C++11 or later
- The companion [`TimerControl_STM32`](../TimerControl_STM32) library; the encoder requires a `TimerControl` object even when rate calculation is disabled
- A `mcu_select.h` reachable through the project's include paths, defining exactly one supported family macro: `STM32F1`, `STM32F4`, or `STM32H7`

Add `EncoderSSI_STM32/src` and `TimerControl_STM32/src` to the project's include paths. Add both `EncoderSSI.cpp` and `TimerControl.cpp` to the build. Ensure the selected HAL and its device headers are also on the include path. The family selection is read by both libraries from `mcu_select.h`.

## Quick start: SPI mode

Configure and initialize the HAL SPI and timer peripherals in the normal project startup code. The timer clock passed to `TimerControl::setClockFrequency()` must be the actual timer input clock in hertz, not necessarily the APB peripheral clock; account for the STM32 timer clock multiplier when the APB prescaler is greater than one. `TimerControl` accepts clock frequencies of at least 1 MHz that are exact multiples of 1 MHz.

```cpp
#include "EncoderSSI.h"

extern SPI_HandleTypeDef hspi3; // Initialized by the application/HAL.
extern TIM_HandleTypeDef htim2; // Initialized by the application/HAL.

TimerControl encoderTimer(&htim2);
EncoderSSI encoder;

bool initEncoder(uint32_t tim2_input_clock_hz)
{
    // Call after HAL_Init(), clock setup, MX_SPI3_Init(), and MX_TIM2_Init().
    if (!encoderTimer.setClockFrequency(tim2_input_clock_hz) ||
        !encoderTimer.init() ||
        !encoderTimer.start()) {
        return false;
    }

    encoder.parameters.HSPI = &hspi3;
    encoder.parameters.TIMER = &encoderTimer;
    encoder.parameters.READ_MODE = EncoderSSI_COM_Mode_SPI;
    encoder.parameters.RESOLUTION_SINGLE_TURN = 17;
    encoder.parameters.RESOLUTION_MULTI_TURN = 0;
    encoder.parameters.DATA_FORAMT = EncoderSSI_DATA_FORMAT_BINARY;
    encoder.parameters.START_BIT = true;
    encoder.parameters.SPI_MODE = 3;
    encoder.parameters.SPI_BAUDRATE_PRESCALER = SPI_BAUDRATEPRESCALER_128;
    encoder.parameters.GEAR_RATIO = 1.0;
    encoder.parameters.RATE_ENA = true;
    encoder.parameters.FLTR = 0.0; // Rate low-pass cutoff; 0 disables filtering.

    return encoder.init();
}

void sampleEncoder()
{
    if (encoder.update()) {
        const double rawDegrees = encoder.value.posRawDeg;
        const double positionDegrees = encoder.value.posDeg;
        const double velocityDegreesPerSecond = encoder.value.velDegSec;
        (void)rawDegrees;
        (void)positionDegrees;
        (void)velocityDegreesPerSecond;
    } else {
        // Inspect encoder.errorMessage; the last valid measurement is retained
        // after a communication failure.
    }
}
```

`encoder.init()` applies the selected SPI mode and baud-rate prescaler to the supplied HAL handle, then calls `HAL_SPI_Init()`. Set the SPI instance and other required HAL fields before calling it. The SPI read timeout defaults to 5 ms and can be overridden by defining `EncoderSSI_SPI_TIMEOUT_MS` before including the library header.

## GPIO mode

Set `READ_MODE` to `EncoderSSI_COM_Mode_GPIO`, then provide the clock and data GPIO ports and pins. The library enables the GPIO port clocks and configures the clock pin as push-pull output and data pin as input during `init()`.

```cpp
encoder.parameters.READ_MODE = EncoderSSI_COM_Mode_GPIO;
encoder.parameters.TIMER = &encoderTimer;
encoder.parameters.CLK_GPIO_PORT = GPIOC;
encoder.parameters.CLK_GPIO_PIN = GPIO_PIN_10;
encoder.parameters.DATA_GPIO_PORT = GPIOC;
encoder.parameters.DATA_GPIO_PIN = GPIO_PIN_11;
encoder.parameters.GPIO_CLOCK_FRQ = 200000; // 0 selects the default slow timing.
encoder.parameters.SPI_MODE = 3;            // Select edge timing to match the encoder.
```

The GPIO implementation uses `SPI_MODE` to choose clock polarity and the data sampling edge. Confirm the required timing against the encoder datasheet. In either communication mode, initialize and start the `TimerControl` instance before calling `encoder.init()`.

## Configuration reference

Set parameters before `init()`. The constructor defaults to SPI mode, 17 single-turn bits, no multi-turn bits, binary data, a start bit, SPI mode 3, SPI prescaler /256, gear ratio 1, and disabled rate calculation and filters. The timer pointer and SPI handle (SPI mode) or GPIO ports (GPIO mode) must be supplied by the application.

| Parameter | Meaning |
| --- | --- |
| `READ_MODE` | `EncoderSSI_COM_Mode_SPI` or `EncoderSSI_COM_Mode_GPIO`; selected before initialization |
| `HSPI` | HAL SPI handle, required in SPI mode |
| `TIMER` | Initialized and running `TimerControl`; required in both modes |
| `CLK_GPIO_PORT`, `CLK_GPIO_PIN` | SSI clock output in GPIO mode |
| `DATA_GPIO_PORT`, `DATA_GPIO_PIN` | SSI data input in GPIO mode |
| `GPIO_CLOCK_FRQ` | Requested GPIO clock frequency in hertz; zero uses the default timing |
| `SPI_MODE` | Clock mode 0–3. Applied to HAL SPI in SPI mode and used for GPIO clock/sample timing |
| `SPI_BAUDRATE_PRESCALER` | HAL SPI divider, one of the STM32 HAL `/2` through `/256` constants |
| `RESOLUTION_SINGLE_TURN` | Number of position bits for one revolution; must be greater than zero |
| `RESOLUTION_MULTI_TURN` | Number of additional turn-count bits; use zero for a single-turn encoder |
| `START_BIT` | Whether the SSI frame includes a leading start bit |
| `DATA_FORAMT` | `EncoderSSI_DATA_FORMAT_BINARY` or `EncoderSSI_DATA_FORMAT_GRAY` (member spelling follows the API) |
| `IGNORE_MULTI_TURN` | Discard multi-turn bits after decoding, retaining only single-turn position |
| `POSRAW_OFFSET_DEG` | Raw angle offset in degrees, subtracted before mapping and gear ratio |
| `GEAR_RATIO` | Multiplier for output position and velocity; use a nonzero value, especially when setting a preset |
| `MAP_ENA`, `MAP_MIN`, `MAP_MAX` | Map angle into the configured degree range; requires `MAP_MAX > MAP_MIN` |
| `RATE_ENA` | Enable angular velocity calculation |
| `RATE_SPS` | Optional sample-rate gate for velocity updates; zero disables the gate |
| `UPDATE_FRQ` | Minimum interval between encoder reads in hertz; zero allows each `update()` call to read |
| `FLTR` | Rate low-pass cutoff frequency in hertz; zero disables rate filtering |
| `FLTA` | Position low-pass cutoff frequency in hertz; zero disables position filtering |
| `FLTM` | Median-filter window: 0 disables; valid nonzero values are 3, 5, 7, 9, 11, or 15 |
| `FLTS` | Step applied in degrees when the requested position jumps by more than 1°; zero disables this jump limiting |

The implementation validates total encoder resolution at no more than 31 bits. Follow the encoder's actual frame format and part datasheet; an incorrect bit count or start-bit setting shifts the decoded position.

## Values and methods

| Member | Meaning |
| --- | --- |
| `value.posRawStep` | Decoded position count after optional Gray conversion and multi-turn masking |
| `value.posRawDeg` | Decoded raw angle in degrees. With multi-turn bits included, this can exceed 360°; `IGNORE_MULTI_TURN` restricts it to the single-turn portion |
| `value.posDeg` | Processed output position in degrees after offset, optional mapping, gear ratio, and enabled filters |
| `value.velDegSec` | Calculated angular velocity in degrees per second when `RATE_ENA` is enabled |
| `errorMessage` | Null-terminated diagnostic buffer (`char[100]`) for the most recent reported error |

- `bool init()` validates parameters and configures SPI or GPIO. Call after the timer and required HAL peripherals are ready.
- `bool update()` reads and processes a sample. `false` indicates an initialization, parameter, or communication error. If `UPDATE_FRQ` throttles a read, the function returns `true` without taking a new sample.
- `bool setPresetValueDeg(double degrees)` reads the current raw position and adjusts `POSRAW_OFFSET_DEG` so the current output corresponds to the requested degrees. Initialize the encoder first, disable angle mapping, and set rotation direction and a nonzero gear ratio before using it.
- `void filterEnable(bool enabled)` enables or bypasses the position slew/median/low-pass processing. Rate filtering is configured separately by `FLTR`.
- `void clean()` clears stored values and filter history; it does not reconfigure hardware.

With position filtering enabled, the library applies jump limiting, then the median filter, then the position low-pass filter. `update()` uses `TimerControl::micros()` for rate and filter timing. Call it regularly from the application loop or a suitable task; keep the call rate high enough for the encoder motion and desired velocity response. Avoid calling blocking `update()` from a high-priority interrupt.

## Error handling

Check the return value from `init()` and `update()`. On failure, inspect `encoder.errorMessage`. SPI transfers use a finite timeout; failed reads return `false` and preserve the last valid position values. A successful call can mean the sample interval has not elapsed when `UPDATE_FRQ` is set, so values may be unchanged.

## Example project

The STM32F407 Keil project in [`examples/STM32F407VGT6/ex1`](examples/STM32F407VGT6/ex1) demonstrates SPI acquisition, a TIM2-backed `TimerControl`, and periodic calls to `update()`. The example is configured for a specific board and encoder wiring; review its clock frequency, SPI mode, resolution, and pin assignments before adapting it.
