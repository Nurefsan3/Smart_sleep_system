#include "stm32f10x.h"

// Pin Definitions
#define LED_PORT    GPIOD
#define LED_PIN     12        // Status LED (Output)

#define FAN_PORT    GPIOA
#define FAN_PIN     0         // DC Fan (Output)

#define HEATER_PORT GPIOA
#define HEATER_PIN  13        // Heater (Output)

#define DHT_PORT    GPIOA
#define DHT_PIN     12        // DHT22 data pin (GPIO, input/output)

#define OLED_PORT   GPIOB
#define OLED_SCL    6         // PB6 (SCL) I2C (OLED Display)
#define OLED_SDA    7         // PB7 (SDA) I2C (OLED Display)

#define UART_TX_PORT GPIOD
#define UART_TX_PIN  8   // UART A TX line is routed through MCU pin PD8.

#define UART_RX_PORT GPIOD
#define UART_RX_PIN  9   // UART A RX line is routed through MCU pin PD9.



// functions that set/clear/read a pin using direct registers.

static inline void gpio_set(GPIO_TypeDef *port, uint8_t pin)
{
    // Set the pin to 1 (HIGH) by writing 1 to the corresponding bit of the BSRR register.
    port->BSRR = (1U << pin);
}

static inline void gpio_clear(GPIO_TypeDef *port, uint8_t pin)
{
    // Set the pin to 0 (LOW) by writing 1 to the corresponding bit of the BRR register.
    port->BRR = (1U << pin);
}

static inline uint8_t gpio_read(GPIO_TypeDef *port, uint8_t pin)
{
    // The IDR register is masked and the pin level is read.
    return ( (port->IDR & (1U << pin)) ? 1U : 0U );
}

// Since the DHT22 protocol requires microsecond precision, the delay function
// is written using the SysTick timer.

static void delay_us(uint32_t us)
{
    // SystemCoreClock is the current CPU frequency.
    // The required number of clock cycles for a 1 µs period is computed
    // and written into SysTick->LOAD.
    uint32_t ticks = SystemCoreClock / 1000000U - 1U;

    SysTick->LOAD = ticks;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    for (uint32_t i = 0; i < us; i++)
    {
        // COUNTFLAG becomes 1 when the timer completes a single reload period (1 µs).
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
        {
            // wait
        }
    }
    SysTick->CTRL = 0;  // Disable SysTick
}

// A simple wrapper for millisecond delays (implemented as 1000 repeated microsecond delays).
static void delay_ms(uint32_t ms)
{
    while (ms--)
    {
        delay_us(1000);
    }
}

// Just letting the compiler know these functions exist before we define them.

static void gpio_init(void);

static void dht_set_output(void);
static void dht_set_input_pullup(void);
static uint8_t dht_read_bit(void);
static uint8_t dht_read_byte(void);
static uint8_t dht_read(uint16_t *temp_x10, uint16_t *hum_x10);

static void control_actuators(uint16_t temp_x10, uint16_t hum_x10);
static void oled_show_status(uint16_t temp_x10, uint16_t hum_x10);
static void pc_send_status(uint16_t temp_x10, uint16_t hum_x10);

// main function
int main(void)
{
    uint16_t temp_x10 = 0;   // Temperature in °C ×10 format (e.g., 253 -> 25.3 °C)
    uint16_t hum_x10  = 0;   // Humidity in %RH ×10 format (e.g., 625 -> 62.5 %RH)
    uint8_t  ok;

    gpio_init();             // Set up all GPIO directions and clocks before the loop.

    while (1)
    {
        // Read data from the DHT22 sensor.
        // ok = 1 means the reading is valid.
        // ok = 0 means something went wrong (checksum error or timeout).
        ok = dht_read(&temp_x10, &hum_x10);

        if (!ok)
        {
            // If reading fails, the LED is blinked quickly.
            // This is used as a simple visual indication that something went wrong
            // (a kind of “sensor error” indicator).
            gpio_set(LED_PORT, LED_PIN);
            delay_ms(100);
            gpio_clear(LED_PORT, LED_PIN);
            delay_ms(100);
            continue;   // Skip the rest of the loop and try again.
        }

        // If the sensor reading is valid, the temperature value is used
        // to control the Heater, Fan, and the status LED.
        // (All the decision logic is inside control_actuators().)
        control_actuators(temp_x10, hum_x10);

        // These two functions are placeholders for now.
        // The GPIO pins for the OLED (I2C) and UART connections are prepared,
        // but the actual communication protocols have not been implemented yet.
        // For the moment, these functions are just empty stubs.
        oled_show_status(temp_x10, hum_x10);
        pc_send_status(temp_x10, hum_x10);

        // According to the DHT22 datasheet, the recommended reading interval
        // is around 2 seconds, so a delay is added here before the next measurement.
        delay_ms(2000);
    }
}

// GPIO INITIALIZATION – REGISTER LEVEL 
// In this section:
//   - The required port clocks are enabled through RCC->APB2ENR.
//   - Pin modes are configured using the CRL and CRH registers.
//
// For STM32F1:
//   - CRL holds the configuration bits for pins 0–7 (4 bits per pin).
//   - CRH holds the configuration bits for pins 8–15 (4 bits per pin).
//   - Each pin is defined by a 4-bit field: [CNF1 CNF0 MODE1 MODE0]
//       MODE[1:0] = speed / input-output mode
//       CNF[1:0]  = output type (e.g., push-pull) or input configuration

static void gpio_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   // GPIOA clock enable
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;   // GPIOB clock enable
    RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;   // GPIOD clock enable
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;   // AFIO (alternate function) clock enable

    // PA3 – DC Fan (Output Push-Pull, 2 MHz)
    // PA3 is located in GPIOA->CRL (pins 0–7).
    // Each pin occupies a 4-bit configuration field.
    // For pin 3, this field is located at bits 12–15.
    // The field is first cleared and then updated with the required mode.
    // MODE = 0b10 -> Output, 2 MHz
    // CNF  = 0b00 -> General-purpose push-pull
    // Combined 4-bit value = 0b0010 = 0x2
    GPIOA->CRL &= ~(0xF << (FAN_PIN * 4));   // The 4-bit configuration field is cleared.
    GPIOA->CRL |=  (0x2 << (FAN_PIN * 4));   // Output, 2 MHz, push-pull mode is applied.

    // PA13 – Heater (Output Push-Pull, 2 MHz)
    // PA13 is located in GPIOA->CRH (pins 8–15).
    // Its index inside CRH is calculated as pin - 8.
    // Example: pin 13 -> index = 13 - 8 = 5.
    // The corresponding 4-bit configuration field is located at bits 20–23.
    uint8_t idx = HEATER_PIN - 8;
    GPIOA->CRH &= ~(0xF << (idx * 4));       // The 4-bit configuration field is cleared.
    GPIOA->CRH |=  (0x2 << (idx * 4));       // Output, 2 MHz, push-pull mode is applied.

    // PD12 – Status LED (Output Push-Pull, 2 MHz)
    // The same procedure is applied here, since PD12 is also located in CRH.
    idx = LED_PIN - 8;
    GPIOD->CRH &= ~(0xF << (idx * 4));
    GPIOD->CRH |=  (0x2 << (idx * 4));       // Output, 2 MHz, push-pull mode is applied.

    // PA12 – DHT22 data pin (initially Input Pull-Up)
    // PA12 is located in CRH.
    // Since the DHT22 protocol requires switching between input and output,
    // the pin is initially configured as input with an internal pull-up.
    dht_set_input_pullup();

    // PB6 / PB7 – OLED I2C pins (SCL/SDA, AF Open-Drain)
    // Only the GPIO configuration is applied at this stage.
    // The actual I2C communication protocol has not been implemented yet.
    // MODE = 0b10 -> 2 MHz output
    // CNF  = 0b11 -> Alternate-function open-drain (suitable for I2C)
    // Combined value = 0b1011 = 0xB
    OLED_PORT->CRL &= ~(0xF << (OLED_SCL * 4));
    OLED_PORT->CRL |=  (0xB << (OLED_SCL * 4));   // PB6 is configured as SCL.

    OLED_PORT->CRL &= ~(0xF << (OLED_SDA * 4));
    OLED_PORT->CRL |=  (0xB << (OLED_SDA * 4));   // PB7 is configured as SDA.

    // UART pin preparation (optional)
    // Basic GPIO configuration for UART pins is provided.
    // However, the full UART/USART communication setup
    // (such as baud rate and frame format)
    // has not been implemented yet.

    // PD8 (TX) – Alternate Function Push-Pull, 50 MHz
idx = UART_TX_PIN - 8;              
GPIOD->CRH &= ~(0xF << (idx * 4));
GPIOD->CRH |=  (0xB << (idx * 4));  // PD8 is configured as AF push-pull (50 MHz).

// PD9 (RX) – Input Floating
idx = UART_RX_PIN - 8;
GPIOD->CRH &= ~(0xF << (idx * 4));
GPIOD->CRH |=  (0x4 << (idx * 4));  // PD9 is configured as floating input.


    // Initial output states:
    // The Fan, Heater, and LED are turned off at startup.
    gpio_clear(FAN_PORT, FAN_PIN);
    gpio_clear(HEATER_PORT, HEATER_PIN);
    gpio_clear(LED_PORT, LED_PIN);
}

// -DHT22 GPIO mode change functions-
// The DHT22 data pin (PA12) is used as an MCU output when the start signal
// is sent, and as an input when the sensor data is read.
// Because of this, the pin mode is changed dynamically while the program runs.

static void dht_set_output(void)
{
    // PA12 is configured as a 2 MHz push-pull output.
    uint8_t idx = DHT_PIN - 8;
    GPIOA->CRH &= ~(0xF << (idx * 4));
    GPIOA->CRH |=  (0x2 << (idx * 4));   // MODE=10, CNF=00 -> output 2 MHz, push-pull
}

static void dht_set_input_pullup(void)
{
    // PA12 is configured as an input with an internal pull-up resistor.
    uint8_t idx = DHT_PIN - 8;
    GPIOA->CRH &= ~(0xF << (idx * 4));
    GPIOA->CRH |=  (0x8 << (idx * 4));   // 0x8 = 1000 -> MODE=00 (input), CNF=10 (pull-up/pull-down)

    // In CNF=10 mode, the ODR bit defines the pull direction:
    // ODR = 1 selects pull-up, ODR = 0 selects pull-down.
    GPIOA->ODR |= (1U << DHT_PIN);       // Pull-up is selected here.
}

// - DHT22 bit-level reading -
// For each data bit, the sensor generates the following timing:
//   - About 50 µs LOW,
//   - Then about 26–28 µs HIGH for a logic 0,
//     or about 70 µs HIGH for a logic 1.
//
// In this implementation, the following strategy is used:
//   - The LOW period and the rising edge to HIGH are waited for,
//   - After the line goes HIGH, a delay of about 35 µs is applied,
//   - The line is sampled after this delay:
//       * If the line is LOW again at 35 µs, the bit is interpreted as 0,
//       * If the line is still HIGH at 35 µs, the bit is interpreted as 1.

static uint8_t dht_read_bit(void)
{
    uint32_t timeout;

    //  The line is waited to go LOW (sensor’s ~50 µs LOW period).
    timeout = 1000;
    while (gpio_read(DHT_PORT, DHT_PIN))
    {
        if (!timeout--) return 0;    // A timeout is used in case the sensor does not respond.
    }

    //  The line is then waited to go HIGH (start of the HIGH period for this bit).
    timeout = 1000;
    while (!gpio_read(DHT_PORT, DHT_PIN))
    {
        if (!timeout--) return 0;
    }

    //  After the line becomes HIGH, a delay of ~35 µs is applied
    //    and then the line level is sampled.
    delay_us(35);

    if (gpio_read(DHT_PORT, DHT_PIN))
        return 1;
    else
        return 0;
}

// One byte (8 bits) is read here, with the most significant bit (MSB) first.
static uint8_t dht_read_byte(void)
{
    uint8_t i, value = 0;
    for (i = 0; i < 8; i++)
    {
        value <<= 1;             // Existing bits are shifted to the left (MSB-first).
        value |= dht_read_bit(); // The new bit is placed into the least significant position.
    }
    return value;
}

// - DHT22 main reading function -
// Output format:
//   - temp_x10: temperature in °C ×10, e.g. 253 represents 25.3 °C
//   - hum_x10 : humidity in %RH ×10, e.g. 625 represents 62.5 %RH
//
// Return value:
//   - 1 -> data is read successfully from the sensor,
//   - 0 -> an error is detected (timeout or checksum mismatch).

static uint8_t dht_read(uint16_t *temp_x10, uint16_t *hum_x10)
{
    uint8_t buf[5];
    uint8_t i;
    uint32_t timeout;

    // 1) MCU start signal:
    //    The data line is held LOW for at least 18 ms (20 ms is used here),
    //    then it is driven HIGH for a short period (20–40 µs),
    //    and finally the pin is switched to input mode so the sensor can drive the line.
    dht_set_output();
    gpio_clear(DHT_PORT, DHT_PIN);
    delay_ms(20);                // 20 ms LOW ->start signal.
    gpio_set(DHT_PORT, DHT_PIN);
    delay_us(30);                // 20–40 µs HIGH.
    dht_set_input_pullup();      // After this point, the sensor response is expected.

    // 2) Sensor response (~80 µs LOW + ~80 µs HIGH).
    //    In this part, only the level changes are checked
    //    to confirm that the sensor is ready.

    // The line is first waited to go LOW.
    timeout = 10000;
    while (gpio_read(DHT_PORT, DHT_PIN))
    {
        if (!timeout--) return 0;
    }

    // Then the line is waited to go HIGH.
    timeout = 10000;
    while (!gpio_read(DHT_PORT, DHT_PIN))
    {
        if (!timeout--) return 0;
    }

    // Finally, the end of this HIGH period is waited (the line goes LOW again).
    timeout = 10000;
    while (gpio_read(DHT_PORT, DHT_PIN))
    {
        if (!timeout--) return 0;
    }

    // 3) 40 bits (5 bytes) of data are read from the sensor.
    //    DHT22 data format:
    //      byte0: humidity high byte
    //      byte1: humidity low byte
    //      byte2: temperature high byte
    //      byte3: temperature low byte
    //      byte4: checksum = (byte0 + byte1 + byte2 + byte3) & 0xFF
    for (i = 0; i < 5; i++)
    {
        buf[i] = dht_read_byte();
    }

    // 4) Checksum verification is performed.
    uint8_t sum = buf[0] + buf[1] + buf[2] + buf[3];
    if ((sum & 0xFF) != buf[4])
    {
        return 0;   // If the checksum does not match, the data is treated as invalid.
    }

    // 5) Humidity and temperature values are calculated.
    //    DHT22 sends values in ×10 format:
    //      Humidity    = (buf0<<8 | buf1) / 10.0
    //      Temperature = (buf2<<8 | buf3) / 10.0
    //    In this code, the values are kept in ×10 format without division.
    uint16_t raw_hum  = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t raw_temp = ((uint16_t)buf[2] << 8) | buf[3];

    // Negative temperature is encoded by using the MSB as a sign bit.
    // For this project, mainly positive temperatures are expected,
    // so the sign is not used further in the logic.
    if (raw_temp & 0x8000)
    {
        raw_temp &= 0x7FFF;   // The sign bit is cleared.
        *temp_x10 = raw_temp;
    }
    else
    {
        *temp_x10 = raw_temp;
    }

    *hum_x10 = raw_hum;

    return 1;  // If this point is reached, the measurement is considered successful.
}

// A simple decision rule is applied here:
//   - temp_x10 < 240  (24.0 °C)   -> Heater is turned ON, Fan is turned OFF, LED is turned ON
//   - temp_x10 > 280  (28.0 °C)   -> Fan is turned ON, Heater is turned OFF, LED is turned ON
//   - 24.0–28.0 °C range          -> Both Heater and Fan are turned OFF, LED is turned OFF
//                                    (this interval is considered as the comfort zone).

static void control_actuators(uint16_t temp_x10, uint16_t hum_x10)
{
    (void)hum_x10;   // The humidity value is not used directly at this stage.

    if (temp_x10 < 240)            // Below 24.0 °C.
    {
        gpio_set(HEATER_PORT, HEATER_PIN);
        gpio_clear(FAN_PORT, FAN_PIN);
        gpio_set(LED_PORT, LED_PIN);
    }
    else if (temp_x10 > 280)       // Above 28.0 °C.
    {
        gpio_clear(HEATER_PORT, HEATER_PIN);
        gpio_set(FAN_PORT, FAN_PIN);
        gpio_set(LED_PORT, LED_PIN);
    }
    else                           // Between 24.0 °C and 28.0 °C.
    {
        gpio_clear(HEATER_PORT, HEATER_PIN);
        gpio_clear(FAN_PORT, FAN_PIN);
        gpio_clear(LED_PORT, LED_PIN);
    }
}

// - OLED and PC (UART) skeleton functions -
// These functions correspond to the "OLED" and "PC" blocks in the project diagram.
// At this stage, they are kept as high-level placeholders and the actual I2C / UART
// protocol code is not implemented. This is mainly done to show the planned
// digital GPIO connections and overall system structure.

static void oled_show_status(uint16_t temp_x10, uint16_t hum_x10)
{
    (void)temp_x10;
    (void)hum_x10;

    // Possible future steps could be:
    //   1) Enabling the I2C peripheral clock (for example, via RCC->APB1ENR for I2C1),
    //   2) Configuring I2C timing and address parameters,
    //   3) Sending the temperature and humidity values to the OLED display.
    //
    // For now, only the pin configuration is prepared, and this function
    // is intentionally left empty. During the presentation, it can be stated that:
    //    The I2C pins (PB6/PB7) are configured as alternate-function open-drain,
    //    and this function is reserved for displaying temperature and humidity
    //    on the OLED screen in a later step.
}

static void pc_send_status(uint16_t temp_x10, uint16_t hum_x10)
{
    (void)temp_x10;
    (void)hum_x10;

    // For now, only the GPIO configuration is prepared and this function
    // is left as an empty stub. On the board, the "UART A" USB connector
    // is expected to act as a USB-UART bridge and to appear as a virtual
    // COM port on the PC side.
}
