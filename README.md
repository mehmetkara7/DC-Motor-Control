# DC-Motor-Control
# Motor Control with ACS712 Current Sensor and Atmega324PB Microcontroller

This project demonstrates motor control using an ACS712 current sensor with an Atmega324PB microcontroller. The microcontroller controls the motor speed and direction based on the current measured by the ACS712 sensor.

## Components

- Atmega324PB microcontroller
- ACS712 current sensor
- Motor driver circuit
- Buzzer
- Relay
- Other necessary components (e.g., resistors, capacitors, power supply)

## Getting Started

1. Clone or download this repository to your local machine.
2. Connect the components as shown in the circuit diagram.
3. Compile and upload the code to your Atmega324PB microcontroller using an AVR programmer.
4. Power up the circuit and observe the motor control based on current readings.

## Code Explanation

- `setup()`: Initializes the microcontroller pins and sets up the Timer/Counter1 for PWM control.
- `setMotorSpeed(uint8_t speed)`: Sets the motor speed using PWM (Pulse Width Modulation).
- `forward()`, `backward()`, `stop()`: Functions to control the motor direction.
- `turnOnRelay()`, `turnOffRelay()`: Functions to control the relay.
- `turnOnBuzzer()`, `turnOffBuzzer()`: Functions to control the buzzer.
- `readACS712()`: Reads the current value from the ACS712 sensor.
- `main()`: The main control loop that reads current values and adjusts motor control accordingly.

## Usage

Modify the code as needed to adjust the current threshold for motor control and the motor speed settings.

```c
if (adcValue > 512) {
    // Current threshold exceeded, take action (e.g., stop motor and sound buzzer)
} else {
    // Current within the acceptable range, adjust motor speed and direction as needed
    // Example: forward(), setMotorSpeed(speed), _delay_ms(delay);
}
