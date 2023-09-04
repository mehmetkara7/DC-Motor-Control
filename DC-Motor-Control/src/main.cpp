#include <avr/io.h>
#include <util/delay.h>

#define MOTOR_IN1 PD6
#define MOTOR_IN2 PD7
#define PWM_PIN PD5
#define RELAY_PIN PD4
#define BUZZER_PIN PB1

void setup()
{
  DDRD |= (1 << MOTOR_IN1) | (1 << MOTOR_IN2) | (1 << PWM_PIN) | (1 << RELAY_PIN);
  PORTD &= ~((1 << MOTOR_IN1) | (1 << MOTOR_IN2) | (1 << RELAY_PIN));

  TCCR1A = (1 << COM1A1) | (1 << WGM11) | (1 << WGM10);
  TCCR1B = (1 << WGM12) | (1 << CS11);

  DDRB |= (1 << BUZZER_PIN);
  PORTB &= ~(1 << BUZZER_PIN);
}

void setMotorSpeed(uint8_t speed)
{
  OCR1A = speed;
}

void forward()
{
  PORTD |= (1 << MOTOR_IN1);
  PORTD &= ~(1 << MOTOR_IN2);
}

void backward()
{
  PORTD &= ~(1 << MOTOR_IN1);
  PORTD |= (1 << MOTOR_IN2);
}

void stop()
{
  PORTD &= ~((1 << MOTOR_IN1) | (1 << MOTOR_IN2));
}

void turnOnRelay()
{
  PORTD |= (1 << RELAY_PIN);
}

void turnOffRelay()
{
  PORTD &= ~(1 << RELAY_PIN);
}

void turnOnBuzzer()
{
  PORTB |= (1 << BUZZER_PIN);
}

void turnOffBuzzer()
{
  PORTB &= ~(1 << BUZZER_PIN);
}

uint16_t readACS712()
{
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC))
    ;
  return ADC;
}

int main(void)
{
  setup();
  turnOnRelay();

  while (1)
  {
    uint16_t adcValue = readACS712();
    if (adcValue > 512)
    {
      stop();
      turnOffRelay();
      turnOnBuzzer();
    }
    else
    {
      turnOnRelay();
      forward();
      setMotorSpeed(100);
      _delay_ms(2000);
      setMotorSpeed(150);
      _delay_ms(1000);
      setMotorSpeed(200);
      _delay_ms(2000);

      backward();
      setMotorSpeed(100);
      _delay_ms(2000);
    }
  }

  return 0;
}
