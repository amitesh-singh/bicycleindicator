/*
 * Bi-cycle indicator based on Attiny13a
 * 
 * Copyright (C) 2017  Amitesh Singh <singh.amitesh@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <util/atomic.h>

// in ms
const static uint16_t BLINK_LEDS_TIMEOUT = 80;

#define BUZZER_PORT     PB0 //OC0A
#define LEFT_SWITCH     PB1
#define RIGHT_SWITCH    PB2
#define LEFT_LED        PB3
#define RIGHT_LED       PB4

// timer0 code
static void
init_timer0()
{
    // Compare Output Mode, non-PWM Mode
    cli();
    TCCR0B |= (1 << CS02) | (1 << CS00); // f_cpu/1024
    TCCR0A |= (1 << COM0A1); // Clear OC0A on Compare Match  
    TIMSK0 |= 1 << TOIE0;

    TCNT0 = 0;
    OCR0A = 250;
    sei();
}

static void
stop_timer()
{
    cli();
    TIMSK0 &= ~(1 << TOIE0);
    sei();
}

// 1 PB1 is pressed
// 2 PB2 is pressed 
// 0 none is pressed
static volatile int8_t key_pressed = 0;

static void
my_delay(uint16_t ms)
{
  uint16_t i = 0;
  for (; i < ms; ++i)
    _delay_ms(1);
}

/*
static void
blink_led(uint8_t pin)
{
  PORTB |= (1 << pin);
  my_delay(BLINK_LEDS_TIMEOUT);

  PORTB &= ~(1 << pin);
  my_delay(BLINK_LEDS_TIMEOUT);
}
*/

const static uint8_t DEBOUNCE_TIMEOUT = 8;

//software debounce
static uint8_t
debounce(uint8_t pin)
{
   if (bit_is_clear(PINB, pin))
     {
        _delay_ms(DEBOUNCE_TIMEOUT);
        if (bit_is_clear(PINB, pin))
          return 1;
     }
   return 0;
}

// PB0 - PCINT0
// PB1 - PCINT1
ISR(PCINT0_vect)
{
  if (debounce(LEFT_SWITCH))
     {
        if (!key_pressed) key_pressed = 1;
        else key_pressed = 0;
     }
  else if (debounce(RIGHT_SWITCH))
     {
        if (!key_pressed) key_pressed = 2;
        else key_pressed = 0;
     }
}

ISR(TIM0_OVF_vect)
{
  if (key_pressed == 1)
    PORTB ^= (1 << LEFT_LED);
  else if (key_pressed == 2)
    PORTB ^= (1 << RIGHT_LED);
}

static void
blink_leds()
{
   PORTB |= (1 << LEFT_LED) | (1 << RIGHT_LED);
   my_delay(BLINK_LEDS_TIMEOUT);

   PORTB &= ~((1 << LEFT_LED) | (1 << RIGHT_LED));
   my_delay(BLINK_LEDS_TIMEOUT);
}

//blink both LEFT and right key for 4 times
static void
start_show()
{
   uint8_t i = 0;
   for (; i < 4; ++i)
     blink_leds();
}

int main(void)
{
   volatile uint8_t timer_started = 0;
   // save power by disabling all the peripherals properties (ADC etcs)
   // since we don't need it.
   power_all_disable();
   power_timer0_enable();
   clock_prescale_set(clock_div_4);
   cli();
                 //left        right  
   DDRB |= (1 << LEFT_LED) | (1 << RIGHT_LED);

   // enable PB0 and PB1 internal pullups
   DDRB &= ~(1 << LEFT_SWITCH); // Left key
   DDRB &= ~(1 << RIGHT_SWITCH); // Right key
   PORTB |= (1 << LEFT_SWITCH) | (1 << RIGHT_SWITCH);

   GIMSK |= (1 << PCIE); // enable PCINT 
   PCMSK |= (1 << PCINT1) | (1 << PCINT2);

   sei(); //set global enable interrupts!

   start_show();

   while (1)
     {
        if (key_pressed != 0)
          {
            //if (key_pressed == 1) blink_led(LEFT_LED);
            //else if (key_pressed == 2) blink_led(RIGHT_LED);
            if (!timer_started)
            {
              init_timer0();
              ATOMIC_BLOCK(ATOMIC_FORCEON)
                {
                   timer_started = 1;
                }
            }
          }
        else
          {
             //stop the timer0 
             stop_timer();
             ATOMIC_BLOCK(ATOMIC_FORCEON)
               {
                  timer_started = 0;
               }
             //sleep
             PORTB &= ~((1 << LEFT_LED) | (1 << RIGHT_LED));
             cli();
             set_sleep_mode(SLEEP_MODE_PWR_DOWN);
             sleep_enable();
             sei();
             sleep_mode();  // <<-- sleep here

             sleep_disable(); // <<-- wake up 
          }
     }
   return 0;
}
