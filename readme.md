# Arduboy test programs

Various emulator tests. See /hex and /ino.

- ssd1306_128x64_spi is the standard Adafruit test program ported to the Arduboy.
  It tests normal drawing, scrolling, and pixel inversion. Source is already widely available.

- ssd1306_probe: AI-generated test program (Claude Opus 5.5) aimed at fixing the MAME SSD1306
  driver behaviors.

Things you should test against:
- Older games using variants on the delay() Arduino API. These all require
  accurate timer 0 and interrupt handling. Incorrect behavior causes delays
  to return immediately.

Specific Arduboy games worth testing:

- ABsynth FM: PWM driver on timer 4; game will freeze without it.
- Evade: Plays music on timer 3.
- Evade 2: Plays music on timer 4.
