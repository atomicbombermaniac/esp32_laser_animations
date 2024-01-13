# esp32_laser_animations
ESP32 laser XY output using internal DAC to draw various animations, including realtime sound oscillogram.

This all started as a way to replace the need for an overkill rPI to use Marcan's (openLase) software to draw a simple Xmas tree ILDA file.

From there is turned into a one-button-press on ESP32, cycle through a few animations, including the Xmas tree, a realtime sound oscillogram, a realtime sound-controlled spirograph and a some other attempts.

The output is fed into some (LM358, I think) opamps that condition the signal for use with cheap 20Kpps laser scanners (subtracting the DC offset and doing some extra scaling). There is also a GPIO that turns the beam on/off since there was no 3rd DAC channel available.

The sound source can be line level from a headphone jack but the code is optimized for the Adafruit MAX9814 auto-gain microphone module, set at middle gain (I think the gain setting was to connect the gain pin to GND iirc). Went wireless for this one so that I can place the laser box anywhere for an event and since the signal level is VERY dependent on microphone-to-speaker distance and current loudness this mode is a blessing for such applications. Of course the beat detection and low-pass fitering code is taken from the Beat-detector repo.

Code is lowest quality. Use at own risk. This was created just for fun.

Below you can find the original SVG file used to convert into a points array and draw using the laser and also the old schematic on which most of the current laser scanner clones are based on (minus the "coil temperature calculator"). I put it here because it is hard to find.

![alt text](https://github.com/atomicbombermaniac/esp32_laser_animations/blob/main/free-svg-file-christmas-tree-400x467.svg?raw=true)
![alt text](https://github.com/atomicbombermaniac/esp32_laser_animations/blob/main/Laser_amp_schematic_ish.gif?raw=true)
