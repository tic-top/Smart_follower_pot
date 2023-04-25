# Smart_flower_pot
#### Author: Yilin Jia, Yiting Lai, Danli Shi, Sikai Li

#### Date: 24/4/2023

### Features

In the test_screen\Core\Src,

- soil.c: temperature reading, humidity reading, water pumping
- ILI9341_GTX.c ILI9341_STM32_Driver.c: Image, line, text, circle plotting
- my_touch.c: reads the position the user touch, temperature reading

In ld3320\Core\Src,

- main.c: Configure timer for ultrasonic sensors, speed of movable base, call the voice recognition function
- motor.c: Configure functions for movable base: move_forward, move_backward, move_left, move_right, move_stop
- LD3320.c: Implement voice recognition functions, call the corresponding move functions to the instructions it recognized.
