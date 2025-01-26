# Meson

Create a build directory with the cross compilation target
```
meson setup --cross-file cross/arm-none-eabi.cross target
```

Compile
```
meson compile -C target
```

Clean
```
meson compile --clean -C target
```

# Orbtrace / Orbuculum

Enable the 3v3 power output
```
orbuculum --monitor 1000 --orbtrace '-p vtref,3.3 -e vtref,on'
```

Disable the 3v3 power output
```
orbuculum --monitor 1000 --orbtrace '-p vtref,3.3 -e vtref,off'
```

# Upload binary / debug
Use the vscode configuration (F5 for debugging).


# Hardware
Place the board so the leds and reset button are in the upper right corner.

## Orbtrace
Connect the orbtrace using SWD 10 pin connector.
The ribbon cable should point to the right when connecting.

## FTDI-232
The FTDI-232 UART to USB converter needs to be set for 3v3. 
Plug it in the first header from the top of the board.
Make sure the chip/led side of the FTDI board is facing towards you.

## DHT22
The DHT22 is connected to PC0 PC1 and PC2. 
This corresponds to the fourth header from the top.
Pin 0 is the left-most pin.
Make sure the plastic case is facing towards you.

## 1.14 inch LCD ST7789 135x240 pixels
The LCD is connected to PB6-PB13.
This corresponds to the fifth header from the top.
Pin 6 is the 7th pin from the left.
Make sure the display is under the pin header.
