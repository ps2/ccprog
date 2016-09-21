# ccprog
Chipcon CC1110 GPIO-based (bitbang) programmer

Used to program cc1110 devices over GPIO lines on operating systems and boards that support libmraa, like the INTEL Edison board.

## Building

`make`

## Usage

```
Usage: ./ccprog command

 Commands supported: erase reset write

 Command line options:
   -p DC,DD,RESET              specify mraa pins for debugging cc chip:
```

# Wiring

The edison natively uses 1.8V logic levels.  The CC1110 requires 3.3V logic levels, so you will need something like the [Sparkfun GPIO Block](https://www.sparkfun.com/products/13038) to shift the levels up. 
