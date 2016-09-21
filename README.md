# ccprog
Chipcon CC1110 GPIO-based (bitbang) programmer

Used to program cc1110 devices over GPIO lines on operating systems and boards that support libmraa.

## Building

`make`

## Usage

```
Usage: ./ccprog command

 Commands supported: erase reset write

 Command line options:
   -p DC,DD,RESET              specify mraa pins for debugging cc chip:
```
