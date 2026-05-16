## Setup on Raspberry Pi

Install dependencies:
```
sudo apt-get install --no-install-recommends git make g++ cmake libsoapysdr-dev libasound2-dev soapysdr-tools python3-soapysdr
```

Some prototype boards do not have the HAT identification EEPROM written.
If you have one of those, write it first by following
[EEPROM writing instructions](dts/README.md).

Compile and install SoapySDR module:
```
cd SoapySX
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
```

Check that the module is found:
```
SoapySDRUtil --probe=driver=sx
```

Read SX1255 temperature and raw Q_OUT-derived ADC values:
```
SoapySDRUtil --probe=driver=sx
SoapySDRUtil --probe=driver=sx | grep -i temperature
```

The SX1255 temperature sensor uses the RX ADC path. SoapySX exposes it as
SoapySDR sensors (`temperature`, `temperature_raw`, `temperature_qout`), but
refuses the measurement while RX/TX streams are active so applications do not
get a hidden receive/transmit interruption.

The default conversion uses the datasheet slope of about -1 LSB/degC and the
typical ambient reference (`TEMP_REF_C=25`, `TEMP_REF_RAW=150`). The SXceiver
I2S path reports Q_OUT as a signed 32-bit value, so the default raw conversion is
`-temperature_qout / 2^23`. For accurate absolute values, calibrate the board at
a known temperature and set:
```
TEMP_REF_C=<known temperature C>
TEMP_REF_RAW=<temperature_raw at that point>
```

## Features
SoapySX provides some support for timestamps which are used by some
applications to obtain a known timing relationship between transmitted and
received signals.
See the
[linear repeater example](example/linear_repeater.py)
for an example on using timestamps to obtain a constant, known latency
from received to transmitted signal.
