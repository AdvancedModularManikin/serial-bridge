# AMM - Serial Bridge

#### Requirements
The Serial Bridge requires the [AMM Standard Library](https://github.com/AdvancedModularManikin/amm-library) be built and available (and so requires FastRTPS and FastCDR).  In addition to the AMM library dependancies, the Serial Bridge also requires:
- tinyxml2 (`apt-get install libtinyxml2-dev`)
- boost::system

The serial bridge is Linux-only and written as an example for connecting to an Arduino device via serial.
 

### Installation
```bash
    $ git clone https://github.com/AdvancedModularManikin/serial-bridge
    $ mkdir serial-bridge/build && cd serial-bridge/build
    $ cmake ..
    $ cmake --build . --target install
```

By default on a Linux system this will install into `/usr/local/bin`

