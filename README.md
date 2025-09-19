# Description
astrostream plugin obtains frames or video stream from INDIlib server. This allows to use in OBS any cameras which have drivers in INDI

# Usage

- Put `astrostream.so` into OBS plugins directory.
- Run kstars and launch EKOS with your config
- Launch OBS, add source `INDI Camera` and select ip:port of indiserver. Port is 7624 by default, but can be 7625 or some other.
- Select CCD from list. If you also have guidescope, there can be 2 CCD
- Start capturing or video streaming in EKOS. Astrostream plugin provides no controls for camera, all controls in EKOS/Indi

# Building Requirements
- libindi-dev
- libobs-dev
- libjpeg-dev
- libcfitsio-dev
- cmake, g++, etc...
