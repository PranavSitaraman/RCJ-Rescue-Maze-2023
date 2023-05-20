#!/bin/bash
cd pi
cd build
cmake ..
make
cd ../
cd ../
cd arduino
pio run --target upload
cd ../
cd master
./master