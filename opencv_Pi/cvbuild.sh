#!/bin/bash
g++ -Wall $1 -o output `pkg-config --cflags --libs opencv` -lwiringPi 
