#!/bin/bash
g++ $1 -o output `pkg-config --cflags --libs opencv`
