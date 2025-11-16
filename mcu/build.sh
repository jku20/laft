#!/bin/bash
cmake -G Ninja -B build . &&
  cd build &&
  bear -- ninja
