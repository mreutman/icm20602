# ICM20602

## Introduction

This is basic embedded driver for InvenSense's ICM20602 6-axis motion tracking
chip. I wrote this after not being able to find any decent code out their that
clearly showed how to configure and use the device.

The driver is far from feature complete currently. Hopefully I can get around
to adding in more support as I go along using it for my own projects, but at
the very least I hope it can help someone with their own project. Feel free to
investigate the code and/or hack away at it freely.

## Guide

The driver code should be device agnostic and compatible with any decent C
compiler out in the wild. All of the actual low level communication
(I2C or SPI) is left up to the developer to implement for their target.

## Notice

Code is very likely to change, it is probably wise to treat the driver as being
in it's Alpha stages. Not sure it's worth doing any sort of semantic versioning
on this currently, but we'll see how things go.
