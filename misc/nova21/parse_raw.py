#!/usr/bin/env python

for line in open("joey.txt", "r").readlines():
    params = line.split(",")
    if len(params) == 9:
        print params[1] + "," + params[2] + "," + params[3] + \
                "," + params[4] + "," + params[5]

