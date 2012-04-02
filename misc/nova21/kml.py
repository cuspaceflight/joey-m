#!/usr/bin/env python
# Jon Sowman 2012
import simplekml

kml = simplekml.Kml()
points = []

for line in open("telemetry.csv", "r").readlines():
    p = line.split(",")
    if len(p) == 5:
        points.append((p[3], p[2], p[4]))

line = kml.newlinestring(name="Nova 21", description="Nova 21 Flight Path",
        coords=points, altitudemode="absolute")
line.linestyle.color = 'ff0000ff'
line.linestyle.width = 5
kml.save("nova21.kml")

