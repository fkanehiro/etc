import math
import sys

argv = sys.argv
argc = len(argv)

r1 = 0.1
r2 = 1.0
ndiv1 = 8
ndiv2 = 8

if argc >= 2:
    r1 = float(argv[1])
if argc >= 3:
    r2 = float(argv[2])
if argc >= 4:
    ndiv1 = int(argv[3])
if argc >= 5:
    ndiv2 = int(argv[4])

print "#VRML V2.0 utf8"
print
print "DEF torus Shape{"
print "  geometry Extrusion{"
print "    crossSection["
for i in range(ndiv1):
    th = -2*math.pi/ndiv1*i
    print r1*math.cos(th),r1*math.sin(th),","
print r1,0
print "    ]"
print "    spine["
for i in range(ndiv2):
    th = -2*math.pi/ndiv2*i
    print r2*math.cos(th),r2*math.sin(th),"0,"
print r2,0,0
print "    ]"
print "  }"
print "  appearance Appearance{"
print "    material Material{} "
print "  }"
print "}"

