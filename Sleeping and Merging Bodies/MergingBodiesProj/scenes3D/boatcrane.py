import xml.etree.ElementTree as ET
from box import Box
root = ET.Element('root')

# Fixed body
Box(root, name='dock', position="-10.0 0.0 0.0", dim='15 2 15', pinned="true")

# Boat
boat = Box(root, name='boat', position="17 0.0 0.0", dim='30 2 15')
boat.addSpring("15 0 7.5")
boat.addSpring("15 0 -7.5")
boat.addSpring("-15 0 -7.5")
boat.addSpring("-15 0 7.5")

# Magnet
magnet = Box(root, name='magnet', position="-10.0 15.0 0.0", dim='8 2 8', magnetic="true")
magnet.addSpring("1 0 1")
magnet.addSpring("1 0 -1")
magnet.addSpring("-1 0 -1")
magnet.addSpring("-1 0 1")

# Bodies pile
y = 0
x = 0
for i in range(15):
    y += 2.5
    if (i%5==0) :
        x+=8.5
        y=2.5
    Box(root, name='box'+str(i), position=str(x)+" "+str(y)+" 0.0", dim='8 2 8')

filename = "boatcrane.xml"
print("Generated file: "+filename)
file = open(filename, "w")
file.write(ET.tostring(root))
file.close
