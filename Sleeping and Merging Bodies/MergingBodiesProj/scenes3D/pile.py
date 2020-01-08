import xml.etree.ElementTree as ET
from box import Box

root = ET.Element('root')

# Plane
plane = ET.SubElement(root, 'body')
plane.set('type','plane')
plane.set('p','0 -1 0')
plane.set('n','0 1 0')
plane.set('name','plane')

# Bodies pile
y = 0
for i in range(15):
    y += 2.5
    Box(root, name='box'+str(i), position="0.0 "+str(y)+" 0.0", dim='8 2 2')

filename = "pile.xml"
print("Generated file: "+filename)
file = open(filename, "w")
file.write(ET.tostring(root))
file.close
