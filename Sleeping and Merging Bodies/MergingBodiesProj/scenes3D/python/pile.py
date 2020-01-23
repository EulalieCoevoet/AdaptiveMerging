import xml.etree.ElementTree as ET
from modules.box import Box
from modules.exportXML import export
import random

root = ET.Element('root')

# Plane
plane = ET.SubElement(root, 'body')
plane.set('type','plane')
plane.set('p','0 -1 0')
plane.set('n','0 1 0')
plane.set('name','plane')

# Bodies pile
y = 0
colors=["0.3 0.3 0.3 1.","0.2 0.5 0.2 1.","0.3 0.3 0.5 1.","0.7 0.7 0.5 1."]
orientations=["0 -1 0 0.1","0 -1 0 0","0 1 0 0.03","0 -1 0 0.05"]
for i in range(10):
    y += 2.5
    Box(root, name='box'+str(i), position="0.0 "+str(y)+" 0.0", dim='8 2 6', color=colors[random.randint(0,3)], orientation=orientations[random.randint(0,3)])

export(root, "../pile.xml")
