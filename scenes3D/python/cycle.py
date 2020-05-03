import xml.etree.ElementTree as ET
from modules.box import Box
from modules.exportXML import export

root = ET.Element('root')

# Fixed bodies
x = 0
for i in range(2):
    if i==1 : x += 18
    Box(root, name='box'+str(i), position=str(x)+" 0.0 0.0", dim='8 2 8', pinned="true")

# Pile bodies
y = 0
x = 4
for i in range(8):
    y += 2.5
    if i==4 :
        x += 10
        y = 2.5
    Box(root, name='box'+str(i), position=str(x)+" "+str(y)+" 0.0", dim='8 2 8')

export(root, "../cycle.xml")
