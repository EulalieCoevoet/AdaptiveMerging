import xml.etree.ElementTree as ET
import xml.dom.minidom as md

from modules.box import Box
from modules.mesh import Mesh
from modules.exportXML import export

root = ET.Element('root')

# Plane
plane = ET.SubElement(root, 'body')
plane.set('type','plane')
plane.set('p','0 -1 0')
plane.set('n','0 1 0')
plane.set('name','plane')

# Bodies pile
y = 15
x = -15
z = -7
for i in range(750):
    y += 2.
    if (i%15==0) :
        x+=2.5
        y=15
    if (i%150==0) :
        x=-15
        y=15
        z+=2.5
    Box(root, name='box'+str(i), position=str(x)+" "+str(y)+" "+str(z), dim='2 2 2')

# chariot
Mesh(root, obj="data/chariot.obj", scale="0.5", name="chariot", st="data/chariot.sph", position="0. 12. 0.")



export(root, "../chariot.xml")
