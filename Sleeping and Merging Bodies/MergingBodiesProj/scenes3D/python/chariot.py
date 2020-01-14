import xml.etree.ElementTree as ET
import xml.dom.minidom as md

from modules.box import Box
from modules.sphere import Sphere
from modules.composite import Composite
from modules.exportXML import export

import random

root = ET.Element('root')

# Plane
plane = ET.SubElement(root, 'body')
plane.set('type','plane')
plane.set('p','0 -1 0')
plane.set('n','0. 1. 0.0')
plane.set('name','plane')
fric = ET.SubElement(plane, 'friction')
fric.text = "0.01"

# Bodies pile
y0 = 2.5
x0 = -2
z0 = -1.9

y=y0; x=x0; z=z0;
dimx=0.5; dimy=0.5; dimz=0.5;
dist=0.01;
for i in range(216):
    y += dimy + dist
    if (i%6==0):
        x+=dimx + dist; y=y0;
    if (i%36==0):
        x=x0; y=y0; z+=dimz + dist;
    orientation = "0 -1 0 0.03" if (i%2) else "0 -1 0 0";
    #color = str(random.uniform(0, 1)) + " " + str(random.uniform(0, 1)) + " " + str(random.uniform(0, 1)) + " 1."
    Box(root, name='box'+str(i), position=str(x)+" "+str(y)+" "+str(z), dim=str(dimx)+" "+str(dimy)+" "+str(dimz))

Sphere(root, name='ball', position="1.8 2.7 0", radius="1.", color="0.8 0.8 0. 1.")
Box(root, name='box'+str(i), position="-30 -2 0", orientation="0 0 1 0", dim="10 5 20", pinned="true")

# chariot
composite = Composite(root, obj="data/chariot.obj", scale="0.1", name="chariot", position="0. 1.7 0.", velocity="0. 0. 0.", color="1. 0. 0. 1.")
composite.addSphere(name='wheel1', position=" 3.15 -1.7  1.9", radius='1')
composite.addSphere(name='wheel2', position=" 3.15 -1.7 -1.9", radius='1')
composite.addSphere(name='wheel3', position="-3.15 -1.7  1.9", radius='1')
composite.addSphere(name='wheel4', position="-3.15 -1.7 -1.9", radius='1')
composite.addBox(name='panier', position="0.2 0.1 0.", dim='7.2 0.2 4')
composite.addBox(name='panierb1', position="0.2 0.75 -1.9", dim='7.2 1.5 0.2')
composite.addBox(name='panierb2', position="3.7 0.75 0.", dim='0.2 1.5 4')
composite.addBox(name='panierb3', position="0.2 0.75 1.9", dim='7.2 1.5 0.2')
composite.addBox(name='panierb4', position="-3.5 0.75 0.", dim='0.2 1.5 4')



export(root, "../chariot.xml")
