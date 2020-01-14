import xml.etree.ElementTree as ET
import xml.dom.minidom as md

from modules.box import Box
from modules.sphere import Sphere
from modules.mesh import Mesh
from modules.composite import Composite
from modules.exportXML import export

import math
import random

root = ET.Element('root')

# Plane
plane = ET.SubElement(root, 'body')
plane.set('type','plane')
plane.set('p','0 0 0')
plane.set('n','0. 1. 0.0')
plane.set('name','plane')

nbFloor=4
nbHouse=1
cardSizex=0.05
cardSizey=3
cardSizez=1
dim=str(cardSizex)+" "+str(cardSizey)+" "+str(cardSizez)
x0=-25.
y=0.
z=0.
shiftx=0.3
shifty=0.15

for k in range(nbHouse):
    nbFloor+=2
    x0+=25
    y=0.

    for i in range(nbFloor):
        y+=cardSizey-shifty
        x=x0+(cardSizey/2.-shiftx)*i+ cardSizey/2. + shiftx

        # Floor cards
        for j in range(nbFloor-i-1):
            y += cardSizex if (j%2) else -cardSizex
            if(j>0): x+=2.4
            orientation = "0 0 -1 "+str(math.pi/2.);
            Box(root, name='book'+str(j), position=str(x)+" "+str(y)+" "+str(z), dim=dim, color="0. 0. 0.5 1.", orientation=orientation, friction="0.3", density="1")

        y-=(cardSizey-shifty)/2.
        x=x0+(cardSizey/2.-shiftx)*i

        # Orientide Cards
        for j in range((nbFloor-i)*2):
            if(j>0): x+=1.2
            orientation = "0 0 -1 0.4" if (j%2==0) else "0 0 1 0.4";
            Box(root, name='book'+str(j), position=str(x)+" "+str(y)+" "+str(z), dim=dim, color="0. 0. 0.5 1.", orientation=orientation, friction="0.3", density="1")
        y+=(cardSizey-shifty)/2.

export(root, "../houseOfCards.xml")
