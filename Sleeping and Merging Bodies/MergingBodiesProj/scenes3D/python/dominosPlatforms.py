import xml.etree.ElementTree as ET
import xml.dom.minidom as md

from modules.box import Box
from modules.sphere import Sphere
from modules.platform import Platform
from modules.exportXML import export

import random

root = ET.Element('root')

# Plane
plane = ET.SubElement(root, 'body')
plane.set('type','plane')
plane.set('p','0 0 0')
plane.set('n','0. 1. 0.0')
plane.set('name','plane')

# System
plane = ET.SubElement(root, 'system')
plane.set('mouseSpringStiffness','3')
plane.set('mouseSpringDamping','1')

x0=0.
y0=0.
z0=0.

dx=0.88
dy=5.

x=x0;
y=y0;
z=z0;

platformDimx=29 #better impair

nbPlatforms=3
nbDominos=33

for k in range(nbPlatforms):
    y+=dy
    x=x0
    x+= dx/2 if (k%2) else -dx/2;
    xp=x;
    for i in range(nbDominos):
        x+=dx
        Box(root, name='domino'+str(i+nbDominos*k), position=str(x)+" "+str(y+1.2)+" "+str(z), dim="0.25 2 1", density="2")
        if (i==nbDominos/2):
            v = "-0.22 0 0" if (k%2) else "0.22 0 0";
            platform=Box(root, name="platform"+str(k), dim=str(platformDimx)+" 0.5 3", position=str(x)+" "+str(y)+" "+str(z), velocity=v)
            height=3
            platform.addSpring(positionB="-"+str(platformDimx/2)+" 0 1.5", positionW=str(-(platformDimx-1)/2.+x)+" "+str(y+height)+" 1.5")
            platform.addSpring(positionB="-"+str(platformDimx/2)+" 0 -1.5", positionW=str(-(platformDimx-1)/2.+x)+" "+str(y+height)+" -1.5")
            platform.addSpring(positionB=str(platformDimx/2)+" 0 -1.5", positionW=str((platformDimx-1)/2.+x)+" "+str(y+height)+" -1.5")
            platform.addSpring(positionB=str(platformDimx/2)+" 0 1.5", positionW=str((platformDimx-1)/2.+x)+" "+str(y+height)+" 1.5")

export(root, "../dominosPlatforms.xml")
