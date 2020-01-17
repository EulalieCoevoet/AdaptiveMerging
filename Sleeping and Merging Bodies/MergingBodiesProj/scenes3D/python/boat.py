import xml.etree.ElementTree as ET
import xml.dom.minidom as md

from modules.box import Box
from modules.sphere import Sphere
from modules.mesh import Mesh
from modules.composite import Composite
from modules.exportXML import export

import random

root = ET.Element('root')

Box(root, name='dock', position="0. -5. -30.", dim="100 5 50", color="0.1 0.1 0.1 1.", pinned="true")

# Bodies pile on the dock
dimx=3; dimy=1; dimz=1;
dist=0.1;
colors=["0.5 0.3 0.3 1.","0.3 0.5 0.3 1.","0.3 0.3 0.5 1.","0.7 0.7 0.5 1."]
for k in range(2):
    y0 = -2.
    x0 = -30. + k*50
    z0 = -45
    y=y0; x=x0; z=z0;
    for i in range(600+k*100):
        y += dimy
        if (i%(3+k*2)==0):
            x+=dimx + dist; y=y0;
        if (i%31==0):
            x=x0; y=y0; z+=dimz + dist;
        orientation = "0 -1 0 0.03" if (i%2) else "0 -1 0 0";
        Box(root, name='boxDock'+str(i), dim=str(dimx)+" "+str(dimy)+" "+str(dimz), color=colors[random.randint(0,3)], position=str(x)+" "+str(y)+" "+str(z))

# Crane on the dock
composite = Composite(root, obj=None, name="crane", position="0. 0. 0.", color="0.5 0.4 0. 1.")
composite.addBox(name='stand1', dim="3 0.1 3", position="-8 -2.5 -10")
composite.addBox(name='crane11', dim="1 10 1", position="-8 2.5 -10")
composite.addBox(name='crane12', dim="1 0.75 16", position="-6 5.5 -6")
composite.addBox(name='stand2', dim="3 0.1 3", position="0 -2.5 -10")
composite.addBox(name='crane21', dim="1 10 1", position="0 2.5 -10")
composite.addBox(name='crane22', dim="1 0.75 16", position="-2 5.5 -6")
composite.addBox(name='stand3', dim="10 1 1", position="-4 5.5 -10")
composite.addBox(name='cockpit1', dim="2 2 2", position="2 5.5 -10")
composite.addBox(name='cockpit2', dim="2 2 2", position="-10 5.5 -10")

magnet = Box(root, name='magnet', dim="1 1 1", position="-2 5 -6", magnetic="true", color="0. 0. 0. 1.")
magnet.addSpring(positionB="-0.4 0.5 0.4", controllable="true", k="100", d="1")
magnet.addSpring(positionB="0.4 0.5 0.4", controllable="true", k="100", d="1")
magnet.addSpring(positionB="-0.4 0.5 -0.4", controllable="true", k="100", d="1")
magnet.addSpring(positionB="0.4 0.5 -0.4", controllable="true", k="100", d="1")

# Boat
boat = Composite(root, obj="data/boat.obj", scale="0.1", name="boat", position="0. -2.5 0.",density="10")
boat.addBox(name="ground", position="0. -1.5 0.", dim="30 3 6")
boat.addSpring(positionB="7. 0 1.8", positionW="7 -1.5 1.8", k="100", d="1")
boat.addSpring(positionB="-7. 0 1.8", positionW="-7 -1.5 1.8", k="100", d="1")
boat.addSpring(positionB="-7. 0 -1.8", positionW="-7 -1.5 -1.8", k="100", d="1")
boat.addSpring(positionB="7. 0 -1.8", positionW="7 -1.5 -1.8", k="100", d="1")

# Bodies pile on the boat
y0 = -2
x0 = -8.
z0 = -2.75

y=y0; x=x0; z=z0;
dimx=3; dimy=1; dimz=1;
dist=0.1;
for i in range(71):
    y += dimy + dist
    if (i%3==0):
        x+=dimx + dist; y=y0;
    if (i%18==0):
        x=x0; y=y0; z+=dimz + dist;
    orientation = "0 -1 0 0.03" if (i%2) else "0 -1 0 0";
    Box(root, name='boxBoat'+str(i), position=str(x)+" "+str(y)+" "+str(z), dim=str(dimx)+" "+str(dimy)+" "+str(dimz), color=colors[random.randint(0,2)])



# Ocean
ocean = Composite(root, obj="data/ocean.obj", scale="0.1", name="ocean", position="0. -2.5 40.", color="0. 0. 0.4 1.", density="10")
ocean.addBox(name="dummy", position="0. -1.5 0.", dim="30 3 6")
ocean.addSpring(positionB="7. 0 1.8", positionW="7 -1.5 41.8", k="100", d="1")
ocean.addSpring(positionB="-7. 0 1.8", positionW="-7 -1.5 41.8", k="100", d="1")
ocean.addSpring(positionB="-7. 0 -1.8", positionW="-7 -1.5 38.2", k="100", d="1")
ocean.addSpring(positionB="7. 0 -1.8", positionW="7 -1.5 38.2", k="100", d="1")


export(root, "../cargoShip.xml")
