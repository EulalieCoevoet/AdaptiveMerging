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
y0 = -2.
x0 = -45.
z0 = -45

y=y0; x=x0; z=z0;
dimx=3; dimy=1; dimz=1;
dist=0.1;
colors=["0.4 0.1 0.1 1.","0.1 0.4 0.1 1.","0.1 0.1 0.4 1."]
for i in range(900):
    y += dimy
    if (i%3==0):
        x+=dimx + dist; y=y0;
    if (i%90==0):
        x=x0; y=y0; z+=dimz + dist;
    orientation = "0 -1 0 0.03" if (i%2) else "0 -1 0 0";
    Box(root, name='boxDock'+str(i), dim=str(dimx)+" "+str(dimy)+" "+str(dimz), color=colors[random.randint(0,2)], position=str(x)+" "+str(y)+" "+str(z))

# Crane on the dock
composite = Composite(root, obj=None, name="crane", position="0. 0. 0.", color="0.5 0.4 0. 1.")
composite.addBox(name='stand', dim="2 0.1 2", position="-8 -2.5 -10")
composite.addBox(name='crane1', dim="1 10 1", position="-8 2.5 -10")
composite.addBox(name='crane2', dim="10 0.75 1", position="-6 5.5 -10")

# Boat
composite = Composite(root, obj="data/boat.obj", scale="0.1", name="boat", position="0. 0. 0.")
composite.addBox(name="ground", position="0. -1.5 0.", dim="25 3 6")
composite.addSpring(positionB="7. -0.5 2.", k="70", d="5")
composite.addSpring(positionB="-7. -0.5 2.", k="70", d="5")
composite.addSpring(positionB="-7. -0.5 -2.", k="70", d="5")
composite.addSpring(positionB="7. -0.5 -2.", k="70", d="5")

# Bodies pile on the boat
y0 = 0.5
x0 = -5.
z0 = -2.75

y=y0; x=x0; z=z0;
dimx=3; dimy=1; dimz=1;
dist=0.1;
colors=["0.4 0.1 0.1 1.","0.1 0.4 0.1 1.","0.1 0.1 0.4 1."]
for i in range(45):
    y += dimy + dist
    if (i%3==0):
        x+=dimx + dist; y=y0;
    if (i%12==0):
        x=x0; y=y0; z+=dimz + dist;
    orientation = "0 -1 0 0.03" if (i%2) else "0 -1 0 0";
    Box(root, name='boxBoat'+str(i), position=str(x)+" "+str(y)+" "+str(z), dim=str(dimx)+" "+str(dimy)+" "+str(dimz), color=colors[random.randint(0,2)])

export(root, "../cargoShip.xml")
