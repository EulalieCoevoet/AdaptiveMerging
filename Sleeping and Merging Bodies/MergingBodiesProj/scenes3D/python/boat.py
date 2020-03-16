import xml.etree.ElementTree as ET
import xml.dom.minidom as md

from modules.box import Box
from modules.sphere import Sphere
from modules.mesh import Mesh
from modules.composite import Composite
from modules.exportXML import export

import random

root = ET.Element('root')

Box(root, name='dock', position="0. -5. -40.", dim="100 5 70", color="0.1 0.1 0.1 1.", pinned="true", friction="0.")

# Containers
dimx=3; dimy=1; dimz=1;
dist=0.1;
colors=["0.5 0.3 0.3 1.","0.3 0.5 0.3 1.","0.3 0.3 0.5 1.","0.7 0.7 0.5 1."]

# Trains
disp=0
x=0; z=-36; y=-1;
for k in range(2):
    train = Composite(root, obj="data/train.obj", scale="0.025", name="train", position=str(x)+" -0.5 "+str(disp+z-2.5),density="1", color="0.3 0.3 0.3", velocity="-0.5 0 0")
    train.addBox(name='box', dim="35 0.95 2.5", position="16 -1.5 1.5")

    Box(root, name='rail1', position="0. -2.5 "+str(disp+z), dim="100 0.2 0.2", color="0.1 0.1 0.1 1.", pinned="true", friction="0.")
    Box(root, name='rail2', position="0. -2.5 "+str(disp+z-2), dim="100 0.2 0.2", color="0.1 0.1 0.1 1.", pinned="true", friction="0.")

    dispContainery=0
    for l in range(2):
        dispContainer=1
        for i in range(3):
            for j in range(2):
                Box(root, name='containerTrain', dim=str(dimx)+" "+str(dimy)+" "+str(dimz), color=colors[random.randint(0,3)], position=str(5+x+dispContainer)+" "+str(y+dispContainery)+" "+str(z-0.4+disp))
                Box(root, name='containerTrain', dim=str(dimx)+" "+str(dimy)+" "+str(dimz), color=colors[random.randint(0,3)], position=str(5+x+dispContainer)+" "+str(y+dispContainery)+" "+str(z-1.6+disp))
                dispContainer+=dimx+dist
            dispContainer+=dimx*1.5+dist
        dispContainery+=dimy

    disp+=5
    x-=20



# Portique
portique = Composite(root, obj="data/portique.obj", scale="0.05", name="portique", position="0 -2.5 -40",density="1", color="0.4 0. 0.")
portique.addBox(name='box', dim="10 1 1", position="0 0.5 0")
portique.addBox(name='box', dim="10 1 1", position="0 0.5 10")

# Bodies pile on the dock
for k in range(2):
    y0 = -2.
    x0 = -32. + k*50
    z0 = -23
    y=y0; x=x0; z=z0;
    for i in range(100):
        y += dimy
        if (i%(3+k)==0):
            x+=dimx + dist; y=y0;
        if (i%31==0):
            x=x0; y=y0; z+=dimz + dist;
        orientation = "0 -1 0 0.03" if (i%2) else "0 -1 0 0";
        Box(root, name='containerDock'+str(i), dim=str(dimx)+" "+str(dimy)+" "+str(dimz), color=colors[random.randint(0,3)], position=str(x)+" "+str(y)+" "+str(z))

# Trucks
positions           = ["-10. -1.5 -15.", "-0. -1.5 -15.",  "30. -1.5 -15."]
positionsContainer1 = ["-10.3 -1.2 -15.","-0.3 -1.2 -15.", "29.7 -1.2 -15."]
positionsContainer2 = ["-13.4 -1.2 -15.","-3.4 -1.2 -15.", "26.6 -1.2 -15."]
positionsContainer3 = ["-12 -.2 -15.",   "-2 -.2 -15.",    "28 -.2 -15."]
for i in range(3):
    truck = Composite(root, obj="data/truckPort.obj", scale="0.005", name="truck"+str(i), position=positions[i], density="1", velocity="-1 0 0")
    truck.addBox(name='box', dim="8 1 3", position="-3 -0.5 0")
    Box(root, name='containerTruck0'+str(i), dim=str(dimx)+" "+str(dimy)+" "+str(dimz), color=colors[random.randint(0,3)], position=positionsContainer1[i])
    Box(root, name='containerTruck1'+str(i), dim=str(dimx)+" "+str(dimy)+" "+str(dimz), color=colors[random.randint(0,3)], position=positionsContainer2[i])
    if not (i%3):
        Box(root, name='containerTruck2'+str(i), dim=str(dimx)+" "+str(dimy)+" "+str(dimz), color=colors[random.randint(0,3)], position=positionsContainer3[i])


disp = -27
boatColors = ["0.7 0.7 0.7", "0.4 0.4 0.4"]
for k in range(2):

    # Crane on the dock
    composite = Composite(root, obj=None, name="crane", position=str(disp)+" 0. 0.", color="0.5 0.4 0. 1.")
    composite.addBox(name='stand1', dim="2 0.1 5", position="-8 -2.5 -8.5")
    composite.addBox(name='crane11', dim="0.5 10 0.5", position="-8 2.5 -10")
    composite.addBox(name='crane12', dim="1 0.5 16", position="-6 5.5 -6")
    composite.addBox(name='stand2', dim="2 0.1 5", position="0 -2.5 -8.5")
    composite.addBox(name='crane21', dim="0.5 10 0.5", position="0 2.5 -10")
    composite.addBox(name='crane22', dim="1 0.5 16", position="-2 5.5 -6")
    composite.addBox(name='stand3', dim="12 0.5 0.5", position="-4 5.5 -10")
    composite.addBox(name='cockpit1', dim="1 1 1", position="2 5.5 -10")
    composite.addBox(name='cockpit2', dim="1 1 1", position="-10 5.5 -10")

    mag = Box(root, name='magnet', dim="1 0.5 1", position=str(disp-2)+" 4 -6", magnetic="true", color="0. 0. 0. 1.")
    mag.addSpring(positionB="-0.4 0 0.4", positionW=str(disp-2.4)+" 5.5 -5.6",controllable="true", k="100", d="1")
    mag.addSpring(positionB="0.4 0. 0.4", positionW=str(disp-1.6)+" 5.5 -5.6", controllable="true", k="100", d="1")
    mag.addSpring(positionB="-0.4 0. -0.4", positionW=str(disp-2.4)+" 5.5 -6.4",controllable="true",k="100", d="1")
    mag.addSpring(positionB="0.4 0. -0.4", positionW=str(disp-1.6)+" 5.5 -6.4",controllable="true", k="100", d="1")

    # Boat
    boat = Composite(root, obj="data/boat.obj", scale="0.1", name="boat", position=str(disp)+" -2.5 0.",density="10", color=boatColors[k])
    boat.addBox(name="ground", position="0. -1.5 0.", dim="30 3 6")
    boat.addSpring(positionB="7. 0 1.8", positionW=str(disp+7)+" -1.5 1.8", k="100", d="1")
    boat.addSpring(positionB="-7. 0 1.8", positionW=str(disp-7)+" -1.5 1.8", k="100", d="1")
    boat.addSpring(positionB="-7. 0 -1.8", positionW=str(disp-7)+" -1.5 -1.8", k="100", d="1")
    boat.addSpring(positionB="7. 0 -1.8", positionW=str(disp+7)+" -1.5 -1.8", k="100", d="1")

    # Bodies pile on the boat
    y0 = -2
    x0 = -8. + disp
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

    disp+=45


# Pointing finger
# boat = Composite(root, obj="data/pointingfinger.obj", scale="0.3", name="finger", position="70 30 -40", density="1", color="1. 1. 1. 1.")
# boat.addBox(name="fingercenterbox", position="0. 0. 0.", dim="5 5 5")
# boat.addSphere(name="fingersphere", position="-18. -12. 0.", radius="2")
# boat.addSpring(positionB="2.5 2.5 2.5", k="100", d="1")
# boat.addSpring(positionB="2.5 2.5 -2.5", k="100", d="1")
# boat.addSpring(positionB="-2.5 2.5 -2.5", k="100", d="1")
# boat.addSpring(positionB="-2.5 2.5 2.5", k="100", d="1")
# boat.addSpring(positionB="2.5 -2.5 2.5", k="100", d="1")
# boat.addSpring(positionB="2.5 -2.5 -2.5", k="100", d="1")
# boat.addSpring(positionB="-2.5 -2.5 -2.5", k="100", d="1")
# boat.addSpring(positionB="-2.5 -2.5 2.5", k="100", d="1")


# Ocean
ocean = Composite(root, obj="data/ocean.obj", scale="0.1", name="ocean", position="0. -2.5 60.", color="0. 0.18 0.29 1.", density="10")
ocean.addBox(name="dummy", position="0. -1.5 0.", dim="30 3 6")
ocean.addSpring(positionB="7. 0 1.8", positionW="7 -1.5 61.8", k="100", d="1")
ocean.addSpring(positionB="-7. 0 1.8", positionW="-7 -1.5 61.8", k="100", d="1")
ocean.addSpring(positionB="-7. 0 -1.8", positionW="-7 -1.5 58.2", k="100", d="1")
ocean.addSpring(positionB="7. 0 -1.8", positionW="7 -1.5 58.2", k="100", d="1")


export(root, "../cargoShip.xml")
