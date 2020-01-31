import xml.etree.ElementTree as ET
import xml.dom.minidom as md

from modules.box import Box
from modules.sphere import Sphere
from modules.mesh import Mesh
from modules.composite import Composite
from modules.exportXML import export

import random
import math

root = ET.Element('root')

# Plane
plane = ET.SubElement(root, 'body')
plane.set('type','plane')
plane.set('p','0 2 0')
plane.set('n','0. 1. 0.0')
plane.set('name','plane')

system = ET.SubElement(root, 'system')
system.set('mouseSpringStiffness','100.')
system.set('mouseSpringDamping','50')

mesh = Mesh(root, name="venus", scale="10", obj="data/scaledtorso10.obj", st="data/torso_flux.sph", position="0 50 0", orientation="-1 0 0 1.57", friction="0.8", density="0.1")
positions=["-10 0 10","10 0 10","-10 0 -10","10 0 -10","-10 10 10","10 10 10","-10 10 -10","10 10 -10"]
for i in range(8):
    mesh.addSpring(positionB="-10 0 10", k="1000", d="900", ls="0.5")

stand = Composite(root, obj=None, name="stand", scale="10", position="0 0 0", friction="0.8", density="0.1", pinned="true")
stand.addBox(name='wall1', position="0. 15 20", orientation="1 0 0 0", dim='50 30 5')
stand.addBox(name='wall3', position="0. 15 -20", orientation="1 0 0 0", dim='50 30 5')
stand.addBox(name='wall2', position="-22.5 15 0.", orientation="1 0 0 0", dim='5 30 40')
stand.addBox(name='wall4', position="22.5 15 0.", orientation="1 0 0 0", dim='5 30 40')

y=98; x0=-2.; z0=4.;
radius=9;
nbPerles=18;
theta0=2.*math.pi/float(nbPerles)
theta=0.
for i in range(nbPerles):
    theta+=theta0
    x=radius*math.cos(theta)+x0
    z=radius*math.sin(theta)+z0
    sphere = Sphere(root, name="perle"+str(i), position=str(x)+" "+str(y)+" "+str(z), radius="1.6", density="1", friction="0.8")
    if i>0:
        sphere.addSpring(positionB="0.8 0 0", k="300.", d="100", ls="0.2", body2="perle"+str(i-1), positionB2="-0.8 0 0")
    if i==nbPerles-1:
        sphere.addSpring(positionB="-0.8 0 0", k="300.", d="100", ls="0.2", body2="perle0", positionB2="0.8 0 0")

export(root, "../venus.xml")
