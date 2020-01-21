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
plane.set('p','0 0 0')
plane.set('n','0. 1. 0.0')
plane.set('name','plane')

system = ET.SubElement(root, 'system')
system.set('mouseSpringStiffness','1.')
system.set('mouseSpringDamping','0.021')

mesh = Mesh(root, name="venus", scale="1", obj="data/scaledtorso10.obj", st="data/torso_flux.sph", position="0 5 0", orientation="-1 0 0 1.57", friction="1.")
mesh.addSpring(positionB="-1 0 1", k="100", d="10")
mesh.addSpring(positionB="1 0 1", k="100", d="10")
mesh.addSpring(positionB="-1 0 -1", k="100", d="10")
mesh.addSpring(positionB="1 0 -1", k="100", d="10")
mesh.addSpring(positionB="-1 1 1", k="100", d="10")
mesh.addSpring(positionB="1 1 1", k="100", d="10")
mesh.addSpring(positionB="-1 1 -1", k="100", d="10")
mesh.addSpring(positionB="1 1 -1", k="100", d="10")


y=10.2; x0=0.; z0=0.;
radius=0.5;
nbPerles=15;
theta0=2.*math.pi/float(nbPerles)
theta=0.
for i in range(nbPerles):
    theta+=theta0
    x=radius*math.cos(theta)+x0
    z=radius*math.sin(theta)+z0
    sphere = Sphere(root, name="perle"+str(i), position=str(x)+" "+str(y)+" "+str(z), radius="0.1", density="1", friction="1.")
    # sphere = Box(root, name="perle"+str(i), position=str(x)+" "+str(y)+" "+str(z), dim="0.2 0.2 0.2", density="1", friction="0.8")
    if i>0:
        sphere.addSpring(positionB="0.08 0 0", k="1.", d="0.021", body2="perle"+str(i-1), positionB2="-0.08 0 0")
    if i==nbPerles-1:
        sphere.addSpring(positionB="-0.08 0 0", k="1.", d="0.021", body2="perle0", positionB2="0.08 0 0")


export(root, "../venus.xml")
