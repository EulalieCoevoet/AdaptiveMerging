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
system.set('mouseSpringStiffness','100.')
system.set('mouseSpringDamping','50')

mesh = Mesh(root, name="venus", scale="10", obj="data/scaledtorso10.obj", st="data/torso_flux.sph", position="0 50 0", orientation="-1 0 0 1.57", friction="0.8")
positions=["-10 0 10","10 0 10","-10 0 -10","10 0 -10","-10 10 10","10 10 10","-10 10 -10","10 10 -10"]
for i in range(8):
    mesh.addSpring(positionB="-10 0 10", k="1000", d="900", ls="0.5")


y=102; x0=-2.; z0=7.;
radius=7;
nbPerles=15;
theta0=2.*math.pi/float(nbPerles)
theta=0.
for i in range(nbPerles):
    theta+=theta0
    x=radius*math.cos(theta)+x0
    z=radius*math.sin(theta)+z0
    sphere = Sphere(root, name="perle"+str(i), position=str(x)+" "+str(y)+" "+str(z), radius="1.6", density="1", friction="0.8")
    if i>0:
        sphere.addSpring(positionB="0.8 0 0", k="400.", d="300", ls="0.6", body2="perle"+str(i-1), positionB2="-0.8 0 0")
    if i==nbPerles-1:
        sphere.addSpring(positionB="-0.8 0 0", k="400.", d="300", ls="0.6", body2="perle0", positionB2="0.8 0 0")


export(root, "../venus.xml")
