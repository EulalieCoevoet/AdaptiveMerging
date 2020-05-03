import xml.etree.ElementTree as ET
import xml.dom.minidom as md

from modules.box import Box
from modules.sphere import Sphere
from modules.platform import Platform
from modules.exportXML import export

import random

root = ET.Element('root')

# Collision
collision = ET.SubElement(root, 'collision')
collision.set('feedbackStiffness','6')
collision.set('enableCompliance','false')
collision.set('enablePostStabilization','true')
collision.set('iterations','10')

# Plane
plane = ET.SubElement(root, 'body')
plane.set('type','plane')
plane.set('p','0 -0.5 0')
plane.set('n','0. 1. 0.0')
plane.set('name','plane')


spinner = Platform(root, spinner=True, dim="15 1 15", height="3", position="0. 5. 0.", omega="0 2.5 0.", friction="0.1", density="4.")
spinner.addStand(width="20", height="30", k="10", d="8", pinned="false", weirdCoeff="-5")

# spinnery=6.5
# dist=2.1
# dx=dy=dz=0
# for i in range(2):
#     dx+=dist
#     dz=0
#     for j in range(2):
#         dz+=dist
#         Sphere(root, position=str(dx+0.75)+" "+str(dy+spinnery)+" "+str(dz+0.75), radius="1", color="0.8 0.6 0.6 1.", density="0.00001", friction="0.1")
#         Sphere(root, position=str(dx-6.75)+" "+str(dy+spinnery)+" "+str(dz-6.75), radius="1", color="0.6 0.8 0.6 1.", density="0.00001", friction="0.1")
#         Box(root, position=str(dx-6.75)+" "+str(dy+spinnery)+" "+str(dz+0.75), orientation="0 -1 0 0", dim="2 2 2", color="0.6 0.8 0.8 1.", density="0.00001", friction="0.1")
#         Box(root, position=str(dx+0.75)+" "+str(dy+spinnery)+" "+str(dz-6.75), orientation="0 -1 0 0", dim="2 2 2", color="0.8 0.8 0.6 1.", density="0.00001", friction="0.1")
# export(root, "../spinnerMultiple.xml")

# spinnery=7
# dx=dy=dz=0
# Sphere(root, position=str(dx+3.75)+" "+str(dy+spinnery)+" "+str(dz+3.75), radius="2", color="0.8 0.6 0.6 1.", density="0.0001")
# Sphere(root, position=str(dx-3.75)+" "+str(dy+spinnery)+" "+str(dz-3.75), radius="2", color="0.6 0.8 0.6 1.", density="0.0001")
# Box(root, position=str(dx-3.75)+" "+str(dy+spinnery)+" "+str(dz+3.75), orientation="0 -1 0 0", dim="3 3 3", color="0.6 0.8 0.8 1.", density="0.0001")
# Box(root, position=str(dx+3.75)+" "+str(dy+spinnery)+" "+str(dz-3.75), orientation="0 -1 0 0", dim="3 3 3", color="0.8 0.8 0.6 1.", density="0.0001")
# export(root, "../spinnerHighRotMassRatio.xml")

spinnery=7
dx=dy=dz=0
Sphere(root, position=str(dx+3.75)+" "+str(dy+spinnery)+" "+str(dz+3.75), radius="2", color="0.8 0.6 0.6 1.", density="0.1")
Sphere(root, position=str(dx-3.75)+" "+str(dy+spinnery)+" "+str(dz-3.75), radius="2", color="0.6 0.8 0.6 1.", density="0.1")
Box(root, position=str(dx-3.75)+" "+str(dy+spinnery)+" "+str(dz+3.75), orientation="0 -1 0 0", dim="3 3 3", color="0.6 0.8 0.8 1.", density="0.1")
Box(root, position=str(dx+3.75)+" "+str(dy+spinnery)+" "+str(dz-3.75), orientation="0 -1 0 0", dim="3 3 3", color="0.8 0.8 0.6 1.", density="0.1")
export(root, "../spinner.xml")
