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

spinnery="7"
spinnersize="8"

spinner = Platform(root, spinner=True, dim="15 1 15", height="3", position="0. 5. 0.", omega="0 0.5 0.", friction="0.1")
spinner.addStand(width="20", height="30", k="12", d="10", pinned="false", weirdCoeff="-5")

Sphere(root, name='ball1', position="3.75 "+spinnery+" 3.75", radius="2", color="0.8 0.6 0.6 1.")
Sphere(root, name='ball2', position="-3.75 "+spinnery+" -3.75", radius="2", color="0.6 0.8 0.6 1.")
Box(root, name='box1', position="-3.75 "+spinnery+" 3.75", orientation="0 -1 0 0", dim="3 3 3", color="0.6 0.8 0.8 1.")
Box(root, name='box2', position="3.75 "+spinnery+" -3.75", orientation="0 -1 0 0", dim="3 3 3", color="0.8 0.8 0.6 1.")

export(root, "../spinner.xml")
