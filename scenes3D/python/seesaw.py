import xml.etree.ElementTree as ET
from modules.box import Box
from modules.composite import Composite
from modules.exportXML import export
import random
import math

root = ET.Element('root')

# Plane
plane = ET.SubElement(root, 'body')
plane.set('type','plane')
plane.set('p','0 0 0')
plane.set('n','0 1 0')
plane.set('name','plane')

# Seesaw
composite = Composite(root, obj="data/seesaw1.obj", scale="0.1", name="seesaw1", position="0 10 0", velocity="0. 0. 0.", color="0.7 0.7 0.7 1.")
composite.addBox(name='seesaw1', position="0 2.0 0", dim='80 5 5')

composite = Composite(root, obj="data/seesaw2.obj", scale="0.1", name="seesaw2", position="0 10 0", velocity="0. 0. 0.", color="0.1 0.1 0.1 1.")
composite.addBox(name='part1', position="0 -4 0", dim='5 5 5', orientation="0. 0. 1. "+str(math.pi/4.))
composite.addBox(name='part2', position="0 -8 0", dim='9 5 5')

export(root, "../seesaw.xml")
