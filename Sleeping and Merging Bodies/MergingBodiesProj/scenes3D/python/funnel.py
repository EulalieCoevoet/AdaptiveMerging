import xml.etree.ElementTree as ET
import xml.dom.minidom as md

from modules.box import Box
from modules.sphere import Sphere
from modules.mesh import Mesh
from modules.stand import Stand
from modules.composite import Composite
from modules.exportXML import export

root = ET.Element('root')

# Plane
plane = ET.SubElement(root, 'body')
plane.set('type','plane')
plane.set('p','0 0 0')
plane.set('n','0. 1. 0.0')
plane.set('name','plane')

# Default Box
Box(root, name='box1', position="0 100 0", orientation="0 0 1 0", dim="1.8 1.8 1.8")
Box(root, name='box2', position="0 100 0", orientation="0 0 1 0", dim="1.8 1.8 1.8")
Box(root, name='box3', position="0 100 0", orientation="0 0 1 0", dim="1.8 1.8 1.8")
# Sphere(root, name='sphere', position="0 100 0", orientation="0 0 1 0", radius="1")
# Mesh(root, name='venus', obj="data/scaletorso10.obj", st="data/torso_flux.sph", position="0 100 0", orientation="0 0 1 0", scale="0.1")

# Funnel
# composite = Composite(root, obj="data/funnel.obj", scale="2", name="funnel", position="0. 50. 0.", velocity="0. 0. 0.", color="0.5 0. 0. 1.")
# composite.addBox(name='wall1', position="0. 0. -9.", orientation="-1 0 0 0.5", dim='28 22 1')
# composite.addBox(name='wall3', position="0. 0. 9.", orientation="1 0 0 0.5", dim='28 22 1')
# composite.addBox(name='wall2', position="-9. 0. 0.", orientation="0 0 1 0.5", dim='1 22 28')
# composite.addBox(name='wall4', position="9. 0. 0.", orientation="0 0 -1 0.5", dim='1 22 28')
composite = Composite(root, obj="data/funnel.obj", scale="1", name="funnel", position="0. 50. 0.", velocity="0. 0. 0.", color="0.5 0. 0. 1.", omega="0 0 -0.05")
composite.addBox(name='wall1', position="0. -0.5 -13.5", orientation="-1 0 0 1.07", dim='48 22 1')
composite.addBox(name='wall3', position="0. -0.5 13.5", orientation="1 0 0 1.07", dim='48 22 1')
composite.addBox(name='wall2', position="-13.5 -0.5 0.", orientation="0 0 1 1.07", dim='1 22 48')
composite.addBox(name='wall4', position="13.5 -0.5 0.", orientation="0 0 -1 1.07", dim='1 22 48')

# Stand(root, body2="funnel", dimBody2="26 20 26", ybody2="74", width="40", height="100", pinned="true")
Stand(root, body2="funnel", dimBody2="46 10 46", ybody2="74", width="40", height="100", pinned="true")

export(root, "../funnel.xml")
