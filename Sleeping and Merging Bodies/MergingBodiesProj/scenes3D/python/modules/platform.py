import xml.etree.ElementTree as ET
from box import Box
from sphere import Sphere
from composite import CompositeBody

class Platform():

    def __init__(self, root, position="0 3 0", orientation="0. -1 0. 0.", scale="1", pinned="false", dim="8 1 8", restitution=None, friction=None):
        self.root = root
        self.position = position
        self.dim = dim.split()

        box = Box( root, name="platform", position=position, orientation=orientation, dim=dim, pinned=pinned, magnetic="false", restitution=restitution, friction=friction)

    def addStand(self, k="3", d="10", width="5", length="20", springShift="0", orientation="0. -1 0. 0.", scale="1", obj=None, restitution=None, friction=None, thickness="1"):

        composite = CompositeBody(self.root, name="stand", obj=obj, position="0 0 0", orientation=orientation, scale=scale, restitution=restitution, friction=friction)
        composite.addBox(dim=str(float(width)*2.+float(thickness))+" "+thickness+" "+thickness, position="0 "+length+" 0")
        composite.addBox(dim=thickness+" "+length+" "+thickness, position=width+" "+str(float(length)/2.)+" 0")
        composite.addBox(dim=thickness+" "+length+" "+thickness, position="-"+width+" "+str(float(length)/2.)+" 0")
        composite.addBox(dim=thickness+" "+thickness+" 10", position=width+" 0 0")
        composite.addBox(dim=thickness+" "+thickness+" 10", position="-"+width+" 0 0")
        stand = composite.get()

        dim = self.dim
        x = str(float(dim[0])/2.)
        y = str(float(dim[1])/2.)
        z = str(float(dim[2])/2.)
        pB = [x+" "+y+" "+z,
              "-"+x+" "+y+" "+z,
              "-"+x+" "+y+" -"+z,
              x+" "+y+" -"+z,]
        y = str(float(length)/2.+float(self.position.split()[1])-float(thickness))
        pB2 = [springShift+" "+y+" 0",
              "-"+springShift+" "+y+" 0",
              "-"+springShift+" "+y+" 0",
              springShift+" "+y+" 0",]
        for i in range(4):
            spring = ET.SubElement(stand, 'spring')
            spring.set('pB', pB2[i])
            spring.set('k', k)
            spring.set('d', d)
            spring.set('body2', "platform")
            spring.set('pB2', pB[i])


# Test
# platform = Platform(root)
# platform.addStand()
