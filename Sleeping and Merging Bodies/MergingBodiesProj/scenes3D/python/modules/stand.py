import xml.etree.ElementTree as ET
from box import Box
from sphere import Sphere
from composite import CompositeBody

class Stand():

    def __init__(self, root, body2, dimBody2, ybody2, k="100", d="10", width="5", length="20", orientation="0. -1 0. 0.", scale="1", restitution=None, friction=None, thickness="1"):

        composite = CompositeBody(root, obj=None, name="stand", position="0 0 0", orientation=orientation, scale=scale, restitution=restitution, friction=friction)
        composite.addBox(dim=str(float(width)*2.+float(thickness))+" "+thickness+" "+thickness, position="0 "+length+" "+width)
        composite.addBox(dim=str(float(width)*2.+float(thickness))+" "+thickness+" "+thickness, position="0 "+length+" -"+width)
        composite.addBox(dim=thickness+" "+thickness+" "+str(float(width)*2.+float(thickness)), position=width+" "+length+" 0")
        composite.addBox(dim=thickness+" "+thickness+" "+str(float(width)*2.+float(thickness)), position=" -"+width+" "+length+" 0")
        composite.addBox(dim=thickness+" "+length+" "+thickness, position=width+" "+str(float(length)/2.)+" "+width)
        composite.addBox(dim=thickness+" "+length+" "+thickness, position="-"+width+" "+str(float(length)/2.)+" "+width)
        composite.addBox(dim=thickness+" "+length+" "+thickness, position=width+" "+str(float(length)/2.)+" -"+width)
        composite.addBox(dim=thickness+" "+length+" "+thickness, position="-"+width+" "+str(float(length)/2.)+" -"+width)
        stand = composite.get()

        dim = dimBody2.split()
        x = str(float(dim[0])/2.)
        y = str(float(dim[1])/2.)
        z = str(float(dim[2])/2.)
        pB2 = [x+" "+y+" "+z,
              "-"+x+" "+y+" "+z,
              "-"+x+" "+y+" -"+z,
              x+" "+y+" -"+z,]
        y = str(float(length)-float(ybody2)+float(thickness)*2)
        pB = [    str(float(width))+" "+y+" " +str(float(width)),
              "-"+str(float(width))+" "+y+" " +str(float(width)),
              "-"+str(float(width))+" "+y+" -" +str(float(width)),
                  str(float(width))+" "+y+" -" +str(float(width))]
        for i in range(4):
            spring = ET.SubElement(stand, 'spring')
            spring.set('pB', pB[i])
            spring.set('k', k)
            spring.set('d', d)
            spring.set('body2', body2)
            spring.set('pB2', pB2[i])
