import xml.etree.ElementTree as ET
from box import Box
from sphere import Sphere
from composite import Composite

class Stand():

    def __init__(self, root, body2, dimBody2, ybody2, k="100", d="10", width="5", height="20", orientation="0. -1 0. 0.", scale="1", restitution=None, friction=None, thickness="1", pinned="false"):

        composite = Composite(root, obj=None, name="stand", position="0 0 0", orientation=orientation, scale=scale, restitution=restitution, friction=friction, pinned=pinned)
        composite.addBox(dim=str(float(width)*2.+float(thickness))+" "+thickness+" "+thickness, position="0 "+height+" "+width)
        composite.addBox(dim=str(float(width)*2.+float(thickness))+" "+thickness+" "+thickness, position="0 "+height+" -"+width)
        composite.addBox(dim=thickness+" "+thickness+" "+str(float(width)*2.+float(thickness)), position=width+" "+height+" 0")
        composite.addBox(dim=thickness+" "+thickness+" "+str(float(width)*2.+float(thickness)), position=" -"+width+" "+height+" 0")
        composite.addBox(dim=thickness+" "+height+" "+thickness, position=width+" "+str(float(height)/2.)+" "+width)
        composite.addBox(dim=thickness+" "+height+" "+thickness, position="-"+width+" "+str(float(height)/2.)+" "+width)
        composite.addBox(dim=thickness+" "+height+" "+thickness, position=width+" "+str(float(height)/2.)+" -"+width)
        composite.addBox(dim=thickness+" "+height+" "+thickness, position="-"+width+" "+str(float(height)/2.)+" -"+width)
        stand = composite.get()

        dim = dimBody2.split()
        x = str(float(dim[0])/2.)
        y = str(float(dim[1])/2.)
        z = str(float(dim[2])/2.)
        pB2 = [x+" "+y+" "+z,
              "-"+x+" "+y+" "+z,
              "-"+x+" "+y+" -"+z,
              x+" "+y+" -"+z,]
        y = str(float(height)-float(ybody2)+float(thickness)*2)
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
