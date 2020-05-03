import xml.etree.ElementTree as ET
from box import Box
from sphere import Sphere
from composite import Composite

class Platform():

    def __init__(self, root, position="0 3 0", orientation="0. -1 0. 0.", density="1", scale="1", pinned="false", dim="8 2 8", restitution=None, friction=None, spinner=False, height="3", velocity="0 0 0", omega="0 0 0"):
        self.root = root
        self.position = position
        self.dim = dim.split()
        d = self.dim

        if not spinner:
            Box( root, name="platform", position=position, orientation=orientation, velocity=velocity, density=density, omega=omega, dim=dim, pinned=pinned, magnetic="false", restitution=restitution, friction=friction)
        else:
            composite = Composite( root, name="platform", obj=None, position=position, orientation=orientation, density=density, pinned=pinned, magnetic="false", restitution=restitution, friction=friction, velocity=velocity, omega=omega)
            dx=str(float(d[0])/2.-float(d[1])/2.)
            dy=str(float(height)/2.-float(d[1])/2.)
            dz=str(float(d[2])/2.-float(d[1])/2.)

            composite.addBox(position="0. 0. 0.", dim=d[0]+" "+d[1]+" "+d[2])
            composite.addBox(position="0. "+dy+" 0.", dim=d[0]+" "+height+" "+d[1])
            composite.addBox(position="0. "+dy+" "+dz, dim=d[0]+" "+height+" "+d[1])
            composite.addBox(position="0. "+dy+" -"+dz, dim=d[0]+" "+height+" "+d[1])
            composite.addBox(position="0. "+dy+" 0.", dim=d[1]+" "+height+" "+d[2])
            composite.addBox(position=" "+dx+" "+dy+" 0.", dim=d[1]+" "+height+" "+d[2])
            composite.addBox(position=" -"+dx+" "+dy+" 0.", dim=d[1]+" "+height+" "+d[2])

    def addStand(self, k="3", d="10", width="5", height="20", springShift="0", orientation="0. -1 0. 0.", scale="1", obj=None, restitution=None, friction=None, thickness="1", pinned="false", weirdCoeff="10"):

        composite = Composite(self.root, name="stand", obj=obj, position="0 0 0", orientation=orientation, scale=scale, restitution=restitution, friction=friction, pinned=pinned)
        composite.addBox(dim=str(float(width)*2.+float(thickness))+" "+thickness+" "+thickness, position="0 "+height+" 0")
        composite.addBox(dim=thickness+" "+height+" "+thickness, position=width+" "+str(float(height)/2.)+" 0")
        composite.addBox(dim=thickness+" "+height+" "+thickness, position="-"+width+" "+str(float(height)/2.)+" 0")
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
        ybody2=self.position.split()[1]
        #com is moving weirdly with respect to width... (something wrong in Java code...)
        y = str(float(height)/2.+float(thickness)*2+float(weirdCoeff))
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
