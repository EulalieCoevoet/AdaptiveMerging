import xml.etree.ElementTree as ET
from box import Box
from sphere import Sphere
import general

class Composite():

    def __init__(self, root, obj, name="composite", position="0 0 0", orientation="0. -1 0. 0.", velocity="0. 0. 0.", omega="0. 0. 0.", scale="1", pinned="false", magnetic="false", restitution=None, friction=None, color=None, density="1"):
        # Fixed body
        self.body = general.body(root, "composite", name, position, orientation, density=density, velocity=velocity, omega=omega, obj=obj, scale=scale, pinned=pinned, magnetic=magnetic, restitution=restitution, friction=friction, color=color)

    def addSpring(self, positionB="0 0 0", k="100", d="10", ls="1", body2=None, positionW=None, positionB2=None):
        spring = ET.SubElement(self.body, 'spring')
        spring.set('pB', positionB)
        spring.set('k', k)
        spring.set('d', d)
        spring.set('ls', ls)

        if (body2!=None and positionB2!=None):
            spring.set('body2', body2)
            spring.set('pB2', positionB2)

        if (positionW!=None):
            spring.set('pW', positionW)

    def addBox(self, **kwargs):
        Box(self.body, **kwargs)

    def addSphere(self, **kwargs):
        Sphere(self.body, **kwargs)

    def get(self):
        return self.body
