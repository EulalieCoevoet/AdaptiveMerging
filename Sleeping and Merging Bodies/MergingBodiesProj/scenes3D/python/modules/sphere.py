import xml.etree.ElementTree as ET
import general

class Sphere():

    def __init__(self, root, name="box", position="0 0 0", orientation="0. -1 0. 0.", velocity="0. 0. 0.", omega="0. 0. 0.",radius="1", pinned="false", magnetic="false", restitution=None, friction=None, color=None):
        # Fixed body
        self.body = general.body(root, "sphere", name, position, orientation, velocity=velocity, omega=omega, radius=radius, pinned=pinned, magnetic=magnetic, restitution=restitution, friction=friction, color=color)

    def addSpring(self, positionB="0 0 0", k="100", d="10", body2=None, positionW=None, positionB2=None):
        spring = ET.SubElement(self.body, 'spring')
        spring.set('pB', positionB)
        spring.set('k', k)
        spring.set('d', d)

        if (body2!=None and positionB2!=None):
            spring.set('body2', body2)
            spring.set('pB2', positionB2)

        if (positionW!=None):
            spring.set('pW', positionW)

    def get(self):
        return self.body
