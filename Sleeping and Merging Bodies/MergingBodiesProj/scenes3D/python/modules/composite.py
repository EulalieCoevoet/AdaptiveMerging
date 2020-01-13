import xml.etree.ElementTree as ET
from box import Box
from sphere import Sphere

class CompositeBody():

    def __init__(self, root, obj, name="composite", position="0 0 0", orientation="0. -1 0. 0.", scale="1", pinned="false", velocity="0. 0. 0.", omega="0. 0. 0.", restitution=None, friction=None):
        # Fixed body
        self.body = ET.SubElement(root, 'body')
        self.body.set('type','composite')
        self.body.set('name', name)
        if (obj is not None):
            self.body.set('obj', obj)
        self.body.set('scale', scale)
        com = ET.SubElement(self.body, 'x')
        com.text = position
        vel = ET.SubElement(self.body, 'v')
        vel.text = velocity
        R = ET.SubElement(self.body, 'R')
        R.text = orientation
        ome = ET.SubElement(self.body, 'omega')
        ome.text = omega
        pin = ET.SubElement(self.body, 'pinned')
        pin.text = pinned
        if(restitution is not None):
            rest = ET.SubElement(self.body, 'restitution')
            rest.text = restitution
        if(friction is not None):
            fric = ET.SubElement(self.body, 'friction')
            fric.text = friction

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

    def addBox(self, **kwargs):
        Box(self.body, **kwargs)

    def addSphere(self, **kwargs):
        Sphere(self.body, **kwargs)

    def get(self):
        return self.body
