import xml.etree.ElementTree as ET

class Mesh():

    def __init__(self, root, obj, name="mesh", position="0 0 0", orientation="0. -1 0. 0.", scale="1", st=None, pinned="false", restitution=None, friction=None):
        # Fixed body
        self.body = ET.SubElement(root, 'body')
        self.body.set('type','mesh')
        self.body.set('name', name)
        self.body.set('obj', obj)
        self.body.set('scale', scale)
        if (st != None):
            self.body.set('st', st)
        com = ET.SubElement(self.body, 'x')
        com.text = position
        R = ET.SubElement(self.body, 'R')
        R.text = orientation
        pin = ET.SubElement(self.body, 'pinned')
        pin.text = pinned
        if(restitution!=None):
            rest = ET.SubElement(self.body, 'restitution')
            rest.text = restitution
        if(friction!=None):
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
