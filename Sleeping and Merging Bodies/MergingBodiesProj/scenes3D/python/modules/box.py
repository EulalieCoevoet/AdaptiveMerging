import xml.etree.ElementTree as ET

class Box():

    def __init__(self, root, name="box", position="0 0 0", orientation="0. -1 0. 0.", dim="2 2 2", pinned="false", magnetic="false", restitution=None, friction=None):
        # Fixed body
        self.body = ET.SubElement(root, 'body')
        self.body.set('type','box')
        self.body.set('dim', dim)
        self.body.set('name',name)
        com = ET.SubElement(self.body, 'x')
        com.text = position
        R = ET.SubElement(self.body, 'R')
        R.text = orientation
        pin = ET.SubElement(self.body, 'pinned')
        pin.text = pinned
        magn = ET.SubElement(self.body, 'magnetic')
        magn.text = magnetic
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



### Test
# root = ET.Element('root')
# Box(root, "0 0 0", dim="15 2 15", pinned="true")
# filename = "box.xml"
# print("Generated file: "+filename)
# file = open(filename, "w")
# file.write(ET.tostring(root))
# file.close
