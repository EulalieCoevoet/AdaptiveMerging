import xml.etree.ElementTree as ET

root = ET.Element('root')

# Plane
plane = ET.SubElement(root, 'body')
plane.set('type','plane')
plane.set('p','0 -1 0')
plane.set('n','0 1 0')
plane.set('name','plane')

# Bodies pile
y = 0
for i in range(15):
    y += 2.5
    body = ET.SubElement(root, 'body')
    body.set('type','box')
    body.set('dim','8 2 2')
    body.set('name','box'+str(i))
    x = ET.SubElement(body, 'x')
    x.text =  "0.0 "+str(y)+" 0.0"
    R = ET.SubElement(body, 'R')
    R.text = "0 -1 0 0.0"

filename = "pile.xml"
print("Generated file: "+filename)
file = open(filename, "w")
file.write(ET.tostring(root))
file.close
