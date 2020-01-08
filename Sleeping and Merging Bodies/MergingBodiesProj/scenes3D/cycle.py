import xml.etree.ElementTree as ET

root = ET.Element('root')

# Fixed bodies
x = 0
for i in range(2):
    if i==1 : x += 18
    body = ET.SubElement(root, 'body')
    body.set('type','box')
    body.set('dim','8 2 8')
    body.set('name','box'+str(i))
    com = ET.SubElement(body, 'x')
    com.text =  str(x)+" 0.0 0.0"
    R = ET.SubElement(body, 'R')
    R.text = "0 -1 0 0.0"
    pinned = ET.SubElement(body, 'pinned')
    pinned.text = "true"

# Pile bodies
y = 0
x = 4
for i in range(8):
    y += 2.5
    if i==4 :
        x += 10
        y = 2.5
    body = ET.SubElement(root, 'body')
    body.set('type','box')
    body.set('dim','8 2 8')
    body.set('name','box'+str(i))
    com = ET.SubElement(body, 'x')
    com.text = str(x)+" "+str(y)+" 0.0"
    R = ET.SubElement(body, 'R')
    R.text = "0 -1 0 0.0"

filename = "cycle.xml"
print("Generated file: "+filename)
file = open(filename, "w")
file.write(ET.tostring(root))
file.close
