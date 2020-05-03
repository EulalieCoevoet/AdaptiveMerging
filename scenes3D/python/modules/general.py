import xml.etree.ElementTree as ET

def body(root, type, name, position="0 0 0", orientation="0. -1 0. 0.", velocity="0. 0. 0.", omega="0. 0. 0.", dim="2 2 2", obj=None, st=None, radius="1", scale="1", pinned="false", magnetic="false", density="1", restitution=None, friction=None, color=None):
    # Fixed body
    body = ET.SubElement(root, 'body')
    body.set('type', type)
    body.set('name', name)
    body.set('scale', scale)
    body.set('density', density)

    if(type == "box"):
        body.set('dim', dim)
    elif (type == "sphere"):
        body.set('r', radius)
    elif ((type == "composite") and (obj is not None)):
        body.set('obj', obj)
    elif ((type == "mesh")):
        body.set('obj', obj)
        body.set("st",st)

    prop = ET.SubElement(body, 'x')
    prop.text = position
    prop = ET.SubElement(body, 'R')
    prop.text = orientation
    prop = ET.SubElement(body, 'v')
    prop.text = velocity
    prop = ET.SubElement(body, 'omega')
    prop.text = omega
    prop = ET.SubElement(body, 'pinned')
    prop.text = pinned
    prop = ET.SubElement(body, 'magnetic')
    prop.text = magnetic
    if(restitution!=None):
        prop = ET.SubElement(body, 'restitution')
        prop.text = restitution
    if(friction!=None):
        prop = ET.SubElement(body, 'friction')
        prop.text = friction
    if(color!=None):
        prop = ET.SubElement(body, 'col')
        prop.text = color

    return body
