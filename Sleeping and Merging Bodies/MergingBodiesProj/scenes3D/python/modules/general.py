import xml.etree.ElementTree as ET

def body(root, type, name, position="0 0 0", orientation="0. -1 0. 0.", velocity="0. 0. 0.", omega="0. 0. 0.", dim="2 2 2", obj=None, radius="1", scale="1", pinned="false", magnetic="false", restitution=None, friction=None, color=None):
    # Fixed body
    body = ET.SubElement(root, 'body')
    body.set('type', type)
    body.set('name', name)
    body.set('scale', scale)

    if(type == "box"):
        body.set('dim', dim)
    elif (type == "sphere"):
        body.set('r', radius)
    elif ((type == "composite") and (obj is not None)):
        body.set('obj', obj)
    elif ((type == "mesh")):
        body.set('obj', obj)

    com = ET.SubElement(body, 'x')
    com.text = position
    R = ET.SubElement(body, 'R')
    R.text = orientation
    vel = ET.SubElement(body, 'v')
    vel.text = velocity
    ome = ET.SubElement(body, 'omega')
    ome.text = omega
    pin = ET.SubElement(body, 'pinned')
    pin.text = pinned
    magn = ET.SubElement(body, 'magnetic')
    magn.text = magnetic
    if(restitution!=None):
        rest = ET.SubElement(body, 'restitution')
        rest.text = restitution
    if(friction!=None):
        fric = ET.SubElement(body, 'friction')
        fric.text = friction
    if(color!=None):
        col = ET.SubElement(body, 'col')
        col.text = color

    return body
