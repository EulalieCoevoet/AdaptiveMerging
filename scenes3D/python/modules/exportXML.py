import xml.etree.ElementTree as ET
import xml.dom.minidom as md

def export(root, filename):
    print("Generated file: "+filename)
    file = open(filename, "w")

    rough_string = ET.tostring(root, 'utf-8')
    reparsed = md.parseString(rough_string)
    file.write(reparsed.toprettyxml(indent="\t"))

    file.close
    return
