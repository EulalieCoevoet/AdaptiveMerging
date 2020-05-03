import xml.etree.ElementTree as ET
from modules.box import Box
from modules.exportXML import export

root = ET.Element('root')

# Spring Default
body = Box(root, name='body1', position="0.0 0.0 0.0", dim='2 2 2')
body.addSpring("0. 0. 0.", k="10")

# Spring World
body = Box(root, name='body2', position="4.0 0.0 0.0", dim='2 2 2')
body.addSpring("0. 0. 0.", positionW="4. 10. 0.", k="10")

# Spring body body
body = Box(root, name='body3', position="8.0 0.0 0.0", dim='2 2 2')
body.addSpring("0. 0. 0.", k="10")
boat = Box(root, name='body4', position="8.0 -5.0 0.0", dim='2 2 2')
boat.addSpring("0. 0. 0.", positionB2="0. 0. 0.", k="10", body2="body3")

export(root, "../springsTest.xml")
