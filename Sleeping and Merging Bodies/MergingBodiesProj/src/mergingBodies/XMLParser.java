package mergingBodies;

import org.w3c.dom.*;
import org.xml.sax.SAXException;

import javax.xml.parsers.*;
import java.io.*;

public class XMLParser {
	
	private RigidBodySystem system;
	private Document document = null;
	private Element eElement;
	
	public void parse(RigidBodySystem system, String filename) {
		
       	String ext = filename.substring(filename.length()-3);
       	filename = filename.replaceFirst(ext,"xml");
	
		this.system = system;
		
		DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
		DocumentBuilder builder = null;
		try {
			builder = factory.newDocumentBuilder();
		} catch (ParserConfigurationException e) {
			e.printStackTrace();
		}
		 
		try {
			document = builder.parse(new File(filename));
		} catch (SAXException | IOException e) {
			// No corresponding xml file
			return;
		}
		 
		document.getDocumentElement().normalize();
		 
		System.out.println("Load XML file");
		System.out.println("-------------");
		processCollision();
		processBody();
		System.out.println("-------------");
	}
	
	protected void processCollision() {
		NodeList nList = document.getElementsByTagName("collision");
		for (int temp = 0; temp < nList.getLength(); temp++)
		{
			System.out.println("Collision node not implemented yet");
			System.out.println("");
		}
	}
	
	protected void processBody() {
		NodeList nList = document.getElementsByTagName("body");
		for (int temp = 0; temp < nList.getLength(); temp++)
		{
			Node node = nList.item(temp);
			if (node.getNodeType() == Node.ELEMENT_NODE)
			{
				eElement = (Element) node;
				int index = Integer.parseInt(eElement.getAttribute("id"));
				
				RigidBody body = null;
				if (system.bodies.size()>index)
					body = system.bodies.get(index);
					
				if (body == null) {
					System.out.println("Body id : "    + eElement.getAttribute("id") + " is invalid");
					System.out.println("");
					continue;
				}
				
				System.out.println("Body id : "    + eElement.getAttribute("id"));

				// TODO: look if there's a way to make correspond the attribute to the variable automatically
				String[] attributes = {"x", "theta", "v", "omega"};
				
				for (String attribute : attributes) {
					Node n = eElement.getElementsByTagName(attribute).item(0);
					if ( n != null) {
						System.out.println(" "+attribute+" : "  + n.getTextContent());
						String[] values = n.getTextContent().split("\\s+");
						switch(attribute) {
							case "x":
								body.x.set(Double.parseDouble(values[0]), Double.parseDouble(values[1]));
								break;
							case "theta":
								body.theta = Double.parseDouble(values[0]);
								break;
							case "v":
								body.v.set(Double.parseDouble(values[0]), Double.parseDouble(values[1]));
								break;
							case "omega":
								body.omega = Double.parseDouble(values[0]);
								break;
							default:
						}
					}
				}
				
				
				System.out.println("");
			}
		}
	}
}
