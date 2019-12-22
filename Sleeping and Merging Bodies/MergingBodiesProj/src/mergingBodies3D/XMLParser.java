package mergingBodies3D;

import org.w3c.dom.*;
import org.xml.sax.SAXException;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;
import javax.xml.parsers.*;
import java.io.*;
import java.util.ArrayList;

public class XMLParser {
	
	private RigidBodySystem system;
	private Document document = null;
	private Element eElement;
	
	/**
	 * Method to call to parse the XML file
	 * @param system
	 * @param filename
	 */
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
			e.printStackTrace();
			return;
		}
		 
		document.getDocumentElement().normalize();
		 
		System.out.println("Load XML file");
		System.out.println("-------------");
		parseCollision(); 
		parseBody();
		System.out.println("-------------");
	}
	
	/**
	 * Parse collision node to set parameters common to ALL bodies?
	 * TODO: should probably have these adjust interface slider values for restitution and friction
	 * And those parameters should perhaps be available as overrides or to modulate the restition and
	 * friction parameters of bodies. 
	 */
	private void parseCollision() {
		NodeList nList = document.getElementsByTagName("collision");
		for (int temp = 0; temp < nList.getLength(); temp++) {
			Node node = nList.item(temp);
			if (node.getNodeType() == Node.ELEMENT_NODE) {
				eElement = (Element) node;
				String[] attributes = {"feedbackStiffness", "restitution", "friction"};
				for (String attribute : attributes) {
					Node n = eElement.getElementsByTagName(attribute).item(0);
					if ( n != null) {
						System.out.println(" "+attribute+" : "  + n.getTextContent());
						String[] values = n.getTextContent().split("\\s+");
						switch(attribute) {
							case "feedbackStiffness":
								system.collision.feedbackStiffness.setValue(Double.parseDouble(values[0]));
								break;
							case "restitution":
								system.collision.restitution.setValue(Double.parseDouble(values[0]));
								for (RigidBody body: system.bodies)
									body.restitution = system.collision.restitution.getValue();
								break;
							case "friction":
								system.collision.friction.setValue(Double.parseDouble(values[0]));
								for (RigidBody body: system.bodies)
									body.friction = system.collision.friction.getValue();
								break;
							default:
						}
					}
				}
				System.out.println("");
			}
		}
	}
	
	/**
	 * Parse body node
	 */
	private void parseBody() {
		NodeList nList = document.getElementsByTagName("body");
		for (int temp = 0; temp < nList.getLength(); temp++) {
			Node node = nList.item(temp);
			if (node.getNodeType() == Node.ELEMENT_NODE) {
				eElement = (Element) node;
				String type = eElement.getAttribute("type");
				String name = eElement.getAttribute("name");
				if ( type.equalsIgnoreCase("box") ) {
					createBox( name, eElement );
				} else if ( type.equalsIgnoreCase("plane") ) {
					createPlane( name, eElement );
				} else if ( type.equalsIgnoreCase("sphere") ) {
					createSphere( name, eElement );
				}
			}
		}
	}
	
	private void createBox( String name, Element eElement ) {

	}

	private void createPlane( String name, Element eElement ) {
		Point3d p = new Point3d();
		Vector3d n = new Vector3d();
		p.set( t3d( eElement.getAttribute("p") ) );
		n.set( t3d( eElement.getAttribute("n") ) );
		RigidBody b = new PlaneRigidBody(p, n); // ALWAYS pinned, 
		setCommonAttributes( b, eElement ); // can still adjust friction and restitution
		system.bodies.add( b );
	}

	private void createSphere( String name, Element eElement ) {
		double r = Double.parseDouble( eElement.getAttribute("r") );
		Matrix3d angularMass = new Matrix3d();
		angularMass.setIdentity();
		double massLinear = 4/3*Math.PI*r*r*r;
		angularMass.mul( 2/5*massLinear*r*r );
		ArrayList<Point3d> bbB = new ArrayList<Point3d>();
		bbB.add( new Point3d( -r, -r, -r ) );
		bbB.add( new Point3d( -r, -r,  r ) );
		bbB.add( new Point3d( -r,  r, -r ) );
		bbB.add( new Point3d( -r,  r,  r ) );
		bbB.add( new Point3d(  r, -r, -r ) );
		bbB.add( new Point3d(  r, -r,  r ) );
		bbB.add( new Point3d(  r,  r, -r ) );
		bbB.add( new Point3d(  r,  r,  r ) );		
		RigidBody body = new RigidBody(massLinear, angularMass, false, bbB );
		setCommonAttributes( body, eElement );
		body.updateTransformations();
        //body.root = new BVNode( , body );
        body.geom = new RigidBodyGeomSphere( r );		
	}

	/**
	 * Sets the attributes common to all bodies here
	 * @param body
	 * @param eElement
	 */
	private void setCommonAttributes( RigidBody body, Element eElement ) {
		NodeList nodeList = eElement.getChildNodes();
        for ( int i = 0; i < nodeList.getLength(); i++ ) {
        	Node n = nodeList.item(i);
            // skip all text, just process the ELEMENT_NODEs
            if ( n.getNodeType() != Node.ELEMENT_NODE ) continue;
            String attribute = n.getNodeName();
            String[] values = n.getTextContent().split("\\s+");        
			System.out.println(" "+attribute+" : "  + n.getTextContent());
			if ( attribute.equalsIgnoreCase("x") ) {
				body.x.set( t3d( values ) );
			} else if ( attribute.equalsIgnoreCase("v") ) {
				body.v.set( t3d( values ) );
			} else if ( attribute.equalsIgnoreCase("omega") ) {
				body.omega.set( t3d( values ) );
			} else if ( attribute.equalsIgnoreCase("restitution") ) {
				body.restitution = Double.parseDouble(values[0]);
			} else if ( attribute.equalsIgnoreCase("friction") ) {
				body.friction = Double.parseDouble(values[0]);
			} else if ( attribute.equalsIgnoreCase("temporarilyPinned") ) {			
				//body.temporarilyPinned = Boolean.parseBoolean(values[0]);
			} else if ( attribute.equalsIgnoreCase("pinned") ) {	
				body.pinned = Boolean.parseBoolean(values[0]);
			} // could complain about unknown attributes here
		}		
		System.out.println("");
	}
	
	private Tuple3d t3d( String[] values ) {
		Vector3d v = new Vector3d();
		v.x = Double.parseDouble(values[0]);
		v.y = Double.parseDouble(values[1]);
		v.z = Double.parseDouble(values[2]);
		return v;
	}

	private Tuple3d t3d( String s ) {
		Vector3d v = new Vector3d();
        String[] values = s.split("\\s+");        
		v.x = Double.parseDouble(values[0]);
		v.y = Double.parseDouble(values[1]);
		v.z = Double.parseDouble(values[2]);
		return v;
	}

}
