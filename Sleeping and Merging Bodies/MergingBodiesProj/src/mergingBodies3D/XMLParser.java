package mergingBodies3D;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import tools.moments.PolygonSoup;
import tools.moments.Polyhedron;
import tools.moments.Vertex;
import tools.moments.VolInt;

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
		 
		parseCollision(); 
		parseBody();
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
			}
		}
	}
	
	/**
	 * Parse body node
	 */
	private void parseBody() {
		NodeList root = document.getElementsByTagName("root");
		Node rootNode = root.item(0);
		NodeList nList = rootNode.getChildNodes();
		for (int temp = 0; temp < nList.getLength(); temp++) {
			Node node = nList.item(temp);
			if ( node.getNodeType() == Node.ELEMENT_NODE && node.getNodeName().equalsIgnoreCase("body") ) {
				eElement = (Element) node;
				String type = eElement.getAttribute("type");
				String name = eElement.getAttribute("name");
				if ( type.equalsIgnoreCase("box") ) {
					RigidBody body = createBox( name, eElement );
					system.bodies.add( body );
				} else if ( type.equalsIgnoreCase("plane") ) {
					RigidBody body = createPlane( name, eElement );
					system.bodies.add( body );
				} else if ( type.equalsIgnoreCase("sphere") ) {
					RigidBody body = createSphere( name, eElement );
					system.bodies.add( body );
				} else if ( type.equalsIgnoreCase("mesh") ) {
					RigidBody body = createMesh( name, eElement );
					system.bodies.add( body );
				} else if ( type.equalsIgnoreCase("composite") ) {
					RigidBody body = createComposite( name, eElement );
					system.bodies.add( body );
				}
			}
		}
	}
	
	/**
	 * Probably want to use composites sparingly, as they will probably be more expensive for
	 * collision detection... or might likewise want to make sure there is a bounding sphere 
	 * at the COM for trivial tests.
	 * Note that a possible extension would be to allow composites to reference bodies by name
	 * for inclusion, to avoid reloading or recreating bodies in the case where many composites 
	 * are desired?
	 * 
	 * @param name
	 * @param eElement
	 */
	private RigidBody createComposite( String name, Element eElement ) {
		RigidBodyGeomComposite compositeGeom = new RigidBodyGeomComposite();

		// get the bodies, harvest their geometries, and compute the composite
		//NodeList nodeList = eElement.getChildNodes();
		NodeList nList = eElement.getChildNodes();

		// we need to keep them to the end to properly compute inertia
		// or likewise, it might be convenient to preserve this composite body
		// list for the sake of easy drawing and collision detection
		// ( i.e., store it with the geometry )
		ArrayList<RigidBody> bodies = new ArrayList<RigidBody>();
		
		for (int temp = 0; temp < nList.getLength(); temp++) {
			Node node = nList.item(temp);
			if (node.getNodeType() == Node.ELEMENT_NODE && node.getNodeName().equalsIgnoreCase("body") ) {
				eElement = (Element) node;
				String type = eElement.getAttribute("type");
				String name2 = eElement.getAttribute("name");
				RigidBody sbody = null;
				if ( type.equalsIgnoreCase("box") ) {
					sbody = createBox( name2, eElement );
				} else if ( type.equalsIgnoreCase("plane") ) {
					sbody = createPlane( name2, eElement );
				} else if ( type.equalsIgnoreCase("sphere") ) {
					sbody = createSphere( name2, eElement );
				} else if ( type.equalsIgnoreCase("mesh") ) {
					sbody = createMesh( name2, eElement );
				}
				if ( sbody != null ) {
					bodies.add( sbody );
				}			
			}
        }

		Vector3d ll = new Vector3d();
		Vector3d ur = new Vector3d();
		double massLinear = 0;
		Matrix3d massAngular = new Matrix3d();

		Point3d com = new Point3d();
		for ( RigidBody b : bodies ) {
			massLinear += b.massLinear;		
			com.scaleAdd( b.massLinear, b.x, com );
			compositeGeom.bodies.add( b );
		}
		com.scale( 1.0/massLinear );
		for ( RigidBody b : bodies ) {
			b.x.sub(com);  // ensure it draws relative to the center of mass
			b.x0.sub(com);
			b.updateTransformations();
			massAngular.add( b.massAngular );
			// should certainly have a b.x squared type term for the mass being at a distance...
//			I [p]    J  0    I   0 
//			0  I    0 mI    [p] I
//			I [p]   J   0
//			0  I   m[p] 0
//			J + I [p][p] in the upper left...
		// recall lemma 2.3: [a] = a a^T - ||a||^2 I
			double x = b.x.x;
			double y = b.x.y;
			double z = b.x.z;			
			double x2 = x*x;
			double y2 = y*y;
			double z2 = z*z;
			Matrix3d op = new Matrix3d();
			op.m00 = y2+z2; op.m01 = x*y; op.m02 = x*z;
			op.m10 = y*x; op.m11 = x2+z2; op.m12 = y*z;
			op.m20 = z*x; op.m21 = z*y; op.m22 = x2 + y2;
			op.mul( b.massLinear );
			massAngular.add( op );			
		}
		
		// TODO: THIS BOUNDING BOX IS POSSIBLY WRONG?
		for ( RigidBody b : bodies ) {
			for ( Point3d p : b.boundingBoxB ) {
				p.sub(com);
				ll.x = Math.min( p.x, ll.x );
				ll.y = Math.min( p.y, ll.y );
				ll.z = Math.min( p.z,  ll.z );
				ur.x = Math.max( p.x, ur.x );
				ur.y = Math.max( p.y, ur.y );
				ur.z = Math.max( p.z,  ur.z );
			}
		}
		
		// set the stuff on the body now!
		ArrayList<Point3d> bbB = new ArrayList<Point3d>();
		bbB.add( new Point3d( ll.x, ll.y, ll.z ) );
		bbB.add( new Point3d( ll.x, ll.y, ur.z ) );
		bbB.add( new Point3d( ll.x, ur.y, ll.z ) );
		bbB.add( new Point3d( ll.x, ur.y, ur.z ) );
		bbB.add( new Point3d( ur.x, ll.y, ll.z ) );
		bbB.add( new Point3d( ur.x, ll.y, ur.z ) );
		bbB.add( new Point3d( ur.x, ur.y, ll.z ) );
		bbB.add( new Point3d( ur.x, ur.y, ur.z ) );
		
		RigidBody body = new RigidBody( massLinear, massAngular, false, bbB );
		setCommonAttributes( body, eElement );
		body.updateTransformations();
		body.geom = compositeGeom;
		
		return body;
	}
	
	/**
	 * TODO: should include ability to change density
	 * @param name
	 * @param eElement
	 */
	private RigidBody createBox( String name, Element eElement ) {
		Vector3d s = new Vector3d( t3d( eElement.getAttribute("dim") ) );
		double density = 1;
		double massLinear = s.x*s.y*s.z * density;
		Matrix3d angularMass = new Matrix3d();
		angularMass.m00 = 1.0/12*massLinear*(s.y*s.y+s.z*s.z);
		angularMass.m11 = 1.0/12*massLinear*(s.x*s.x+s.z*s.z);
		angularMass.m22 = 1.0/12*massLinear*(s.x*s.x+s.y*s.y);				
		ArrayList<Point3d> bbB = new ArrayList<Point3d>();
		bbB.add( new Point3d( -s.x, -s.y, -s.z ) );
		bbB.add( new Point3d( -s.x, -s.y,  s.z ) );
		bbB.add( new Point3d( -s.x,  s.y, -s.z ) );
		bbB.add( new Point3d( -s.x,  s.y,  s.z ) );
		bbB.add( new Point3d(  s.x, -s.y, -s.z ) );
		bbB.add( new Point3d(  s.x, -s.y,  s.z ) );
		bbB.add( new Point3d(  s.x,  s.y, -s.z ) );
		bbB.add( new Point3d(  s.x,  s.y,  s.z ) );		
		RigidBody body = new RigidBody(massLinear, angularMass, false, bbB );
		setCommonAttributes( body, eElement );
		body.updateTransformations();
        body.geom = new RigidBodyGeomBox( s );	
        
        Vector3d tmp = new Vector3d( s );
        tmp.scale(0.5);        
        body.radius = tmp.length();
        
        // Silly to deal with boxes with spheres, but this is the quick solution for now... 
        // Assume unit size tiling, and choose centers along faces
        // ... and now in comments because we have box-box collisions
        
//        double radius = 0.5;
//        int nx = (int) Math.ceil( s.x/2/radius );
//        int ny = (int) Math.ceil( s.y/2/radius );
//        int nz = (int) Math.ceil( s.z/2/radius );
//        
//        ArrayList<BVSphere> L = new ArrayList<BVSphere>();
//        Point3d p = new Point3d();
//        
//        // stupid, but simple: loop over the volume, and only 
//        // generate spheres at surface points
//        for ( int i = 0; i < nx; i++ ) {
//        	boolean iboundary = i == 0 || i == nx-1;
//        	for ( int j = 0; j < ny; j++ ) {
//            	boolean jboundary = j == 0 || j == ny-1;
//        		for ( int k = 0; k < nz; k++ ) {
//                	boolean kboundary = k == 0 || k == nz-1;
//                	if ( !iboundary && !jboundary && !kboundary ) continue;
//            		p.x = -s.x/2 + radius + i*2*radius;
//            		p.y = -s.y/2 + radius + j*2*radius;
//            		p.z = -s.z/2 + radius + k*2*radius;
//            		L.add( new BVSphere( p, radius * Math.sqrt(2) , body ) ); // should probably be sqrt 3... but balance what is in and what is out/
//        		}
//        	}
//        }        
//        body.root = new BVNode( L, body );
        
        body.name = name;
		return body;
	}

	private RigidBody createPlane( String name, Element eElement ) {
		Point3d p = new Point3d();
		Vector3d n = new Vector3d();
		p.set( t3d( eElement.getAttribute("p") ) );
		n.set( t3d( eElement.getAttribute("n") ) );
		RigidBody b = new PlaneRigidBody(p, n); // ALWAYS pinned, 
		setCommonAttributes( b, eElement ); // can still adjust friction and restitution
		b.name = name;
		return b;
	}
	
	/**
	 * TODO: should include ability to change density
	 * @param name
	 * @param eElement
	 */
	private RigidBody createSphere( String name, Element eElement ) {
		double r = Double.parseDouble( eElement.getAttribute("r") );
		double density = 1;
		double massLinear = 4.0/3*Math.PI*r*r*r * density;
		Matrix3d angularMass = new Matrix3d();
		angularMass.setIdentity();
		angularMass.mul( 2.0/5*massLinear*r*r );
		ArrayList<Point3d> bbB = new ArrayList<Point3d>();
		bbB.add( new Point3d( -r, -r, -r ) );
		bbB.add( new Point3d( -r, -r,  r ) );
		bbB.add( new Point3d( -r,  r, -r ) );
		bbB.add( new Point3d( -r,  r,  r ) );
		bbB.add( new Point3d(  r, -r, -r ) );
		bbB.add( new Point3d(  r, -r,  r ) );
		bbB.add( new Point3d(  r,  r, -r ) );
		bbB.add( new Point3d(  r,  r,  r ) );
		// seems stupid to have a set of bounding box points for a sphere, but 
		// it is needed for how we currently do relative velocity bounds.
		RigidBody body = new RigidBody(massLinear, angularMass, false, bbB );
		setCommonAttributes( body, eElement );
        body.geom = new RigidBodyGeomSphere( r );		
		BVSphere bvSphere = new BVSphere( new Point3d(), r, body );
        body.root = new BVNode( bvSphere );
		body.name = name;
		return body;
	}
	
	private RigidBody createMesh( String name, Element eElement ) {
		double scale = Double.parseDouble( eElement.getAttribute("scale") );
		double density = 1;
		String objfname = eElement.getAttribute("obj");
		String stfname = eElement.getAttribute("st");
		
		PolygonSoup soup = new PolygonSoup( objfname );
		for ( Vertex v : soup.vertexList ) {
			v.p.scale( scale );
		}
		Polyhedron poly = soup.getPolyhedron();
		Matrix3d inertia = new Matrix3d();
		Vector3d com = new Vector3d();
		double mass = VolInt.computeMassProperties( poly, density, inertia, com );
        if ( mass < 0 ) {
        	mass = -mass;
        	System.err.println( "mesh " + objfname + " appears to be inside out" );
        }
        // translate mesh to COM
		for ( Vertex v : soup.vertexList ) {
			v.p.sub( com );
		}

		// compute the axis aligned bounding box
	    Vector3d ll = new Vector3d( Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE );
	    Vector3d ur = new Vector3d( Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE );
		for ( Vertex v : soup.vertexList ) {
			ll.x = Math.min( v.p.x, ll.x );
			ll.y = Math.min( v.p.y, ll.y );
			ll.z = Math.min( v.p.z, ll.z );
			ur.x = Math.max( v.p.x, ur.x );
			ur.y = Math.max( v.p.y, ur.y );
			ur.z = Math.max( v.p.z, ur.z );			
		}
		ArrayList<Point3d> bbB = new ArrayList<Point3d>();
		bbB.add( new Point3d( ll.x, ll.y, ll.z ) );
		bbB.add( new Point3d( ll.x, ll.y, ur.z ) );
		bbB.add( new Point3d( ll.x, ur.y, ll.z ) );
		bbB.add( new Point3d( ll.x, ur.y, ur.z ) );
		bbB.add( new Point3d( ur.x, ll.y, ll.z ) );
		bbB.add( new Point3d( ur.x, ll.y, ur.z ) );
		bbB.add( new Point3d( ur.x, ur.y, ll.z ) );
		bbB.add( new Point3d( ur.x, ur.y, ur.z ) );
		
		RigidBody body = new RigidBody( mass, inertia, false, bbB );
		setCommonAttributes( body, eElement );
        body.geom = new RigidBodyGeomMesh( soup );		
		
		// load the corresponding sphere tree and note that you 
        ArrayList<BVNode> spheres = new ArrayList<BVNode>();
        int numSpheres = 0;                
        try {
            Scanner s = new Scanner( new FileInputStream( stfname ) );            
            String line = s.nextLine();
            Scanner s2 = new Scanner( line );
            numSpheres = s2.nextInt();            
            for ( int i = 0; i < numSpheres; i++ ) {
                line = s.nextLine();            
                s2 = new Scanner( line );
                Point3d pB = new Point3d( s2.nextDouble(), s2.nextDouble(), s2.nextDouble() );
                pB.scale( scale );
                pB.sub( com );
                double radius = s2.nextDouble();
                radius *= scale;                
                BVSphere bvSphere = new BVSphere(pB, radius, body);
                BVNode sphere = new BVNode( bvSphere );
                spheres.add( sphere );
            }            
            while ( s.hasNextLine() ) {
                line = s.nextLine();
                s2 = new Scanner( line );
                int parent = s2.nextInt();
                int count = s2.nextInt();
                BVNode sphere = spheres.get( parent -1 );
                sphere.children = new BVNode[count];
                for ( int i = 0; i < count; i++ ) {
                    sphere.children[i] = spheres.get( s2.nextInt() - 1 );
                }
            }
            s.close();
            s2.close();
        } catch ( Exception e ) {
            e.printStackTrace();
        }    
        body.root = spheres.get(0);

        body.name = name;	
		return body;
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
            String tag = n.getNodeName();
            String[] values = n.getTextContent().trim().split("\\s+");        
			if ( tag.equalsIgnoreCase("x") ) {
				body.x.set( t3d( values ) );
				body.x0.set( body.x );
				body.updateTransformations();
			} else if ( tag.equalsIgnoreCase("R") ) {
				AxisAngle4d aa = new AxisAngle4d();
				aa.set( asDoubles(values) );
				body.theta.set( aa );
				body.theta0.set( aa );
				body.updateTransformations();
			} else if ( tag.equalsIgnoreCase("v") ) {
				body.v.set( t3d( values ) );
			} else if ( tag.equalsIgnoreCase("omega") ) {
				body.omega.set( t3d( values ) );
			} else if ( tag.equalsIgnoreCase("restitution") ) {
				body.restitution = Double.parseDouble(values[0]);
			} else if ( tag.equalsIgnoreCase("friction") ) {
				body.friction = Double.parseDouble(values[0]);
			} else if ( tag.equalsIgnoreCase("temporarilyPinned") ) {			
				//body.temporarilyPinned = Boolean.parseBoolean(values[0]);
			} else if ( tag.equalsIgnoreCase("pinned") ) {	
				body.pinned = Boolean.parseBoolean(values[0]);
				if ( body.pinned ) {
					body.minv = 0;
					body.jinv0.setZero();
					body.jinv.setZero();
				}
			} else if ( tag.equalsIgnoreCase("factorypart") ) {	
				body.factoryPart = Boolean.parseBoolean(values[0]);
			} else if ( tag.equalsIgnoreCase("spring") ) {
				Element e = (Element) n;
				Point3d pB = new Point3d( t3d( e.getAttribute("pB") ) );
				Spring s = new Spring( pB, body );
				body.springs.add( s );
			} else if ( tag.equalsIgnoreCase("col")) {
				double [] col = asDoubles( values );
				body.col = new float[] { (float) col[0], (float) col[1], (float) col[2], 1 };
			}
			// could complain about unknown attributes here
		}		
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
        String[] values = s.trim().split("\\s+");        
		v.x = Double.parseDouble(values[0]);
		v.y = Double.parseDouble(values[1]);
		v.z = Double.parseDouble(values[2]);
		return v;
	}
	
	private double[] asDoubles( String[] s ) {
		double[] d = new double[s.length];
		for ( int i = 0; i < s.length; i++ ) {
			d[i] = Double.parseDouble(s[i]);
		}
		return d;
	}

}
