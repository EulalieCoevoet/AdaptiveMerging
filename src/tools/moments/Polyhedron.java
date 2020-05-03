/*
 * Created on 15-Dec-2004
 */
package tools.moments;

import java.util.LinkedList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;

import static tools.moments.VolInt.X;
import static tools.moments.VolInt.Y;
import static tools.moments.VolInt.Z;

/**
 * This class defines a polyhedron, and provides a means for classes which
 * represent polyhedrons (such as meshes) to easily build a representation of
 * themselves that can be passed to VolInt. <p>
 * <p>
 * Faces are added with the addFace function.  Note that order of vertices
 * in the face is important as order determines inward or outward facing
 * polygons.  VolInt will likely do bad things if you're polyhedron is 
 * inside out.
 * 
 * 
 */
public class Polyhedron {

    /**
     * inner class for keeping track of faces 
     */
    public static class Face {

        /** Number of vertices in the face (always triangles) */
        public int numVerts = 3;

        /** face normal */
        public double norm[] = new double[3];

        /** face area ? */
        public double w;

        /** vertex positions */
        public double verts[][] = new double[3][3];

    }

    /**
     * The faces of this polyhedron.
     */
    public List<Face> faces;

    /**
     * A count of the number of degenerate triangles that were encountered
     */
    public int numDegenerateTriangles = 0;
    
    /**
     * Create a new polyhedron. You must call addFace for all faces of the
     * polyhedron before giving this polyhedron object to VolInt.
     */
    public Polyhedron() {
        faces = new LinkedList<Face>();
    }

    /**
     * Add a face to this polyhedron
     * @param p1
     * @param p2
     * @param p3
     */
    public void addFace(double[] p1, double[] p2, double[] p3) {
        if (p1.length != 3 || p2.length != 3 || p3.length != 3) {
            System.out
                    .println("Error:  VolInt.java does not currently handle objects with non-triangular faces");
            return;
        }
        Face f = new Face();
        f.verts[0][X] = p1[0];
        f.verts[0][Y] = p1[1];
        f.verts[0][Z] = p1[2];
        f.verts[1][X] = p2[0];
        f.verts[1][Y] = p2[1];
        f.verts[1][Z] = p2[2];
        f.verts[2][X] = p3[0];
        f.verts[2][Y] = p3[1];
        f.verts[2][Z] = p3[2];
        addFaceHelper(f);
    }

    /**
     * Add a face to this polyhedron
     * @param p1
     * @param p2
     * @param p3
     */
    public void addFace(float[] p1, float[] p2, float[] p3) {
        if (p1.length != 3 || p2.length != 3 || p3.length != 3) {
            System.out
                    .println("Error:  VolInt.java does not currently handle objects with non-triangular faces");
            return;
        }
        Face f = new Face();
        f.verts[0][X] = p1[0];
        f.verts[0][Y] = p1[1];
        f.verts[0][Z] = p1[2];
        f.verts[1][X] = p2[0];
        f.verts[1][Y] = p2[1];
        f.verts[1][Z] = p2[2];
        f.verts[2][X] = p3[0];
        f.verts[2][Y] = p3[1];
        f.verts[2][Z] = p3[2];
        addFaceHelper(f);
    }

    /**
     * Add a face to this polyhedron
     * @param p1
     * @param p2
     * @param p3
     */
    public void addFace(Point3d p1, Point3d p2, Point3d p3) {        
        Face f = new Face();
        f.verts[0][X] = p1.x;
        f.verts[0][Y] = p1.y;
        f.verts[0][Z] = p1.z;
        f.verts[1][X] = p2.x;
        f.verts[1][Y] = p2.y;
        f.verts[1][Z] = p2.z;
        f.verts[2][X] = p3.x;
        f.verts[2][Y] = p3.y;
        f.verts[2][Z] = p3.z;
        addFaceHelper(f);
    }
    
    /**
     * Add a face to this polyhedron
     * @param p1
     * @param p2
     * @param p3
     */
    public void addFace(Tuple3f p1, Tuple3f p2, Tuple3f p3) {
        Face f = new Face();
        f.verts[0][X] = p1.x;
        f.verts[0][Y] = p1.y;
        f.verts[0][Z] = p1.z;
        f.verts[1][X] = p2.x;
        f.verts[1][Y] = p2.y;
        f.verts[1][Z] = p2.z;
        f.verts[2][X] = p3.x;
        f.verts[2][Y] = p3.y;
        f.verts[2][Z] = p3.z;
        addFaceHelper(f);
    }

    /**
     * Add a face to this polyhedron
     * @param p1
     * @param p2
     * @param p3
     */
    public void addFace(Tuple3d p1, Tuple3d p2, Tuple3d p3) {
        Face f = new Face();
        f.verts[0][X] = p1.x;
        f.verts[0][Y] = p1.y;
        f.verts[0][Z] = p1.z;
        f.verts[1][X] = p2.x;
        f.verts[1][Y] = p2.y;
        f.verts[1][Z] = p2.z;
        f.verts[2][X] = p3.x;
        f.verts[2][Y] = p3.y;
        f.verts[2][Z] = p3.z;
        addFaceHelper(f);
    }

    /**
     * Add a face to this polyhedron
     * @param p
     */
    public void addFace(Tuple3f[] p) {
        if (p.length != 3) {
            System.out
                    .println("Error:  VolInt.java does not currently handle objects with non-triangular faces");
            return;
        }
        Face f = new Face();
        for (int j = 0; j < 3; j++) {
            f.verts[j][X] = p[j].x;
            f.verts[j][Y] = p[j].y;
            f.verts[j][Z] = p[j].z;
        }
        addFaceHelper(f);
    }

    /**
     * Add a face to this polyhedron
     * @param p
     */
    public void addFace(Tuple3d[] p) {
        if (p.length != 3) {
            System.out
                    .println("Error:  VolInt.java does not currently handle objects with non-triangular faces");
            return;
        }
        Face f = new Face();
        for (int j = 0; j < 3; j++) {
            f.verts[j][X] = p[j].x;
            f.verts[j][Y] = p[j].y;
            f.verts[j][Z] = p[j].z;
        }
        addFaceHelper(f);
    }

    private void addFaceHelper(Face f) {
        /* compute face normal and offset w from first 3 vertices */
        double dx1 = f.verts[1][X] - f.verts[0][X];
        double dy1 = f.verts[1][Y] - f.verts[0][Y];
        double dz1 = f.verts[1][Z] - f.verts[0][Z];
        double dx2 = f.verts[2][X] - f.verts[1][X];
        double dy2 = f.verts[2][Y] - f.verts[1][Y];
        double dz2 = f.verts[2][Z] - f.verts[1][Z];
        double nx = dy1 * dz2 - dy2 * dz1;
        double ny = dz1 * dx2 - dz2 * dx1;
        double nz = dx1 * dy2 - dx2 * dy1;
        double len = Math.sqrt(nx * nx + ny * ny + nz * nz);
        if ( len == 0 ) {
            numDegenerateTriangles++;            
            return;
        }
        f.norm[X] = nx / len;
        f.norm[Y] = ny / len;
        f.norm[Z] = nz / len;
        f.w = -f.norm[X] * f.verts[0][X] - f.norm[Y] * f.verts[0][Y]
                - f.norm[Z] * f.verts[0][Z];
        faces.add(f);
    }

}