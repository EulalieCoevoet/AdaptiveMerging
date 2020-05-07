/*
 Revision history

 26 Jan 1996    Program creation.

 3 Aug 1996 Corrected bug arising when polyhedron density
 is not 1.0.  Changes confined to function main().
 Thanks to Zoran Popovic for catching this one.

 27 May 1997     Corrected sign error in translation of inertia
 product terms to center of mass frame.  Changes
 confined to function main().  Thanks to
 Chris Hecker.
 */

package tools.moments;

import javax.vecmath.Matrix3d;
import javax.vecmath.Tuple3d;

import tools.moments.Polyhedron.Face;

/**
 * A port of Brian Mirtich's volInt.c to Java.
 * <p>
 * Ported by Mike Cline
 * 
 * <p>
 * This version is modified to work with the subdivision code. The interface is
 * a single static method, computeMassProperties. See the comment for this
 * method for details.
 * 
 * <p>
 * Original copyright notice appears below
 * 
 * <p>
 * cline cs.ubc.ca
 * <p>
 * May 25th, 2001
 * 
 * <p>
 * Modified again to take triangle soups. Note that bad results will occur if
 * non manifold soups are used!
 * 
 * <p>
 * kry cs.rutgers.edu
 * <p>
 * 2004-12-15
 * 
 * <p>
 * volInt.c
 * 
 * <p>
 * This code computes volume integrals needed for determining mass properties of
 * polyhedral bodies.
 * 
 * <p>
 * For more information, see the accompanying README file, and the paper
 * 
 * <p>
 * Brian Mirtich, "Fast and Accurate Computation of Polyhedral Mass Properties,"
 * journal of graphics tools, volume 1, number 1, 1996.
 * 
 * <p>
 * This source code is public domain, and may be used in any way, shape or form,
 * free of charge.
 * 
 * <p>
 * Copyright 1995 by Brian Mirtich
 * 
 * <p>
 * mirtich cs.berkeley.edu
 * <p>
 * http://www.cs.berkeley.edu/~mirtich
 * 
 */

public class VolInt {

	static final int X = 0;

	static final int Y = 1;

	static final int Z = 2;

	/** 
	 * @param x
	 * @return Returns the square of x
	 */
	private static double SQR(double x) {
		return x * x;
	}

	/** 
	 * @param x
	 * @return Returns the cube of x
	 */
	private static double CUBE(double x) {
		return x * x * x;
	}
	
	/*
	 * ============================================================================
	 * globals
	 * ============================================================================
	 */

	private static int A; /* alpha */

	private static int B; /* beta */

	private static int C; /* gamma */

	/* projection integrals */
	private static double P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;

	/* face integrals */
	private static double Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab,
			Fbbc, Fcca;

	/* volume integrals */
	private static double T0;

	private static double T1[] = new double[3];

	private static double T2[] = new double[3];

	private static double TP[] = new double[3];

	/*
	 * ============================================================================
	 * compute mass properties
	 * ============================================================================
	 */

	/**
	 * compute various integrations over projection of face 
	 * @param f 
	 */
	private static void compProjectionIntegrals(Face f) {
		double a0, a1, da;
		double b0, b1, db;
		double a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
		double a1_2, a1_3, b1_2, b1_3;
		double C1, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
		double Cab, Kab, Caab, Kaab, Cabb, Kabb;
		int i;

		P1 = Pa = Pb = Paa = Pab = Pbb = Paaa = Paab = Pabb = Pbbb = 0.0;

		for (i = 0; i < f.numVerts; i++) {
			a0 = f.verts[i][A];
			b0 = f.verts[i][B];
			a1 = f.verts[(i + 1) % f.numVerts][A];
			b1 = f.verts[(i + 1) % f.numVerts][B];
			da = a1 - a0;
			db = b1 - b0;
			a0_2 = a0 * a0;
			a0_3 = a0_2 * a0;
			a0_4 = a0_3 * a0;
			b0_2 = b0 * b0;
			b0_3 = b0_2 * b0;
			b0_4 = b0_3 * b0;
			a1_2 = a1 * a1;
			a1_3 = a1_2 * a1;
			b1_2 = b1 * b1;
			b1_3 = b1_2 * b1;

			C1 = a1 + a0;
			Ca = a1 * C1 + a0_2;
			Caa = a1 * Ca + a0_3;
			Caaa = a1 * Caa + a0_4;
			Cb = b1 * (b1 + b0) + b0_2;
			Cbb = b1 * Cb + b0_3;
			Cbbb = b1 * Cbb + b0_4;
			Cab = 3 * a1_2 + 2 * a1 * a0 + a0_2;
			Kab = a1_2 + 2 * a1 * a0 + 3 * a0_2;
			Caab = a0 * Cab + 4 * a1_3;
			Kaab = a1 * Kab + 4 * a0_3;
			Cabb = 4 * b1_3 + 3 * b1_2 * b0 + 2 * b1 * b0_2 + b0_3;
			Kabb = b1_3 + 2 * b1_2 * b0 + 3 * b1 * b0_2 + 4 * b0_3;

			P1 += db * C1;
			Pa += db * Ca;
			Paa += db * Caa;
			Paaa += db * Caaa;
			Pb += da * Cb;
			Pbb += da * Cbb;
			Pbbb += da * Cbbb;
			Pab += db * (b1 * Cab + b0 * Kab);
			Paab += db * (b1 * Caab + b0 * Kaab);
			Pabb += da * (a1 * Cabb + a0 * Kabb);
		}

		P1 /= 2.0;
		Pa /= 6.0;
		Paa /= 12.0;
		Paaa /= 20.0;
		Pb /= -6.0;
		Pbb /= -12.0;
		Pbbb /= -20.0;
		Pab /= 24.0;
		Paab /= 60.0;
		Pabb /= -60.0;
	}

	private static void compFaceIntegrals(Face f) {
		double w;
		double n[];
		double k1, k2, k3, k4;

		compProjectionIntegrals(f);

		w = f.w;
		n = f.norm;
		k1 = 1 / n[C];
		k2 = k1 * k1;
		k3 = k2 * k1;
		k4 = k3 * k1;

		Fa = k1 * Pa;
		Fb = k1 * Pb;
		Fc = -k2 * (n[A] * Pa + n[B] * Pb + w * P1);

		Faa = k1 * Paa;
		Fbb = k1 * Pbb;
		Fcc = k3
				* (SQR(n[A]) * Paa + 2 * n[A] * n[B] * Pab + SQR(n[B]) * Pbb + w
						* (2 * (n[A] * Pa + n[B] * Pb) + w * P1));

		Faaa = k1 * Paaa;
		Fbbb = k1 * Pbbb;
		Fccc = -k4
				* (CUBE(n[A])
						* Paaa
						+ 3
						* SQR(n[A])
						* n[B]
						* Paab
						+ 3
						* n[A]
						* SQR(n[B])
						* Pabb
						+ CUBE(n[B])
						* Pbbb
						+ 3
						* w
						* (SQR(n[A]) * Paa + 2 * n[A] * n[B] * Pab + SQR(n[B])
								* Pbb) + w * w
						* (3 * (n[A] * Pa + n[B] * Pb) + w * P1));

		Faab = k1 * Paab;
		Fbbc = -k2 * (n[A] * Pabb + n[B] * Pbbb + w * Pbb);
		Fcca = k3
				* (SQR(n[A]) * Paaa + 2 * n[A] * n[B] * Paab + SQR(n[B]) * Pabb + w
						* (2 * (n[A] * Paa + n[B] * Pab) + w * Pa));
	}

	private static void compVolumeIntegrals(Polyhedron p) {		
		double nx, ny, nz;
		T0 = T1[X] = T1[Y] = T1[Z] = T2[X] = T2[Y] = T2[Z] = TP[X] = TP[Y] = TP[Z] = 0;
		for ( Face f : p.faces ) {
            if ( f.norm[X] == 0 && f.norm[Y] == 0 && f.norm[Z] == 0 ) {
                System.err.println("problems!");
            }
			nx = Math.abs(f.norm[X]);
			ny = Math.abs(f.norm[Y]);
			nz = Math.abs(f.norm[Z]);
			if (nx > ny && nx > nz) {
				C = X;
			} else {
				C = (ny > nz) ? Y : Z;
			}
			A = (C + 1) % 3;
			B = (A + 1) % 3;
			compFaceIntegrals(f);
			T0 += f.norm[X] * ((A == X) ? Fa : ((B == X) ? Fb : Fc));
			T1[A] += f.norm[A] * Faa;
			T1[B] += f.norm[B] * Fbb;
			T1[C] += f.norm[C] * Fcc;
			T2[A] += f.norm[A] * Faaa;
			T2[B] += f.norm[B] * Fbbb;
			T2[C] += f.norm[C] * Fccc;
			TP[A] += f.norm[A] * Faab;
			TP[B] += f.norm[B] * Fbbc;
			TP[C] += f.norm[C] * Fcca;
		}
		T1[X] /= 2;
		T1[Y] /= 2;
		T1[Z] /= 2;
		T2[X] /= 3;
		T2[Y] /= 3;
		T2[Z] /= 3;
		TP[X] /= 2;
		TP[Y] /= 2;
		TP[Z] /= 2;
	}

    /**
     * Controls the verbosity of output.
     */
    static public Boolean quiet = true;
    
	/**
 	 * Compute the mass and inertia matrix for an object, then translate the
	 * mesh so that the origin of body coordinates corresponds to the centre of
	 * mass of the object. The object is assumed to be a constant density, given
	 * by the second parameter to this method. The value returned by this method
	 * is the mass of the object.
	 * @param p			the polyhedron defining the volume
	 * @param density	the density
	 * @param inertia	to be filled with inertia tensor
	 * @param com		to be filled with center of mass
	 * @return	the mass (or volume if density is given as 1)
	 */
	public static double computeMassProperties( Polyhedron p, double density,
			Matrix3d inertia, Tuple3d com) {
		
		double mass;
		double r[] = new double[3]; /* center of mass */
		double J[][] = new double[3][3]; /* inertia tensor */

		compVolumeIntegrals(p);

        if ( ! quiet ) {
    		System.out.println("\nT1 =   " + T0);
    
    		System.out.println("Tx =   " + T1[X]);
    		System.out.println("Ty =   " + T1[Y]);
    		System.out.println("Tz =   " + T1[Z]);
    
    		System.out.println("Txx =  " + T2[X]);
    		System.out.println("Tyy =  " + T2[Y]);
    		System.out.println("Tzz =  " + T2[Z]);
    
    		System.out.println("Txy =  " + TP[X]);
    		System.out.println("Tyz =  " + TP[Y]);
    		System.out.println("Tzx =  " + TP[Z]);
        }
		mass = density * T0;

		/* compute center of mass */
		r[X] = T1[X] / T0;
		r[Y] = T1[Y] / T0;
		r[Z] = T1[Z] / T0;

		/* compute inertia tensor */
		J[X][X] = density * (T2[Y] + T2[Z]);
		J[Y][Y] = density * (T2[Z] + T2[X]);
		J[Z][Z] = density * (T2[X] + T2[Y]);
		J[X][Y] = J[Y][X] = -density * TP[X];
		J[Y][Z] = J[Z][Y] = -density * TP[Y];
		J[Z][X] = J[X][Z] = -density * TP[Z];

		/* translate inertia tensor to center of mass */
		J[X][X] -= mass * (r[Y] * r[Y] + r[Z] * r[Z]);
		J[Y][Y] -= mass * (r[Z] * r[Z] + r[X] * r[X]);
		J[Z][Z] -= mass * (r[X] * r[X] + r[Y] * r[Y]);
		J[X][Y] = J[Y][X] += mass * r[X] * r[Y];
		J[Y][Z] = J[Z][Y] += mass * r[Y] * r[Z];
		J[Z][X] = J[X][Z] += mass * r[Z] * r[X];

        if ( !quiet ) {
    		System.out.println("center of mass:  " + r[X] + " " + r[Y] + " " + r[Z]);
    
    		System.out.println("inertia tensor with origin at c.o.m. :\n");
    		System.out.println(J[X][X] + " " + J[X][Y] + " " + J[X][Z]);
    		System.out.println(J[Y][X] + " " + J[Y][Y] + " " + J[Y][Z]);
    		System.out.println(J[Z][X] + " " + J[Z][Y] + " " + J[Z][Z]);
        }

        inertia.m00 = J[X][X];
		inertia.m01 = J[X][Y];
		inertia.m02 = J[X][Z];
		inertia.m10 = J[Y][X];
		inertia.m11 = J[Y][Y];
		inertia.m12 = J[Y][Z];
		inertia.m20 = J[Z][X];
		inertia.m21 = J[Z][Y];
		inertia.m22 = J[Z][Z];

		com.set(r[X], r[Y], r[Z]);

		return mass;
	}
}
