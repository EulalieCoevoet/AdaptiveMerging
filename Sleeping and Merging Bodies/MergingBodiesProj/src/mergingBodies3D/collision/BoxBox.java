package mergingBodies3D.collision;

import static org.ode4j.ode.internal.Common.dRecip;

import java.util.ArrayList;

import javax.management.RuntimeErrorException;
import javax.vecmath.Matrix3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

/*************************************************************************
 * Adapted from the ODE port to Java DxBox class, see license info below 
 * TODO: MEMORY: consider having this class hold worker variables as 
 * members rather than allocating on each test (memory/garbage issues?)
/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2007-2010 Tilmann Zäschke      *
 * All rights reserved.  Email: ode4j@gmx.de   Web: www.ode4j.org        *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT.         *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT, ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT for more   *
 * details.                                                              *
 *                                                                       *
 *************************************************************************/

/**
 * standard ODE geometry primitives: public API and pairwise collision functions.
 * 
 * the rule is that only the low level primitive collision functions should set
 * dContactGeom::g1 and dContactGeom::g2.
 */
public class BoxBox {

	private static void ASSERT( boolean b ) {
		if (b) {
			throw new RuntimeErrorException( new Error("avenge my death!") );
		}
	}

	private static double getComp( Tuple3d v, int i ) {
		if ( i == 0 ) return v.x;
		if ( i == 1 ) return v.y;
		return v.z;
	}
	
	private static void setComp( Tuple3d v, int i, double val ) {
		if ( i == 0 ) {
			v.x = val;
		} else if (i == 1 ) {
			v.y = val;
		} else {
			v.z = val;
		}
	}
	
	/** 
	 * Dot product the columns of R with v2 and store in v1
	 * This is equivalent to v1 = R^T * v2
	 * @param v1
	 * @param R
	 * @param v2
	 */
	private static void dMULTIPLY1_331( Vector3d v1, Matrix3d R, Vector3d v2 ) {
		v1.x = R.m00*v2.x + R.m10*v2.y + R.m20*v2.z;
		v1.y = R.m01*v2.x + R.m11*v2.y + R.m21*v2.z;
		v1.z = R.m02*v2.x + R.m12*v2.y + R.m22*v2.z;
	}
	
	/** Working variables for various DOT routines */
	private static Vector3d v1 = new Vector3d(); 
	private static Vector3d v2 = new Vector3d();

	private static double dDOT44(Matrix3d a, int o1, Matrix3d b, int o2) {
		a.getColumn(o1, v1);
		b.getColumn(o2, v2);
		return v1.dot(v2);
	}
	
	private static double dDOT41(Matrix3d a, int o1, Vector3d b) {
		a.getColumn(o1,v1);
		return v1.dot(b);
	}
	
	private static double dDOT( Vector3d a, double[] b, int o2 ) {
		return a.x*b[0+o2] + a.y*b[1+o2] + a.z*b[2+o2];
	}
	
	private static void memcpy(double[] to, int i, double[] from, int j, int count) {
		System.arraycopy(from, j, to, i, count);
	}
	
	/**
	 * Returns a dotted with column o2 of matrix b
	 * @param a
	 * @param b
	 * @param o2
	 * @return
	 */
	private static double dDOT14(Vector3d a, Matrix3d b, int o2)  {
		b.getColumn( o2, v1 );
		return a.dot(v1);
	}
	
	//****************************************************************************
	// box-box collision utility


	/** for intersectRectQuad */
	private static double[] buffer = new double[16];

	// find all the intersection points between the 2D rectangle with vertices
	// at (+/-h[0],+/-h[1]) and the 2D quadrilateral with vertices (p[0],p[1]),
	// (p[2],p[3]),(p[4],p[5]),(p[6],p[7]).
	//
	// the intersection points are returned as x,y pairs in the 'ret' array.
	// the number of intersection points is returned by the function (this will
	// be in the range 0 to 8).

	//static int intersectRectQuad (double h[2], double p[8], double ret[16])
	private static int intersectRectQuad (double h[], double p[], double ret[]) {
		// q (and r) contain nq (and nr) coordinate points for the current (and
		// chopped) polygons
		int nq=4,nr = 0;
		//  double *q = p;
		//  double *r = ret;
		double[] r = ret;
		double[] q = p;
		int pos_q = 0; // p
		int pos_r = 0; // ret;
		for (int dir=0; dir <= 1; dir++) {
			// direction notation: xy[0] = x axis, xy[1] = y axis
			for (int sign=-1; sign <= 1; sign += 2) {
				// chop q along the line xy[dir] = sign*h[dir]
				//        double *pq = q;
				//        double *pr = r;
				int pos_pq = pos_q;
				int pos_pr = pos_r;
				nr = 0;
				for (int i=nq; i > 0; i--) {
					// go through all points in q and all lines between adjacent points
					//if (sign*pq[dir] < h[dir]) {
					if (sign*q[pos_pq + dir] < h[dir]) {
						// this point is inside the chopping line
						r[pos_pr] = q[pos_pq];//pr[0] = pq[0];
						r[pos_pr+1] = q[pos_pq+1];//pr[1] = pq[1];
						pos_pr +=2;//pr += 2;
						nr++;
						if ((nr & 8)!=0) {
							q = r;  //TZ
							pos_q = pos_r;
							//TZ goto done;
							if (q != ret) memcpy (ret,0,q, pos_q,nr*2);//*sizeof(double));
							return nr;
						}
					}
					//double *nextq = (i > 1) ? pq+2 : q;
					int next_q = (i > 1) ? pos_pq+2 : 0;
					//if ((sign*pq[dir] < h[dir]) ^ (sign*nextq[dir] < h[dir])) {
					if ((sign*q[pos_pq+dir] < h[dir]) ^ (sign*q[next_q+dir] < h[dir])) {
						// this line crosses the chopping line
						//						pr[1-dir] = pq[1-dir] + (nextq[1-dir]-pq[1-dir]) /
						//						(nextq[dir]-pq[dir]) * (sign*h[dir]-pq[dir]);
						r[pos_pr+1-dir] = q[pos_pq+1-dir] + 
							(q[next_q+1-dir]-q[pos_pq+1-dir]) /
							(q[next_q+dir]-q[pos_pq+dir]) * (sign*h[dir]-q[pos_pq+dir]);
						r[pos_pr+dir] = sign*h[dir];//pr[dir] = sign*h[dir];
						pos_pr += 2;//pr += 2;
						nr++;
						if ((nr & 8)!=0) {
							q = r;
							pos_q = pos_r; //TZ
							//TZ goto done;
							if (q != ret) memcpy (ret,0, q, pos_q,nr*2);//*sizeof(double));
							return nr;
						}
					}
					pos_pq += 2;//pq += 2;
				}
				q = r;
				pos_q = pos_r;  //TZ
				r = (q==ret) ? buffer : ret;
				pos_r = 0; //TZ
				nq = nr;
			}
		}
		//TZ	done:
		if (q != ret) memcpy (ret,0,q,pos_q,nr*2);//*sizeof(double));
		return nr;
	}

	private static boolean dNODEBUG = false;

	/** For cullPoints */
	private static double[] Acp = new double[8];
	private static int[] avail = new int[8];

	/**
	 * given n points in the plane (array p, of size 2*n), generate m points that
	 * best represent the whole set. the definition of 'best' here is not
	 * predetermined - the idea is to select points that give good box-box
	 * collision detection behavior. the chosen point indexes are returned in the
	 * array iret (of size m). 'i0' is always the first entry in the array.
	 * n must be in the range [1..8]. m must be in the range [1..n]. i0 must be
	 * in the range [0..n-1].
	 */
	private static void cullPoints (int n, double p[], int m, int i0, int iretA[]) {
		int iretP = 0;
		// compute the centroid of the polygon in cx,cy
		int i,j;
		double a,cx,cy,q;
		if (n==1) {
			cx = p[0];
			cy = p[1];
		} else if (n==2) {
			cx = (0.5)*(p[0] + p[2]);
			cy = (0.5)*(p[1] + p[3]);
		} else {
			a = 0;
			cx = 0;
			cy = 0;
			for (i=0; i<(n-1); i++) {
				q = p[i*2]*p[i*2+3] - p[i*2+2]*p[i*2+1];
				a += q;
				cx += q*(p[i*2]+p[i*2+2]);
				cy += q*(p[i*2+1]+p[i*2+3]);
			}
			q = p[n*2-2]*p[1] - p[0]*p[n*2-1];
			a = 1.0/((3.0)*(a+q));
			cx = a*(cx + q*(p[n*2-2]+p[0]));
			cy = a*(cy + q*(p[n*2-1]+p[1]));
		}

		// compute the angle of each point w.r.t. the centroid
		for (i=0; i<n; i++) Acp[i] = Math.atan2(p[i*2+1]-cy,p[i*2]-cx);

		// search for points that have angles closest to A[i0] + i*(2*pi/m).
		for (i=0; i<n; i++) avail[i] = 1;
		avail[i0] = 0;
		iretA[iretP] = i0;//iret[0] = i0;
		iretP++;//iret++;
		for (j=1; j<m; j++) {
			a = (double)((double)j*(2.*Math.PI/m) + Acp[i0]);
			if (a > Math.PI) a -= (double)(2*Math.PI);
			double maxdiff=1e9,diff;
			if (!dNODEBUG) {//#ifndef dNODEBUG
				iretA[iretP] = i0; //*iret = i0;			// iret is not allowed to keep this value
			}//#endif
			for (i=0; i<n; i++) {
				if (avail[i]!=0) {
					diff = Math.abs(Acp[i]-a);
					if (diff > Math.PI) diff = (double) (2*Math.PI - diff);
					if (diff < maxdiff) {
						maxdiff = diff;
						iretA[iretP] = i;//*iret = i;
					}
				}
			}
			if (!dNODEBUG) {//#ifndef dNODEBUG
				//dIASSERT (*iret != i0);	// ensure iret got set
				ASSERT (iretA[iretP] != i0);	// ensure iret got set
			}//#endif
			avail[iretA[iretP]] = 0; //avail[*iret] = 0;
			iretP++;//iret++;
		}
	}

	private static boolean CONTACTS_UNIMPORTANT = false; // we care about contacts! 
	public static final int NUMC_MASK = 0xffff;  // this is a rather large number... 
	
	//#define TST(expr1,expr2,norm,cc) 
	private static boolean TST1(double expr1, double expr2, Matrix3d norm_A, int norm_O, int cc, TstClass tst1) {
		double expr1_val = (expr1); /* Avoid duplicate evaluation of expr1 */ 
		double s2 = Math.abs(expr1_val) - (expr2); 
		if (s2 > 0) return false; 
		if (s2 > tst1._s + 1e-5) { // NOTE: we put a tiny buffer to better encourage consistent contact numbering!
			// Not sure what the rigth number is here, but 1e-5 seems reasonable for the scale of objects we have here
			tst1._s = s2; 
//			tst1._normalR_A = norm_A.v; 
//			tst1._normalR_O = norm_O; 
			tst1._normalR_M = norm_A; 
			tst1._normalR_col = norm_O; 
			tst1._invert_normal = ((expr1_val) < 0); 
			tst1._code = (cc); 
			if (CONTACTS_UNIMPORTANT) {
				tst1._break = true;
				return true;
			}
		}
		return true;
	}

	//#define TST(expr1,expr2,n1,n2,n3,cc) 
	private static boolean TST2(double expr1, double expr2, double n1, double n2, double n3, int cc, TstClass tst2) {
		double expr1_val = (expr1); /* Avoid duplicate evaluation of expr1 */ 
		double s2 = Math.abs(expr1_val) - (expr2); 
		if (s2 > 0) return false; 
		double l = Math.sqrt((n1)*(n1) + (n2)*(n2) + (n3)*(n3)); 
		if (l > 0) { 
			s2 /= l; 
			if (s2*tst2._fudge_factor > tst2._s) { 
				tst2._s = s2; 
				tst2._normalR_M = null; 
				tst2._normalR_col = 0; 
				//tst2._normalC.[0] = (n1)/l; tst2._normalC[1] = (n2)/l; tst2._normalC[2] = (n3)/l; 
				tst2._normalC.set( (n1)/l, (n2)/l, (n3)/l ); 
				tst2._invert_normal = ((expr1_val) < 0); 
				tst2._code = (cc); 
				if (CONTACTS_UNIMPORTANT) {
					tst2._break = true;
					return true;
				}
			} 
		}
		return true;
	}

	private static class TstClass {
		int _code;
		//TODO use RefInt?
//		double[] _normalR_A;
//		int _normalR_O;
		Matrix3d _normalR_M;
		int _normalR_col;
		Vector3d _normalC = new Vector3d();
		double _s;
		double _fudge_factor;
		//final int _flags;  
		boolean _invert_normal;
		TstClass(int flags, double fudge_factor) { // TODO: remove flags?  or put contacts_unimportant here.
			//_flags = flags;
			_fudge_factor = fudge_factor;
		}
		boolean _break = false;
	}
	
	/** for line closest approach (LCP) */
	private static Vector3d pLCP = new Vector3d();

	/**
	 * given two lines
	 * qa = pa + alpha* ua
	 * qb = pb + beta * ub
	 * where pa,pb are two points, ua,ub are two unit length vectors, and alpha,
	 * beta go from [-inf,inf], return alpha and beta such that qa and qb are
	 * as close as possible
	 */
	//void dLineClosestApproach (final dVector3 pa, final dVector3 ua,
	//			   final dVector3 pb, final dVector3 ub,
	//			   double *alpha, double *beta)
	private static void dLineClosestApproach (final Vector3d pa, final Vector3d ua,
			final Vector3d pb, final Vector3d ub,
			double[] alpha, double[] beta)
	{
		pLCP.sub( pb, pa );
		double uaub = ua.dot(ub);
		double q1 =  ua.dot(pLCP);
		double q2 = -ub.dot(pLCP);
		double d = 1-uaub*uaub;
		if (d <= (0.0001)) { // @@@ this needs to be made more robust
			alpha[0] = 0;
			beta[0]= 0;
		} else {
			d = dRecip(d);
			alpha[0] = ( (q1 + uaub*q2)*d);
			beta[0] = ( (uaub*q1 + q2)*d);
		}
	}
	
	/** 
	 * given two boxes (p1,R1,side1) and (p2,R2,side2), collide them together and
	 * generate contact points. this returns 0 if there is no contact otherwise
	 * it returns the number of contacts generated.		<br>
	 * `normal' returns the contact normal.				<br>
	 * `depth' returns the maximum penetration depth (positive) along that normal.	<br>
	 * `return_code' returns a number indicating the type of contact that was
	 * detected:<br>
	 *        1,2,3 = box 2 intersects with a face of box 1		<br>
	 *        4,5,6 = box 1 intersects with a face of box 2		<br>
	 *        7..15 = edge-edge contact							<br>
	 * `maxc' is the maximum number of contacts allowed to be generated, i.e.
	 * the size of the `contact' array.		<br>
	 * `contact' and `skip' are the contact array information provided to the
	 * collision functions. this function only fills in the position and depth
	 * fields.
	 */

	/** for dBoxBox */
	private static Vector3d p = new Vector3d();
	private static Vector3d pp = new Vector3d();
	private static Vector3d B = new Vector3d() ;//double A[3],B[3];
	private static Vector3d A = new Vector3d();
	private static TstClass tst = new TstClass(0xffff, 1.05);//fudge_factor); // we always collect all the contacts, so don't really need the flags!
	private static Vector3d pa = new Vector3d();
	private static Vector3d pb = new Vector3d();
	private static double[] rect=new double[2];
	private static double[] alpha = new double[1];
	private static double[] beta = new double[1]; //RefDouble(0);
	private static Vector3d ua = new Vector3d();
	private static Vector3d ub = new Vector3d();
	private static Vector3d normal2 = new Vector3d();
	private static Vector3d nr = new Vector3d();
	private static Vector3d anr = new Vector3d();
	private static Vector3d center = new Vector3d();
	private static int[] iret=new int[8];


	//	int dBoxBox (const Vector3d p1, const dMatrix3 R1,
	//		     const Vector3d side1, const Vector3d p2,
	//		     const dMatrix3 R2, const Vector3d side2,
	//		     Vector3d normal, dReal *depth, int *return_code,
	//		     int flags, dContactGeom *contact, int skip)
	public static int dBoxBox (final Tuple3d p1, final Matrix3d R1,
			final Vector3d side1, final Tuple3d p2,
			final Matrix3d R2, final Vector3d side2,
			Vector3d normal, double[] depth, int[] return_code, ArrayList<DContactGeom> contacts, int skip )
	{
		//TZ final double fudge_factor = (1.05);
		//,normalC=new Vector3d(0,0,0);
		//final double *normalR = 0;
		//final Vector3d normalR;

		double R11,R12,R13,R21,R22,R23,R31,R32,R33,
		       Q11,Q12,Q13,Q21,Q22,Q23,Q31,Q32,Q33;  //,s,s2,l,expr1_val;
		int i,j;//,invert_normal;//,code;
		//RefInt code = new RefInt();

		// get vector from centers of box 1 to box 2, relative to box 1
		p.sub(p2,p1);
		dMULTIPLY1_331 (pp,R1,p);		// get pp = p relative to body 1

		// get side lengths / 2
		A.scale( 0.5, side1 );
		B.scale( 0.5, side2 );

		// Rij is R1'*R2, i.e. the relative rotation between R1 and R2
		R11 = dDOT44(R1,0,R2,0); R12 = dDOT44(R1,0,R2,1); R13 = dDOT44(R1,0,R2,2);
		R21 = dDOT44(R1,1,R2,0); R22 = dDOT44(R1,1,R2,1); R23 = dDOT44(R1,1,R2,2);
		R31 = dDOT44(R1,2,R2,0); R32 = dDOT44(R1,2,R2,1); R33 = dDOT44(R1,2,R2,2);

		Q11 = Math.abs(R11); Q12 = Math.abs(R12); Q13 = Math.abs(R13);
		Q21 = Math.abs(R21); Q22 = Math.abs(R22); Q23 = Math.abs(R23);
		Q31 = Math.abs(R31); Q32 = Math.abs(R32); Q33 = Math.abs(R33);

		
		// for all 15 possible separating axes:
		//   * see if the axis separates the boxes. if so, return 0.
		//   * find the depth of the penetration along the separating axis (s2)
		//   * if this is the largest depth so far, record it.
		// the normal vector will be set to the separating axis with the smallest
		// depth. note: normalR is set to point to a column of R1 or R2 if that is
		// the smallest depth normal so far. otherwise normalR is 0 and normalC is
		// set to a vector relative to body 1. invert_normal is 1 if the sign of
		// the normal should be flipped.

		do {
			//	#define TST(expr1,expr2,norm,cc) \
			//	    expr1_val = (expr1); /* Avoid duplicate evaluation of expr1 */ \
			//	    s2 = Math.abs(expr1_val) - (expr2); \
			//	    if (s2 > 0) return 0; \
			//	    if (s2 > s) { \
			//	      s = s2; \
			//	      normalR = norm; \
			//	      invert_normal = ((expr1_val) < 0); \
			//	      code = (cc); \
			//		  if (flags & CONTACTS_UNIMPORTANT) break; \
			//		}
			//
			tst._s = Double.NEGATIVE_INFINITY;
			tst._invert_normal = false;
			tst._code = 0;

			// separating axis = u1,u2,u3
			if (!TST1 (pp.x,(A.x + B.x*Q11 + B.y*Q12 + B.z*Q13),R1,0,1,tst)) return 0;
			if (tst._break) break;
			if (!TST1 (pp.y,(A.y + B.x*Q21 + B.y*Q22 + B.z*Q23),R1,1,2,tst)) return 0;
			if (tst._break) break;
			if (!TST1 (pp.z,(A.z + B.x*Q31 + B.y*Q32 + B.z*Q33),R1,2,3,tst)) return 0;
			if (tst._break) break;

			// separating axis = v1,v2,v3
			if (!TST1 (dDOT41(R2,0,p),(A.x*Q11 + A.y*Q21 + A.z*Q31 + B.x),R2,0,4,tst)) return 0;
			if (tst._break) break;
			if (!TST1 (dDOT41(R2,1,p),(A.x*Q12 + A.y*Q22 + A.z*Q32 + B.y),R2,1,5,tst)) return 0;
			if (tst._break) break;
			if (!TST1 (dDOT41(R2,2,p),(A.x*Q13 + A.y*Q23 + A.z*Q33 + B.z),R2,2,6,tst)) return 0;
			if (tst._break) break;

			// note: cross product axes need to be scaled when s is computed.
			// normal (n1,n2,n3) is relative to box 1.
			//	#undef TST
			//	#define TST(expr1,expr2,n1,n2,n3,cc) \
			//	    expr1_val = (expr1); /* Avoid duplicate evaluation of expr1 */ \
			//	    s2 = Math.abs(expr1_val) - (expr2); \
			//	    if (s2 > 0) return 0; \
			//	    l = dSqrt ((n1)*(n1) + (n2)*(n2) + (n3)*(n3)); \
			//	    if (l > 0) { \
			//	      s2 /= l; \
			//	      if (s2*fudge_factor > s) { \
			//	        s = s2; \
			//	        normalR = 0; \
			//	        normalC[0] = (n1)/l; normalC[1] = (n2)/l; normalC[2] = (n3)/l; \
			//	        invert_normal = ((expr1_val) < 0); \
			//	        code = (cc); \
			//	        if (flags & CONTACTS_UNIMPORTANT) break; \
			//		  } \
			//		}

			// We only need to check 3 edges per box 
			// since parallel edges are equivalent.

			// separating axis = u1 x (v1,v2,v3)
			if (!TST2(pp.z*R21-pp.y*R31,(A.y*Q31+A.z*Q21+B.y*Q13+B.z*Q12),0,-R31,R21,7, tst)) return 0;
			if (tst._break) break;
			if (!TST2(pp.z*R22-pp.y*R32,(A.y*Q32+A.z*Q22+B.x*Q13+B.z*Q11),0,-R32,R22,8, tst)) return 0;
			if (tst._break) break;
			if (!TST2(pp.z*R23-pp.y*R33,(A.y*Q33+A.z*Q23+B.x*Q12+B.y*Q11),0,-R33,R23,9, tst)) return 0;
			if (tst._break) break;

			// separating axis = u2 x (v1,v2,v3)
			if (!TST2(pp.x*R31-pp.z*R11,(A.x*Q31+A.z*Q11+B.y*Q23+B.z*Q22),R31,0,-R11,10, tst)) return 0;
			if (tst._break) break;
			if (!TST2(pp.x*R32-pp.z*R12,(A.x*Q32+A.z*Q12+B.x*Q23+B.z*Q21),R32,0,-R12,11, tst)) return 0;
			if (tst._break) break;
			if (!TST2(pp.x*R33-pp.z*R13,(A.x*Q33+A.z*Q13+B.x*Q22+B.y*Q21),R33,0,-R13,12, tst)) return 0;
			if (tst._break) break;

			// separating axis = u3 x (v1,v2,v3)
			if (!TST2(pp.y*R11-pp.x*R21,(A.x*Q21+A.y*Q11+B.y*Q33+B.z*Q32),-R21,R11,0,13, tst)) return 0;
			if (tst._break) break;
			if (!TST2(pp.y*R12-pp.x*R22,(A.x*Q22+A.y*Q12+B.x*Q33+B.z*Q31),-R22,R12,0,14, tst)) return 0;
			if (tst._break) break;
			if (!TST2(pp.y*R13-pp.x*R23,(A.x*Q23+A.y*Q13+B.x*Q32+B.y*Q31),-R23,R13,0,15, tst)) return 0;
			if (tst._break) break;
			//	#undef TST
		} while (false);//(0);

		if (tst._code == 0) return 0;//(!code) return 0;

		// if we get to this point, the boxes interpenetrate. compute the normal
		// in global coordinates.
		if (tst._normalR_M != null) {
			//	    normal[0] = normalR[0];
			//	    normal[1] = normalR[4];
			//	    normal[2] = normalR[8];
//			normal.set(tst._normalR_M[tst._normalR_O+0], 
//					tst._normalR_M[tst._normalR_O+4], 
//					tst._normalR_M[tst._normalR_O+8]);
			//normal.set(tst._normalR_M.viewCol(tst._normalR_col));
			tst._normalR_M.getColumn( tst._normalR_col, normal );
		} else {
			R1.transform( tst._normalC, normal );
			//dMULTIPLY0_331 (normal,R1,tst._normalC);
		}
		if (tst._invert_normal) {
			normal.scale(-1);
		}
		depth[0] = -tst._s;

		// compute contact point(s)

		if (tst._code > 6) {
			// An edge from box 1 touches an edge from box 2.
			// find a point pa on the intersecting edge of box 1
			double sign;
			pa.set( p1 );
			// Get world position of p2 into pa
			for (j=0; j<3; j++) {
				sign = (dDOT14(normal,R1,j) > 0) ? (1.0) : (-1.0);
				//for (i=0; i<3; i++) pa.v[i] += sign * A.v[j] * R1.v[i*4+j];
				//for (i=0; i<3; i++) pa.add(i, sign * getComp(A,j) * R1.getElement(i, j) );
				pa.x += sign * getComp(A,j) * R1.getElement(0, j);
				pa.y += sign * getComp(A,j) * R1.getElement(1, j);
				pa.z += sign * getComp(A,j) * R1.getElement(2, j);				
			}

			// find a point pb on the intersecting edge of box 2
			pb.set( p2 );
			// Get world position of p2 into pb
			for (j=0; j<3; j++) {
				sign = (dDOT14(normal,R2,j) > 0) ? (-1.0) : (1.0);
				//for (i=0; i<3; i++) pb.v[i] += sign * B.v[j] * R2.v[i*4+j];
				//for (i=0; i<3; i++) pb.add(i, sign * getComp(B,j) * R2.getElement(i, j) );
				pb.x += sign * getComp(B,j) * R2.getElement(0, j);
				pb.y +=	sign * getComp(B,j) * R2.getElement(1, j);
				pb.z += sign * getComp(B,j) * R2.getElement(2, j);
			}

			// Get direction of first edge
			//for (i=0; i<3; i++) ua.set(i, R1.v[((tst._code)-7)/3 + i*4] );		
			for (i=0; i<3; i++) setComp( ua, i, R1.getElement(i,(tst._code-7)/3) ); 
			// Get direction of second edge
			//for (i=0; i<3; i++) ub.set(i, R2.v[((tst._code)-7)%3 + i*4] );
			for (i=0; i<3; i++) setComp( ub, i, R2.getElement(i,(tst._code-7)%3) );
			// Get closest points between edges (one at each)
			dLineClosestApproach (pa,ua,pb,ub,alpha,beta);    
//			for (i=0; i<3; i++) pa.v[i] += ua.v[i]*alpha.get();

			pa.scaleAdd(alpha[0], ua, pa);
//			for (i=0; i<3; i++) pb.v[i] += ub.v[i]*beta.get();
			pb.scaleAdd(beta[0], ub, pb);
			// Set the contact point as halfway between the 2 closest points
			//for (i=0; i<3; i++) contact[0].pos[i] = (0.5)*(pa[i]+pb[i]);
			DContactGeom con = new DContactGeom();
			contacts.add( con );
			con.pos.add(pa, pb);
			con.pos.scale(0.5);
			con.depth = depth[0];
			return_code[0] = tst._code;
			// TODO: Add information to help with warm starts?  This edge edge case will be trivial
			// to match up as it only generates one contact between two bodies!
			return 1;
		}

		// okay, we have a face-something intersection (because the separating
		// axis is perpendicular to a face). define face 'a' to be the reference
		// face (i.e. the normal vector is perpendicular to this) and face 'b' to be
		// the incident face (the closest face of the other box).
		// Note: Unmodified parameter values are being used here
		//final double *Ra,*Rb,*pa,*pb,*Sa,*Sb;
		Matrix3d Ra, Rb;
		Tuple3d pa, pb, Sa, Sb;
		if (tst._code <= 3) { // One of the faces of box 1 is the reference face
			Ra = R1; // Rotation of 'a'
			Rb = R2; // Rotation of 'b'
			pa = p1; // Center (location) of 'a'
			pb = p2; // Center (location) of 'b'
			Sa = A;  // Side Lenght of 'a'
			Sb = B;  // Side Lenght of 'b'
		} else { // One of the faces of box 2 is the reference face
			Ra = R2; // Rotation of 'a'
			Rb = R1; // Rotation of 'b'
			pa = p2; // Center (location) of 'a'
			pb = p1; // Center (location) of 'b'
			Sa = B;  // Side Lenght of 'a'
			Sb = A;  // Side Lenght of 'b'
		}

		// nr = normal vector of reference face dotted with axes of incident box.
		// anr = absolute values of nr.
		/*
		The normal is flipped if necessary so it always points outward from box 'a',
		box 'b' is thus always the incident box
		 */

		if (tst._code <= 3) {
			normal2.set(normal);
		} else {
			normal2.scale( -1, normal );
		}
		// Rotate normal2 in incident box opposite direction
		dMULTIPLY1_331 (nr,Rb,normal2);
		anr.absolute(nr);

		// find the largest compontent of anr: this corresponds to the normal
		// for the incident face. the other axis numbers of the incident face
		// are stored in a1,a2.
		int lanr,a1,a2;
		if (anr.y > anr.x) {
			if (anr.y > anr.z) {
				a1 = 0;
				lanr = 1;
				a2 = 2;
			} else {
				a1 = 0;
				a2 = 1;
				lanr = 2;
			}
		} else {
			if (anr.x > anr.z) {
				lanr = 0;
				a1 = 1;
				a2 = 2;
			} else {
				a1 = 0;
				a2 = 1;
				lanr = 2;
			}
		}

		// compute center point of incident face, in reference-face coordinates
		if ( getComp( nr, lanr ) < 0) {
			//for (i=0; i<3; i++) center.set(i, pb.get(i) - pa.get(i) + Sb.get(lanr) * Rb.v[i*4+lanr] );
			for (i=0; i<3; i++) setComp( center, i, getComp(pb,i) - getComp(pa,i) + getComp(Sb,lanr) * Rb.getElement(i, lanr) );
		} else {
			//for (i=0; i<3; i++) center.set(i, pb.get(i) - pa.get(i) - Sb.get(lanr) * Rb.v[i*4+lanr] );
			for (i=0; i<3; i++) setComp( center, i, getComp(pb,i) - getComp(pa,i) - getComp(Sb,lanr) * Rb.getElement(i, lanr) );
		}

		// find the normal and non-normal axis numbers of the reference box
		int codeN,code1,code2;
		if (tst._code <= 3) codeN = tst._code-1; else codeN = tst._code-4;
		if (codeN==0) {
			code1 = 1;
			code2 = 2;
		} else if (codeN==1) {
			code1 = 0;
			code2 = 2;
		} else {
			code1 = 0;
			code2 = 1;
		}

		// find the four corners of the incident face, in reference-face coordinates
		double[] quad=new double[8];	// 2D coordinate of incident face (x,y pairs)
		double c1,c2,m11,m12,m21,m22;
		c1 = dDOT14 (center,Ra,code1);
		c2 = dDOT14 (center,Ra,code2);
		// optimize this? - we have already computed this data above, but it is not
		// stored in an easy-to-index format. for now it's quicker just to recompute
		// the four dot products.
		m11 = dDOT44 (Ra,code1,Rb,a1);
		m12 = dDOT44 (Ra,code1,Rb,a2);
		m21 = dDOT44 (Ra,code2,Rb,a1);
		m22 = dDOT44 (Ra,code2,Rb,a2);
		{
			double k1 = m11*getComp(Sb,a1);
			double k2 = m21*getComp(Sb,a1);
			double k3 = m12*getComp(Sb,a2);
			double k4 = m22*getComp(Sb,a2);
			quad[0] = c1 - k1 - k3;
			quad[1] = c2 - k2 - k4;
			quad[2] = c1 - k1 + k3;
			quad[3] = c2 - k2 + k4;
			quad[4] = c1 + k1 + k3;
			quad[5] = c2 + k2 + k4;
			quad[6] = c1 + k1 - k3;
			quad[7] = c2 + k2 - k4;
		}

		// find the size of the reference face
		rect[0] = getComp(Sa,code1);
		rect[1] = getComp(Sa,code2);

		// intersect the incident and reference faces
		double[] ret=new double[16];
		int n = intersectRectQuad (rect,quad,ret);
		if (n < 1) return 0;		// this should never happen

		// convert the intersection points into reference-face coordinates,
		// and compute the contact position and depth for each point. only keep
		// those points that have a positive (penetrating) depth. delete points in
		// the 'ret' array as necessary so that 'point' and 'ret' correspond.
		double[] point=new double[3*8];		// penetrating contact points
		double[] dep=new double[8];			// depths for those points
		double det1 = 1.0/(m11*m22 - m12*m21);
		m11 *= det1;
		m12 *= det1;
		m21 *= det1;
		m22 *= det1;
		int cnum = 0;			// number of penetrating contact points found
		for (j=0; j < n; j++) {
			double k1 =  m22*(ret[j*2]-c1) - m12*(ret[j*2+1]-c2);
			double k2 = -m21*(ret[j*2]-c1) + m11*(ret[j*2+1]-c2);
			for (i=0; i<3; i++) point[cnum*3+i] =
				//center.get(i) + k1*Rb.v[i*4+a1] + k2*Rb.v[i*4+a2];
				getComp( center,i ) + k1*Rb.getElement(i,a1) + k2*Rb.getElement(i,a2); 
			dep[cnum] = getComp(Sa,codeN) - dDOT(normal2, point,cnum*3);
			if (dep[cnum] >= 0) {
				ret[cnum*2] = ret[j*2];
				ret[cnum*2+1] = ret[j*2+1];
				cnum++;
				if ( CONTACTS_UNIMPORTANT ) break;
				//if (cnum == (flags & NUMC_MASK)) break;
			}
		}
		if (cnum < 1) { 
			return 0;	// this should not happen, yet does at times (demo_plane2d single precision).
		}

		// we can't generate more contacts than we actually have
		// (Actually, now shouldn't generate more than requested as they are allocated here... )
		int maxc = 0xffff;// flags & NUMC_MASK;
		if (maxc > cnum) maxc = cnum;
		// Even though max count must not be zero this check is kept for backward 
		// compatibility as this is a public function
		if (maxc < 1) maxc = 1;	

		if (cnum <= maxc) {
			// we have less contacts than we need, so we use them all
			for (j=0; j < cnum; j++) {
				//dContactGeom *con = CONTACT(contact,skip*j);
				DContactGeom con = new DContactGeom();
				contacts.add( con );
				//DContactGeom con = contacts.get(skip*j);
				for (i=0; i<3; i++) setComp( con.pos, i, point[j*3+i] + getComp(pa,i) );
				con.depth = dep[j];
			}
		} else {
			// cnum should be generated not greater than maxc so that "then" clause is executed
			//dIASSERT(!(flags & CONTACTS_UNIMPORTANT)); 
			
			//ASSERT(0==(flags & CONTACTS_UNIMPORTANT));
			
			// we have more contacts than are wanted, some of them must be culled.
			// find the deepest point, it is always the first contact.
			int i1 = 0;
			double maxdepth = dep[0];
			for (i=1; i<cnum; i++) {
				if (dep[i] > maxdepth) {
					maxdepth = dep[i];
					i1 = i;
				}
			}

			cullPoints (cnum,ret,maxc,i1,iret);

			for (j=0; j < maxc; j++) {
				//dContactGeom *con = CONTACT(contact,skip*j);
				//DContactGeom con = contacts.get(skip*j);
				DContactGeom con = new DContactGeom();
				contacts.add( con );
				for (i=0; i<3; i++) setComp( con.pos, i, point[iret[j]*3+i] + getComp( pa, i ) );
				con.depth = dep[iret[j]];
			}
			cnum = maxc;
		}

		return_code[0] = tst._code;
		return cnum;
	}
	
}
