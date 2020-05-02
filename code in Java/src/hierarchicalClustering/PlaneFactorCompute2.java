package hierarchicalClustering;

import java.lang.Math;
import uk.ac.ucl.jpl.*;

//region growing from seed point, algorithm 2

public class PlaneFactorCompute2
{
	private static Coordinates ref_coords;
	private static SurfaceNormals ref_sn;
	private Point3 sp;
	private Point3 op;

	public PlaneFactorCompute2() {
		sp = new Point3();
		op = new Point3();
	}
	
	public double process(int k, int k_new, int i, double d){
	    Point3 sn =
	        new Point3(ref_sn.mData[k * 3], ref_sn.mData[k * 3 + 1],
	            ref_sn.mData[k * 3 + 2]);
	
	    ref_coords.getPoint(i, sp); 
	    ref_coords.getPoint(k_new, op);
	
	    Point3 a = new Point3();
	    a.x = op.x - sp.x;
	    a.y = op.y - sp.y;
	    a.z = op.z - sp.z;
	
	    double sum = sn.x * a.x + sn.y * a.y + sn.z * a.z;
	    double PAF = 0;
	    double d1 =
	        Math.sqrt(Math.pow(sn.x, 2) + Math.pow(sn.y, 2) + Math.pow(sn.z, 2));
	    double d2 =
	        Math.sqrt(Math.pow(a.x, 2) + Math.pow(a.y, 2) + Math.pow(a.z, 2));
	    sum = sum / (d1 * d2);
	    double angle = Math.abs(Math.acos(sum));
	    PAF = Math.asin(d / d2) / (Math.abs(Math.PI / 2 - angle));
	    return PAF;
    }

	public static void setSharedReferences(SurfaceNormals sn,Coordinates coords){
		ref_sn = sn;
		ref_coords = coords;
    }
}