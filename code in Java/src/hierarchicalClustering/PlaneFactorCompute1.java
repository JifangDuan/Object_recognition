package hierarchicalClustering;
import java.util.ArrayList;
import java.lang.Math;

import uk.ac.ucl.jpl.*;
import uk.ac.ucl.jpl.spatialindexing.OctreeIndex;

//region growing from seed point, algorithm 1

public class PlaneFactorCompute1{
	
	private static Coordinates ref_coords;
	private static OctreeIndex ref_oct;
	private static float radius;
	private static SurfaceNormals ref_sn;
	private Point3 sp;
	private Point3 op;
	private ArrayList<Integer> returnIndex;
	
	public PlaneFactorCompute1() {
		sp = new Point3();
		op = new Point3();
		returnIndex = new ArrayList<Integer>();
	}

	public double process(int k, int i,double angle_threshold){
		Point3 sn = new Point3(ref_sn.mData[k*3],ref_sn.mData[k*3+1],ref_sn.mData[k*3+2]);
		
		ref_coords.getPoint(i,sp); 
		ref_oct.radiusSearch(sp, radius, returnIndex);
		ref_coords.getPoint(k,sp);

		double angle = 0;
		double d2 = 0;
		
		// loop over each neighbour
		for (int j:returnIndex){
			ref_coords.getPoint(j, op);
			
			Point3 a = new Point3();
			a.x = op.x - sp.x;
			a.y = op.y - sp.y;
			a.z = op.z - sp.z;
			
			double sum = sn.x * a.x + sn.y * a.y + sn.z * a.z;
		
			double d1 = Math.sqrt(Math.pow(sn.x, 2)+Math.pow(sn.y, 2)+Math.pow(sn.z, 2));
			d2 = Math.sqrt(Math.pow(a.x, 2)+Math.pow(a.y, 2)+Math.pow(a.z, 2));
			sum = sum/(d1*d2);
			angle += Math.abs(Math.acos(sum));
		}
		angle /= returnIndex.size();
		double PAF = angle_threshold / (Math.abs(Math.PI/2 - angle) /  Math.atan(0.01/d2));
		return PAF;
	}
	
	public static void setSharedReferences(SurfaceNormals sn,Coordinates coords,OctreeIndex oct,float radius){
		ref_sn = sn;
		ref_coords = coords;
		ref_oct = oct;
		PlaneFactorCompute1.radius = radius;
    }
}