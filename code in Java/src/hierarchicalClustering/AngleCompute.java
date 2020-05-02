package hierarchicalClustering;
import java.util.ArrayList;
import java.lang.Math;




import uk.ac.ucl.jpl.Coordinates;
import uk.ac.ucl.jpl.Point3;
import uk.ac.ucl.jpl.SurfaceNormals;
//import uk.ac.ucl.jpl.*;
import uk.ac.ucl.jpl.spatialindexing.OctreeIndex;

//help finding seed points by computing angles between normal vectors of one point and its sounding points

public class AngleCompute{
	private static Coordinates ref_coords;
	private static OctreeIndex ref_oct;
	private static float radius;
	private static SurfaceNormals ref_sn;
	private Point3 sp;
	private ArrayList<Integer> returnIndex;
	
	public AngleCompute() {
		sp = new Point3();
		returnIndex = new ArrayList<Integer>();
	}

	public double process(int i, int sizeThreshold)
	{
		ref_coords.getPoint(i,sp); 
		ref_oct.radiusSearch(sp, radius, returnIndex);
		double angle = 0;
		
		// loop over each neighbour
		for (int j:returnIndex)
		{
			//do not consider low density points
			if (returnIndex.size() <= sizeThreshold){
					angle = -1;
					break;
			}
			
			//compute angle between normal vectors of point i and its neighbouring points 
			double sum = ref_sn.mData[i*3]*ref_sn.mData[j*3]+ref_sn.mData[i*3+1]*ref_sn.mData[j*3+1]+ref_sn.mData[i*3+2]*ref_sn.mData[j*3+2];
			double d1 = Math.sqrt(Math.pow(ref_sn.mData[i*3], 2)+Math.pow(ref_sn.mData[i*3+1], 2)+Math.pow(ref_sn.mData[i*3+2], 2));
			double d2 = Math.sqrt(Math.pow(ref_sn.mData[j*3], 2)+Math.pow(ref_sn.mData[j*3+1], 2)+Math.pow(ref_sn.mData[j*3+2], 2));
			sum = sum/(d1*d2);
			angle += Math.abs(Math.acos(sum));
		}
		angle /= returnIndex.size();
		return angle;	
	}

	public static void setSharedReferences(SurfaceNormals sn,Coordinates coords,OctreeIndex oct,float radius){
		ref_sn = sn;
		ref_coords = coords;
		ref_oct = oct;
		AngleCompute.radius = radius;
    }
}