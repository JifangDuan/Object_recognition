package ransac;

import java.io.IOException;
import java.util.ArrayList;

import uk.ac.ucl.jpl.Point3;
import uk.ac.ucl.jpl.PointCloud;
import uk.ac.ucl.jpl.SurfaceNormals;
import uk.ac.ucl.jpl.colour.Colour;
import uk.ac.ucl.jpl.io.IOFunctions;
import uk.ac.ucl.jpl.spatialindexing.OctreeIndex;

public class RansacTest {
	public static void main(String[] args) throws IOException {

	    long start = System.currentTimeMillis();

	    //parameters of point cloud
	    float octreeRadius = 0.05f;
	    float snormRadius = 0.08f;

	    String fileName =
	        "C:/Users/PangTouXian/Documents/UCL/java eclipse/data/UCLB_meetingRoomSmall.txt";
	    PointCloud pc = IOFunctions.readTextXYZ(fileName);
	    System.out.println("1:file read!");
	      
	    OctreeIndex oct = new OctreeIndex(pc, octreeRadius);
	    SurfaceNormals snorm = new SurfaceNormals(pc.getNumPoints());
	    snorm.calculateFromRadius(pc, oct, snormRadius);
	    snorm.fixNormalsToViewpoint(new Point3(1000, 1000, 1000), pc.mCoordinates);
	    pc.setSurfaceNormals(snorm);
	    System.out.println("2:point cloud established!");
	    
	    // parameters of three point selection 
	    OriginalRansacPlaneDetector.setUpper_threshold_angle(0.01f);
	    OriginalRansacPlaneDetector.setLower_min_triangle_edge(0.1f);
	    OriginalRansacPlaneDetector.setIterations(1000000);
	      
	    // parameters of planar points selection
	    OriginalRansacPlaneDetector.setThreshold_distance_plane(0.1f);
	    OriginalRansacPlaneDetector.setThreshold_angle_plane(0.5f);
	    
	    // remove all the planes
	    ArrayList<PointCloud> planes =
	      OriginalRansacPlaneDetector.findPlanes(pc, oct, snorm);
	    System.out.println("3:plane extraction done!");
	    
	    PointCloud pc_all = new PointCloud(0);
	    Colour c1 = new Colour(pc_all.getNumPoints(), 255, 255, 255);
	    pc_all.setColours(c1);
	    c1 = null;
	    
	    PointCloud pc_rest = planes.get(planes.size() - 1);
	    
	    int j = 1;
	    for (PointCloud pc1 : planes) {
	        if(j<planes.size()){
		    	IOFunctions.writeTextXYZ(
		  	          pc1,
		  	          "C:/Users/PangTouXian/Documents/UCL/java eclipse/result/plane_detection/plane/Plane_"
		  	              + Integer.toString(j) + ".xyz");
		    	pc_all.append(pc1);
		    	 j++;
	        }	
	    }
	    
	    IOFunctions.writeTextXYZ(pc_all,
	            "C:/Users/PangTouXian/Documents/UCL/java eclipse/result/plane_detection/planeAll.txt");
	    
	    IOFunctions.writeTextXYZ(pc_rest,
	            "C:/Users/PangTouXian/Documents/UCL/java eclipse/result/plane_detection/rest.txt");
	    
	    long end = System.currentTimeMillis();
	    System.out.println("time:" + (end - start));
	}

}
