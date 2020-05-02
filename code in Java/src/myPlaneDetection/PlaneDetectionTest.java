package myPlaneDetection;

import java.io.IOException;
import java.util.ArrayList;
import uk.ac.ucl.jpl.Point3;
import uk.ac.ucl.jpl.PointCloud;
import uk.ac.ucl.jpl.SurfaceNormals;
import uk.ac.ucl.jpl.colour.Colour;
import uk.ac.ucl.jpl.io.IOFunctions;
import uk.ac.ucl.jpl.spatialindexing.OctreeIndex;

public class PlaneDetectionTest {
	  public static void main(String[] args) throws IOException {
	      long start = System.currentTimeMillis();
          
	      //parameters of point cloud
		  float octreeRadius = 0.1f;
		  float snormRadius = 0.1f;

		  //read file 
	      String fileName =
			        "C:/Users/PangTouXian/Desktop/paper picture & data/shopping mall/EM001_500000.txt";
		  PointCloud pc = IOFunctions.readTextXYZ(fileName);
		  System.out.println("1:file read!");
		  OctreeIndex oct = new OctreeIndex(pc, octreeRadius);
		  SurfaceNormals snorm = new SurfaceNormals(pc.getNumPoints());
		  snorm.calculateFromRadius(pc, oct, snormRadius);
		  snorm.fixNormalsToViewpoint(new Point3(1000, 1000, 1000), pc.mCoordinates);
		  pc.setSurfaceNormals(snorm);
		  System.out.println("2:point cloud established!");

		  //Ransac parameters on plane model establishment 
		  RansacPlaneDetector.setThreshold_angle(0.01f);
		  RansacPlaneDetector.setUpper_threshold_angle(0.08f);
		  RansacPlaneDetector.setStep_threshold_angle(0.01f);
		  RansacPlaneDetector.setMin_triangle_edge(1f);
		  RansacPlaneDetector.setLower_min_triangle_edge(0f);
		  RansacPlaneDetector.setStep_min_triangle_edge(0.1f);
		  RansacPlaneDetector.setIterations(10000);
		      
		  //Ransac parameters on planar point selection, edge detection, edge region grow
		  RansacPlaneDetector.setThreshold_distance_plane(0.1f);
		  RansacPlaneDetector.setThreshold_angle_plane(0.8f);
		  RansacPlaneDetector.setThreshold_plane_point_number(500);
		  RansacPlaneDetector.setThreshold_point_number(0);
		  float edgeRadius = 0.1f;
		  int plane_density_threshold = 0;
		     
		  // remove all the planes
		  ArrayList<PointCloud> completePlanes =
		  RansacPlaneDetector.findCompletePlanesFast(pc, oct, snorm, edgeRadius);
		  System.out.println("3:plane extraction done!");
		  
		  //save each plane to file
		  int j = 1;
		    for (PointCloud pc1 : completePlanes) {
		      IOFunctions.writeTextXYZ(
		          pc1,
		          "C:/Users/PangTouXian/Desktop/paper picture & data/shopping mall/my method/plane"
		              + Integer.toString(j) + ".xyz");
		      j++;
		    }

		    PointCloud pc_complete_all = new PointCloud(0);
		    Colour c1 = new Colour(pc_complete_all.getNumPoints(), 255, 255, 255);
		    pc_complete_all.setColours(c1);
		    c1 = null;

		    PointCloud pc_complete_large = new PointCloud(0);
		    Colour c4 = new Colour(pc_complete_all.getNumPoints(), 255, 255, 255);
		    pc_complete_large.setColours(c4);
		    c4 = null;

		    PointCloud pc_rest = completePlanes.get(completePlanes.size() - 1);
		      
		    for (int k = 0; k < completePlanes.size() - 1; k++) {
		      pc_complete_all.append(completePlanes.get(k));
		      System.out.print((k + 1) + ":" + completePlanes.get(k).getNumPoints()
		          / RansacPlaneDetector.getEdgeSize().get(k) + ", ");
		      //add low plane density points back to the cloud (e.g. chair seats)
		      if (completePlanes.get(k).getNumPoints()
		          / RansacPlaneDetector.getEdgeSize().get(k) > plane_density_threshold)
		        pc_complete_large.append(completePlanes.get(k));
		      else
		        pc_rest.append(completePlanes.get(k));
		    }

		    IOFunctions
		      .writeTextXYZ( pc_complete_all,
		        "C:/Users/PangTouXian/Desktop/paper picture & data/shopping mall/my method/my_plane_all.txt"
		     );

		    IOFunctions
		        .writeTextXYZ(pc_rest,
		        "C:/Users/PangTouXian/Desktop/paper picture & data/shopping mall/my method/my_rest.txt");
		    OctreeIndex oct_rest = new OctreeIndex(pc_rest, octreeRadius);
		     
		    // filter low density points 
		    SegmentObjects.setRadiusSearch(0.05f);
		    SegmentObjects.setPointNumThreshold(1);
		  
		    PointCloud pc_denoise1 = SegmentObjects.denoise(pc_rest, oct_rest);
		    OctreeIndex oct_denoise1 = new OctreeIndex(pc_denoise1, octreeRadius);
		    
		    SegmentObjects.setRadiusSearch(0.2f);
		    SegmentObjects.setPointNumThreshold(20);
		    
		    PointCloud pc_denoise2 = SegmentObjects.denoise(pc_denoise1, oct_denoise1);
		
		    IOFunctions
		      .writeTextXYZ(
		          pc_denoise2,
		      "C:/Users/PangTouXian/Desktop/paper picture & data/shopping mall/my method/my_denoise.txt");
		    System.out.println("  point number change:" + pc_rest.getNumPoints() + " ; " + pc_denoise2.getNumPoints());
		    System.out.println("4:denoising done!");
		    System.out.println("End!");
		    long end = System.currentTimeMillis();
		    System.out.println("time:" + (end - start));
		  }

}
