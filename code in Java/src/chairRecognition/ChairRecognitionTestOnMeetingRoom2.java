package chairRecognition;

import java.io.IOException;
import java.util.ArrayList;

import uk.ac.ucl.jpl.Point3;
import uk.ac.ucl.jpl.PointCloud;
import uk.ac.ucl.jpl.SurfaceNormals;
import uk.ac.ucl.jpl.colour.Colour;
import uk.ac.ucl.jpl.io.IOFunctions;
import uk.ac.ucl.jpl.spatialindexing.Box;
import uk.ac.ucl.jpl.spatialindexing.OctreeIndex;

public class ChairRecognitionTestOnMeetingRoom2 {

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
  
    // calculate overall point density
    Box bb = pc.mCoordinates.findBoundingBox();
    float ceil = bb.zMax;
    float floor = bb.zMin;
    int pointDensity = (int) (pc.getNumPoints()/(2*(bb.xLength()*bb.yLength()+bb.xLength()*bb.zLength()+bb.yLength()*bb.zLength())));
    System.out.println("pointDensity:"+pointDensity);
    
    // parameters of three point selection 
    RansacPlaneDetector.setThreshold_angle(0.01f);
    RansacPlaneDetector.setUpper_threshold_angle(0.07f);
    RansacPlaneDetector.setStep_threshold_angle(0.01f);
    RansacPlaneDetector.setMin_triangle_edge(1f);
    RansacPlaneDetector.setLower_min_triangle_edge(0.1f);
    RansacPlaneDetector.setStep_min_triangle_edge(0.1f);
    RansacPlaneDetector.setIterations(1000000);
      
    // parameters of planar points selection
    RansacPlaneDetector.setThreshold_distance_plane(0.05f);
    RansacPlaneDetector.setThreshold_angle_plane(1.5f);
    RansacPlaneDetector.setThreshold_plane_point_number(pointDensity*2);
    float edgeRadius = 0.06f;
    
    // put extracted plane with low plane density back to original cloud
    int plane_density_threshold = pointDensity/2;

    // parameter for filter
    RansacPlaneDetector.setRadius_count(1f);
    RansacPlaneDetector.setThreshold_point_number(pointDensity/4);
  
    float filterSize1 = 0.05f;
    float filterSize2 = 0.4f;
    float filterParameter = 0.5f;
    
    // parameter of object
    float chairSize = 0.8f;
    float chairRange = 3f;//1.5f
    int objectPointNumberThreshold = pointDensity/2;
    float growRadius = 0.03f;
    
    // remove all the planes
    ArrayList<PointCloud> completePlanes =
      RansacPlaneDetector.findCompletePlanesFast(pc, oct, snorm, edgeRadius);
    System.out.println("3:plane extraction done!");

    PointCloud pc_complete_all = new PointCloud(0);
    Colour c1 = new Colour(pc_complete_all.getNumPoints(), 255, 255, 255);
    pc_complete_all.setColours(c1);
    c1 = null;

    PointCloud pc_complete_large = new PointCloud(0);
    Colour c4 = new Colour(pc_complete_all.getNumPoints(), 255, 255, 255);
    pc_complete_large.setColours(c4);
    c4 = null;

    PointCloud pc_rest = completePlanes.get(completePlanes.size() - 1);
      
    int j = 1;
    for (int k = 0; k < completePlanes.size() - 1; k++) {
      pc_complete_all.append(completePlanes.get(k));
      System.out.print((k + 1) + ":" + completePlanes.get(k).getNumPoints()
          / RansacPlaneDetector.getEdgeSize().get(k) + ", ");
      if ((ceil-completePlanes.get(k).mCoordinates.findBoundingBox().zMax<=0.5 && 
    		  completePlanes.get(k).mCoordinates.findBoundingBox().zMin-floor<=0.5) ||
    		  completePlanes.get(k).getNumPoints()
          / RansacPlaneDetector.getEdgeSize().get(k) > plane_density_threshold){
    	  IOFunctions.writeTextXYZ(
    			  completePlanes.get(k),
    	          "C:/Users/PangTouXian/Documents/UCL/java eclipse/result/object_segmentation/plane/completePlane_"
    	              + Integer.toString(j++) + ".xyz");
    	  pc_complete_large.append(completePlanes.get(k));
      }
      else
        pc_rest.append(completePlanes.get(k));
    }

    IOFunctions.writeTextXYZ(pc_complete_large,
        "C:/Users/PangTouXian/Documents/UCL/java eclipse/result/object_segmentation/plane_complete_large.txt");

    IOFunctions
        .writeTextXYZ(pc_rest,
        "C:/Users/PangTouXian/Documents/UCL/java eclipse/result/object_segmentation/rest.txt");
    OctreeIndex oct_rest = new OctreeIndex(pc_rest, octreeRadius);
     
    // filter low density points 
    SegmentObjects.setRadiusSearch(filterSize1);
    SegmentObjects.setPointNumThreshold((int) (pointDensity*filterSize1*filterSize1*filterParameter));
  
    PointCloud pc_denoise1 = SegmentObjects.denoise(pc_rest, oct_rest);
    OctreeIndex oct_denoise1 = new OctreeIndex(pc_denoise1, octreeRadius);
   
    SegmentObjects.setRadiusSearch(filterSize2);
    SegmentObjects.setPointNumThreshold((int) (pointDensity*filterSize2*filterSize2*filterParameter)); 

    PointCloud pc_denoise2 = SegmentObjects.denoise(pc_denoise1, oct_denoise1);

    IOFunctions
        .writeTextXYZ(
            pc_denoise2,
        "C:/Users/PangTouXian/Documents/UCL/java eclipse/result/object_segmentation/rest_denoise.txt");
    System.out.println("  point number change:" + pc_rest.getNumPoints()
        +
     " ; " + pc_denoise2.getNumPoints());
    System.out.println("4:denoising done!");
      
    // segment objects 
    ArrayList<PointCloud> pc_object = SegmentObjects.findObjectsRegionGrow(pc_denoise2,
 octreeRadius, growRadius);
      
    PointCloud pc_object_all = new PointCloud(0); 
    Colour c2 = new Colour(pc_object_all.getNumPoints(), 255, 255, 255);
    pc_object_all.setColours(c2); 
    c2 = null;
    
    PointCloud pc_chair_all = new PointCloud(0);
    Colour c3 = new Colour(pc_chair_all.getNumPoints(), 255, 255, 255);
    pc_chair_all.setColours(c3);
    c3 = null;
      
    // save each object and select potential chairs
    ArrayList<Integer> chairIndex = new ArrayList<Integer>();
    for (int i = 0; i < pc_object.size(); i++) {
    	if(pc_object.get(i).getNumPoints() < objectPointNumberThreshold)
    		continue;
    	IOFunctions.writeTextXYZ(pc_object.get(i),
          "C:/Users/PangTouXian/Documents/UCL/java eclipse/result/object_segmentation/object/object_"
              + Integer.toString(i) + ".txt");
    	pc_object_all.append(pc_object.get(i));
    	double xLength = pc_object.get(i).mCoordinates.findBoundingBox().xLength();
    	double yLength = pc_object.get(i).mCoordinates.findBoundingBox().yLength();
    	double zLength = pc_object.get(i).mCoordinates.findBoundingBox().zLength();
    	if (xLength > chairSize / chairRange && xLength < chairSize * chairRange
    	          && yLength > chairSize / chairRange
    	 && yLength < chairSize * chairRange
    	          && zLength > chairSize / chairRange
    	 && zLength < chairSize * chairRange){
    		pc_chair_all.append(pc_object.get(i));
        	chairIndex.add(i);
        	IOFunctions.writeTextXYZ(
                    pc_object.get(i),
                    "C:/Users/PangTouXian/Documents/UCL/java eclipse/result/object_segmentation/chair/object_"
                        + Integer.toString(i) + ".txt");
    	}	
    }

    IOFunctions
        .writeTextXYZ(pc_object_all,
        "C:/Users/PangTouXian/Documents/UCL/java eclipse/result/object_segmentation/object_all.txt");

    // chair recognition
    System.out.print("  recognised chair number: ");
    IOFunctions
        .writeTextXYZ(pc_chair_all,
        "C:/Users/PangTouXian/Documents/UCL/java eclipse/result/object_segmentation/chairs.txt");

    // import cnn parameters to java
    PointCloud pc_chair_rec = new PointCloud(0);
    Colour c5 = new Colour(pc_chair_rec.getNumPoints(), 255, 255, 255);
    pc_chair_rec.setColours(c5);
    c5 = null;

    int[] classNum = new int[chairIndex.size()];
    for (int i = 0; i < chairIndex.size(); i++) {
      classNum[i] = cnn16for4angles.classification(chairIndex.get(i));
      if (classNum[i] == 3) {
        pc_chair_rec.append(pc_object.get(chairIndex.get(i)));
        System.out.print(chairIndex.get(i) + ", ");
      }
    }

    IOFunctions.writeTextXYZ(pc_chair_rec,
        "C:/Users/PangTouXian/Documents/UCL/java eclipse/result/object_segmentation/chairs_rec.txt");

    System.out.println("End!");
    long end = System.currentTimeMillis();
    System.out.println("time:" + (end - start));
  }
}