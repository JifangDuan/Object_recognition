package ransac;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Random;
import java.util.Set;

import uk.ac.ucl.jpl.Coordinates;
import uk.ac.ucl.jpl.Point3;
import uk.ac.ucl.jpl.PointCloud;
import uk.ac.ucl.jpl.SurfaceNormals;
import uk.ac.ucl.jpl.colour.Colour;
import uk.ac.ucl.jpl.spatialindexing.OctreeIndex;

public class OriginalRansacPlaneDetector {
//most of the parameters need to be adjusted according to specific case.
	
  // parameters of random three point selection
  private static float upper_threshold_angle = 0.03f;
  private static float lower_min_triangle_edge = 0.1f;
  private static int iterations = 1000000;
  
  // parameters of planar points selection
  private static float threshold_distance_plane = 0.05f;
  private static float threshold_angle_plane = 0.1f;
  private static ArrayList<Point3> n = new ArrayList<Point3>();
  
  // main program1: basic RANSAC
  public static ArrayList<PointCloud> findPlanes(PointCloud pointcloud,
      OctreeIndex oct, SurfaceNormals snorm) {
    ArrayList<PointCloud> planes = new ArrayList<PointCloud>();
    ArrayList<Integer> planar_points = new ArrayList<Integer>();
    PointCloud pc = new PointCloud(pointcloud);
    pc.setSurfaceNormals(snorm);
    float octMinNodeSize = oct.getMinNodeSize();
    n.clear();
    System.out.println("  input:" + pc.getNumPoints());
    
    if (pointcloud.hasColours()) {
        Colour c = pointcloud.getColours();
        pc.setColours(c);
      }
    
    int nIndex = 0;
    int i = 0;// iteration index
    do {
          int[] rand_ints = generate3RandomPoints(pc);
          i++;
          boolean isPlane = isPlane(rand_ints, pc, oct);
          if (isPlane) {
            System.out.print("    plane found" + planes.size() + 1 + ":");
            planar_points =
                findPlanarPointsSimple(pc, n.get(nIndex), rand_ints, octMinNodeSize);
            System.out.println(planar_points.size());
            PointCloud pc_small = extractPlane(pc, planar_points);
            PointCloud pc_small_colour =
                addColourToEachPlane(planes.size() + 1, pc_small);
            planes.add(pc_small_colour);
            Collections.sort(planar_points);
            pc.clearIndexed(planar_points);
            oct = new OctreeIndex(pc, octMinNodeSize);
            i = 0;
            nIndex++;
            planar_points.clear();
          }
    } while (i <= iterations);


    System.out.println("  Number of plane:" + planes.size());
    planes.add(pc);
    System.out.println("  rest:" + pc.getNumPoints());
    return planes;
  }
  
  
// classes used in main programs
  
  // Identifies 3 unique random numbers (index points) from a point cloud
  private static int[] generate3RandomPoints(PointCloud pc) {
    int pointcloud_size = pc.getNumPoints();
    Random random_gen = new Random();
    int[] three_randints = new int[3];
    
    // add random integers to set until its size is 3 or greater
    Set<Integer> three_randintset = new LinkedHashSet<Integer>();
    do {
      three_randintset.add(random_gen.nextInt(pointcloud_size));
    } while (three_randintset.size() < 3);
    Iterator<Integer> iter = three_randintset.iterator();
    int i = 0;
    while (iter.hasNext()) {
      three_randints[i] = iter.next();
      i++;
    }
    return three_randints;
  }

  /*
   * Identify a plane from the normals of 3 points. If the normals of the 3
   * points and the normal of the plane formed by the three points is
   * sufficiently close, then the points belong to a plane
   */
  private static boolean isPlane(int[] three_randints, PointCloud pc,
      OctreeIndex oct) {

    // get normals of three random integers
    SurfaceNormals snorm = pc.getSurfaceNormals();
    Point3 n1 = snorm.getPoint(three_randints[0]);
    Point3 n2 = snorm.getPoint(three_randints[1]);
    Point3 n3 = snorm.getPoint(three_randints[2]);

    // get coordinates of three random integers
    Coordinates coords = pc.mCoordinates;
    Point3 p1 = new Point3();
    coords.getPoint(three_randints[0], p1);
    Point3 p2 = new Point3();
    coords.getPoint(three_randints[1], p2);
    Point3 p3 = new Point3();
    coords.getPoint(three_randints[2], p3);

    // get distance of the edge of the triangle
    float d1 = (float) Math.sqrt(p1.distSqr(p2));
    float d2 = (float) Math.sqrt(p2.distSqr(p3));
    float d3 = (float) Math.sqrt(p3.distSqr(p1));

    // find the normal of the plane
    Point3 p1p2 = p1.minus(p2);
    Point3 p2p3 = p2.minus(p3);
    Point3 np = p1p2.cross(p2p3);

    if (np.angle(n1) < upper_threshold_angle
        && np.angle(n2) < upper_threshold_angle
        && np.angle(n3) < upper_threshold_angle && d1 > lower_min_triangle_edge
        && d2 > lower_min_triangle_edge && d3 > lower_min_triangle_edge) {
      n.add(np);
      return true;
    }
    return false;
  }

  /*
   * Find other planar points by comparison with normal of the plane and
   * distance to the plane
   */
  private static ArrayList<Integer> findPlanarPointsSimple(PointCloud pc,
      Point3 p_norm, int[] three_randints, float octMinNodeSize) {

    ArrayList<Integer> planar_points = new ArrayList<Integer>();
    Point3 p = new Point3();
    pc.mCoordinates.getPoint(three_randints[0], p);
    Point3 test_point;
    Point3 test_point_norm = new Point3();
    SurfaceNormals snorm = pc.getSurfaceNormals();
    float distToPlane;

    for (int i = 0; i < pc.getNumPoints(); i++) {
      // get distance of test point to plane
      Point3 a = new Point3();
      pc.mCoordinates.getPoint(i, a);
      test_point = a.minus(p);
      p_norm.normalise();
      distToPlane = Math.abs(test_point.dot(p_norm));

      // get normal of test point
      snorm.getPoint(i, test_point_norm);

      /*
       * if distance and angle of test point fall below threshold, then add
       * point as member of a plane
       */
      if (distToPlane < threshold_distance_plane
          && test_point_norm.angle(p_norm) < threshold_angle_plane) {
        planar_points.add(i);
      }
    }
    return planar_points;
  }

  //add colour to each plane
  public static PointCloud addColourToEachPlane(int plane_num,
      PointCloud pc_small) {
    int R = 0;
    int G = 0;
    int B = 0;

    switch (plane_num % 20) {
    case 1:
      R = 200;
      G = 0;
      B = 0;
      break;
    case 2:
      R = 200;
      G = 71;
      B = 0;
      break;
    case 3:
      R = 200;
      G = 143;
      B = 0;
      break;
    case 4:
      R = 91;
      G = 36;
      B = 164;
      break;
    case 5:
      R = 195;
      G = 200;
      B = 0;
      break;
    case 6:
      R = 124;
      G = 200;
      B = 0;
      break;
    case 7:
      R = 52;
      G = 200;
      B = 0;
      break;
    case 8:
      R = 0;
      G = 200;
      B = 52;
      break;
    case 9:
      R = 110;
      G = 124;
      B = 76;
      break;
    case 10:
      R = 0;
      G = 200;
      B = 195;
      break;
    case 11:
      R = 0;
      G = 138;
      B = 200;
      break;
    case 12:
      R = 0;
      G = 67;
      B = 200;
      break;
    case 13:
      R = 15;
      G = 147;
      B = 18;
      break;
    case 14:
      R = 14;
      G = 3;
      B = 159;
      break;
    case 15:
      R = 92;
      G = 62;
      B = 79;
      break;
    case 16:
      R = 34;
      G = 42;
      B = 120;
      break;
    case 17:
      R = 200;
      G = 0;
      B = 119;
      break;
    case 18:
      R = 173;
      G = 161;
      B = 163;
      break;
    case 19:
      R = 200;
      G = 0;
      B = 48;
      break;
    case 0:
      R = 164;
      G = 36;
      B = 103;
      break;
    }
    Colour c = new Colour(pc_small.getNumPoints(), R, G, B);
    pc_small.setColours(c);
    return pc_small;
  }

  private static PointCloud extractPlane(PointCloud pc,
      ArrayList<Integer> planar_points) {
    PointCloud plane = pc.makeSubCloud(planar_points);
    return plane;
  }

  public static void setThreshold_angle_plane(float threshold_angle_plane) {
    OriginalRansacPlaneDetector.threshold_angle_plane = threshold_angle_plane;
  }

  public static float getThreshold_angle_plane() {
    return threshold_angle_plane;
  }

  public static float getUpper_threshold_angle() {
    return upper_threshold_angle;
  }

  public static void setUpper_threshold_angle(float upper_threshold_angle) {
    OriginalRansacPlaneDetector.upper_threshold_angle = upper_threshold_angle;
  }

  public static float getLower_min_triangle_edge() {
    return lower_min_triangle_edge;
  }

  public static void setLower_min_triangle_edge(float lower_min_triangle_edge) {
    OriginalRansacPlaneDetector.lower_min_triangle_edge = lower_min_triangle_edge;
  }

  public static int getIterations() {
    return iterations;
  }

  public static void setIterations(int iterations) {
    OriginalRansacPlaneDetector.iterations = iterations;
  }

  public static float getThreshold_distance_plane() {
    return threshold_distance_plane;
  }

  public static void setThreshold_distance_plane(float threshold_distance_plane) {
    OriginalRansacPlaneDetector.threshold_distance_plane = threshold_distance_plane;
  }
}
