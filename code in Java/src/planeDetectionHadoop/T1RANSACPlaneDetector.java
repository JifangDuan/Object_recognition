package planeDetectionHadoop;

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

public class T1RANSACPlaneDetector {

  private static float threshold_angle;
  private static float threshold_distance;
  private static int iterations;
  private static float min_triangle_edge;
  private static ArrayList<Point3> normals = null;
  private static ArrayList<Point3> normals_accurate = null;
  private static int planeNumbers;

  public T1RANSACPlaneDetector() {
    threshold_angle = (float) (5f / 180 * Math.PI);
    threshold_distance = 0.2f;
    iterations = 100000;
    min_triangle_edge = 1f;
    normals = new ArrayList<Point3>();
    normals_accurate = new ArrayList<Point3>();
    planeNumbers = 0;
  }

  public ArrayList<PointCloud> findPlanes(PointCloud pointCloud,
      OctreeIndex oct, SurfaceNormals snorm) {

    ArrayList<PointCloud> planes = new ArrayList<PointCloud>();
    ArrayList<Integer> planar_points = new ArrayList<Integer>();
    PointCloud pc = new PointCloud(pointCloud);
    pc.setSurfaceNormals(snorm);
    if (pointCloud.hasColours()) {
      Colour c = pointCloud.getColours();
      pc.setColours(c);
    }
    float octMinNodeSize = oct.getMinNodeSize();
    Point3 n = new Point3();
    int i = 0;

    do {

      int[] rand_ints = generate3RandomPoints(pc);
      i++;

      pc.getSurfaceNormals().getPoint(rand_ints[0], n);

      boolean isPlane = isPlane(rand_ints, pc, oct, n);
      if (isPlane) {
        System.out.print("plane found ");
        planar_points =
            findPlanarPoints(pc, n, rand_ints, getThreshold_distance(),
                getThreshold_angle());
        System.out.println(planar_points.size());

        PointCloud pc_sub = extractPlane(pc, planar_points);
        planes.add(pc_sub);
        normals_accurate.add(findAccurateNormals(pc_sub));
        Collections.sort(planar_points);
        pc.clearIndexed(planar_points);
        oct = new OctreeIndex(pc, octMinNodeSize);
        i = 0;
        Point3 n1 = new Point3();
        n1.x = n.x;
        n1.y = n.y;
        n1.z = n.z;
        normals.add(n1);

        planeNumbers++;
      }

    } while (i <= getIterations());
    planes.add(pc);
    return planes;
  }

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
      OctreeIndex oct, Point3 n) {

    // get normals of three random integers
    SurfaceNormals snorm = pc.getSurfaceNormals();
    Point3 n1 = new Point3();
    snorm.getPoint(three_randints[0], n1);
    Point3 n2 = new Point3();
    snorm.getPoint(three_randints[1], n2);
    Point3 n3 = new Point3();
    snorm.getPoint(three_randints[2], n3);

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
    p1.minus(p2);
    p2.minus(p3);

    Point3.cross(p1, p2, n);

    if (n.angle(n1) < getThreshold_angle()
        && n.angle(n2) < getThreshold_angle()
        && n.angle(n3) < getThreshold_angle() && d1 > getMin_triangle_edge()
        && d2 > getMin_triangle_edge() && d3 > getMin_triangle_edge()) {
      return true;
    }

    return false;
  }

  /*
   * Find other planar points by comparison with normal of the plane and
   * distance to the plane
   */
  private static ArrayList<Integer> findPlanarPoints(PointCloud pc,
      Point3 p_norm, int[] three_randints, float threshold_distance,
      float threshold_angle) {

    ArrayList<Integer> planar_points = new ArrayList<Integer>();
    Point3 p = new Point3();
    pc.mCoordinates.getPoint(three_randints[0], p);
    Point3 test_point = new Point3();
    Point3 test_point_norm = new Point3();
    SurfaceNormals snorm = pc.getSurfaceNormals();
    float distToPlane;


    for (int i = 0; i < pc.getNumPoints(); i++) {
      
      // get distance of test point to plane
      pc.mCoordinates.getPoint(i, test_point);
      test_point.minus(p);

      p_norm.normalise();
      distToPlane = Math.abs(test_point.dot(p_norm));

      // get normal of test point
      snorm.getPoint(i, test_point_norm);

      /*
       * if distance and angle of test point fall below threshold, then add point
       * as member of a plane
       */
      if (distToPlane < threshold_distance
          && test_point_norm.angle(p_norm) < threshold_angle) {
        planar_points.add(i);
      }
    }

    return planar_points;
  }

  private Point3 findAccurateNormals(PointCloud pc) {
    Point3 n = new Point3();
    Point3 n0 = new Point3();
    for (int i = 0; i < pc.getNumPoints(); i++) {
      pc.getSurfaceNormals().getPoint(i, n);
      n0.add(n);

    }
    n0.normalise();
    return n0;
  }

  private PointCloud extractPlane(PointCloud pc,
      ArrayList<Integer> planar_points) {

    PointCloud plane = pc.makeSubCloud(planar_points);
    return plane;
  }

  public static float getThreshold_angle() {
    return threshold_angle;
  }

  public void setThreshold_angle(float threshold_angle) {
    T1RANSACPlaneDetector.threshold_angle = threshold_angle;
  }

  public float getThreshold_distance() {
    return threshold_distance;
  }

  public void setThreshold_distance(float threshold_distance) {
    T1RANSACPlaneDetector.threshold_distance = threshold_distance;
  }

  public int getIterations() {
    return iterations;
  }

  public void setIterations(int iterations) {
    T1RANSACPlaneDetector.iterations = iterations;
  }

  public static float getMin_triangle_edge() {
    return min_triangle_edge;
  }

  public void setMin_triangle_edge(float min_triangle_edge) {
    T1RANSACPlaneDetector.min_triangle_edge = min_triangle_edge;
  }

  public ArrayList<Point3> getNormals() {
    return normals;
  }

  public ArrayList<Point3> getAccurateNormals() {
    return normals_accurate;
  }

  public int getPlaneNumers() {
    return planeNumbers;
  }

}
