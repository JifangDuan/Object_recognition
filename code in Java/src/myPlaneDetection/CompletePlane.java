package myPlaneDetection;

import java.util.ArrayList;

import Jama.Matrix;
import uk.ac.ucl.jpl.Point3;
import uk.ac.ucl.jpl.PointCloud;
import uk.ac.ucl.jpl.scalarfields.Scalar;

public class CompletePlane {

  // u and v are the two coordinates in the reference system of each plane, p is
  // the origin of the reference system
  private static Point3 u = new Point3();
  private static Point3 v = new Point3();
  private static Point3 p = new Point3();
  private static float grid_size = 0.08f;
  private static double plane_size;

  // find edge points of all the planes in pc, pc is an extracted plane by using
  // RANSAC (may contain a lot of parts)
  public static PointCloud EdgePoints(PointCloud pc){

    Matrix featVect = PlaneFitting.getFeatVect(pc);
    double[][] uv = featVect.getArray();
    u.x = (float) uv[0][0];
    u.y = (float) uv[1][0];
    u.z = (float) uv[2][0];
    v.x = (float) uv[0][1];
    v.y = (float) uv[1][1];
    v.z = (float) uv[2][1];
    
    pc.mCoordinates.getPoint(0, p);

    Scalar ua = new Scalar(pc.getNumPoints());
    Scalar va = new Scalar(pc.getNumPoints());
    Point3 a = new Point3();
    pc.mCoordinates.getPoint(0, a);
    a.minus(p);
    ua.mData[0] = a.dot(u);
    va.mData[0] = a.dot(v);
    double x_min = ua.mData[0];
    double x_max = ua.mData[0];
    double y_min = va.mData[0];
    double y_max = va.mData[0];

    // find the plane length and width
    for (int i = 1; i < pc.getNumPoints(); i++) {
      pc.mCoordinates.getPoint(i, a);
      a.minus(p);
      ua.mData[i] = a.dot(u);
      va.mData[i] = a.dot(v);
      if (ua.mData[i] < x_min)
        x_min = ua.mData[i];
      if (ua.mData[i] > x_max)
        x_max = ua.mData[i];
      if (va.mData[i] < y_min)
        y_min = va.mData[i];
      if (va.mData[i] > y_max)
        y_max = va.mData[i];
    }
    plane_size = (x_max - x_min) * (y_max - y_min);

    // separate the plane into m rows and n columns
    int m = (int) ((x_max - x_min) / grid_size + 2);
    int n = (int) ((y_max - y_min) / grid_size + 2);
    int s[][] = new int[m][n];
    Scalar h1 = new Scalar(pc.getNumPoints());
    Scalar h2 = new Scalar(pc.getNumPoints());

    int ind1 = pc.addScalar(h1);
    int ind2 = pc.addScalar(h2);

    for (int i = 0; i < pc.getNumPoints(); i++) {
      s[(int) Math.floor((ua.mData[i] - x_min) / grid_size) + 1][(int) Math
          .floor((va.mData[i] - y_min) / grid_size) + 1] = 1;
      h1.mData[i] = (float) (Math.floor((ua.mData[i] - x_min) / grid_size) + 1);
      h2.mData[i] = (float) (Math.floor((va.mData[i] - y_min) / grid_size) + 1);
    }

    pc.setScalar(ind1, h1);
    pc.setScalar(ind2, h2);

    ArrayList<Integer> l1 = new ArrayList<Integer>();
    ArrayList<Integer> l2 = new ArrayList<Integer>();

    for (int i = 1; i < m - 1; i++) {
      for (int j = 1; j < n - 1; j++) {
        int t =
            s[i - 1][j - 1] + s[i - 1][j] + s[i - 1][j + 1] + s[i][j - 1]
                + s[i][j + 1] + s[i + 1][j - 1] + s[i + 1][j] + s[i + 1][j + 1]
                + s[i][j];
        if (t < 9 && t > 4) {
          l1.add(i);
          l2.add(j);
        }
      }
    }

    ArrayList<Integer> n0 = new ArrayList<Integer>();

    for (int k = 0; k < pc.getNumPoints(); k++) {
      for (int i = 0; i < l1.size(); i++) {
        if (l1.get(i) == (int) (pc.getScalar(ind1).mData[k])
            && l2.get(i) == (int) (pc.getScalar(ind2).mData[k]))
          n0.add(k);
      }

    }
    return pc.makeSubCloud(n0);
  }

  public static Point3 getU() {
    return u;
  }

  public static Point3 getV() {
    return v;
  }

  public static double getPlaneSize() {
    return plane_size;
  }

  public static void setGridSize(float gs) {
    grid_size = gs;
  }

}
