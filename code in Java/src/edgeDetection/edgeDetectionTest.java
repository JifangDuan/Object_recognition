package edgeDetection;
//BAO's plane based edge detection method 
//edge detection from extracted planes. 
//input is a series of planes (can be obtained by ugridSizeng RANSAC)
//output is extracted edges for each plane and for all the planes

import java.util.ArrayList;

import uk.ac.ucl.jpl.*;
import uk.ac.ucl.jpl.colour.Colour;
import uk.ac.ucl.jpl.io.IOFunctions;
import uk.ac.ucl.jpl.scalarfields.Scalar;
import uk.ac.ucl.jpl.spatialindexing.OctreeIndex;

public class edgeDetectionTest {

  public static void main(String[] args) throws Exception {
	//  More accurate edge detection result can be obtained with smaller gridSize, but with more processing time.
	// gridSize set to too small will cause errors in detection result (e.g. smaller than average distance between two points)
	double gridSize = 0.03;
	
    PointCloud pc0 = new PointCloud(0);
    Colour c1 = new Colour(pc0.getNumPoints(), 255, 255, 255);
    pc0.setColours(c1);
    c1 = null;
    ArrayList<Integer> gridSizeze = new ArrayList<Integer>();
    gridSizeze.add(0);

    for (int num = 1; num < 6; num++)//num need to be adjusted according to specific case
    {
      //each input file should be an extracted plane (can be obtained by using ransac, hierarchicalClustering or my plane detection method)
      String fileName1 =
    		  "C:/Users/PangTouXian/Documents/UCL/java eclipse/result/object_segmentation/plane/completePlane_"
    	              + num + ".xyz";
      PointCloud pc = IOFunctions.readTextXYZ(fileName1);
      OctreeIndex oct = new OctreeIndex(pc, 0.05f);
      SurfaceNormals snorm = new SurfaceNormals(pc.getNumPoints());

      snorm.calculateFromRadius(pc.mCoordinates, oct, oct.getMinNodeSize());
      pc.setSurfaceNormals(snorm);
      pc.getSurfaceNormals().fixNormalsToViewpoint(
          new Point3(1000, 1000, 1000), pc.mCoordinates);
      
      Point3 np = new Point3();
      Point3 p = new Point3();
      Point3 u = new Point3();
      Point3 v = new Point3();

      for (int i = 0; i < pc.getNumPoints(); i++) {
        Point3 n0 = new Point3();
        snorm.getPoint(i, n0);
        np.add(n0);
      }
      
      float nx = np.x / pc.getNumPoints();
      float ny = np.y / pc.getNumPoints();
      float nz = np.z / pc.getNumPoints();
      np.set(nx, ny, nz);

      pc.mCoordinates.getPoint(0, p);

      u.x = (np.x * p.x + np.y * p.y + np.z * p.z) / np.x;
      u.y = 0;
      u.z = 0;
      Point3 pu = new Point3();
      pu = p.minus0(u);
      pu.normalise();

      v.z = 0;
      v.y =
          (p.y * np.y * u.x - p.x * p.y * np.y + p.z * np.z * u.x - p.z * p.x
              * np.z + np.x * p.y * p.y + np.x * p.z * p.z)
              / (p.y * np.x + np.y * u.x - np.y * p.x);
      v.x = (p.y * np.y + p.z * np.z - v.y * np.y + p.x * np.x) / np.x;
      Point3 pv = new Point3();
      pv = p.minus0(v);
      pv.normalise();
      
      Point3 a = new Point3();
      Scalar ua = new Scalar(pc.getNumPoints());
      Scalar va = new Scalar(pc.getNumPoints());

      int ind1 = pc.addScalar(ua);
      int ind2 = pc.addScalar(va);

      pc.mCoordinates.getPoint(0, a);
      a.minus1(p);
      ua.mData[0] = a.dot(pu);
      va.mData[0] = a.dot(pv);
      double x_min = ua.mData[0];
      double x_max = ua.mData[0];
      double y_min = va.mData[0];
      double y_max = va.mData[0];

      for (int i = 1; i < pc.getNumPoints(); i++) {
        pc.mCoordinates.getPoint(i, a);
        a.minus1(p);
        ua.mData[i] = a.dot(pu);
        va.mData[i] = a.dot(pv);
        if (ua.mData[i] < x_min)
          x_min = ua.mData[i];
        if (ua.mData[i] > x_max)
          x_max = ua.mData[i];
        if (va.mData[i] < y_min)
          y_min = va.mData[i];
        if (va.mData[i] > y_max)
          y_max = va.mData[i];
      }
      
      pc.setScalar(ind1, ua);
      pc.setScalar(ind2, va);
   
      int m = (int) ((x_max - x_min) / gridSize + 2);
      int n = (int) ((y_max - y_min) / gridSize + 2);
      int s[][] = new int[m][n];

      Scalar h1 = new Scalar(pc.getNumPoints());
      Scalar h2 = new Scalar(pc.getNumPoints());

      int ind3 = pc.addScalar(h1);
      int ind4 = pc.addScalar(h2);

      for (int i = 0; i < pc.getNumPoints(); i++) {
        s[(int) Math.floor((ua.mData[i] - x_min) / gridSize) + 1][(int) Math
            .floor((va.mData[i] - y_min) / gridSize) + 1] = 1;
        h1.mData[i] = (float) (Math.floor((ua.mData[i] - x_min) / gridSize) + 1);
        h2.mData[i] = (float) (Math.floor((va.mData[i] - y_min) / gridSize) + 1);
      }

      pc.setScalar(ind3, h1);
      pc.setScalar(ind4, h2);
      
      ArrayList<Integer> l1 = new ArrayList<Integer>();
      ArrayList<Integer> l2 = new ArrayList<Integer>();

      for (int i = 1; i < m - 1; i++) {
        for (int j = 1; j < n - 1; j++) {
          int t =
              s[i - 1][j - 1] + s[i - 1][j] + s[i - 1][j + 1] + s[i][j - 1]
                  + s[i][j + 1] + s[i + 1][j - 1] + s[i + 1][j]
                  + s[i + 1][j + 1];
          if (t < 7 && t > 4)
          {
            l1.add(i);
            l2.add(j);
          }
        }
      }
    
      ArrayList<Integer> n0 = new ArrayList<Integer>();

      for (int k = 0; k < pc.getNumPoints(); k++) {
        for (int i = 0; i < l1.size(); i++) {
          if (l1.get(i) == (int) (pc.getScalar(ind3).mData[k])
              && l2.get(i) == (int) (pc.getScalar(ind4).mData[k]))
            n0.add(k);
        }

      }
      PointCloud pc1 = pc.makeSubCloud(n0);

      gridSizeze.add(n0.size());

      PointCloud pc1_colour = addColourToEachPlane(num, pc1);

      String fileName2 =
    		  "C:/Users/PangTouXian/Documents/UCL/java eclipse/result/object_segmentation/edge/part_edge"+num+".txt";
      IOFunctions.writeTextXYZ(pc1_colour, fileName2);
      pc0.append(pc1_colour);
      System.out.println(num + "done");
    }

    String fileName3 =
    		"C:/Users/PangTouXian/Documents/UCL/java eclipse/result/object_segmentation/part_edge_all.txt";
    IOFunctions.writeTextXYZ(pc0, fileName3);  
}

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
}

