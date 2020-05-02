package hierarchicalClustering;
import java.util.ArrayList;

import uk.ac.ucl.jpl.Point3;
import uk.ac.ucl.jpl.PointCloud;
import uk.ac.ucl.jpl.SurfaceNormals;
import uk.ac.ucl.jpl.colour.Colour;
import uk.ac.ucl.jpl.io.IOFunctions;
import uk.ac.ucl.jpl.spatialindexing.OctreeIndex;

//BAO's method, seed point fixed.  

public class RegionGrowSeedPointFixTest {
  public static void main(String[] args) throws Exception {
	 
    long start = System.currentTimeMillis();

    String name1 =
        "C:/Users/PangTouXian/Desktop/paper picture & data/shopping mall/EM001_250000.txt";
    
    //point cloud parameters
    float snRadius = 0.1f;
    float octreeRadius = 0.1f;

    //parameters for seed point selection
    double angleThreshold = 0.05f;
    float angularRadius = 0.1f;
    int sizeThreshold = 30;
    
    //parameters for planar point selection
    float radiusSearch = 0.1f;
    double angle_threshold = 12f;
    float pf_radius = 0.05f;
    
    PointCloud pc1 = IOFunctions.readTextXYZ(name1);
    
    PointCloud pc_all_planes = new PointCloud(0);
    Colour c0 = new Colour(pc_all_planes.getNumPoints(), 255, 255, 255);
    pc_all_planes.setColours(c0);
    c0 = null;

    PointCloud pc_all_planes_colour = new PointCloud(0);
    Colour c00 = new Colour(pc_all_planes_colour.getNumPoints(), 255, 255, 255);
    pc_all_planes_colour.setColours(c00);
    c00 = null;

    ArrayList<Integer> index1 = new ArrayList<Integer>();
    ArrayList<Integer> n2 = new ArrayList<Integer>();
    ArrayList<Integer> n3 = new ArrayList<Integer>();
    ArrayList<Integer> n4 = new ArrayList<Integer>();
    ArrayList<Integer> ns = new ArrayList<Integer>();
    
    PlaneFactorCompute1 pfc = new PlaneFactorCompute1();
    AngleCompute ac = new AngleCompute();

    int num = 0;

    int R = 0;
    int G = 0;
    int B = 0;

    int stop_loop = 0;

    do {
      System.out.println("num:" + num);
      if (stop_loop == 1)
        break;
      OctreeIndex oct1 = new OctreeIndex(pc1.mCoordinates, octreeRadius);
      
      System.out.println("pc1:" + pc1.getNumPoints());
      
      SurfaceNormals snorm1 = new SurfaceNormals(pc1.getNumPoints());
      snorm1.calculateFromRadius(pc1.mCoordinates, oct1, snRadius);
      pc1.setSurfaceNormals(snorm1);
      pc1.getSurfaceNormals().fixNormalsToViewpoint(
          new Point3(1000, 1000, 1000), pc1.mCoordinates);

      AngleCompute.setSharedReferences(snorm1, pc1.mCoordinates, oct1,
          angularRadius);

      int seedpointIndex = 0;
      boolean test = true;
      int k1 = -1;

      //make an array of the same size as pc1, 0 represents the point is not tested, 1 represents tested
      int[] pointStatus = new int[pc1.getNumPoints()];
      
      // find seed point
      do {
        double angle = ac.process(seedpointIndex, sizeThreshold);
        if ( angle < angleThreshold && angle >= 0){
          test = false;
          ns.add(seedpointIndex);
          pointStatus[seedpointIndex]=1;
        } else
          seedpointIndex++;
      } while (test && seedpointIndex < pc1.getNumPoints() - 1);

      if (seedpointIndex == pc1.getNumPoints() - 1) {
        stop_loop = 1;
        System.out.println("break!");
        break;
      }
      
      // find plane points
      PlaneFactorCompute1.setSharedReferences(snorm1, pc1.mCoordinates, oct1,
          pf_radius);

      do {
        for (int l : ns) {
          Point3 p1 = new Point3();
          pc1.mCoordinates.getPoint(l, p1);
          oct1.radiusSearch(p1, radiusSearch, index1);

          for (int j : index1) {
            if (pointStatus[j]==1) 
            	continue;
            double PF = pfc.process(seedpointIndex, j, angle_threshold);
            if (PF >= 1) {
              n2.add(j);
              pointStatus[j]=1;
            } 
          }
        }
        n3.addAll(n2);

        // update seed points
        ns.clear();
        ns.addAll(n2);
        k1 = n2.size();
        n2.clear();
      } while (k1 > 0);

      if (n3.size() == 0) {
        n3.add(seedpointIndex);
        pc1.clearIndexed(n3);
        continue;
      }
      
      PointCloud pc1_plane = pc1.makeSubCloud(n3);

      System.out.println("plane_point:" + n3.size());
      num++;

      //add colour to plane
      switch (num%20) {
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

      pc_all_planes.append(pc1_plane);
      Colour c = new Colour(pc1_plane.getNumPoints(), R, G, B);
      pc1_plane.setColours(c);
      pc_all_planes_colour.append(pc1_plane);

      System.out.println("pc_all_planes:" + pc_all_planes.getNumPoints());

      String name6 =
          "C:/Users/PangTouXian/Desktop/paper picture & data/shopping mall/region grow/plane"
              + num
              + ".txt";
      IOFunctions.writeTextXYZ(pc1_plane, name6);

      for (int j = 0; j < pc1.getNumPoints(); j++) {
        if (!n3.contains(j)) 
          n4.add(j);
      }

      PointCloud rest = pc1.makeSubCloud(n4);
      n3.clear();
      n4.clear();
      pc1 = rest;
    } while (true);


    String name2 =
        "C:/Users/PangTouXian/Desktop/paper picture & data/shopping mall/region grow/plane_all.txt";
    IOFunctions.writeTextXYZ(pc_all_planes, name2);
    String name3 =
        "C:/Users/PangTouXian/Desktop/paper picture & data/shopping mall/region grow/plane_all_colour.txt";
    IOFunctions.writeTextXYZ(pc_all_planes_colour, name3);
    String name4 =
        "C:/Users/PangTouXian/Desktop/paper picture & data/shopping mall/region grow/not_plane.txt";
    IOFunctions.writeTextXYZ(pc1, name4);

    long end = System.currentTimeMillis();
    System.out.println("time:" + (end - start));
  }

}

