package planeDetectionHadoop;


import java.util.ArrayList;

import uk.ac.ucl.jpl.Point3;
import uk.ac.ucl.jpl.PointCloud;
import uk.ac.ucl.jpl.SurfaceNormals;
import uk.ac.ucl.jpl.io.IOFunctions;
import uk.ac.ucl.jpl.spatialindexing.OctreeIndex;


public class DataSplit {

  public static void main(String[] args) throws Exception {

    float oct_size = 0.01f;
    float sn_radius = 0.05f;

    String fileName1 =
        "/scratch/uceedua/data/data_shoppingmall/EM013_cloud.txt";
    PointCloud lab = IOFunctions.readTextXYZ(fileName1);
    OctreeIndex oct = new OctreeIndex(lab, oct_size);
    SurfaceNormals snorm = new SurfaceNormals(lab.getNumPoints());
    snorm.calculateFromRadius(lab, oct, sn_radius);
    snorm.fixNormalsToViewpoint(new Point3(), lab.mCoordinates);
    lab.setSurfaceNormals(snorm);

    System.out.println("number of points all:" + lab.getNumPoints());

    // make 3 sub pointclouds
    ArrayList<Integer> subcloud = new ArrayList<Integer>();
    for (int i = 0; i < lab.getNumPoints() / 3; i++) {
      subcloud.add(i);
    }
    PointCloud sub1 = lab.makeSubCloud(subcloud);
    subcloud.clear();

    for (int i = lab.getNumPoints() / 3; i < 2 * lab.getNumPoints() / 3; i++) {
      subcloud.add(i);
    }
    PointCloud sub2 = lab.makeSubCloud(subcloud);
    subcloud.clear();

    for (int i = 2 * lab.getNumPoints() / 3; i < lab.getNumPoints(); i++) {
      subcloud.add(i);
    }
    PointCloud sub3 = lab.makeSubCloud(subcloud);

    System.out.println("number of points each:" + sub1.getNumPoints() + ","
        + sub2.getNumPoints() + "," + sub3.getNumPoints());

    String fileName2 =
        "/scratch/uceedua/data/data_shoppingmall/sn/EM13/EM013_cloud_sn";
    IOFunctions.writeTextXYZ(sub1, fileName2 + "_1.txt");
    IOFunctions.writeTextXYZ(sub2, fileName2 + "_2.txt");
    IOFunctions.writeTextXYZ(sub3, fileName2 + "_3.txt");

    System.out.println("separate original file into 3 files: done!");
  }
}
