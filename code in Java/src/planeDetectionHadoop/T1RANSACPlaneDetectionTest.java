package planeDetectionHadoop;


import java.io.IOException;
import java.util.ArrayList;

import uk.ac.ucl.jpl.Point3;
import uk.ac.ucl.jpl.PointCloud;
import uk.ac.ucl.jpl.SurfaceNormals;
import uk.ac.ucl.jpl.TestCloudGenerator;
import uk.ac.ucl.jpl.io.IOFunctions;
//import uk.ac.ucl.jpl.planedetection.PCAPlaneFitting;

import uk.ac.ucl.jpl.spatialindexing.OctreeIndex;

public class T1RANSACPlaneDetectionTest {

  public static void main(String[] args) throws IOException {

    int numpoints = 60000;
    PointCloud cube =
        TestCloudGenerator.makeRandomHollowCube(numpoints, 1f, 0.01f);
    IOFunctions.writeTextXYZ(cube, "cube.xyz");

    OctreeIndex oct = new OctreeIndex(cube, 0.01f);
    SurfaceNormals snorm = new SurfaceNormals(cube.getNumPoints());
    snorm.calculateFromRadius(cube, oct, 0.05f);
    snorm.fixNormalsToViewpoint(new Point3(), cube.mCoordinates);
    cube.setSurfaceNormals(snorm);

    T1RANSACPlaneDetector t1 = new T1RANSACPlaneDetector();

    ArrayList<PointCloud> planes =
 t1.findPlanes(cube, oct, snorm);

    ArrayList<Point3> normals = t1.getNormals();
    ArrayList<Point3> normals_accurate = t1.getAccurateNormals();

    System.out.println(normals.get(0));
    System.out.println(normals.get(1));
    System.out.println(normals_accurate.get(0));
    System.out.println(normals_accurate.get(1));

	int i = 1;
    for (PointCloud pc : planes) {

      IOFunctions.writeTextXYZ(pc,
 "cube" + Integer.toString(i) + ".xyz");
      i++;
    }
    System.out.println("done");
  }
}

