package planeDetectionHadoop;

import java.io.IOException;
import java.math.BigDecimal;
import java.util.ArrayList;

import org.apache.hadoop.io.Text;
import org.apache.hadoop.mapreduce.Mapper;

import uk.ac.ucl.jpl.Point3;
import uk.ac.ucl.jpl.PointCloud;

import uk.ac.ucl.jpl.spatialindexing.OctreeIndex;

import uk.ac.ucl.jpl_hadoop.writables.PointCloudWritable;

public class T1Mapper extends
    Mapper<Text, PointCloudWritable, T1MapOutputKey, PointCloudWritable> {
	
  T1RANSACPlaneDetector t1 = new T1RANSACPlaneDetector();

  @Override
  public void setup(Context context) throws IOException {

  }

  @Override
  public void map(Text keyIn, PointCloudWritable valIn, Context context)
      throws IOException, InterruptedException {

    System.out.println("start Mapper");
    System.out.println("======================================");
    System.out.println("\tkey in : " + keyIn);
    System.out.println("\tnum points: " + valIn.getNumPoints());
    System.out.println("\trecord name : " + valIn.recordName);
    System.out.println("======================================");

    OctreeIndex oct = new OctreeIndex(valIn, 0.05f);

    ArrayList<PointCloud> planes =
        t1.findPlanes(valIn, oct, valIn.getSurfaceNormals());

    System.out.println("planes size:" + (planes.size() - 1));
    
    float thresholdP = 2f;
    float thresholdN = 5;
    //data centre coordinates(better to write a function to compute)
    float x_origin = -3;
    float y_origin = -3;
    float z_origin = 94;

    for (int i = 0; i < planes.size() - 1; i++) {
      PointCloud pc = new PointCloud(planes.get(i));
      Point3 n = new Point3();
      n = t1.getAccurateNormals().get(i);

      System.out.println(n);

      Point3 p = new Point3();
      pc.mCoordinates.getPoint(0, p);

      float D = 0;

      if (Math.abs(n.z) > Math.abs(n.y) && Math.abs(n.z) > Math.abs(n.x)) {
        if (n.z < 0) {
          n.x *= -1;
          n.y *= -1;
          n.z *= -1;
        }

      } else if (Math.abs(n.y) > Math.abs(n.x) && Math.abs(n.y) > Math.abs(n.z)) {
        if (n.y < 0) {
          n.x *= -1;
          n.y *= -1;
          n.z *= -1;
        }

      } else {
        if (n.x < 0) {
          n.x *= -1;
          n.y *= -1;
          n.z *= -1;
        }

      }
      D =
          p.x * n.x + p.y * n.y + p.z * n.z - x_origin * n.x - y_origin * n.y
              - z_origin * n.z;
      float pz = D;

      BigDecimal a = BigDecimal.valueOf(pz * thresholdP);
      pz = a.setScale(0, BigDecimal.ROUND_HALF_UP).floatValue() / thresholdP;

      Point3Writable pw = new Point3Writable();
      pw.set(0, 0, pz);

      Point3Writable nw = new Point3Writable(n);

      BigDecimal x = BigDecimal.valueOf(nw.x * thresholdN);
      nw.x = x.setScale(0, BigDecimal.ROUND_HALF_UP).floatValue() / thresholdN;
      BigDecimal y = BigDecimal.valueOf(nw.y * thresholdN);
      nw.y = y.setScale(0, BigDecimal.ROUND_HALF_UP).floatValue() / thresholdN;
      BigDecimal z = BigDecimal.valueOf(nw.z * thresholdN);
      nw.z = z.setScale(0, BigDecimal.ROUND_HALF_UP).floatValue() / thresholdN;

      System.out.println("plane" + i + ":" + nw.x + "," + nw.y + "," + nw.z
          + "    " + pz);

      T1MapOutputKey mok = new T1MapOutputKey(pw, nw);

      PointCloudWritable pcw = new PointCloudWritable();
      try {
        pcw = new PointCloudWritable(pc);
      } catch (Exception e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
      context.write(mok, pcw);
    }
    System.out.println("Mapper done");
  }
}
