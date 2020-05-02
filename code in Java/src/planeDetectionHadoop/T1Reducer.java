package planeDetectionHadoop;

import java.io.IOException;

import org.apache.hadoop.mapreduce.Reducer;
import uk.ac.ucl.jpl_hadoop.writables.PointCloudWritable;
import org.apache.hadoop.mapreduce.lib.output.MultipleOutputs;

public class T1Reducer
    extends
    Reducer<T1MapOutputKey, PointCloudWritable, T1MapOutputKey, PointCloudWritable> {

  MultipleOutputs<T1MapOutputKey, PointCloudWritable> mMultiOutputs;
  private PointCloudWritable pcw = new PointCloudWritable();
  private int i = 0;

  @Override
  protected void setup(Context context) throws IOException,
      InterruptedException {
    mMultiOutputs =
        new MultipleOutputs<T1MapOutputKey, PointCloudWritable>(context);
    super.setup(context);
  }

  protected void cleanup(Context context) throws IOException,
      InterruptedException {
    mMultiOutputs.close();
    super.cleanup(context);
  }

  @Override
  public void reduce(T1MapOutputKey key, Iterable<PointCloudWritable> values,
      Context context) throws IOException, InterruptedException {
    i++;
    pcw = new PointCloudWritable();

    System.out.println("-----------REDUCE WITH KEY" + i + "(" + key.hashCode()
        + ")"
        + ":"
        + key.getNormal().x
        + "," + key.getNormal().y + "," + key.getNormal().z + ";"
        + key.getPoint().z + "------------");

    for (PointCloudWritable val : values) {
      if (pcw.isEmpty()) {
        try {
          pcw = new PointCloudWritable(val);
        } catch (Exception e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }

      } else {
        pcw.append(val);
      }
    }
    System.out.println(pcw.getNumPoints());

    try {
      mMultiOutputs.write(key, pcw, "plane" + i);
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }
}
