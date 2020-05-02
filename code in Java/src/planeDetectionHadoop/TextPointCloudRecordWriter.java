package planeDetectionHadoop;

import java.io.BufferedWriter;
import java.io.IOException;

import org.apache.hadoop.conf.Configuration;

import org.apache.hadoop.mapreduce.RecordWriter;
import org.apache.hadoop.mapreduce.TaskAttemptContext;

import uk.ac.ucl.jpl.io.IOFunctions;
import uk.ac.ucl.jpl_hadoop.hdfsutils.Outputs;
import uk.ac.ucl.jpl_hadoop.writables.PointCloudWritable;

public class TextPointCloudRecordWriter extends
    RecordWriter<T1MapOutputKey, PointCloudWritable> {
  Configuration mConf = null;
  String mOutputPath = "";

  public TextPointCloudRecordWriter(String outputPath) {
    super();
    StringBuilder sb = new StringBuilder();
    sb.append(outputPath);
    if (!outputPath.endsWith("/"))
      sb.append("/");
    mOutputPath = sb.toString();
  }

  @Override
  public void close(TaskAttemptContext arg0) throws IOException,
      InterruptedException {
  }

  @Override
  public void write(T1MapOutputKey key, PointCloudWritable value)
      throws IOException, InterruptedException {
    if (null == value)
      throw new IOException("PointCloudWritable vale is null");

    String outFileName = mOutputPath;

    outFileName += value.getNumPoints() + ".txt";

    if (null == mConf)
      throw new IOException("mConf is null, set it fist using setConf");

    BufferedWriter bout =
        Outputs.createBufferedWriter(outFileName, mConf, true);
    IOFunctions.writeTextXYZ(value, bout);
    bout.flush();
    bout.close();
  }

  public void setConf(Configuration conf) {
    mConf = conf;
  }
}
