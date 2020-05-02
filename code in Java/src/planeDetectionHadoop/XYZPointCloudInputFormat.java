package planeDetectionHadoop;

/**
 * requires following .jar files to compile:
 *  hadoop-common-2.2.0.jar
 *  hadoop-mapreduce-client-core-2.2.0.jar
 *  commons-logging-1.1.1.jar
 */
import java.io.IOException;

import org.apache.hadoop.io.*;
import org.apache.hadoop.fs.*;
import org.apache.hadoop.mapreduce.InputSplit;
import org.apache.hadoop.mapreduce.JobContext;
import org.apache.hadoop.mapreduce.TaskAttemptContext;
import org.apache.hadoop.mapreduce.lib.input.FileInputFormat;
import org.apache.hadoop.mapreduce.RecordReader;

import uk.ac.ucl.jpl_hadoop.inputs.XYZFileRecordReader;
import uk.ac.ucl.jpl_hadoop.writables.PointCloudWritable;

public class XYZPointCloudInputFormat extends
    FileInputFormat<Text, PointCloudWritable> {
  @Override
  protected boolean isSplitable(JobContext context, Path filename) {
    return false;
  }

  @Override
  public RecordReader<Text, PointCloudWritable> createRecordReader(
      InputSplit split, TaskAttemptContext context) throws IOException,
      InterruptedException {
    XYZFileRecordReader reader = new XYZFileRecordReader();
    reader.initialize(split, context);
    return reader;
  }
}
