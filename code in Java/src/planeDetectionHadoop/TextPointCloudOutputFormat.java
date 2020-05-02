package planeDetectionHadoop;

import java.io.IOException;

import org.apache.hadoop.conf.Configuration;
import org.apache.hadoop.fs.Path;

import org.apache.hadoop.mapreduce.JobContext;
import org.apache.hadoop.mapreduce.OutputCommitter;
import org.apache.hadoop.mapreduce.OutputFormat;
import org.apache.hadoop.mapreduce.RecordWriter;
import org.apache.hadoop.mapreduce.TaskAttemptContext;
import org.apache.hadoop.mapreduce.lib.output.FileOutputCommitter;
import org.apache.hadoop.mapreduce.lib.output.FileOutputFormat;


import uk.ac.ucl.jpl_hadoop.writables.PointCloudWritable;

public class TextPointCloudOutputFormat extends
    OutputFormat<T1MapOutputKey, PointCloudWritable> {

  protected FileOutputCommitter committer;

  public TextPointCloudOutputFormat() {
    super();
  }

  @Override
  public RecordWriter<T1MapOutputKey, PointCloudWritable> getRecordWriter(
      TaskAttemptContext context) throws IOException, InterruptedException {

    String outputPath = FileOutputFormat.getOutputPath(context).toString();
    TextPointCloudRecordWriter recordWriter =
        new TextPointCloudRecordWriter(outputPath);
    Configuration conf = context.getConfiguration();
    recordWriter.setConf(conf);
    return recordWriter;
  }

  @Override
  public void checkOutputSpecs(JobContext arg0) throws IOException,
      InterruptedException {
    // TODO Auto-generated method stub
  }

  @Override
  public OutputCommitter getOutputCommitter(TaskAttemptContext context)
      throws IOException, InterruptedException {
    if (committer == null) {
      Path oPath = FileOutputFormat.getOutputPath(context);
      committer = new FileOutputCommitter(oPath, context);
    }
    return committer;
  }
}
