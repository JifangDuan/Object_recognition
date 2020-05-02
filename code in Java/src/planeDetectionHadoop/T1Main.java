package planeDetectionHadoop;

import org.apache.hadoop.conf.Configuration;
import org.apache.hadoop.conf.Configured;
import org.apache.hadoop.fs.Path;
//import org.apache.hadoop.io.Text;

import org.apache.hadoop.mapreduce.Job;
import org.apache.hadoop.mapreduce.lib.input.FileInputFormat;
import org.apache.hadoop.mapreduce.lib.output.FileOutputFormat;

import org.apache.hadoop.util.Tool;
import org.apache.hadoop.util.ToolRunner;

import uk.ac.ucl.jpl_hadoop.writables.PointCloudWritable;

//import org.apache.hadoop.mapred.lib.MultipleOutputs;
//import org.apache.hadoop.mapreduce.lib.output.MultipleOutputs;

public class T1Main extends Configured implements Tool {

  public static String mInputClouds;
  public static String mOutputPath;

  public static void main(String[] args) throws Exception {
    Configuration conf = new Configuration();
    int res = ToolRunner.run(conf, new T1Main(), args);
    System.out.println("EXIT OK");
    System.exit(res);
  }

  @Override
  public int run(String[] args) throws Exception {
    if (args.length != 2) {
      throw new IllegalArgumentException(
          "expected 2 input arguments for job, got " + args.length);
    } else {
      mInputClouds = args[0];
      mOutputPath = args[1];

      if (!mOutputPath.endsWith("/"))
        mOutputPath += "/";
    }

    Configuration conf = this.getConf();
    Job job = Job.getInstance(conf);
    
    job.setJarByClass(T1Main.class);
    job.setMapperClass(T1Mapper.class);
    job.setCombinerClass(T1Combiner.class);
    job.setReducerClass(T1Reducer.class);

    FileInputFormat.addInputPath(job, new Path(mInputClouds));
    FileOutputFormat.setOutputPath(job, new Path(mOutputPath));

    job.setInputFormatClass(uk.ac.ucl.jpl_hadoop.inputs.XYZFileInputFormat.class);
    job.setMapOutputKeyClass(T1MapOutputKey.class);
    job.setMapOutputValueClass(PointCloudWritable.class);
    job.setOutputFormatClass(TextPointCloudOutputFormat.class);

    System.exit(job.waitForCompletion(true) ? 0 : 1);
    return 0;
  }
}
