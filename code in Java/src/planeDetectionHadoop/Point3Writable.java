package planeDetectionHadoop;

import java.io.DataInput;
import java.io.DataOutput;
import java.io.IOException;

import org.apache.hadoop.io.Writable;

import uk.ac.ucl.jpl.Point3;

public class Point3Writable extends Point3 implements Writable {

  public Point3Writable() {
    super();
  }

  public Point3Writable(Point3 p) {
    super(p);
  }

  @Override
  public void readFields(DataInput in) throws IOException {
    this.x = in.readFloat();
    this.y = in.readFloat();
    this.z = in.readFloat();
  }

  @Override
  public void write(DataOutput out) throws IOException {
    out.writeFloat(this.x);
    out.writeFloat(this.y);
    out.writeFloat(this.z);
  }
}
