package planeDetectionHadoop;

import java.io.DataInput;
import java.io.DataOutput;
import java.io.IOException;

import org.apache.hadoop.io.WritableComparable;


public class T1MapOutputKey implements WritableComparable<T1MapOutputKey> {

  private Point3Writable point = null;
  private Point3Writable normal = null;

  public T1MapOutputKey() {
    try {
      point = new Point3Writable();
      normal = new Point3Writable();

    } catch (Exception e) {
      throw new Error(e.getMessage());
    }
  }

  public T1MapOutputKey(Point3Writable point, Point3Writable normal) {
    this.point = point;
    this.normal = normal;
  }

  @Override
  public void readFields(DataInput in) throws IOException {
    point.readFields(in);
    normal.readFields(in);
  }

  @Override
  public void write(DataOutput out) throws IOException {
    point.write(out);
    normal.write(out);
  }
  
  @Override
  public int compareTo(T1MapOutputKey t) {

    if (this.hashCode() < t.hashCode())
      return -1;
    else if (this.hashCode() == t.hashCode())
      return 0;
    else
      return 1;
  }

  @Override
  public int hashCode() {
    return (int) (point.z * 1000000 + normal.x * 100000 + normal.y * 1000 + normal.z * 10);
  }

  public void setPoint(Point3Writable point) {
    this.point = point;
  }

  public void setNormal(Point3Writable normal) {
    this.normal = normal;
  }

  public Point3Writable getPoint() {
    return point;
  }

  public Point3Writable getNormal() {
    return normal;
  }

}