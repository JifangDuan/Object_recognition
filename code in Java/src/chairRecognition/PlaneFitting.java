package chairRecognition;

import Jama.EigenvalueDecomposition;
import Jama.Matrix;
import uk.ac.ucl.jpl.Coordinates;
import uk.ac.ucl.jpl.Point3;
import uk.ac.ucl.jpl.PointCloud;


//use PCA for dimension reduction. A 3D plane model is converted to 2D plane
public class PlaneFitting {

  private static Point3 mean;
  private static Matrix featVect;

  public static PointCloud FlattenPlane(PointCloud pointcloud) {
    getFeatVect(pointcloud);
    Matrix dataAdjust = DataAdjust(pointcloud);
    Matrix transformedData = TransformedData(dataAdjust);
    Matrix reducedData = ReducedData(transformedData);
    PointCloud flattenedPlane = MatrixToPointCloud(reducedData);
    flattenedPlane.setColours(pointcloud.getColours());
    return flattenedPlane;
  }

  public static Matrix CovarianceMatrix(PointCloud pointcloud) {
    Point3 mean = new Point3();
    Point3 p = new Point3();
    Coordinates coords = pointcloud.mCoordinates;
    for (int i = 0; i < pointcloud.getNumPoints(); i++) {
      coords.getPoint(i, p);
      mean.plus(p, mean);
    }
    mean.scale(1f / pointcloud.getNumPoints());
    setMean(mean);
    double[][] cov = new double[3][3];
    for (int i = 0; i < pointcloud.getNumPoints(); i++) {
      coords.getPoint(i, p);
      p.minus(mean);
      cov[0][0] += p.x * p.x;
      cov[0][1] += p.x * p.y;
      cov[0][2] += p.x * p.z;
      cov[1][1] += p.y * p.y;
      cov[1][2] += p.y * p.z;
      cov[2][2] += p.z * p.z;
    }
    
    cov[1][0] = cov[0][1];
    cov[2][0] = cov[0][2];
    cov[2][1] = cov[1][2];

    for (int i = 0; i < cov.length; i++) {
      for (int j = 0; j < cov.length; j++) {
        cov[i][j] /= (pointcloud.getNumPoints() - 1);
      }
    }

    Matrix covM = new Matrix(cov);
    return covM;
  }

  public static Matrix FeatureVector(Matrix covM) {
    EigenvalueDecomposition eigenDecomp = new EigenvalueDecomposition(covM);
    Matrix eigenVal = eigenDecomp.getD();
    Matrix eigenVec = eigenDecomp.getV();

    double eigenMin =
        Math.min(eigenVal.get(0, 0),
            Math.min(eigenVal.get(1, 1), eigenVal.get(2, 2)));

    int[] eigenMaxIdxs = new int[2];
    if (eigenMin == eigenVal.get(0, 0)) {
      eigenMaxIdxs[0] = 1;
      eigenMaxIdxs[1] = 2;
    } else if (eigenMin == eigenVal.get(1, 1)) {
      eigenMaxIdxs[0] = 0;
      eigenMaxIdxs[1] = 2;
    } else {
      eigenMaxIdxs[0] = 0;
      eigenMaxIdxs[1] = 1;
    }

    Matrix featureVec = eigenVec.getMatrix(0, 2, eigenMaxIdxs);
    return featureVec;
  }

  public static Matrix DataAdjust(PointCloud pointcloud) {
    double[][] dataAdjustArray = new double[3][pointcloud.getNumPoints()];
    Coordinates coords = pointcloud.mCoordinates;
    Point3 p = new Point3();
    for (int i = 0; i < pointcloud.getNumPoints(); i++) {
      coords.getPoint(i, p);
      p = p.minus(getMean());

      dataAdjustArray[0][i] = p.x;
      dataAdjustArray[1][i] = p.y;
      dataAdjustArray[2][i] = p.z;
    }

    Matrix dataAdjust = new Matrix(dataAdjustArray);
    return dataAdjust;
  }

  public static Matrix TransformedData(Matrix rowDataAdjust) {
    Matrix rowFeatureVect = featVect.transpose();

    return rowFeatureVect.times(rowDataAdjust);
  }

  public static Matrix ReducedData(Matrix transformedData) {
    return featVect.times(transformedData);
  }

  public static PointCloud MatrixToPointCloud(Matrix reducedData) {
    PointCloud pointcloud = new PointCloud(reducedData.getColumnDimension());
    Point3 p;

    for (int i = 0; i < pointcloud.getNumPoints(); i++) {
      p =
          new Point3((float) reducedData.get(0, i), (float) reducedData.get(1,
              i), (float) reducedData.get(2, i));
      p.plus(getMean(), p);
      pointcloud.mCoordinates.setPoint(i, p);
    }

    return pointcloud;
  }

  private static void setMean(Point3 mean) {
    PlaneFitting.mean = mean;
  }

  private static Point3 getMean() {
    return PlaneFitting.mean;
  }

  public static Matrix getFeatVect(PointCloud pc) {
    Matrix covM = CovarianceMatrix(pc);
    featVect = FeatureVector(covM);
    return featVect;
  }
}
