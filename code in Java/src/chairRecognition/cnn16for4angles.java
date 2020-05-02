package chairRecognition;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;

import uk.ac.ucl.jpl.PointCloud;
import uk.ac.ucl.jpl.io.IOFunctions;

public class cnn16for4angles {

  public static int classification(int s) throws IOException {
    int n = 0;

    float[] opttheta1 = new float[100525];
    String pathname1 =
        "C:/Users/PangTouXian/Documents/UCL/java eclipse/cnn/cnn3D_2filters_16_4/opttheta1.txt";
    File filename1 = new File(pathname1);
    InputStreamReader reader1 =
        new InputStreamReader(new FileInputStream(filename1));
    BufferedReader br1 = new BufferedReader(reader1);
    String line1 = "";
    line1 = br1.readLine();
    int i = 0;
    while (line1 != null) {
      opttheta1[i] = Float.parseFloat(line1);
      i++;
      line1 = br1.readLine();
    }

    float[] opttheta2 = new float[584329];
    String pathname2 =
        "C:/Users/PangTouXian/Documents/UCL/java eclipse/cnn/cnn3D_2filters_16_4/opttheta2.txt";
    File filename2 = new File(pathname2);
    InputStreamReader reader2 =
        new InputStreamReader(new FileInputStream(filename2));
    BufferedReader br2 = new BufferedReader(reader2);
    String line2 = "";
    line2 = br2.readLine();
    i = 0;
    while (line2 != null) {
      opttheta2[i] = Float.parseFloat(line2);
      i++;
      line2 = br2.readLine();
    }

    float[] softmax = new float[140000];
    String pathname3 =
        "C:/Users/PangTouXian/Documents/UCL/java eclipse/cnn/cnn3D_2filters_16_4/softmaxModel.txt";
    File filename3 = new File(pathname3);
    InputStreamReader reader3 =
        new InputStreamReader(new FileInputStream(filename3));
    BufferedReader br3 = new BufferedReader(reader3);
    String line3 = "";
    line3 = br3.readLine();
    i = 0;
    while (line3 != null) {
      softmax[i] = Float.parseFloat(line3);
      i++;
      line3 = br3.readLine();
    }

    float[][] softmaxModel = new float[10][14000];
    for (int j = 0; j < 140000; j++) {
      softmaxModel[j - 10 * (j / 10)][j / 10] = softmax[j];
    }

    String fileName1 =
        "C:/Users/PangTouXian/Documents/UCL/java eclipse/result/object_segmentation/chair/object_"
            + Integer.toString(s) + ".txt";

    PointCloud pc = IOFunctions.readTextXYZ(fileName1);
    int[][][] testData = pointcloudToVoxel.voxelise(pc);
    convPool.setTestData(testData);
    float[][][][] pooledFeatures1 = convPool.convolvePool(opttheta1, 5);
    float[][][][] pooledFeatures2 = convPool.convolvePool(opttheta2, 9);
    float[][] pooledFeatures = new float[400][35];
    for (int j = 0; j < 400; j++) {
      for (int k = 0; k < 35; k++) {
        if (k < 27)
          pooledFeatures[j][k] =
              pooledFeatures1[j][k % 3][(int) Math.floor((k % 9) / 3)][(int) Math
                  .floor(k / 9)];
        else
          pooledFeatures[j][k] =
              pooledFeatures2[j][(k - 27) % 2][(int) Math
                  .floor(((k - 27) % 4) / 2)][(int) Math.floor((k - 27) / 4)];
      }
    }
    float[] pooledFeaturesInARow = new float[14000];
    for (int j = 0; j < 14000; j++) {
      pooledFeaturesInARow[j] =
          pooledFeatures[j % 400][(int) Math.floor(j / 400)];
    }

    float[] val = new float[10];
    for (int j = 0; j < 10; j++) {
      for (int k = 0; k < 14000; k++) {
        val[j] += softmaxModel[j][k] * pooledFeaturesInARow[k];
      }

    }

    float max = val[0];
    for (int j = 1; j < 10; j++) {
      if (val[j] > max) {
        max = val[j];
        n = j;
      }
    }

    n++;
    return n;
  }

}
