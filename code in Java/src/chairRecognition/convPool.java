package chairRecognition;

public class convPool {

  private static int cubeDim = 16;
  private static int[][][] testData = new int[cubeDim][cubeDim][cubeDim];
  private static int poolDim = 4;
  private static int hiddenSize = 400;
  public static float[][][][] convolvePool(float[] optTheta, int patchDim) {
    int visibalSize = patchDim * patchDim * patchDim;
    float[][] w = new float[hiddenSize][visibalSize];
    float[] b = new float[hiddenSize];
    for (int i = 0; i < hiddenSize; i++) {
      b[i] = optTheta[2 * hiddenSize * visibalSize + i];
      for (int j = 0; j < visibalSize; j++) {
        w[i][j] = optTheta[j * hiddenSize + i];
      }
    }
    float[][][][] convolvedFeatures = convolve3D(patchDim, w, b);
    float[][][][] pooledFeatures = pool3D(patchDim, poolDim, convolvedFeatures);
    return pooledFeatures;
  }

  public static float[][][][] convolve3D(int patchDim,
      float[][] w,
      float[] b) {
	  
    int patchSize = patchDim * patchDim * patchDim;
    int convolvedDim = cubeDim - patchDim + 1;
    float[][][][] convolvedFeatures =
        new float[hiddenSize][convolvedDim][convolvedDim][convolvedDim];
    int[] convolvedData = new int[patchSize];
    float x = 0;
    for (int i = 0; i < hiddenSize; i++) {
      for (int j = 0; j < convolvedDim; j++) {
        for (int k = 0; k < convolvedDim; k++) {
          for (int l = 0; l < convolvedDim; l++) {
            for (int m = 0; m < patchSize; m++) {
              convolvedData[m] =
                  testData[m % patchDim + j][(int) Math
                      .floor((m % (patchDim * patchDim)) / patchDim) + k][(int) Math
                      .floor(m / (patchDim * patchDim)) + l];

              x += w[i][m] * convolvedData[m];
            }
            convolvedFeatures[i][j][k][l] =
                (float) (1 / (1 + Math.exp(-(x + b[i]))));
            x = 0;
          }
        }
      }
    }
    return convolvedFeatures;
  }

  public static float[][][][] pool3D(int patchDim, int poolDim,
      float[][][][] convolvedFeatures) {
    int convolvedDim = cubeDim - patchDim + 1;
    int resultDim = convolvedDim / poolDim;
    float[][][][] pooledFeatures =
        new float[hiddenSize][resultDim][resultDim][resultDim];

    for (int i = 0; i < hiddenSize; i++) {
      for (int j = 0; j < convolvedDim - poolDim + 1; j += poolDim) {
        for (int k = 0; k < convolvedDim - poolDim + 1; k += poolDim) {
          for (int l = 0; l < convolvedDim - poolDim + 1; l += poolDim) {
            float sum = 0;
            for (int m = j; m < j + poolDim; m++) {
              for (int n = k; n < k + poolDim; n++) {
                for (int o = l; o < l + poolDim; o++) {
                  sum += convolvedFeatures[i][m][n][o];
                }
              }
            }
            pooledFeatures[i][j / poolDim][k / poolDim][l / poolDim] =
                (float) (sum / (Math.pow(poolDim, 3)));
          }
        }
      }
    }
    return pooledFeatures;
  }

  public static void setTestData(int[][][] testData) {
    convPool.testData = testData;
  }
}

