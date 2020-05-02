package chairRecognition;

import uk.ac.ucl.jpl.PointCloud;
import uk.ac.ucl.jpl.spatialindexing.VoxelGridIndex;
import uk.ac.ucl.jpl.spatialindexing.VoxelIndex;

// convert point cloud data to voxel data
public class pointcloudToVoxel {

  private static int nx = 16;
  private static int ny = 16;
  private static int nz = 16;

  public static int[][][] voxelise(PointCloud pc) {
    VoxelGridIndex vind = new VoxelGridIndex(pc.mCoordinates, nx, ny, nz);
    int gridNum = vind.getNumVoxels();
    int[][][] voxel = new int[nx][ny][nz];

    for (int i = 0; i < gridNum; i++) {
      VoxelIndex v = vind.getVoxel(i);
      if (v.getIndexCount() > 0) {
        voxel[i % nx][(int) Math.floor((i % (nx * ny)) / nx)][(int) Math
            .floor(i / (nx * ny))] = 1;
      } else
        voxel[i % nx][(int) Math.floor((i % (nx * ny)) / nx)][(int) Math
            .floor(i / (nx * ny))] = 0;
    }
    return voxel;
  }

}
