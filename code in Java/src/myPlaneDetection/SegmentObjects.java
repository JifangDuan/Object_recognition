package myPlaneDetection;

import java.util.ArrayList;
import java.util.Collections;

import uk.ac.ucl.jpl.Point3;
import uk.ac.ucl.jpl.PointCloud;
import uk.ac.ucl.jpl.colour.Colour;
import uk.ac.ucl.jpl.spatialindexing.OctreeIndex;
import uk.ac.ucl.jpl.spatialindexing.VoxelGridIndex;
import uk.ac.ucl.jpl.spatialindexing.VoxelIndex;

public class SegmentObjects {

  private static float radiusSearch = 0.03f;
  private static int pointNumThreshold = 3;
  private static ArrayList<Integer> objectGridNumber = new ArrayList<Integer>();
  private static ArrayList<Integer> lengthX = new ArrayList<Integer>();
  private static ArrayList<Integer> lengthY = new ArrayList<Integer>();
  private static ArrayList<Integer> lengthZ = new ArrayList<Integer>();

  //denoise
  public static PointCloud denoise(PointCloud pointcloud, OctreeIndex oct) {
    ArrayList<Integer> n = new ArrayList<Integer>();
    for (int i = 0; i < pointcloud.getNumPoints(); i++) {
      Point3 p = new Point3();
      pointcloud.mCoordinates.getPoint(i, p);
      int pointNum = oct.radiusCount(p, radiusSearch);
      if (pointNum > pointNumThreshold) {
        n.add(i);
      }
    }
    return pointcloud.makeSubCloud(n);
  }

  //segment each object using voxel grid based region growing
  public static ArrayList<PointCloud> findObjects(PointCloud pc,
      OctreeIndex oct, float dx, float dy, float dz) {

    lengthX = new ArrayList<Integer>();
    lengthY = new ArrayList<Integer>();
    lengthZ = new ArrayList<Integer>();

    ArrayList<PointCloud> objects = new ArrayList<PointCloud>();
    VoxelGridIndex vind = new VoxelGridIndex(pc.mCoordinates, dx, dy, dz);
    int gridNum = vind.getNumVoxels();
    ArrayList<Integer> index = new ArrayList<Integer>();// add all the grid that has point inside

    for (int i = 0; i < gridNum; i++) {
        VoxelIndex v = vind.getVoxel(i);
        if (v.getIndexCount() != 0 ) {
          index.add(i);
        }
    }

    System.out.println("  All grid:" + gridNum + "    Grid have point:"
        + index.size());

    ArrayList<Integer> n = new ArrayList<Integer>();
    ArrayList<Integer> n1 = new ArrayList<Integer>();
    ArrayList<Integer> n_all = new ArrayList<Integer>();
    ArrayList<Integer> n_object = new ArrayList<Integer>();

    do {
      n.add(index.get(0));
      n_all.add(index.get(0));
      do {
        for (int i = 0; i < n.size(); i++) {
          ArrayList<Integer> neighbour =
              findNeighbouringGrid(n.get(i), vind.getNumX(), vind.getNumY(),
                  vind.getNumZ());
          
          for (int j = 0; j < neighbour.size(); j++){
            if (index.contains(neighbour.get(j))
                && !n_all.contains(neighbour.get(j))
                && !n.contains(neighbour.get(j))) {
              n1.add(neighbour.get(j));
              n_all.add(neighbour.get(j));
            }
          }
        }
        n.clear();
        n.addAll(n1);
        n1.clear();
      } while (n.size() > 0);

      VoxelIndex v1 = vind.getVoxel(n_all.get(0));
      int xMin = v1.xStepCount();
      int xMax = v1.xStepCount();
      int yMin = v1.yStepCount();
      int yMax = v1.yStepCount();
      int zMin = v1.zStepCount();
      int zMax = v1.zStepCount();
      for (int i = 0; i < n_all.size(); i++) {
        v1 = vind.getVoxel(n_all.get(i));
        n_object.addAll(v1.getAllIndices());
        if (v1.xStepCount() < xMin)
          xMin = v1.xStepCount();
        if (v1.xStepCount() > xMax)
          xMax = v1.xStepCount();
        if (v1.yStepCount() < yMin)
          yMin = v1.yStepCount();
        if (v1.yStepCount() > yMax)
          yMax = v1.yStepCount();
        if (v1.zStepCount() < zMin)
          zMin = v1.zStepCount();
        if (v1.zStepCount() > zMax)
          zMax = v1.zStepCount();
      }

      lengthX.add(xMax - xMin + 1);
      lengthY.add(yMax - yMin + 1);
      lengthZ.add(zMax - zMin + 1);

      index.removeAll(n_all);
      System.out.println("    find object" + (objects.size() + 1) + ":"
          + n_all.size()
          + "   index:" + index.size());
      objectGridNumber.add(n_all.size());

      n_all.clear();
      PointCloud pc_colour = pc.makeSubCloud(n_object);
      RansacPlaneDetector.addColourToEachPlane(objects.size() + 1, pc_colour);
      objects.add(pc_colour);
      n_object.clear();
    } while (index.size() > 0);
    return objects;
  }

  //segment each object using voxel grid based region growing
  public static ArrayList<PointCloud> findObjectsFast(PointCloud pc,
	      OctreeIndex oct, float dx, float dy, float dz) {

    lengthX = new ArrayList<Integer>();
    lengthY = new ArrayList<Integer>();
    lengthZ = new ArrayList<Integer>();

    ArrayList<PointCloud> objects = new ArrayList<PointCloud>();
    VoxelGridIndex vind = new VoxelGridIndex(pc.mCoordinates, dx, dy, dz);
    int gridNum = vind.getNumVoxels();
    ArrayList<Integer> index = new ArrayList<Integer>();// add all the grid that has point inside
    int[] gridStatus = new int[gridNum];

    for (int i = 0; i < gridNum; i++) {
        VoxelIndex v = vind.getVoxel(i);
        if (v.getIndexCount() != 0 ) {
          index.add(i);
        }
    }

    System.out.println("  All grid:" + gridNum + "    Grid have point:"
        + index.size());

    ArrayList<Integer> n = new ArrayList<Integer>();
    ArrayList<Integer> n1 = new ArrayList<Integer>();
    ArrayList<Integer> n_all = new ArrayList<Integer>();
    ArrayList<Integer> n_object = new ArrayList<Integer>();

    do {
      n.add(index.get(0));
      n_all.add(index.get(0));


      do {
        for (int i = 0; i < n.size(); i++) {
          ArrayList<Integer> neighbour =
              findNeighbouringGrid(n.get(i), vind.getNumX(), vind.getNumY(),
                  vind.getNumZ());
	      gridStatus[n.get(i)] = 1;

          for (int j = 0; j < neighbour.size(); j++){
            if (index.contains(neighbour.get(j))&&gridStatus[neighbour.get(j)]==0) {
              n1.add(neighbour.get(j));
              n_all.add(neighbour.get(j));
              gridStatus[neighbour.get(j)]=1;
            }
          }
        }
        n.clear();
        n.addAll(n1);
        n1.clear();
      } while (n.size() > 0);

      VoxelIndex v1 = vind.getVoxel(n_all.get(0));
      int xMin = v1.xStepCount();
      int xMax = v1.xStepCount();
      int yMin = v1.yStepCount();
      int yMax = v1.yStepCount();
      int zMin = v1.zStepCount();
      int zMax = v1.zStepCount();
      for (int i = 0; i < n_all.size(); i++) {
        v1 = vind.getVoxel(n_all.get(i));
        n_object.addAll(v1.getAllIndices());
        if (v1.xStepCount() < xMin)
          xMin = v1.xStepCount();
        if (v1.xStepCount() > xMax)
          xMax = v1.xStepCount();
        if (v1.yStepCount() < yMin)
          yMin = v1.yStepCount();
        if (v1.yStepCount() > yMax)
          yMax = v1.yStepCount();
        if (v1.zStepCount() < zMin)
          zMin = v1.zStepCount();
        if (v1.zStepCount() > zMax)
          zMax = v1.zStepCount();
      }

      lengthX.add(xMax - xMin + 1);
      lengthY.add(yMax - yMin + 1);
      lengthZ.add(zMax - zMin + 1);

      index.removeAll(n_all);
      System.out.println("    find object" + (objects.size() + 1) + ":"
          + n_all.size()
          + "   index:" + index.size());
      objectGridNumber.add(n_all.size());

      n_all.clear();
      PointCloud pc_colour = pc.makeSubCloud(n_object);
      RansacPlaneDetector.addColourToEachPlane(objects.size() + 1, pc_colour);
      objects.add(pc_colour);
      n_object.clear();
	  } while (index.size() > 0);
	  return objects;
  }
  
  public static ArrayList<PointCloud> findObjectsRegionGrow(PointCloud pc, float octreeRadius, float growRadius){
	  ArrayList<PointCloud> objects = new ArrayList<PointCloud>();
	  ArrayList<Integer> index1 = new ArrayList<Integer>();
	  ArrayList<Integer> n = new ArrayList<Integer>();
	  ArrayList<Integer> nObject = new ArrayList<Integer>();
	  ArrayList<Integer> nAll = new ArrayList<Integer>();//update seed point
	  ArrayList<Integer> nRest = new ArrayList<Integer>();
      int R = 0;
      int G = 0;
      int B = 0;
      int num = 0;
  
	  do{
		  int k1 = -1;//new added points to an objects
		  int[] pointStatus = new int[pc.getNumPoints()];
		  OctreeIndex oct = new OctreeIndex(pc.mCoordinates, octreeRadius);
	      //System.out.println("pc1:" + pc.getNumPoints());

    	  Point3 p1 = new Point3();
    	  pc.mCoordinates.getPoint(0, p1);
    	  nAll.add(0);
    	  nObject.add(0);
    	  
    	  do{
    		  for(int l:nAll){
    			  pointStatus[l]=1;
    			  pc.mCoordinates.getPoint(l, p1);
    	    	  oct.radiusSearch(p1, growRadius, index1);
    	    	  for (int j : index1) {
    	              if (pointStatus[j]==1) {
    	                continue;
    	              }
    	              n.add(j);
    	              pointStatus[j]=1;      
    	    	  }
    			  
    		  }
    		  nObject.addAll(n);
    		  nAll.clear();
    		  nAll.addAll(n);
    		  k1 = n.size();
    		  n.clear();
    	  }while(k1 > 0);
    	  
          if (nObject.size() == 0) {
              nObject.add(0);
              pc.clearIndexed(nObject);
              continue;
            }	  
          Collections.sort(nObject);
          PointCloud pcObject = pc.makeSubCloud(nObject);
          Colour c0 = new Colour(pcObject.getNumPoints(), 255, 255, 255);
          pcObject.setColours(c0);
          c0 = null;

          //System.out.println("Object:" + nObject.size());
          num++;

          switch (num%20) {
          case 1:
            R = 200;
            G = 0;
            B = 0;
            break;
          case 2:
            R = 200;
            G = 71;
            B = 0;
            break;
          case 3:
            R = 200;
            G = 143;
            B = 0;
            break;
          case 4:
            R = 91;
            G = 36;
            B = 164;
            break;
          case 5:
            R = 195;
            G = 200;
            B = 0;
            break;
          case 6:
            R = 124;
            G = 200;
            B = 0;
            break;
          case 7:
            R = 52;
            G = 200;
            B = 0;
            break;
          case 8:
            R = 0;
            G = 200;
            B = 52;
            break;
          case 9:
            R = 110;
            G = 124;
            B = 76;
            break;
          case 10:
            R = 0;
            G = 200;
            B = 195;
            break;
          case 11:
            R = 0;
            G = 138;
            B = 200;
            break;
          case 12:
            R = 0;
            G = 67;
            B = 200;
            break;
          case 13:
            R = 15;
            G = 147;
            B = 18;
            break;
          case 14:
            R = 14;
            G = 3;
            B = 159;
            break;
          case 15:
            R = 92;
            G = 62;
            B = 79;
            break;
          case 16:
            R = 34;
            G = 42;
            B = 120;
            break;
          case 17:
            R = 200;
            G = 0;
            B = 119;
            break;
          case 18:
            R = 173;
            G = 161;
            B = 163;
            break;
          case 19:
            R = 200;
            G = 0;
            B = 48;
            break;
          case 0:
            R = 164;
            G = 36;
            B = 103;
            break;

          }

          Colour c = new Colour(pcObject.getNumPoints(), R, G, B);
          pcObject.setColours(c);
          objects.add(pcObject);

          for (int j = 0; j < pc.getNumPoints(); j++) {
            if (pointStatus[j] == 0) {
              nRest.add(j);
            }
          }
          
          if (nRest.size() == 0)
        	  break;
          
          PointCloud rest = pc.makeSubCloud(nRest);
          System.out.println("Object:" + pcObject.getNumPoints()+"  Rest:"+rest.getNumPoints());
          
          nObject.clear();
          nRest.clear();
          pc = rest;
	  }while(true);
      return objects;
  }


  public static ArrayList<Integer> findNeighbouringGrid(int gridIndx, int numX,
      int numY, int numZ) {

    int[][][] xyz = new int[numX][numY][numZ];
    
    for (int i = 0; i < numX * numY * numZ; i++) {
      int z = (int) Math.floor(i / (numX * numY));
      int y = (int) Math.floor((i % (numX * numY)) / numX);
      int x = i - z * numX * numY - y * numX;
      xyz[x][y][z]=i;
    }

    int zg = (int) Math.ceil(gridIndx / (numX * numY));
    int yg = (int) Math.ceil((gridIndx % (numX * numY)) / numX);
    int xg = gridIndx - zg * numX * numY - yg * numX;
    int x1 = 0;
    int x2 = 0;
    int y1 = 0;
    int y2 = 0;
    int z1 = 0;
    int z2 = 0;
    if (xg == 0) {
      x1 = 1;
    }
    if (xg == numX - 1) {
      x2 = 1;
    }
    if (yg == 0) {
      y1 = 1;
    }
    if (yg == numY - 1) {
      y2 = 1;
    }
    if (zg == 0) {
      z1 = 1;
    }
    if (zg == numZ - 1) {
      z2 = 1;
    }

    return searchNeighbour(xyz, xg, yg, zg, xg - 1 + x1, xg + 2 - x2, yg - 1
        + y1, yg + 2 - y2, zg - 1 + z1, zg + 2 - z2);


  }
  
  public static ArrayList<Integer> searchNeighbour(int[][][] xyz,
      int xg, int yg, int zg, int minX, int maxX, int minY, int maxY, int minZ,
      int maxZ) {
    ArrayList<Integer> ng = new ArrayList<Integer>();
    for (int i = minX; i < maxX; i++) {
      for (int j = minY; j < maxY; j++) {
        for (int k = minZ; k < maxZ; k++) {
          if (xg != i || yg != j || zg != k) {
            ng.add(xyz[i][j][k]);
          }
        }
      }
    }

    return ng;
  }

  public static void setRadiusSearch(float r) {
    SegmentObjects.radiusSearch = r;
  }

  public static float getRadiusSearch() {
    return radiusSearch;
  }

  public static void setPointNumThreshold(int p) {
    SegmentObjects.pointNumThreshold = p;
  }

  public static float getPointNumThreshold() {
    return pointNumThreshold;
  }

  public static ArrayList<Integer> getObjectGridNumber() {
    return objectGridNumber;
  }

  public static ArrayList<Integer> getLengthX() {
    return lengthX;
  }

  public static ArrayList<Integer> getLengthY() {
    return lengthY;
  }

  public static ArrayList<Integer> getLengthZ() {
    return lengthZ;
  }
}

