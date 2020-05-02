package sphereDetection;
import java.util.ArrayList;

import uk.ac.ucl.jpl.Point3;
import uk.ac.ucl.jpl.PointCloud;
import uk.ac.ucl.jpl.SurfaceNormals;
import uk.ac.ucl.jpl.io.IOFunctions;
import uk.ac.ucl.jpl.scalarfields.ScalarCalculator;
import uk.ac.ucl.jpl.spatialindexing.OctreeIndex;

//this is code for range method to recognise rigid spheres

public class sphere_range
{
	public static void main(String [] args) throws Exception
	{
		Point3 ref = new Point3(0, 0, 36);      
		//use different radius to calculate curvature and roughness
        float r1 = 0.02f;
        float r2 = 0.03f;
        float r3 = 0.04f;
        
        //parameters of range threshold
        float thre1 = 0.85f;
        float thre2 = 1.15f;
        
        //radius to calculate surface normal
        float r_sn = 0.03f;
        
        //remove low-density points   
        float r_filter = 0.1f;
        int num_min = 150;
        
        //radius of octree
        float r_oct = 0.01f;
   
        //there are 10 sphere samples to choose from
        String name1 = "E:/David/data for David/sphere_all/sphere_sample1.txt";
		PointCloud pc = IOFunctions.readTextXYZ(name1);
		
		String name2 = "E:/David/data for David/sphere_all/708_007_part_octree.txt";
	    PointCloud pc_s1 = IOFunctions.readTextXYZ(name2);	
		
        //find min max
		OctreeIndex oct = new OctreeIndex(pc, r_oct);	
    	ArrayList<Integer> x1= new ArrayList<Integer>();
		ArrayList<Integer> x2= new ArrayList<Integer>();
		Point3 p1 = new Point3();
  		for(int i = 0; i < pc.getNumPoints(); i++){
  			ArrayList<Integer> returnIndex = new ArrayList<Integer>();
			pc.mCoordinates.getPoint(i, p1);
			oct.radiusSearch(p1,  r_filter, returnIndex);
			if(returnIndex.size() > num_min ) x1.add(i);
			else x2.add(i);
  		}
  		
  		PointCloud pc1 = pc.makeSubCloud(x1);	
  		System.out.println(pc.getNumPoints());
  		System.out.println(x1.size());
	   
 		SurfaceNormals snorm1 = new SurfaceNormals(pc1.getNumPoints());
  		OctreeIndex oct1 = new OctreeIndex(pc1, r_oct);
        snorm1.calculateFromRadius(pc1.mCoordinates, oct1, r_sn);
  		pc1.setSurfaceNormals(snorm1);
   		pc1.getSurfaceNormals().fixNormalsToViewpoint(ref, pc1.mCoordinates); 
	

   	    //radius r1
		int IND_SURF_CURV1 = pc1.addScalar(ScalarCalculator.calculateSurfaceCurvature(pc1.mCoordinates, oct1, r1));
		int IND_SURF_ROUG1 = pc1.addScalar(ScalarCalculator.calculateSurfaceRoughness(pc1.mCoordinates, oct1, r1, snorm1));
		
		float sc_min1 = 1;
		float sc_max1 = 0;
		float sr_min1 = 1;
		float sr_max1 = 0;

		for(int i = 0; i < pc1.getNumPoints(); i++){
			if(pc1.getScalar(IND_SURF_CURV1).mData[i]<sc_min1)
				sc_min1 = pc1.getScalar(IND_SURF_CURV1).mData[i];
			if(pc1.getScalar(IND_SURF_CURV1).mData[i]>sc_max1)
				sc_max1 = pc1.getScalar(IND_SURF_CURV1).mData[i];
			if(pc1.getScalar(IND_SURF_ROUG1).mData[i]<sr_min1)
				sr_min1 = pc1.getScalar(IND_SURF_ROUG1).mData[i];
			if(pc1.getScalar(IND_SURF_ROUG1).mData[i]>sr_max1)
				sr_max1 = pc1.getScalar(IND_SURF_ROUG1).mData[i];
		}
		
		System.out.println(sc_min1+","+sc_max1);
		System.out.println(sr_min1+","+sr_max1);
        
		//radius r2
		int IND_SURF_CURV2 = pc1.addScalar(ScalarCalculator.calculateSurfaceCurvature(pc1.mCoordinates, oct1, r2));
		int IND_SURF_ROUG2 = pc1.addScalar(ScalarCalculator.calculateSurfaceRoughness(pc1.mCoordinates, oct1, r2, snorm1));
		
		float sc_min2 = 1;
		float sc_max2 = 0;
		float sr_min2 = 1;
		float sr_max2 = 0;

		for(int i = 0; i < pc1.getNumPoints(); i++){
			if(pc1.getScalar(IND_SURF_CURV2).mData[i]<sc_min2)
				sc_min2 = pc1.getScalar(IND_SURF_CURV2).mData[i];
			if(pc1.getScalar(IND_SURF_CURV2).mData[i]>sc_max2)
				sc_max2 = pc1.getScalar(IND_SURF_CURV2).mData[i];
			if(pc1.getScalar(IND_SURF_ROUG2).mData[i]<sr_min2)
				sr_min2 = pc1.getScalar(IND_SURF_ROUG2).mData[i];
			if(pc1.getScalar(IND_SURF_ROUG2).mData[i]>sr_max2)
				sr_max2 = pc1.getScalar(IND_SURF_ROUG2).mData[i];		
		}
		
		System.out.println(sc_min2+","+sc_max2);
		System.out.println(sr_min2+","+sr_max2);
		
        //radius r3
		int IND_SURF_CURV3 = pc1.addScalar(ScalarCalculator.calculateSurfaceCurvature(pc1.mCoordinates, oct1, r3));
		int IND_SURF_ROUG3 = pc1.addScalar(ScalarCalculator.calculateSurfaceRoughness(pc1.mCoordinates, oct1, r3, snorm1));
		
		float sc_min3 = 1;
		float sc_max3 = 0;
		float sr_min3 = 1;
		float sr_max3 = 0;

		for(int i = 0; i < pc1.getNumPoints(); i++){
			if(pc1.getScalar(IND_SURF_CURV3).mData[i]<sc_min3)
				sc_min3 = pc1.getScalar(IND_SURF_CURV3).mData[i];
			if(pc1.getScalar(IND_SURF_CURV3).mData[i]>sc_max3)
				sc_max3 = pc1.getScalar(IND_SURF_CURV3).mData[i];
			if(pc1.getScalar(IND_SURF_ROUG3).mData[i]<sr_min3)
				sr_min3 = pc1.getScalar(IND_SURF_ROUG3).mData[i];
			if(pc1.getScalar(IND_SURF_ROUG3).mData[i]>sr_max3)
				sr_max3 = pc1.getScalar(IND_SURF_ROUG3).mData[i];
		}
		
		System.out.println(sc_min3+","+sc_max3);
		System.out.println(sr_min3+","+sr_max3);
		
		sc_min1 *= thre1;
	    sc_max1 *= thre2;
	    sr_min1 *= thre1;
	    sr_max1 *= thre2;
	     
	    sc_min2 *= thre1;
	    sc_max2 *= thre2;
	    sr_min2 *= thre1;
	    sr_max2 *= thre2;

	    sc_min3 *= thre1;
	    sc_max3 *= thre2;
	    sr_min3 *= thre1;
	    sr_max3 *= thre2;

         //separate
	     int k1 = 0;
	     int times = 0;
		 do{
		    times++;
			System.out.println(times);
		    OctreeIndex oct_s1 = new OctreeIndex(pc_s1, r_oct);
	        SurfaceNormals snorm_s1 = new SurfaceNormals(pc_s1.getNumPoints());
	        snorm_s1.calculateFromRadius(pc_s1.mCoordinates, oct_s1, r_sn);
	 		pc_s1.setSurfaceNormals(snorm_s1);
	  		pc_s1.getSurfaceNormals().fixNormalsToViewpoint(ref, pc_s1.mCoordinates); 
	  		System.out.println(pc_s1.getNumPoints());
	  	
	  	  	//surface roughness curvature 1
			IND_SURF_CURV1 = pc_s1.addScalar(ScalarCalculator.calculateSurfaceCurvature(pc_s1.mCoordinates, oct_s1,r1));
			IND_SURF_ROUG1 = pc_s1.addScalar(ScalarCalculator.calculateSurfaceRoughness(pc_s1.mCoordinates, oct_s1, r1, snorm_s1));
			
			//surface roughness curvature 2
			IND_SURF_CURV2 = pc_s1.addScalar(ScalarCalculator.calculateSurfaceCurvature(pc_s1.mCoordinates, oct_s1,r2));
			IND_SURF_ROUG2 = pc_s1.addScalar(ScalarCalculator.calculateSurfaceRoughness(pc_s1.mCoordinates, oct_s1, r2, snorm_s1));
		
			//surface roughness curvature 3
			IND_SURF_CURV3 = pc_s1.addScalar(ScalarCalculator.calculateSurfaceCurvature(pc_s1.mCoordinates, oct_s1,r3));
			IND_SURF_ROUG3 = pc_s1.addScalar(ScalarCalculator.calculateSurfaceRoughness(pc_s1.mCoordinates, oct_s1, r3, snorm_s1));
		
			ArrayList<Integer> p = new ArrayList<Integer>();
			ArrayList<Integer> t = new ArrayList<Integer>();
			
		    for(int i = 0; i < pc_s1.getNumPoints(); i ++){
    	        if(pc_s1.getScalar(IND_SURF_CURV1).mData[i]> sc_min1 && pc_s1.getScalar(IND_SURF_CURV1).mData[i] < sc_max1
    			&& pc_s1.getScalar(IND_SURF_ROUG1).mData[i]> sr_min1 && pc_s1.getScalar(IND_SURF_ROUG1).mData[i] < sr_max1
    			&& pc_s1.getScalar(IND_SURF_CURV2).mData[i]> sc_min2 && pc_s1.getScalar(IND_SURF_CURV2).mData[i] < sc_max2
    			&& pc_s1.getScalar(IND_SURF_ROUG2).mData[i]> sr_min2 && pc_s1.getScalar(IND_SURF_ROUG2).mData[i] < sr_max2
    			&& pc_s1.getScalar(IND_SURF_CURV3).mData[i]> sc_min3 && pc_s1.getScalar(IND_SURF_CURV3).mData[i] < sc_max3
    			&& pc_s1.getScalar(IND_SURF_ROUG3).mData[i]> sr_min3 && pc_s1.getScalar(IND_SURF_ROUG3).mData[i] < sr_max3) p.add(i);
		    	else
		    		t.add(i);   
		    }
		    System.out.println(p.size());
			
		    PointCloud pc_s2 = pc_s1.makeSubCloud(p);	
	  		SurfaceNormals snorm_s2 = new SurfaceNormals(pc_s2.getNumPoints());
	  		OctreeIndex oct_s2 = new OctreeIndex(pc_s2, r_oct);
	        snorm_s2.calculateFromRadius(pc_s2.mCoordinates, oct_s2, r_sn);//sphere 0.03f 
	  		pc_s2.setSurfaceNormals(snorm_s2);
	   		pc_s2.getSurfaceNormals().fixNormalsToViewpoint(ref, pc_s2.mCoordinates); 
			
	  		ArrayList<Integer> x3= new ArrayList<Integer>();
			ArrayList<Integer> x4= new ArrayList<Integer>();
			Point3 p0 = new Point3();
	  		for(int i = 0; i < pc_s2.getNumPoints(); i++){
	  			ArrayList<Integer> returnIndex = new ArrayList<Integer>();
				pc_s2.mCoordinates.getPoint(i, p0);
				oct_s2.radiusSearch(p0,  r_filter, returnIndex);
				if(returnIndex.size() > num_min  ) x3.add(i);
				else x4.add(i);
	  		}
	  		
	  		PointCloud pc_s3 = pc_s2.makeSubCloud(x3);	
	  		System.out.println(x3.size());
	  		PointCloud pc_s4 = pc_s2.makeSubCloud(x4);
			k1 = pc_s4.getNumPoints();
				
			String name3 = "E:/David/data for David/sphere_all/result/sphere_filter" + times + ".txt";
			IOFunctions.writeTextXYZ(pc_s3, name3);

		    PointCloud pc0 = new PointCloud(pc_s3);
			pc_s1 = pc0;	
		  }while(k1>0);
	}
}