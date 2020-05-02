%%prepare the train/test data. generate .mat files


cubeDim = 16;
normalise_mesh_4(106,156,'bathtub',cubeDim);
normalise_mesh_4(515,615,'bed',cubeDim);
normalise_mesh_4(889,989,'chair',cubeDim);
normalise_mesh_4(200,286,'desk',cubeDim);
normalise_mesh_4(200,286,'dresser',cubeDim);
normalise_mesh_4(465,565,'monitor',cubeDim);
normalise_mesh_4(200,286,'night_stand',cubeDim);
normalise_mesh_4(680,780,'sofa',cubeDim);
normalise_mesh_4(392,492,'table',cubeDim);
normalise_mesh_4(344,444,'toilet',cubeDim);

