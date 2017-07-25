#pragma once
#ifndef ENDOMODEL_H
#define ENDOMODEL_H

#include <Vision.h>
#include <LinAlg.h>

//PCL includes
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>
#include <pcl/common/common_headers.h>

#include <iostream>
#include <fstream>

using namespace std; 

class EndoModel
{
private:

public:

	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;
	pcl::PolygonMesh::Ptr surfaceMesh; 

	EndoModel();
	void addPointToPointCloud(linalg::EndoPt point);

	void savePointCloudAsPLY(string &filename); 
	void savePointCloudAsPCD(string &filename);
	void saveMesh(string &filename);
	void removeOutliers(int meanK, float SD);
	size_t getCloudSize(); 
	void convertCloudToSurface();

//Static functions:	
	static void viewPointCloud(string &filename, int fileType, linalg::EndoPt camera);
};

#endif // !ENDOMODEL_H
