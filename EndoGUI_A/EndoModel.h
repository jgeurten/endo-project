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

#include <iostream>
#include <fstream>

using namespace std; 

class EndoModel
{
private:

public:

	EndoModel();
	void convertPointCloudToSurfaceMesh();
	void savePointCloudAsPLY(string &filename); 
	void savePointCloudAsPCD(string &filename);
	void addPointToPointCloud(EndoPt point);
	void saveData(EndoPt point);

	ofstream myfile;
	pcl::PointCloud<pcl::PointXYZ> *pointCloud;
};

#endif // !ENDOMODEL_H
