
//PCL includes
#include <pcl/common/projection_matrix.h>
//#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

//boost
#include <boost/thread/thread.hpp>


//Local includes
#include "EndoModel.h"
#include "defines.h"
#include <MainWindow.h>
#include <linalg.h>

//Qt includes
#include <qdebug.h>
#include <qdialog.h>
#include <qfiledialog.h>
#include <qfile.h>

//MSDN includes
#include <iostream>
#include <fstream>	

using namespace std;

EndoModel::EndoModel()
{
	pointCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	this->pointCloud = cloud;			//May be redundant

	surfaceMesh.reset(new pcl::PolygonMesh); 
	pcl::PolygonMesh::Ptr _surfMesh(new pcl::PolygonMesh); 
	this->surfaceMesh = _surfMesh;

}

void EndoModel::addPointToPointCloud(linalg::EndoPt point)
{
	//convert EndoPt to pcl point:
	pcl::PointXYZ pc;
	pc.x = point.x;
	pc.y = point.y;
	pc.z = point.z;

	pointCloud->push_back(pc);
}

void EndoModel::savePointCloudAsPLY(string &filename)
{
	if (pointCloud->size() == 0) return;
	pcl::io::savePLYFileASCII(filename, *pointCloud);
}

void EndoModel::savePointCloudAsPCD(string &filename)
{
	if (pointCloud->size() == 0) return;
	pcl::io::savePCDFileASCII(filename, *pointCloud);
}

void EndoModel::saveMesh(string &filename)
{
	if (pointCloud->size() == 0) return;		//point cloud will be non-null since pointcloud is converted to polygon mesh.
	pcl::io::saveOBJFile(filename, *surfaceMesh);
}

void EndoModel::viewPointCloud(string &filename, int fileType)	//filetype 1:pcd, 2:ply
{
	//create instance of pcloud visualizer

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);	//black 

	if (fileType < 3) {	//PLY or PCL -> view simply point cloud

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		if (fileType == 1) {
			if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1)	//store in pcloud ptr
			{
				ERROR("Unable to read file:", filename);
				return;
			}
		}
		else if (fileType == 2) {
			if (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud) == -1) {
				ERROR("Unable to read file:", filename);
				return;
			}
		}
		else
		{
			ERROR("unrecognized filetype");
			return;
		}

		viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	}

	if (fileType == 3) {		//view mesh
		pcl::PolygonMesh mesh;
		if (pcl::io::loadOBJFile(filename, mesh) == -1) {
			ERROR("Unable to read file:", filename);
			return;
		}

		viewer->addPolygonMesh(mesh, "mesh", 0);
		//setpointcloudrendering can still be used while using add polygon mesh
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "mesh");	
	}

	viewer->addCoordinateSystem(1.0);// , origin.x, origin.y, origin.z);
	viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	};
}

void EndoModel::removeOutliers(int meanK, float SD)
{
	//filters the this->Pointcloud and returns filtered cloud.
	//Called between ending a scan and saving PC as ply or pcd.
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr downSampCld(new pcl::PointCloud<pcl::PointXYZ>);

	//create stat outlier removal object. NN to analyze = meanK.Points with > SD of the mean distance
	//of the query point will be marked = outlier. 
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> remover;
	remover.setInputCloud(pointCloud);
	remover.setMeanK(meanK);
	remover.setStddevMulThresh(SD);
	remover.filter(*filtCloud);

	//Apply downsampling point cloud using voxel grid filter. 
	//Approach: define voxel dimensions in world coordinates. Per voxel, will estimate the 
	//centroid which will be the only point kept.
	pcl::VoxelGrid<pcl::PointXYZ> sampler;
	sampler.setInputCloud(filtCloud);
	sampler.setLeafSize(0.01f, 0.01f, 0.01f);	//parameters are in meters
	sampler.filter(*downSampCld);

	//outliers are removed Cloud has been approximated. 
	pointCloud = downSampCld;
}

size_t EndoModel::getCloudSize()
{
	return pointCloud->size();
}

void EndoModel::convertCloudToSurface()
{
	//Perform normal estimation 1st
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	int ksearch = 20;
	tree->setInputCloud(pointCloud);
	norm.setInputCloud(pointCloud);
	norm.setSearchMethod(tree);
	norm.setKSearch(ksearch);
	norm.compute(*normals);

	//Add point cloud and normals:
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudAndNormals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*pointCloud, *normals, *cloudAndNormals);

	//Create tree:
	pcl::search::KdTree<pcl::PointNormal>::Ptr searchTree(new pcl::search::KdTree<pcl::PointNormal>);
	searchTree->setInputCloud(cloudAndNormals);

	//SEt traingulation params:
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	gp3.setSearchRadius(15.00);					//max search radius between connected pts
	gp3.setMu(2.5);								//sets multiplier for calc of final search radius
	gp3.setMaximumSurfaceAngle(M_PI / 4);		// 45 degrees
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMinimumAngle(M_PI / 18);				// 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3);			// 120 degrees
	gp3.setNormalConsistency(false);			//consistent normal orientation

	pcl::PolygonMesh _surfaceMesh;
	//Resultingly:
	gp3.setInputCloud(cloudAndNormals);
	gp3.setSearchMethod(searchTree);
	gp3.reconstruct(_surfaceMesh);
	
	*surfaceMesh = _surfaceMesh;			//populate global variable  
}