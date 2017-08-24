
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
#include <pcl/common/centroid.h>
#include <pcl/surface/mls.h>
#include <pcl/point_types.h>
#include <pcl/pcl_exports.h>

//VTK Includes:
#include <vtkCellArray.h>
#include <vtkProperty.h>
#include <vtkDataSetMapper.h>
#include <vtkActor.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolygon.h>
#include <vtkSmartPointer.h>
#include <vtkMath.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCleanPolyData.h>
#include <vtkDelaunay3D.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPolyDataReader.h>
#include <vtkPlyReader.h>
#include <vtkObjReader.h>
#include <vtkSurfaceReconstructionFilter.h>
#include <vtkContourFilter.h>
#include <vtkPLYWriter.h>
#include <vtkReverseSense.h>


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

	this->points = vtkPoints::New();
	pointCount = 0; 
	sum.x = 0; 
	sum.y = 0;
	sum.z = 0;

	SD = 0; 
}

void EndoModel::addPointToPointCloud(linalg::EndoPt point)
{
	pcl::PointXYZ pt;
	pt.x = point.x;
	pt.y = point.y;
	pt.z = point.z;
	pointCloud->push_back(pt);
	//calculate total sum in x,y,z direction
	
	sum.x += pt.x;
	sum.y += pt.y;
	sum.z += pt.z;
	
	//update count and prev point data
	pointCount++; 
}

pcl::PointCloud<pcl::PointXYZ>::Ptr EndoModel::filterCloud()
{
	//create int mat to hold row values
	const int nrows = (int) pointCount;
	std::vector<int> rowIndex(nrows);
	int colIn = 24;			//column of interest
	int ncols = 59;
	string line; 
	//Parse Results.csv to get associated row # for Intersection X,Y,Z
	string openFile = "./Results/Results.csv";
	ifstream resultsFile(openFile);

	for (int row = 0; row < nrows; row++)
	{
		for (int i = 0; i < colIn; i++)
			getline(resultsFile, line);
		rowIndex[row] = stoi(line);
		for (int j = colIn; j < ncols; j++)
			getline(resultsFile, line);
	}

	string fileLocation = "./Results/PolyData.csv";
	ofstream ResultsFile;
	ResultsFile.open(fileLocation, ios::out | ios::binary |ios::trunc);		//discard previous contents

	//Results file header:
	ResultsFile << "X," << "Y," << "Z," << "Index" << endl;

	//calculate centroid
	linalg::EndoPt centroid;
	centroid.x = sum.x / pointCount;
	centroid.y = sum.y / pointCount;
	centroid.z = sum.z / pointCount;

	//calculate mean and SD of points from centroid:
	linalg::EndoPt SD, mean, dist;
	mean.x = 0;
	mean.y = 0;
	mean.z = 0;
	SD.x = 0; 
	SD.y = 0;
	SD.z = 0;

	//calculate mean distance from centroid
	for (int i = 0; i < pointCount; i++)
	{
		mean.x += sqrt(std::pow(pointCloud->points[i].x -centroid.x, 2));		
		mean.y += sqrt(std::pow(pointCloud->points[i].y -centroid.y, 2));
		mean.z += sqrt(std::pow(pointCloud->points[i].z -centroid.z, 2));
	}

	mean.x = mean.x/pointCount;
	mean.y = mean.y/pointCount;
	mean.z = mean.z/pointCount;
	
	//calculate sd distance from centroid
	for (int i = 0; i < pointCount; i++)
	{
		dist.x = sqrt(std::pow(pointCloud->points[i].x - centroid.x,2));
		dist.y = sqrt(std::pow(pointCloud->points[i].y - centroid.y,2));
		dist.z = sqrt(std::pow(pointCloud->points[i].z - centroid.z,2));
		SD.x += abs(dist.x - mean.x);
		SD.y += abs(dist.y - mean.y);
		SD.z += abs(dist.z - mean.z);
	}

	SD.x = sqrt(SD.x / (pointCount - 1));	//SD form
	SD.y = sqrt(SD.y / (pointCount - 1));
	SD.z = sqrt(SD.z / (pointCount - 1));

	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
	//remove outliers:
	pcl::PointXYZ pt; 
	for (int i = 0; i < pointCount ; i++)
	{	
		if (sqrt(std::pow(pointCloud->points[i].x - centroid.x, 2)) < 2*SD.x + mean.x)		//if within 2 sd of mean in each direction from the centroid --> ADD
		{
			if (sqrt(std::pow(pointCloud->points[i].y - centroid.y, 2)) < 2*SD.y + mean.y)	//will drop ~5% of points
			{
				if (sqrt(std::pow(pointCloud->points[i].z - centroid.z, 2)) < 2*SD.z + mean.z)
				{
					pt.x = pointCloud->points[i].x;
					pt.y = pointCloud->points[i].y;
					pt.z = pointCloud->points[i].z;
					filteredCloud->push_back(pt);
					ResultsFile <<pt.x << ",";													//save filtered data for polyline writer
					ResultsFile <<pt.y << ",";
					ResultsFile <<pt.z << ",";
					ResultsFile << rowIndex[i] << endl;
				}
			}
		}
	}
	return filteredCloud;
}

void EndoModel::savePointCloudAsPLY(string &filename)
{
	if (pointCloud->size() == 0) return;
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud;
	filteredCloud = filterCloud();
	pcl::io::savePLYFileASCII(filename, *filteredCloud);
}

void EndoModel::savePointCloudAsPCD(string &filename)
{
	if (pointCloud->size() == 0) return;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud;
	//filteredCloud = filterCloud();
	pcl::io::savePCDFileASCII(filename, *pointCloud);
}

void EndoModel::saveMesh(string &filename)
{
	if (pointCloud->size() == 0) return;		//point cloud will be non-null since pointcloud is converted to polygon mesh.
	pcl::io::saveOBJFile(filename, *surfaceMesh);
}



size_t EndoModel::getCloudSize()
{
	return pointCloud->size();
}

void EndoModel::convertCloudToSurface(string &filename)
{/*
	//Perform normal estimation 1st
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	cloud->points.resize(pointCloud->size());
	for (size_t i = 0; i < pointCloud->points.size(); i++) {
		cloud->points[i].x = pointCloud->points[i].x;
		cloud->points[i].y = pointCloud->points[i].y;
		cloud->points[i].z = pointCloud->points[i].z;
	}


	int ksearch = 20;
	tree->setInputCloud(cloud);
	norm.setInputCloud(cloud);
	norm.setSearchMethod(tree);

	norm.setKSearch(ksearch);
	norm.compute(*normals);

	//Add point cloud and normals:
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudAndNormals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloudAndNormals);

	//Create tree:
	pcl::search::KdTree<pcl::PointNormal>::Ptr searchTree(new pcl::search::KdTree<pcl::PointNormal>);
	searchTree->setInputCloud(cloudAndNormals);

	//SEt traingulation params:
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	gp3.setSearchRadius(150);					//max search radius between connected pts
	gp3.setMu(2.0);								//sets multiplier for calc of final search radius
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


	*/

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile(filename, cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *pcloud);
	//* the data should be available in pcloud

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(pcloud);
	n.setInputCloud(pcloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*pcloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = pcloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	pcl::io::saveOBJFile(filename, triangles);
}

void EndoModel::createVTKSurface(string &filename)
{
	//Open file to get point cloud. convert point cloud to vtk surface using 
	//delaunay algo: http://www.vtk.org/Wiki/VTK/Examples/Cxx/Modelling/Delaunay3D

	//Create vtk mapper. Mapper class interfaces data geometry and graphics
	vtkSmartPointer<vtkDataSetMapper> originalMapper = vtkSmartPointer<vtkDataSetMapper>::New();

	// Filter the polydata. removes duplicated points
	vtkSmartPointer<vtkCleanPolyData> filterer = vtkSmartPointer<vtkCleanPolyData>::New();

	//Open file:
	string type = vtksys::SystemTools::GetFilenameExtension(filename);

	if (type == ".ply")
	{
		vtkSmartPointer<vtkPLYReader> plyreader = vtkSmartPointer<vtkPLYReader>::New();
		plyreader->SetFileName(filename.c_str());
		plyreader->Update();				//get data
		originalMapper->SetInputConnection(plyreader->GetOutputPort());
		filterer->SetInputConnection(plyreader->GetOutputPort());
	}
	else if (type == ".OBJ")
	{
		vtkSmartPointer<vtkOBJReader> objreader = vtkSmartPointer<vtkOBJReader>::New();
		objreader->SetFileName(filename.c_str());
		objreader->Update();
		originalMapper->SetInputConnection(objreader->GetOutputPort());
		filterer->SetInputConnection(objreader->GetOutputPort());
	}

	//Create actor. Actor represents an object while rendering. Maintains reference to geometry:
	vtkSmartPointer<vtkActor> originalActor = 	vtkSmartPointer<vtkActor>::New();
	originalActor->SetMapper(originalMapper);
	originalActor->GetProperty()->SetColor(1, 0, 0);

	// Generate a tetrahedral mesh from the input points.
	vtkSmartPointer<vtkDelaunay3D> delaunay3D =	vtkSmartPointer<vtkDelaunay3D>::New();
	delaunay3D->SetInputConnection(filterer->GetOutputPort());

	//Create a mapper and actor for delauney output
	vtkSmartPointer<vtkDataSetMapper> delaunayMapper =	vtkSmartPointer<vtkDataSetMapper>::New();
	vtkSmartPointer<vtkActor> delaunayActor =	vtkSmartPointer<vtkActor>::New();
	delaunayMapper->SetInputConnection(filterer->GetOutputPort()); 
	delaunayActor->SetMapper(delaunayMapper); 
	delaunayActor->GetProperty()->SetColor(1, 0, 0);

	// Generate a mesh from the input points. If Alpha is non-zero, then
	// tetrahedra, triangles, edges and vertices that lie within the
	// alpha radius are output.
	vtkSmartPointer<vtkDelaunay3D> delaunay3DAlpha =
		vtkSmartPointer<vtkDelaunay3D>::New();
	delaunay3DAlpha->SetInputConnection(filterer->GetOutputPort());
	delaunay3DAlpha->SetAlpha(0.1);

	vtkSmartPointer<vtkDataSetMapper> delaunayAlphaMapper =
		vtkSmartPointer<vtkDataSetMapper>::New();
	delaunayAlphaMapper->SetInputConnection(delaunay3DAlpha->GetOutputPort());

	vtkSmartPointer<vtkActor> delaunayAlphaActor =
		vtkSmartPointer<vtkActor>::New();
	delaunayAlphaActor->SetMapper(delaunayAlphaMapper);
	delaunayAlphaActor->GetProperty()->SetColor(1, 0, 0);

	//Create visualization/interactor, renderer, render window:
	double leftViewport[4] = { 0.0, 0.0, 0.33, 1.0 };
	double centerViewport[4] = { 0.33, 0.0, 0.66, 1.0 };
	double rightViewport[4] = { 0.66, 0.0, 1.0, 1.0 };
	vtkSmartPointer<vtkRenderer> originalRenderer =	vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderer> delaunayRenderer =	vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderer> delaunayAlphaRenderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow =	vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->SetSize(900, 300);

	renderWindow->AddRenderer(originalRenderer);
	originalRenderer->SetViewport(leftViewport);
	renderWindow->AddRenderer(delaunayRenderer);
	delaunayRenderer->SetViewport(centerViewport);
	renderWindow->AddRenderer(delaunayAlphaRenderer);
	delaunayAlphaRenderer->SetViewport(rightViewport);

	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =	vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	originalRenderer->AddActor(originalActor);
	delaunayRenderer->AddActor(delaunayActor);
	delaunayAlphaRenderer->AddActor(delaunayAlphaActor);

	originalRenderer->SetBackground(.3, .6, .3);
	delaunayRenderer->SetBackground(.4, .6, .3);
	delaunayAlphaRenderer->SetBackground(.5, .6, .3);

	// Render and interact
	renderWindow->Render();
	renderWindowInteractor->Start();

}

void EndoModel::createVTKPC(string filename)
{
	
	//Create polydata structure and set points equal to vtkpoints pointer
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	
	/*
	vtkPoints* elves = vtkPoints::New();
	float x, y, z;
	// generate random points on unit sphere
	for (int i = 0; i<5000; i++)
	{

		double u = vtkMath::Random(0.0, 1.0);
		double v = vtkMath::Random(0.0, 1.0);
		double phi = 2.0*3.14159265*u;
		double theta = acos(2.0*v - 1.0);

		x = std::cos(phi)*std::sin(theta);
		y = std::sin(phi)*std::sin(theta);
		z = std::cos(theta);

		elves->InsertNextPoint(x, y, z);
	}
	*/

	polydata->SetPoints(points); 

	// Construct the surface and create isosurface.	
	vtkSmartPointer<vtkSurfaceReconstructionFilter> surface = vtkSmartPointer<vtkSurfaceReconstructionFilter>::New();

	//If vtk version > 5
	surface->SetInputData(polydata); 
	//else: surface->SetInput(polydata); 

	vtkSmartPointer<vtkContourFilter> contourFilter = vtkSmartPointer<vtkContourFilter>::New();
	contourFilter->SetInputConnection(surface->GetOutputPort());
	contourFilter->SetValue(0, 0.0);

	// Sometimes the contouring algorithm can create a volume whose gradient
	// vector and ordering of polygon (using the right hand rule) are
	// inconsistent. vtkReverseSense cures this problem.
	

	string temp = filename.substr(0, filename.size() - 4);
	string newfilename = temp + "reverse.ply";

	//Save as PLY
	vtkSmartPointer<vtkPLYWriter> plyWriter =
		vtkSmartPointer<vtkPLYWriter>::New();
	plyWriter->SetFileName(filename.c_str());
	plyWriter->SetInputConnection(contourFilter->GetOutputPort());
	plyWriter->Write();

	/*
	vtkSmartPointer<vtkReverseSense> reverse = vtkSmartPointer<vtkReverseSense>::New();
	reverse->SetInputConnection(contourFilter->GetOutputPort());
	reverse->ReverseCellsOn();
	reverse->ReverseNormalsOn();
	reverse->Update();

	vtkSmartPointer<vtkPLYWriter> plyReverseWriter =
		vtkSmartPointer<vtkPLYWriter>::New();
	plyReverseWriter->SetFileName(newfilename.c_str());
	plyReverseWriter->SetInputConnection(reverse->GetOutputPort());
	plyReverseWriter->Write();
	*/
}

