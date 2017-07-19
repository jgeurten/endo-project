
//PCL includes
//include "c://Program Files/PCL 1.8.0/include/pcl-1.8/pcl/common/projection_matrix.h"
#include <pcl/common/projection_matrix.h>


//Local includes
#include "EndoModel.h"
#include "defines.h"
#include <MainWindow.h>

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
	//this->pointCloud = new pcl::PointCloud<pcl::PointXYZ>;
	this->test = 3;
	ofstream myfile("./Data/Scan.csv");
	myfile << "Cam X," << "Cam Y," << "Cam Z,"
		<< "Tool X," << "Tool Y," << "Tool Z,"
		<< "Laser X," << "Laser Y," << "Laser Z" << endl;
}
/*
void EndoModel::convertPointCloudToSurfaceMesh()
{

}

void EndoModel::savePointCloudAsPLY(string &filename)
{
	if (pointCloud->size() == 0) return;
	pcl::io::savePLYFileASCII(filename, *pointCloud);

}



void EndoModel::addPointToPointCloud(EndoPt point)
{
	//convert EndoPt to pcl point:
	pcl::PointXYZ pc;
	pc.x = point.x;
	pc.y = point.y;
	pc.z = point.z;
	
	pointCloud->push_back(pc);
	this->saveData(point);
}


void EndoModel::savePointCloudAsPCD(string &filename)
{
	//if (pointCloud->size() == 0) return;
	//pcl::io::savePCDFileASCII(filename, *pointCloud);
	myfile.close(); 
}

void EndoModel::saveData(linalg::EndoPt point)
{
	for (int i = 0; i < 3; i++)
		myfile << MainWindow::getCameraPosition(i, 3) << ",";

	for (int i = 0; i < 3; i++)
		myfile << MainWindow::getLaserPosition(i, 3) << ",";

	myfile << point.x << "," << point.y << "," << point.z << endl; 

}
*/