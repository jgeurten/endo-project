//PCL includes
#include <c://Documents/pcl-bin>

//need to get PCL

//Local includes
#include "EndoModel.h"
#include "defines.h"
#include "LinAlg.h"
#include <MainWindow.h>

//Qt includes
#include <qdebug.h>
#include <qdialog.h>
#include <qfiledialog.h>
#include <qfile.h>

//MSDN includes
#include <iostream>
#include <fstream>

using namespace pcl;
using namespace std; 

EndoModel::EndoModel()
{
	pointCloud.reset(new PointCloud<PointXYZRGB>);

	ofstream myfile("./Data/Scan.csv"); 
	myfile << "Cam X," << "Cam Y," << "Cam Z,"
		<< "Tool X," << "Tool Y," << "Tool Z,"
		<< "Laser X," << "Laser Y," << "Laser Z" << endl;
}

void EndoModel::convertPointCloudToSurfaceMesh()
{

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

void EndoModel::addPointToPointCloud(EndoPt point)
{
	//convert EndoPt to pcl point:
	PointXYZRGB pc;
	pc.x = point.x;
	pc.y = point.y;
	pc.z = point.z;
	
	pointCloud->push_back(pc);
	pointCloud->saveData(point); 
}

void EndoModel::savePointCloud()
{
	QFileDialog d(*pointCloud, "Save File", "", "PCD (*.pcd) ;; PLY (*.ply)");
	d.setAcceptMode(QFileDialog::AcceptSave);
	if (d.exec()) {
		QString fileName = d.selectedFiles()[0];
		//fileName.append(d.selectedNameFilter());
		if (fileName.isEmpty()) return;
		qDebug() << fileName;
		if (fileName.endsWith(".pcd", Qt::CaseInsensitive)) {
			qDebug() << "Save as pcd file.";
			savePointCloudAsPCD(fileName.toStdString());
		}
		else if (fileName.endsWith(".ply", Qt::CaseInsensitive)) {
			qDebug() << "Save as ply file.";
			savePointCloudAsPLY(fileName.toStdString());
		}
	}
}

void EndoModel::saveData(EndoPt point)
{
	for (int i = 0; i < 3; i++)
		myfile << MainWindow::getCameraPosition(i, 3) << ",";

	for (int i = 0; i < 3; i++)
		myfile << MainWindow::getToolPosition(i, 3) << ",";

	myfile << point.x << "," << point.y << "," << point.z << endl; 

}