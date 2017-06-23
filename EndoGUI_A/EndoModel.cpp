//PCL includes

//need to get PCL

//Local includes
#include "EndoModel.h"
#include "defines.h"
#include "LinAlg.h"

//Qt includes
#include <qdebug.h>
#include <qdialog.h>
#include <qfiledialog.h>
#include <qfile.h>

using namespace pcl;
using namespace std; 

EndoModel::EndoModel()
{
	pointCloud.reset(new PointCloud<PointXYZRGB>);
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