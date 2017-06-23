#include "EndoModel.h"


using namespace pcl;
using namespace std; 




EndoModel::EndoModel()
{
	pointcloud.reset(new PointCloud<PointXYZRGB>);
}

void EndoModel::convertPointCloudToSurfaceMesh()
{

}

void EndoModel::savePointCloudAsPLY(string &filename)
{

}

void EndoModel::addPointToPointCloud(FSPoint point)
{
	//qDebug()<<"added Point to cloud";
	PointXYZRGB pc;
	pc.x = point.x;
	pc.y = point.y;
	pc.z = point.z;
	
	pointCloud->push_back(pc);
}

void MainWindow::savePointCloud()
{
	QFileDialog d(this, "Save File", "", "PCD (*.pcd) ;; PLY (*.ply)");
	d.setAcceptMode(QFileDialog::AcceptSave);
	if (d.exec()) {
		QString fileName = d.selectedFiles()[0];
		//fileName.append(d.selectedNameFilter());
		if (fileName.isEmpty()) return;
		qDebug() << fileName;
		if (fileName.endsWith(".pcd", Qt::CaseInsensitive)) {
			qDebug() << "Save as pcd file.";
			FSController::getInstance()->model->savePointCloudAsPCD(fileName.toStdString());
		}
		else if (fileName.endsWith(".ply", Qt::CaseInsensitive)) {
			qDebug() << "Save as ply file.";
			FSController::getInstance()->model->savePointCloudAsPLY(fileName.toStdString());
		}
	}

	ui->widget->drawState = 0;
	ui->widget->updateGL();
}
