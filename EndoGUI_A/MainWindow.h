#pragma once
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//Local includes
#include "Serial.h"
#include "qlightwidget.h"
#include "ControlWidget.h"
#include "LinAlg.h"
#include "EndoModel.h"

//QT includes
#include <qmainwindow.h>
#include <qmenu.h>
#include <qpushbutton.h>
#include <qaction.h>
#include <qwidget.h>
#include <qlabel.h>	
#include <QImage>
#include <qtimer.h>
#include <QPaintEvent>
#include <qpainter.h>
#include <qmessagebox.h>
#include <qfile>
#include <qstring.h>
#include <qmediaplayer.h>
#include <qthread.h>

// VTK Includes
#include <vtkSmartPointer.h>
#include <vtkMatrix4x4.h>
#include <vtkActor.h>
#include <vtkTexture.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkImageImport.h>
#include <vtk_glew.h>

//opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Plus includes:

#include <vtkPlusDataCollector.h>
#include "PlusConfigure.h"
#include "vtkPlusTransformRepository.h"
#include "PlusTrackedFrame.h"
#include "vtkPlusChannel.h"
#include <vtkPlusNDITracker.h>
#include <vtkPlusVolumeReconstructor.h>
#include <vtkPlusMmfVideoSource.h>
#include <vtkPlusOpenIGTLinkVideoSource.h>
#include "C:/RVTK-bin/Deps/Plus-bin/PlusApp/fCal/Toolboxes/QAbstractToolbox.h"


//using namespace cv;
using namespace std;

// VTK forward declaration
class QVTKWidget;
class vtkRenderer;
class vtkTexture;
class vtkImageImport;
class vtkTrackerTool;
class ControlWidget; 

namespace Ui {
	class MainWindow;
}

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = Q_NULLPTR);
	~MainWindow();

private:
		
	void createMenus(); 

	void createControlDock();

	void createStatusBar();

	void createVTKObject();

	QMenu			*fileMenu;
	QMenu			*helpMenu;
	QMenu			*cameraMenu; 
	QAction			*openAct;
	QAction			*saveAct;
	QAction			*exitAct;
	QAction			*aboutAct;
	QAction			*helpAct;
	QAction			*webcam; 
	QAction			*endoCam; 

	QWidget			*videoWidget;
	QPushButton		*pushButton;
	QPushButton		*pushButton_2;
	QPushButton		*pushButton_3;
	QPushButton		*pushButton_4;

	QMediaPlayer	*player;
	QTimer			*trackTimer;
	QTimer			*scanTimer; 
	QDockWidget		*controlDock; 
	ControlWidget	*controlWidget;
	QImage			 image;
	QPixmap			 pixLabel;
	QLabel			*videoLabel;
	QSize			*size;
	QThread			*streamThread; 

	EndoModel		*model; 
	

	// Plus members
	vtkSmartPointer<vtkXMLDataElement>				configRootElement = vtkSmartPointer<vtkXMLDataElement>::New();
	vtkSmartPointer<vtkPlusDataCollector>			dataCollector = vtkSmartPointer<vtkPlusDataCollector>::New();
	vtkSmartPointer<vtkPlusTransformRepository>		repository = vtkSmartPointer<vtkPlusTransformRepository>::New();

	vtkPlusDevice									*trackerDevice;

	// Video Devices
	vtkPlusDevice									*webcamDevice;
	vtkPlusDevice									*endoDevice;

	//Plus transforms
	vtkSmartPointer<vtkMatrix4x4>					camera2Image = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					laser2Tracker = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					camera2Tracker = vtkSmartPointer<vtkMatrix4x4>::New();

	// Plus Transform Names
	PlusTransformName								camera2TrackerName = PlusTransformName("Camera", "Tracker");
	PlusTransformName								laser2TrackerName = PlusTransformName("Laser", "Tracker");
	PlusTransformName								camera2ImageName = PlusTransformName("Camera", "ImagePlane");

	// Mixers
	vtkPlusDevice									*mixerDevice;
	vtkPlusDevice									*leftMixerDevice;
	vtkPlusDevice									*rightMixerDevice;
	vtkSmartPointer<vtkPlusVirtualMixer>			mixer = vtkSmartPointer<vtkPlusVirtualMixer>::New();

	// Channels
	vtkPlusChannel									*trackerChannel;
	vtkPlusChannel									*videoChannel;
	vtkPlusChannel									*leftVideoChannel;
	vtkPlusChannel									*rightVideoChannel;
	vtkPlusChannel									*leftMixerChannel;
	vtkPlusChannel									*rightMixerChannel;
	vtkPlusChannel									*mixerChannel;

	PlusTrackedFrame								leftVideoFrame;
	PlusTrackedFrame								rightVideoFrame;
	PlusTrackedFrame								videoFrame;
	PlusTrackedFrame								leftMixerFrame;
	PlusTrackedFrame								rightMixerFrame;
	PlusTrackedFrame								mixerFrame;
	PlusTrackedFrame								trackedFrame;


	vtkPlusNDITracker								*ndiTracker;
	vtkPlusMmfVideoSource							*webcamVideo;
	vtkPlusMmfVideoSource							*endoVideo;
	vtkPlusOpenIGTLinkVideoSource					*ultrasoundVideo;

	QImage mat_to_qimage(cv::Mat frame, QImage::Format format);

	int					framePd;	// period of frame rate
	bool				isReadyToSave, isSaving, playing, mcuConnected, laserOn, trackerInit, isScanning;

	vector<cv::Vec4i>	lines;
	cv::Point			point1, point2;

	string				portname;
	string				configFile, intrinsicsFile, resultsDir, calibDir;

	Serial				*comPort;
	cv::Mat				streamImg, laserOnImg, laserOffImg;
	cv::Mat				frame;
	cv::Mat				savingMat;
	string				fileName;
	cv::VideoCapture	capture;
	cv::VideoWriter		gVideoWrite;
	int					frameWidth, frameHeight;

	int					scancount = 0;
	int					togglecount = 0;
	int					brightness = 6;
	int					contrast = 18;
	ofstream			myfile; 

	private slots:

	void toggleLaser();
	void camera_button_clicked();
	
	void saveButtonPressed();
	void connectMCU();
	void startTracker();
	void scanButtonPress();
	void scan();
	void camWebcam(bool);
	void camEndocam(bool);
	void updateTracker(); 
	void savePointCloud(); 

	void saveData(linalg::EndoPt point);
	void update_image();
	void saveVideo();
	void help();
	void about();
	void framePointsToCloud(cv::Mat &laserOff, cv::Mat &laserOn, int res, EndoModel* model);
	cv::Mat subtractLaser(cv::Mat &laserOff, cv::Mat &laserOn);
	vector<cv::Vec4i> detectLaserLine(cv::Mat &laserOff, cv::Mat &laserOn);

	public:
	double getCameraPosition(int i, int j);
	double getLaserPosition(int i, int j);


	

protected:
	
	void paintEvent(QPaintEvent* event);

	QImage _qimage;
	cv::Mat _tmp;
	cv::Mat* imagerd;
};
#endif // MAINWINDOW_H
