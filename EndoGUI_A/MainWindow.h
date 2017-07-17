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
#include <vtkMatrix3x3.h>
#include <vtkTransform.h>

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

	// Video Devices
	vtkPlusDevice									*webcamDevice;
	vtkPlusDevice									*endoDevice;
	vtkPlusDevice									*trackerDevice;
	//Plus transforms
	vtkSmartPointer<vtkMatrix4x4>					camera2Image = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					laser2Tracker = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					camera2Tracker = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					normal2Tracker = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					origin2Tracker = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					imagePlane2Tracker = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					cameraInv = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					point2ImagePlane = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tracker2ImagePlane = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tracker2Point = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					imagePlane2Point = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					pixel2Tracker = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4>					tracker2Pixel = vtkSmartPointer<vtkMatrix4x4>::New();

	vtkSmartPointer<vtkMatrix3x3>					intrinsicsMat = vtkSmartPointer<vtkMatrix3x3>::New();


	vtkTransform									*point2Tracker = vtkTransform::New();

	// Plus Transform Names
	PlusTransformName								camera2TrackerName = PlusTransformName("Camera", "Tracker");
	PlusTransformName								laser2TrackerName = PlusTransformName("Laser", "Tracker");
	PlusTransformName								camera2ImageName = PlusTransformName("Camera", "ImagePlane");
	PlusTransformName								normal2TrackerName = PlusTransformName("PlaneNormal", "Tracker");
	PlusTransformName								origin2TrackerName = PlusTransformName("PlaneOrigin", "Tracker");
	PlusTransformName								normal2LaserName = PlusTransformName("PlaneNormal", "Laser");
	PlusTransformName								imagePlane2TrackerName = PlusTransformName("ImagePlane", "Tracker");
	PlusTransformName								point2imagePlaneName = PlusTransformName("Point", "ImagePlane");
	PlusTransformName								tracker2PixelName = PlusTransformName("Tracker", "Pixel");
	PlusTransformName								tracker2ImagePlaneName = PlusTransformName("Tracker", "ImagePlane");


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
	bool				trackReady, isReadyToSave, isSaving, playing, mcuConnected, laserOn, trackerInit, isScanning;
	int dimensions[3];

	vector<cv::Vec4i>	lines;
	cv::Point			point1, point2;

	string				portname;
	string				configFile, intrinsicsFile, resultsDir, calibDir;

	Serial				*comPort;
	cv::Mat				distStreamImg, streamImg, laserOnImg, laserOffImg, distlaserOnImg, distlaserOffImg;
	cv::Mat				frame, intrinsics, distortion, savingMat, newCamMat, map1, map2;
	string				fileName;
	cv::VideoCapture	capture;
	cv::VideoWriter		gVideoWrite;
	int					frameWidth, frameHeight;
	cv::Point pt1;
	int					scanNumber = 0;
	int					scancount = 0;
	int					togglecount = 0;
	int					brightness = 6;
	int					contrast = 18;
	int					prevBrightness = 6;
	int					prevContrast = 18;
	ofstream			myfile;
<<<<<<< HEAD
	double				currentTime, prevTime;
	linalg::EndoPt camera, normal, origin;

=======
	double				currentTime, prevTime; 
	linalg::EndoPt camera, normal, origin;
	
>>>>>>> 587263f880ac41302d976457d8150b5af85e59f0

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
	void saveData(linalg::EndoPt camera, linalg::EndoPt pixel, linalg::EndoPt normal, linalg::EndoPt origin, linalg::EndoLine camLine,
		int col, int row, linalg::EndoPt calcPixel, linalg::EndoPt inter);
	void update_image();
	void saveVideo();
	void help();
	void about();
	void framePointsToCloud(cv::Mat &laserOff, cv::Mat &laserOn, int res);//, EndoModel* model);
	cv::Mat subtractLaser(cv::Mat &laserOff, cv::Mat &laserOn);
	vector<cv::Vec4i> detectLaserLine(cv::Mat &laserOff, cv::Mat &laserOn);
	//cv::Mat changeBCImage(int alpha, int beta);
	//cv::Mat getImg();

	linalg::EndoPt validatePixel(linalg::EndoPt point);

public:
	void getProjectionPosition();

	void getNormalPosition();
	void getOriginPosition();
	linalg::EndoPt getPixelPosition(int row, int col);
<<<<<<< HEAD

=======
	
>>>>>>> 587263f880ac41302d976457d8150b5af85e59f0



protected:

	void paintEvent(QPaintEvent* event);

	QImage _qimage;
	cv::Mat _tmp;
	cv::Mat* imagerd;
};
#endif // MAINWINDOW_H