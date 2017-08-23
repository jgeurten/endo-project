//Local includes
#include "MainWindow.h"
#include "ControlWidget.h"
#include "qlightwidget.h"
#include "defines.h"
#include <EndoModel.h>
#include <LinAlg.h>
#include <matrix.h>
#include <imageUtil.h>
#include <matrixUtil.h>
//#include "SerialPort.h"

//OpenCv includes
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <opencv2/opencv.hpp>

//QT includes
#include <QtWidgets>
#include <QThread>
#include <QPainter>
#include <QMainWindow>
#include <QDesktopWidget>
#include <qmessagebox.h>
#include <qfile>
#include <qstring.h>
#include <QAction>
#include <QMenu>
#include <QStatusBar>
#include <QPushButton>
#include <qabstractbutton.h>
#include <QStyle>
#include <qfiledialog.h>
#include <qmediaplayer.h>
#include <qcolor.h>
#include <qdir.h>
#include <qlineedit.h>
#include <qframe.h>
#include <qdockwidget.h>
#include <qsize.h>
#include <QFuture>
#include <QtConcurrent\qtconcurrentrun.h>
#include <QtSerialPort/QSerialPort.h>


//VTK includes
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkTransform.h>
#include <vtkImageData.h>
#include <vtkImageImport.h>
#include <vtkImageMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkSmartPointer.h>
#include <vtkTexture.h>
#include <vtkImageImport.h>
#include <vtkImageMapper.h>
#include <vtkMatrix3x3.h>
#include <vtkMatrix4x4.h>
#include <vtkVector.h>



// Plus
#include <vtkPlusNDITracker.h>
#include "PlusTrackedFrame.h"
#include "PlusConfigure.h"
#include "vtkCommand.h"
#include "vtkCallbackCommand.h"
#include "vtkPlusDataCollector.h"
#include "vtkPlusChannel.h"
#include "vtkPlusDataSource.h"
#include "vtkPlusDevice.h"
#include "vtkPlusRfProcessor.h"
#include "vtkPlusSavedDataSource.h"
#include "vtkPlusVirtualMixer.h"
#include "C:/RVTK-bin/Deps/Plus-bin/PlusApp/fCal/Toolboxes/QAbstractToolbox.h"
#include "PlusMath.h"
#include "PlusXMLUtils.h"

//PCL
#include <pcl/common/projection_matrix.h>
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


//MSDN includes
#include <Windows.h>
#include <WinBase.h>
#include <synchapi.h>
#include <string>
#include <math.h>
#include <iostream>
#include <fstream>

using namespace std;

class ControlWidget;
class MCUControlWidget;
class QSerialPort;

MainWindow::MainWindow(QWidget *parent)
	:QMainWindow(parent)

{
	playing = false;
	isReadyToSave = true;
	isSaving = false;
	mcuConnected = false;
	laserOn = false;
	trackerInit = false;
	isScanning = false;
	saveDataBool = true;
	saveAsMesh = true;
	paused = false;

	trackReady = false;

	//By default expecting red laser:
	usingRedLaser = true;
	usingGreenLaser = false;
	//cv::VideoCapture capture = new cv::VideoCapture();

	createMenus();
	createControlDock();	//create control dock in videowidget
	createStatusBar();
	createVTKObject();
	resize(QDesktopWidget().availableGeometry(this).size()*0.6);
	setWindowTitle(tr("Endo Scanner"));
	size = this->size;

	////Parameters from ./config/LogitechC920_Distortion(or Intrinsics)3.xml
	//intrinsics = (cv::Mat1d(3, 3) << 1.6676388e+03, 0, 1.172495287e+03, 0, 1.66362177e+03, 6.38079903e+02, 0, 0, 1);
	//distortion = (cv::Mat1d(1, 4) << 6.8932389700580091e-02, 1.7232721105351856e-01, -3.7101608187164112e-03, 4.9653539798082211e-03);// , -2.5076577085136855e+00);
	////distortion = (cv::Mat1d(1, 4) << 8.99827331e-002, -2.04057172e-001, -3.27174924e-003, -2.31121108e-003);

	intrinsics = (cv::Mat1d(3, 3) << 609.710537, 0, 303.200522, 0, 606.011374, 258.905227, 0, 0, 1);
	distortion = (cv::Mat1d(1, 4) << 0.084930, -0.153198, 0.011283, -0.000882);// , -2.5076577085136855e+00); 


}

//destructor

MainWindow::~MainWindow()
{
	if (capture.isOpened())
		capture.release();
}

void MainWindow::createMenus()
{
	fileMenu = menuBar()->addMenu(tr("&File"));

	exitAct = new QAction(tr("&Exit"), this);
	exitAct->setShortcuts(QKeySequence::Quit);
	exitAct->setStatusTip(tr("Exit application"));
	connect(exitAct, SIGNAL(triggered()), this, SLOT(exit()));


	// File Menu

	fileMenu->addAction(exitAct);

	//Camera menu actions:
	webcam = new QAction(tr("Webcam"), this);
	webcam->setCheckable(true);
	webcam->setChecked(true);
	connect(webcam, SIGNAL(toggled(bool)), this, SLOT(camWebcam(bool)));

	endoCam = new QAction(tr("Endocam"), this);
	endoCam->setCheckable(true);
	endoCam->setChecked(false);
	connect(endoCam, SIGNAL(toggled(bool)), this, SLOT(camEndocam(bool)));

	//Camera menu:

	cameraMenu = menuBar()->addMenu(tr("&Camera"));
	cameraMenu->addAction(webcam);
	cameraMenu->addAction(endoCam);

	//Scan menu actions
	saveScanData = new QAction(tr("Save CSV"), this);
	saveScanData->setCheckable(true);
	saveScanData->setChecked(true);
	connect(saveScanData, SIGNAL(toggled(bool)), this, SLOT(saveDataClicked(bool)));

	surfMesh = new QAction(tr("Save Mesh"), this);
	surfMesh->setCheckable(true);
	surfMesh->setChecked(true);
	connect(surfMesh, SIGNAL(toggled(bool)), this, SLOT(surfMeshClicked(bool)));


	//Scan menu:

	scanMenu = menuBar()->addMenu(tr("&Scan"));
	scanMenu->addAction(saveScanData);
	scanMenu->addAction(surfMesh);

	//Laser menu actions:
	redLaser = new QAction(tr("Red Laser"), this);
	redLaser->setCheckable(true);
	redLaser->setChecked(true);
	connect(redLaser, SIGNAL(toggled(bool)), this, SLOT(useRedLaser(bool)));

	greenLaser = new QAction(tr("Green Laser"), this);
	greenLaser->setCheckable(true);
	greenLaser->setChecked(false);
	connect(greenLaser, SIGNAL(toggled(bool)), this, SLOT(useGreenLaser(bool)));

	//laser Menu:
	laserMenu = menuBar()->addMenu(tr("Laser"));
	laserMenu->addAction(redLaser);
	laserMenu->addAction(greenLaser);


	//Help menu actions:
	aboutAct = new QAction(tr("&About"), this);
	aboutAct->setStatusTip(tr("About application"));
	connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));

	helpAct = new QAction(tr("&Help"), this);
	helpAct->setStatusTip(tr("Help with application"));
	connect(helpAct, SIGNAL(triggered()), this, SLOT(help()));

	// Help menu
	helpMenu = menuBar()->addMenu(tr("&Help"));
	helpMenu->addAction(helpAct);
	helpMenu->addAction(aboutAct);
}

void MainWindow::createStatusBar()
{
	statusBar()->showMessage(tr("Ready"));
}

void MainWindow::createControlDock()
{
	//Create dock for webcam controller:
	webcamDock = new QDockWidget(this);
	webcamDock->setAllowedAreas(Qt::LeftDockWidgetArea);
	webcamDock->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable);
	addDockWidget(Qt::LeftDockWidgetArea, webcamDock);
	webcamDock->setMinimumWidth(180);
	webcamDock->setMaximumWidth(300);

	QLabel *wlabel = new QLabel("Webcam Controls", webcamDock);
	wlabel->setStyleSheet("font-size: 10pt; font-weight: bold;");
	//wlabel->setStyleSheet("font-weight: bold;");
	webcamDock->setTitleBarWidget(wlabel);

	QGridLayout* webcamControlsLayout = new QGridLayout;
	webcamControlsLayout->setMargin(0);
	webcamControlsLayout->setSpacing(10);
	webcamControlsLayout->setAlignment(Qt::AlignTop);

	QFrame* webcamFrame = new QFrame;
	webcamFrame->setFrameStyle(QFrame::WinPanel | QFrame::Sunken);
	webcamFrame->setLineWidth(2);
	webcamFrame->setLayout(webcamControlsLayout);

	webcamDock->setWidget(webcamFrame);
	webcamControl = new WebcamControlWidget(this);
	webcamControlsLayout->addWidget(webcamControl);


	//Create dock for laser controller:
	mcuDock = new QDockWidget(this);
	mcuDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
	mcuDock->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable);
	addDockWidget(Qt::LeftDockWidgetArea, mcuDock);
	mcuDock->setMinimumWidth(180);
	mcuDock->setMaximumWidth(300);// (round(size->width()*0.33));

	QLabel *label = new QLabel("Laser Controls", mcuDock);
	label->setStyleSheet("font-size: 10pt; font-weight: bold;");
	mcuDock->setTitleBarWidget(label);

	QGridLayout* mcuControlsLayout = new QGridLayout;
	mcuControlsLayout->setMargin(0);
	mcuControlsLayout->setSpacing(10);
	mcuControlsLayout->setAlignment(Qt::AlignTop);

	QFrame* mcuFrame = new QFrame;
	mcuFrame->setFrameStyle(QFrame::WinPanel | QFrame::Sunken);
	mcuFrame->setLineWidth(2);
	mcuFrame->setLayout(mcuControlsLayout);

	mcuDock->setWidget(mcuFrame);
	mcuControl = new MCUControlWidget(this);
	mcuControlsLayout->addWidget(mcuControl);


	//Tracker controller:
	trackerDock = new QDockWidget(this);
	trackerDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
	trackerDock->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable);
	addDockWidget(Qt::LeftDockWidgetArea, trackerDock);
	trackerDock->setMinimumWidth(180);
	trackerDock->setMaximumWidth(300);// (round(size->width()*0.33));

	QLabel *tlabel = new QLabel("Tracker Controls", trackerDock);
	tlabel->setStyleSheet("font-size: 10pt; font-weight: bold;");
	trackerDock->setTitleBarWidget(tlabel);


	QGridLayout* trackerControlsLayout = new QGridLayout;
	trackerControlsLayout->setMargin(0);
	trackerControlsLayout->setSpacing(10);
	trackerControlsLayout->setAlignment(Qt::AlignTop);

	QFrame* trackerFrame = new QFrame;
	trackerFrame->setFrameStyle(QFrame::WinPanel | QFrame::Sunken);
	trackerFrame->setLineWidth(2);
	trackerFrame->setLayout(trackerControlsLayout);

	trackerDock->setWidget(trackerFrame);
	trackerControl = new ControlWidget(this);
	trackerControlsLayout->addWidget(trackerControl);

	//connect all push buttons:
	connect(webcamControl->streamButton, SIGNAL(clicked()), this, SLOT(camera_button_clicked()));
	connect(trackerControl->saveButton, SIGNAL(clicked()), this, SLOT(saveButtonPressed()));
	connect(mcuControl->mcuButton, SIGNAL(clicked()), this, SLOT(connectMCU()));
	connect(mcuControl->laserButton, SIGNAL(clicked()), this, SLOT(toggleLaser()));
	connect(trackerControl->trackerButton, SIGNAL(clicked()), this, SLOT(startTracker()));
	connect(trackerControl->scanButton, SIGNAL(clicked()), this, SLOT(scanButtonPress()));
	connect(trackerControl->viewCloud, SIGNAL(clicked()), this, SLOT(viewCloudClicked()));

	//Set intitial positions:
	webcamControl->brightSlider->setSliderPosition(brightness);
	webcamControl->contrastSlider->setSliderPosition(contrast);

	//Connect sliders:
	connect(webcamControl->brightSlider, SIGNAL(valueChanged(int)), this, SLOT(brightnessChanged(int)));
	connect(webcamControl->contrastSlider, SIGNAL(valueChanged(int)), this, SLOT(contrastChanged(int)));

	//create trackTimer to refresh the image every x milliseconds depending on the framerate of the camera
	trackTimer = new QTimer(this);

	connect(trackTimer, SIGNAL(timeout()), this, SLOT(updateTracker()));
}


void MainWindow::startTracker()
{

	trackerControl->trackerButton->setChecked(false);
	if (!trackerInit)
	{
		if (dataCollector->GetDevice(trackerDevice, "TrackerDevice") != PLUS_SUCCESS) {
			statusBar()->showMessage(tr("Unable to find Tracker device"), 5000);
			return;
		}
		if (webcam->isChecked())
		{
			// Get Ovrvision Pro device
			if (dataCollector->GetDevice(webcamDevice, "VideoDevice") != PLUS_SUCCESS)
			{
				qDebug() << "Unable to locate the device with ID = \"VideoDevice\". Check config file.";
				return;
			}

			// Get virtual mixer
			if (dataCollector->GetDevice(mixerDevice, "TrackedVideoDevice") != PLUS_SUCCESS)
			{
				qDebug() << "Unable to locate the device with ID = \"TrackedVideoDevice\". Check config file.";
				return;
			}
			webcamVideo = dynamic_cast<vtkPlusMmfVideoSource *>(webcamDevice);
			mixer = dynamic_cast<vtkPlusVirtualMixer *>(mixerDevice);
		}

		if (endoCam->isChecked())
		{
			// Get Ovrvision Pro device
			if (dataCollector->GetDevice(endoDevice, "VideoDevice") != PLUS_SUCCESS)
			{
				qDebug() << "Unable to locate the device with ID = \"VideoDevice\". Check config file.";
				return;
			}

			// Get virtual mixer
			if (dataCollector->GetDevice(mixerDevice, "TrackedVideoDevice") != PLUS_SUCCESS)
			{
				qDebug() << "Unable to locate the device with ID = \"TrackedVideoDevice\". Check config file.";
				return;
			}
			endoVideo = dynamic_cast<vtkPlusMmfVideoSource *>(endoDevice);
			mixer = dynamic_cast<vtkPlusVirtualMixer *>(mixerDevice);
		}

		ndiTracker = dynamic_cast<vtkPlusNDITracker *>(trackerDevice);
		if (ndiTracker == NULL)
		{
			LOG_ERROR("Tracking device is not NDI/Polaris/Aurora. Could not connect.");
			statusBar()->showMessage(tr("Could not connect!"), 5000);
			return;
		}

		// Connect to devices
		std::cout << "Connecting to NDI Polaris through COM" << ndiTracker->GetSerialPort();
		if (dataCollector->Connect() != PLUS_SUCCESS)
		{
			std::cout << "....................... [FAILED]" << std::endl;
			LOG_ERROR("Failed to connect to devices!");
			statusBar()->showMessage(tr("Failed to connect to devices!"), 5000);
			return;
		}

		if (dataCollector->Start() != PLUS_SUCCESS)
		{
			LOG_ERROR("Failed to connect to devices!");
			statusBar()->showMessage(tr("Failed to connect to devices!"), 5000);
			return;
		}

		std::cout << "....................... [OK]" << std::endl;

		if (repository->ReadConfiguration(configRootElement) != PLUS_SUCCESS)
		{
			LOG_ERROR("Configuration incorrect for vtkPlusTransformRepository.");
			statusBar()->showMessage(tr("Configuration incorrect for vtkPlusTransformRepository."), 5000);
			return;
		}

		if (ndiTracker->GetOutputChannelByName(trackerChannel, "TrackerStream") != PLUS_SUCCESS)
		{
			LOG_ERROR("Unable to locate the channel with Id = \"TrackerStream\". check config file.");
			statusBar()->showMessage(tr("Unable to locate the channel"), 5000);
			return;
		}


		if (!playing)
			trackTimer->start(40); //minimum is 17 ms
		trackerInit = true;
		trackerControl->trackerButton->setText(tr("Stop Tracking"));
		trackerControl->trackerButton->setChecked(true);
	}

	else  //already initalized - unitialize...
	{
		trackTimer->stop();
		trackerInit = false;
		trackerControl->trackerButton->setText(tr("Start Tracking"));
		trackerControl->trackerButton->setChecked(false);
		statusBar()->showMessage(tr("Stopping Tracking"), 2000);
		trackerControl->lightWidgets[0]->setBlue();
		trackerControl->lightWidgets[1]->setBlue();
	}

}

void MainWindow::createVTKObject()
{
	if (webcam->isChecked())
		configFile = "./config/configWebcam.xml";

	if (endoCam->isChecked())
		configFile = "./config/configEndoscope.xml";

	// Create output directory
	resultsDir = "./Results/";
	if (CreateDirectory(resultsDir.c_str(), NULL) ||
		ERROR_ALREADY_EXISTS != GetLastError())
		std::cout << "Output directory created." << std::endl;

	// Create calibration saves directory
	calibDir = "./Calibration";
	if (CreateDirectory(calibDir.c_str(), NULL) ||
		ERROR_ALREADY_EXISTS != GetLastError())
		std::cout << "calibration directory created." << std::endl;

	trackerDevice = NULL;

	// Read configuration file
	if (PlusXmlUtils::ReadDeviceSetConfigurationFromFile(configRootElement, configFile.c_str()) == PLUS_FAIL)
	{
		cout << "Unable to read configuration from file" << configFile.c_str() << endl;
		return;
	}

	vtkPlusConfig::GetInstance()->SetDeviceSetConfigurationData(configRootElement);

	// Read configuration file
	if (dataCollector->ReadConfiguration(configRootElement) != PLUS_SUCCESS)
	{
		cout << "Configuration incorrect for vtkPlusDataCollector." << endl;
		return;
	}
	//Old calibration:
	/*
	intrinsicsMat->SetElement(0, 0, 6.21962708e+002);
	intrinsicsMat->SetElement(0, 1, 0);
	intrinsicsMat->SetElement(0, 2, 3.18246521e+002);
	intrinsicsMat->SetElement(1, 0, 0);
	intrinsicsMat->SetElement(1, 1, 6.19908875e+002);
	intrinsicsMat->SetElement(1, 2, 2.36307892e+002);
	intrinsicsMat->SetElement(2, 0, 0);
	intrinsicsMat->SetElement(2, 1, 0);
	intrinsicsMat->SetElement(2, 2, 1);
	*/
	//New calibration:
	intrinsicsMat->SetElement(0, 0, 609.710537);
	intrinsicsMat->SetElement(0, 1, 0);
	intrinsicsMat->SetElement(0, 2, 303.200522);
	intrinsicsMat->SetElement(1, 0, 0);
	intrinsicsMat->SetElement(1, 1, 606.011374);
	intrinsicsMat->SetElement(1, 2, 258.905227);
	intrinsicsMat->SetElement(2, 0, 0);
	intrinsicsMat->SetElement(2, 1, 0);
	intrinsicsMat->SetElement(2, 2, 1);

	vtkMatrix3x3::Invert(intrinsicsMat, invA);

	laser2Normal->SetElement(0, 0, -0.9998151);
	laser2Normal->SetElement(0, 1, 0);
	laser2Normal->SetElement(0, 2, 0);
	laser2Normal->SetElement(0, 3, 0);
	laser2Normal->SetElement(1, 0, 0.00514673);
	laser2Normal->SetElement(1, 1, 1);
	laser2Normal->SetElement(1, 2, 0);
	laser2Normal->SetElement(1, 3, 0);
	laser2Normal->SetElement(2, 0, 0.01852691);
	laser2Normal->SetElement(2, 1, 0);
	laser2Normal->SetElement(2, 2, 1);
	laser2Normal->SetElement(2, 3, 0);
	laser2Normal->SetElement(3, 0, 0);
	laser2Normal->SetElement(3, 1, 0);
	laser2Normal->SetElement(3, 2, 0);
	laser2Normal->SetElement(3, 3, 1);

	//No need to invert
	//vtkMatrix4x4::Invert(laser2Normal, normal2Laser);

	laser2Origin->SetElement(0, 0, 1);
	laser2Origin->SetElement(0, 1, 0);
	laser2Origin->SetElement(0, 2, 0);
	laser2Origin->SetElement(0, 3, -0.9316);
	laser2Origin->SetElement(1, 0, 0);
	laser2Origin->SetElement(1, 1, 1);
	laser2Origin->SetElement(1, 2, 0);
	laser2Origin->SetElement(1, 3, -42.8642);
	laser2Origin->SetElement(2, 0, 0);
	laser2Origin->SetElement(2, 1, 0);
	laser2Origin->SetElement(2, 2, 1);
	laser2Origin->SetElement(2, 3, -28.1282);
	laser2Origin->SetElement(3, 0, 0);
	laser2Origin->SetElement(3, 1, 0);
	laser2Origin->SetElement(3, 2, 0);
	laser2Origin->SetElement(3, 3, 1);

	vtkMatrix4x4::Invert(laser2Origin, origin2Laser);
}

void MainWindow::camera_button_clicked()
{
	if (!playing) {
		capture = cv::VideoCapture(0);		//consider changing to plus get frame
		capture.open(0);
		if (capture.isOpened()) {
			capture.set(CV_CAP_PROP_FPS, 30);
			capture.set(CV_CAP_PROP_AUTOFOCUS, 0);
			frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);
			frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
			framePd = 1000 / (int)capture.get(CV_CAP_PROP_FPS);
			playing = true;
			webcamControl->streamButton->setText(tr("Stop Stream"));

			//Get undistorted transform map first time:
			capture >> distStreamImg;
			cv::undistort(distStreamImg, streamImg, intrinsics, distortion);
			cv::initUndistortRectifyMap(intrinsics, distortion, cv::Mat(), intrinsics, cv::Size(distStreamImg.cols, distStreamImg.rows),
				CV_32FC1, map1, map2);

			if (!trackerInit)
				trackTimer->start(40);

		}
		else
			statusBar()->showMessage(tr("Unable to Detect Camera"), 3000);
	}

	else {
		webcamControl->streamButton->setText(tr("Stream Video"));
		statusBar()->showMessage(tr("Ready"), 7000);
		playing = false;
		trackTimer->stop();
	}
}


void MainWindow::update_image()
{
	if (capture.isOpened())
	{
		capture.set(CV_CAP_PROP_CONTRAST, (double)contrast);
		capture.set(CV_CAP_PROP_BRIGHTNESS, (double)brightness);

		capture >> distStreamImg;
		cv::remap(distStreamImg, streamImg, map1, map2, cv::INTER_CUBIC);
		cv::Size s = streamImg.size();
		image = QImage((const unsigned char*)(streamImg.data), streamImg.cols, streamImg.rows, streamImg.cols*streamImg.channels(), QImage::Format_RGB888).rgbSwapped();

		switch (streamImg.type())	//CONVERT MAT TO QIMAGE
		{
		case CV_8UC4:
		{
			image = QImage((const unsigned char*)(streamImg.data), streamImg.cols, streamImg.rows, streamImg.cols*streamImg.channels(), QImage::Format_ARGB32);
			break;
		}

		case CV_8UC3:
		{
			image = QImage((const unsigned char*)(streamImg.data), streamImg.cols, streamImg.rows, streamImg.cols*streamImg.channels(), QImage::Format_RGB888).rgbSwapped();
			break;
		}

		default:
			qWarning() << "Type Not Handled";
			break;
		}

		if (isSaving)
			saveVideo();

		repaint();
	}
	else
		statusBar()->showMessage(tr("Unable to Detect Camera"), 5000);
}

void MainWindow::scanButtonPress()
{
	if (!mcuConnected)
	{
		statusBar()->showMessage(tr("Connect MCU first"));
		return;
	}

	if (!isScanning) {
		scanTimer = new QTimer(this);
		connect(scanTimer, SIGNAL(timeout()), this, SLOT(scan()));
		trackerControl->scanButton->setText(tr("Stop Scan"));
		trackerControl->scanButton->setChecked(true);
		scanTimer->start(30);
		isScanning = true;

		//create new instance of model to add points
		Model = new EndoModel();

		string fileLocation = "./Results/Results.csv";
		ResultsFile.open(fileLocation, ios::out | ios::ate | ios::app | ios::binary);

		//Results file header:
		ResultsFile << "Cam X," << "Cam Y," << "Cam Z,"
			<< "VecProj X," << "VecProj Y," << "VecProj Z,"
			<< "Plane Normal X," << "Plane Normal Y," << "Plane Normal Z,"
			<< "Laser Origin X," << "Laser Origin Y," << "Laser Origin Z,"
			<< "Plane A," << "B," << "C," << "D,"
			<< "CamLine.A.X," << "CamLine.A.Y," << "CamLine.A.Z,"
			<< "CamLine.B.X," << "CamLine.B.Y," << "CamLine.B.Z,"
			<< "U," << "V," //<< "Calc U," << "Calc V,"
			<< "Intersection X," << "Y," << "Z,"
			<< "Laser Trans R11," << "R12," << "R13," << "T1,"
			<< "R21," << "R22," << "R23," << "T2,"
			<< "R31," << "R32," << "R33," << "T3,"
			<< "R41," << "R42," << "R43," << "T4,"
			<< "Camera Trans R11," << "R12," << "R13," << "T1,"
			<< "R21," << "R22," << "R23," << "T2,"
			<< "R31," << "R32," << "R33," << "T3,"
			<< "R41," << "R42," << "R43," << "T4" << endl;


	}
	else		//finish scan:
	{
		trackerControl->scanButton->setText(tr("Start Scan"));
		trackerControl->scanButton->setChecked(false);
		scanTimer->stop();

		//Turn off laser if still on at end of scan.
		if (laserOn)
			toggleLaser();

		isScanning = false;
		ResultsFile.close();

		/*		//Remove outliers and down sample point cloud before saving
				int meanNN = 150;
				float StdDev = 0.9;
				statusBar()->showMessage(tr("Filtering Point Cloud. Please wait."), 8000);
				//Want to see raw PC
				Model->removeOutliers(meanNN, StdDev);	//updates pointcloud to point to a filtered pt cloud
		*/		statusBar()->showMessage(tr("Saving Point Cloud. Please Wait."), 0);
		string filename = savePointCloud();
		//convert to surface mesh and save, if enabled
		if (saveAsMesh) {
			statusBar()->showMessage(tr("Converting Point Cloud to Surface Mesh. This can take awhile."), 0);
			string st = filename.substr(0, filename.size() - 3);
			string filenameOBJ = st + "OBJ";
			//Model->convertCloudToSurface(filenameOBJ);
			statusBar()->showMessage(tr("Finished Saving Point Cloud and Mesh. Ready."), 3000);

		}
	}
}

string MainWindow::savePointCloud()
{
	//function saves both as ply and pcd
	QString filename = QFileDialog::getSaveFileName(this, "Save File", tr("./Results"), "PCD (*.pcd) ;; PLY (*.ply)");

	if (filename.isEmpty()) return "";
	qDebug() << filename;
	if (filename.endsWith(".pcd", Qt::CaseInsensitive)) {
		qDebug() << "Save as pcd file.";
		Model->savePointCloudAsPCD(filename.toStdString());
		string st = filename.toStdString();
		string filenamePly = st.substr(0, filename.size() - 3) + "PLY";
		Model->savePointCloudAsPLY(filenamePly);
		return filename.toStdString();
	}
	else if (filename.endsWith(".ply", Qt::CaseInsensitive)) {
		qDebug() << "Save as ply file.";
		Model->savePointCloudAsPLY(filename.toStdString());
		string st = filename.toStdString();
		string filenamePcd = st.substr(0, filename.size() - 3) + "PCD";
		Model->savePointCloudAsPCD(filenamePcd);
		return filename.toStdString();
	}
}

void MainWindow::scan()
{
	scancount++;
	if (scancount == 1) {
		toggleLaser();
		togglecount++;
	}

	else if (scancount == 7) {			//effectively delayed 7*30ms = 210ms

		if (togglecount % 2 == 0)
			capture >> distlaserOnImg;

		else
			capture >> distlaserOffImg;

		scancount = 0;
	}

	if (distlaserOnImg.empty() || distlaserOffImg.empty())	//only true for toggle count = 1
		return;

	if (togglecount % 2 == 0) {	//have both laser on and off successive images

		cv::remap(distlaserOnImg, laserOnImg, map1, map2, cv::INTER_CUBIC);
		cv::remap(distlaserOffImg, laserOffImg, map1, map2, cv::INTER_CUBIC);

		framePointsToCloud(laserOffImg, laserOnImg, 2);// , model);
	}
}

void MainWindow::arduinoScanPress()
{
	if (isScanning)					//if scanning, ignore
		return;
	else if (paused)				//if paused, resume scan
		paused = false;
	else
		scanButtonPress();			//not scanning and not paused --> first go button press

}

void MainWindow::arduinoPausePress()
{
	if (!isScanning)
		return;

	if (!paused)
		paused = true;
	else					//already paused -> stop scanning (double press of pause)
		scanButtonPress();
}


void MainWindow::updateTracker()
{
	if (mcuConnected)
	{
		if (serialPort->bytesAvailable() > 0)		//check for data synch
		{
			char data[7];
			qint64 maxSize = 6;
			Sleep(10);								//allow for all data to terminate writing before reading
			serialPort->read(data, maxSize);
			data[6] = '\0';
			if (strcmp(data, gobuttonres) == 0)
				arduinoScanPress();

			if (data == pauseButtonRes)
				arduinoPausePress();

			serialPort->clear();						//clear buffer
		}
	}

	if (playing)
		update_image();

	if (trackerInit)
	{
		trackerChannel->GetTrackedFrame(trackedFrame);
		repository->SetTransforms(trackedFrame);

		//check if laser and camera are seen first to set lights. If laser or camera are not seen, will break out at normal2tracker
		//origin2tracker or imageplane2 tracker. Therefore, trackReady --> false

		bool isValid(false);
		trackerChannel->GetTrackedFrame(trackedFrame);
		repository->SetTransforms(trackedFrame);


		bool isCameraMatrixValid = false;
		bool isToolMatrixValid = false;
		//Camera transforms first



		if (repository->GetTransform(camera2TrackerName, camera2Tracker, &isCameraMatrixValid) == PLUS_SUCCESS && isCameraMatrixValid)
			trackerControl->lightWidgets[0]->setGreen();

		else
			trackerControl->lightWidgets[0]->setRed();

		if (usingRedLaser)
		{
			if (repository->GetTransform(rLaser2TrackerName, rLaser2Tracker, &isToolMatrixValid) == PLUS_SUCCESS && isToolMatrixValid)
				trackerControl->lightWidgets[1]->setGreen();
			else
				trackerControl->lightWidgets[1]->setRed();

			if (repository->GetTransform(rNormal2TrackerName, rNormal2Tracker, &isValid) != PLUS_SUCCESS || !isValid)
			{
				LOG_ERROR("Unable to successfully transform red plane normal to tracker");
				trackReady = false;
				return;
			}

			if (repository->GetTransform(rOrigin2TrackerName, rOrigin2Tracker, &isValid) != PLUS_SUCCESS || !isValid)
			{
				LOG_ERROR("Unable to successfully transform red plane origin to tracker");
				trackReady = false;
				return;
			}
		}

		if (usingGreenLaser)
		{
			bool isToolMatrixValid = false;

			if (repository->GetTransform(gLaser2TrackerName, gLaser2Tracker, &isToolMatrixValid) == PLUS_SUCCESS && isToolMatrixValid)
				trackerControl->lightWidgets[1]->setGreen();
			else
				trackerControl->lightWidgets[1]->setRed();


			if (repository->GetTransform(gNormal2TrackerName, gNormal2Tracker, &isValid) != PLUS_SUCCESS || !isValid)
			{
				LOG_ERROR("Unable to successfully transform plane normal to tracker");
				trackReady = false;
				return;
			}

			if (repository->GetTransform(gOrigin2TrackerName, gOrigin2Tracker, &isValid) != PLUS_SUCCESS || !isValid)
			{
				LOG_ERROR("Unable to successfully transform plane origin to tracker");
				trackReady = false;
				return;
			}

		}

		if (repository->GetTransform(imagePlane2TrackerName, imagePlane2Tracker, &isValid) != PLUS_SUCCESS || !isValid)
		{
			LOG_ERROR("Unable to successfully transform image plane to tracker");
			trackReady = false;
			return;
		}

		if (repository->GetTransform(tracker2ImagePlaneName, tracker2ImagePlane, &isValid) != PLUS_SUCCESS || !isValid)
		{
			LOG_ERROR("Unable to successfully transform tracker 2 image plane");
			trackReady = false;
			return;
		}

		trackReady = true;
	}
}

linalg::EndoPt MainWindow::getCameraPosition()
{
	linalg::EndoPt camera;
	camera.x = camera2Tracker->GetElement(0, 3);		// camera w.r.t. tracker. Forms the origin of the line to pixel
	camera.y = camera2Tracker->GetElement(1, 3);
	camera.z = camera2Tracker->GetElement(2, 3);
	return camera;

}

linalg::EndoPt MainWindow::getNormalPosition()
{
	linalg::EndoPt normal, unitNorm;
	// 
	// //Get normals components:
	// normal.x = rNormal2Tracker->GetElement(0, 0);
	// normal.y = rNormal2Tracker->GetElement(1, 0);
	// normal.z = rNormal2Tracker->GetElement(2, 0);
	// 
	vtkSmartPointer<vtkMatrix4x4> normal2Tracker = vtkSmartPointer<vtkMatrix4x4>::New();

	bool isToolMatrixValid(false);
	//origin 2 tracker = laser 2 tracker * normal 2 laser
	repository->GetTransform(rLaser2TrackerName, rLaser2Tracker, &isToolMatrixValid);
	vtkMatrix4x4::Multiply4x4(rLaser2Tracker, laser2Normal, normal2Tracker);

	normal.x = normal2Tracker->GetElement(0, 0);
	normal.y = normal2Tracker->GetElement(1, 0);
	normal.z = normal2Tracker->GetElement(2, 0);

	//Normalize vector
	double norm = sqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
	unitNorm.x = normal.x / norm;
	unitNorm.y = normal.y / norm;
	unitNorm.z = normal.z / norm;

	return unitNorm;
}

linalg::EndoPt MainWindow::getOriginPosition()
{
	linalg::EndoPt origin;
	vtkSmartPointer<vtkMatrix4x4> origin2Tracker = vtkSmartPointer<vtkMatrix4x4>::New();
	bool isToolMatrixValid(false);
	repository->GetTransform(rLaser2TrackerName, rLaser2Tracker, &isToolMatrixValid);
	vtkMatrix4x4::Multiply4x4(rLaser2Tracker, origin2Laser, origin2Tracker);

	origin.x = origin2Tracker->GetElement(0, 3);
	origin.y = origin2Tracker->GetElement(1, 3);
	origin.z = origin2Tracker->GetElement(2, 3);

	return origin;
}

linalg::EndoPt MainWindow::getGreenNormalPosition()
{
	linalg::EndoPt normal, unitNorm;
	normal.x = gNormal2Tracker->GetElement(0, 0);
	normal.y = gNormal2Tracker->GetElement(1, 0);
	normal.z = gNormal2Tracker->GetElement(2, 0);

	double norm = sqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
	unitNorm.x = normal.x / norm;
	unitNorm.y = normal.y / norm;
	unitNorm.z = normal.z / norm;

	return unitNorm;
}

linalg::EndoPt MainWindow::getGreenOriginPosition()
{
	linalg::EndoPt origin;

	origin.x = gOrigin2Tracker->GetElement(0, 3);
	origin.y = gOrigin2Tracker->GetElement(1, 3);
	origin.z = gOrigin2Tracker->GetElement(2, 3);

	return origin;
}


void MainWindow::saveButtonPressed()
{
	if (isReadyToSave && capture.isOpened()) {

		int format = capture.get(CV_CAP_PROP_FORMAT);
		int mode = capture.get(CV_CAP_PROP_MODE);

		QString filename = QFileDialog::getSaveFileName(this, tr("Save File"),
			"./Videos/untitled",
			tr("AWI File (*.avi);; All Files(*.)"));

		if (!filename.isEmpty())
		{
			string fileName = filename.toStdString();
			int fps = (int)capture.get(CV_CAP_PROP_FPS);
			cv::Size lVideoSize = cv::Size(frameWidth, frameHeight);

			gVideoWrite = cv::VideoWriter(fileName, CV_FOURCC('D', 'I', 'V', 'X'), fps / 2, lVideoSize, true);	//video is sped up by a factor of 2

			if (gVideoWrite.isOpened())
			{
				trackerControl->saveButton->setText("End Saving Video");
				isReadyToSave = false;
				isSaving = true;
			}
		}
		else
			statusBar()->showMessage(tr("Unable to save video"));
	}
	else
	{
		trackerControl->saveButton->setText("Save Video");
		gVideoWrite.release();		//release to close open file and finish saving process
		isReadyToSave = true;
		isSaving = false;
	}
}

void MainWindow::saveVideo()
{
	if (capture.isOpened()) {
		//capture.grab();
		capture >> savingMat;
		gVideoWrite.write(savingMat);
		qDebug() << "Save Video Thread";
	}
	else
		statusBar()->showMessage(tr("Unable to open video stream"), 2000);
}

void MainWindow::paintEvent(QPaintEvent*)
{
	QPainter painter(this);
	painter.drawImage(QRectF(300, 100, frameWidth, frameHeight), image);
}

void MainWindow::toggleLaser()
{
	mcuControl->laserButton->setChecked(false);

	if (mcuConnected && !laserOn)
	{
		//serialPort->flush();			//end of transmission 
		const char hg[1] = { '1' };
		QByteArray writeDataon(hg);

		qint64 bytesWritten = serialPort->write("1");
		if (bytesWritten > 0)
		{
			laserOn = true;
			mcuControl->laserButton->setText(tr("Laser Off"));
			mcuControl->laserButton->setChecked(true);
			/*	Sleep(10);
				if (serialPort->bytesAvailable() > 0)
				{
					char test[3] = { 'O', 'N', '\0' };
					char data[3];
					qint64 maxSize = 3;
					serialPort->readLine(data, maxSize);
					data[2] = '\0';
					if (strcmp(data, test) == 0)
					{
						laserOn = true;
						mcuControl->laserButton->setText(tr("Laser Off"));
						mcuControl->laserButton->setChecked(true);
					}
					else
						toggleLaser();			//laser not on, try again.
				}
				*/
		}
	}
	//All takes 1ms + sleep delay

	else if (mcuConnected && laserOn)
	{
		serialPort->flush();
		QByteArray writeData("2");

		qint64 bytesWritten = serialPort->write(writeData);
		if (bytesWritten > 0)
		{
			//Read response protocol:

			/*
			Sleep(10);
			if (serialPort->bytesAvailable() > 0)
			{
				char offtest[4] = { 'O', 'F', 'F','\0' };
				char offdata[10];
				qint64 maxSize = 10;
				serialPort->read(offdata, maxSize);
				offdata[3] = '\0';
				if (strcmp(offdata, offtest) == 0)
				{
					laserOn = false;
					mcuControl->laserButton->setText(tr("Laser On"));
					mcuControl->laserButton->setChecked(false);
				}
				else
					toggleLaser();			//laser not on, try again.
			}
			*/
			laserOn = false;
			mcuControl->laserButton->setText(tr("Laser On"));
			mcuControl->laserButton->setChecked(false);
		}
	}

	else
	{
		statusBar()->showMessage(tr("Connect MCU First"), 2000);
	}
}

QImage MainWindow::mat_to_qimage(cv::Mat laserOffImg, QImage::Format format)
{
	return QImage(laserOffImg.data, laserOffImg.cols, laserOffImg.rows,
		static_cast<int>(laserOffImg.step), format);
}

//get serial com port object
void MainWindow::connectMCU() {

	mcuControl->mcuButton->setChecked(false);
	if (!mcuConnected)
	{
		//Initialize all serial commands: msg = send to arduino. res = received from arduino

		//turn laser on command:
		//  laserOnMsg.resize(4);
		//  laserOnMsg[0] = 0xFF;		//Msg byte					//46, 46
		//  laserOnMsg[1] = 0xFF;		//On	
		//  laserOnMsg[2] = 0xAA;		//Laser specific tag byte		41,41
		//  laserOnMsg[3] = 0x00;		//Sent							0,0

		QString temp("FFFFAA00");
		laserOnMsg = temp.toLatin1();

		//turn laser off command
		laserOffMsg.resize(4);
		laserOffMsg[0] = 0xFF;
		laserOffMsg[1] = 0x00;		//Off
		laserOffMsg[2] = 0xAA;
		laserOffMsg[3] = 0x00;

		//Receive from Arduino signalling received laser on msg
		laserOnRes.resize(4);
		laserOnRes[0] = 0x00;		//Response
		laserOnRes[1] = 0xFF;		//On		
		laserOnRes[2] = 0xAA;		//Laser
		laserOnRes[3] = 0x01;		//received

		//Receive from Arduino -> received laser off msg
		laserOffRes.resize(3);
		laserOffRes[0] = 0xFF;		//Response
		laserOffRes[1] = 0xAA;		//On		
		laserOffRes[2] = 0xA3;		//Laser


		//send to mcu when go button res has been received
		goButtonMsg.resize(4);
		goButtonMsg[0] = 0xFF;		//Message
		goButtonMsg[1] = 0xFF;		//Go		
		goButtonMsg[2] = 0xBB;		//Button
		goButtonMsg[3] = 0x01;		//

		//Sent from arduino when go button is pressed
		//goButtonRes.resize(3);
		//goButtonRes[0] = 0xFF;		//Response
		//goButtonRes[1] = 0xAA;		//Go		
		//goButtonRes[2] = 0xB3;		//Button
		//
		//QString temp2("FFAAB3");
		//goButtonRes = temp2.toLatin1();


		//Send to mcu after getting pause res
		pauseButtonMsg.resize(4);
		pauseButtonMsg[0] = 0xFF;		//Message
		pauseButtonMsg[1] = 0x00;		//Pasue		
		pauseButtonMsg[2] = 0xBB;		//Button
		pauseButtonMsg[3] = 0x01;		//

		//Sent from arduino to signal pause
		pauseButtonRes.resize(4);
		goButtonRes[0] = 0x00;		//Response
		goButtonRes[1] = 0x00;		//pause		
		goButtonRes[2] = 0xBB;		//Button
		goButtonRes[3] = 0x01;

		bool okay;

		int portnumber = QInputDialog::getInt(this, tr("Connect MCU"), tr("Enter COM Port #:"), 0, 0, 100, 1, &okay);
		portname = "COM" + to_string(portnumber);

		if (okay && portnumber > 0) {
			serialPort = new QSerialPort();
			serialPort->setPortName(QString::fromStdString(portname));
			serialPort->setBaudRate(QSerialPort::Baud9600);
			serialPort->setParity(QSerialPort::NoParity);			//no parity
			serialPort->setReadBufferSize(4);						//qserialport will buffer 4 bytes( 0x 33 22 11 00) 
			serialPort->setFlowControl(QSerialPort::NoFlowControl);

			if (!serialPort->open(QIODevice::ReadWrite))
			{
				statusBar()->showMessage(tr("Unable to connect to COM Port") + QString::number(portnumber), 3000);
				mcuConnected = false;
				return;
			}
			else
			{
				mcuControl->mcuButton->setText(tr("Disconnect MCU"));
				mcuControl->mcuButton->setChecked(true);
				statusBar()->showMessage(tr("MCU Connected"), 2000);
				mcuConnected = true;
				serialPort->flush();
			}
		}
	}
	else {
		mcuConnected = false;
		statusBar()->showMessage(tr("MCU Disconnected."), 2000);
		mcuControl->mcuButton->setText(tr("Connect MCU"));
	}
}

void MainWindow::viewCloudClicked()
{
	if (isScanning) {
		trackerControl->viewCloud->setChecked(false);
		return;
	}

	trackerControl->viewCloud->setChecked(true);

	QString filename = QFileDialog::getOpenFileName(this, tr("Open File"),
		"./Results", tr("3D Scan Files (*.pcd *.ply *.OBJ)"));

	if (filename.isEmpty())
		return;

	else
		return;
	//TO DO:
	//EndoModel::createVTKPC(filename.toStdString());
}

void MainWindow::camWebcam(bool checked)
{
	if (!trackerInit) {
		statusBar()->showMessage(tr("Start Tracker First"), 5000);
		return;
	}

	if (checked)
	{
		dataCollector->Stop();
		dataCollector->Disconnect();
		dataCollector = NULL;

		dataCollector = vtkSmartPointer<vtkPlusDataCollector>::New();

		endoCam->setChecked(false);

		createVTKObject();
		mixer->GetChannel()->GetTrackedFrame(mixerFrame);

		bool isMatrixValid(false);
		repository->SetTransforms(mixerFrame);
		if (repository->GetTransform(PlusTransformName("Webcam", "Tracker"), camera2Tracker, &isMatrixValid) == PLUS_SUCCESS && isMatrixValid)
		{
			statusBar()->showMessage(tr("Webcam now being used & tracked"), 5000);
		}
	}

	else
	{
		dataCollector->Stop();
		dataCollector->Disconnect();
		dataCollector = NULL;

		dataCollector = vtkSmartPointer<vtkPlusDataCollector>::New();

		createVTKObject();
		webcam->setCheckable(true);
		endoCam->setCheckable(true);
	}


}

void MainWindow::saveDataClicked(bool checked)
{
	saveDataBool = !saveDataBool;
	if (!saveDataBool)
		saveScanData->setChecked(false);
	else
		saveScanData->setChecked(true);

}

void MainWindow::surfMeshClicked(bool checked)
{
	saveAsMesh = !saveAsMesh;
	if (saveAsMesh)
		surfMesh->setChecked(true);
	else
		surfMesh->setChecked(false);
}


void MainWindow::camEndocam(bool checked)
{
	if (!trackerInit) {
		statusBar()->showMessage(tr("Start Tracker First"), 5000);
		return;
	}

	if (checked)
	{
		dataCollector->Stop();
		dataCollector->Disconnect();
		dataCollector = NULL;

		dataCollector = vtkSmartPointer<vtkPlusDataCollector>::New();

		// Ensure that all other cameras are unselected
		webcam->setChecked(false);

		createVTKObject();

		mixer->GetChannel()->GetTrackedFrame(mixerFrame);

		bool isMatrixValid(false);
		repository->SetTransforms(mixerFrame);

		if (repository->GetTransform(PlusTransformName("Endo", "Tracker"), camera2Tracker, &isMatrixValid) == PLUS_SUCCESS && isMatrixValid)
			statusBar()->showMessage(tr("Endoscope now being used & tracked"), 5000);
	}

	else
	{
		dataCollector->Stop();
		dataCollector->Disconnect();
		dataCollector = NULL;

		dataCollector = vtkSmartPointer<vtkPlusDataCollector>::New();

		createVTKObject();
		endoCam->setChecked(false);
	}
}

cv::Mat MainWindow::subtractLaser(cv::Mat &laserOff, cv::Mat &laserOn)
{
	//take in image with and without laser ON and return subtracted cv mat image
	cv::Mat bwLaserOn, bwLaserOff, subImg, subImgBW, threshImg;
	cv::Mat lineImg(480, 640, CV_8U, cv::Scalar(0));	//fill with 0's for black and white img 

	cv::subtract(laserOn, laserOff, subImg);
	cv::cvtColor(subImg, subImgBW, CV_RGB2GRAY);
	//cv::cvtColor(laserOn, bwLaserOn, CV_RGB2GRAY);
	//cv::subtract(bwLaserOn, bwLaserOff, subImgBW);

	cv::threshold(subImgBW, threshImg, 10, 255, CV_THRESH_TOZERO);
	cv::blur(threshImg, threshImg, cv::Size(5, 5));

	for (int rowN = 0; rowN < laserOff.rows; rowN++)
	{
		int count = 0;
		int avgCol = 0;
		int columns[640];
		for (int colN = 0; colN < laserOff.cols; colN++)
		{
			if (threshImg.at<uchar>(rowN, colN) > 5)				//using diff img. Clear and thin line
			{
				columns[count] = colN;
				count++;
			}
		}

		for (int index = 0; index < count - 2; index++)
		{
			if (columns[index + 2] - columns[index] < 5) {	//if > 5 away, new line or noise
				int middleCol = round((columns[index + 1] + columns[index]) / 2);
				lineImg.at<uchar>(rowN, middleCol) = 255;
			}
		}
	}
	return lineImg;
}

cv::Mat MainWindow::subImAlgo(cv::Mat &laserOff, cv::Mat &laserOn, int laserColor)
{

	//Define local cv"mats
	cv::Mat subImg, bgr[3], filteredRed, threshImg, greyMat, convImg;
	//subtract laser on and off matricies to get the laser
	cv::subtract(laserOn, laserOff, subImg);

	//split subtracted image into red channel
	cv::split(subImg, bgr);

	//use median blur to eliminate noise in background
	cv::medianBlur(bgr[laserColor], filteredRed, 9);


	//Perform convolution using formula from conv2 matlab
	int const gsize = 30;

	int const sigma = 30;
	double sumGaussfilt = 28.7156;	//derived in matlab
	int linspace[gsize];
	double gaussFilt[gsize];
	cv::Mat gaussDist(1, gsize, CV_32F);
	for (int i = 0; i < gsize; i++)
	{
		linspace[i] = -gsize / 2 + i*gsize / (gsize - 1);
		//gaussFilt[i] = (exp(-1 * (linspace[i] ^ 2) / 2 * sigma ^ 2))/sumGaussfilt;
		gaussDist.at<float>(0, i) = (exp(-1 * (linspace[i] ^ 2) / 2 * sigma ^ 2)) / sumGaussfilt;
	}

	//Matrix<double> h(1, 30);

	cv::GaussianBlur(filteredRed, convImg, cv::Size(29, 29), (double)sigma, 0, 4);
	//filter2D(filteredRed, convImg, -1, gaussDist, cv::Point(-1, -1), 0, 0);

	//threshold image: if > 10, otherwise, possibly noise
	cv::threshold(filteredRed, threshImg, 10, 255, CV_THRESH_TOZERO);
	return threshImg;
}

vector<cv::Vec4i> MainWindow::detectLaserLine(cv::Mat &laserOff, cv::Mat &laserOn)
{
	cv::Mat laserLine = subtractLaser(laserOff, laserOn);

	vector<cv::Vec4i> lines;
	cv::Mat laserLineBW(laserOff.rows, laserOff.cols, CV_8U, cv::Scalar(0));
	cv::cvtColor(laserLine, laserLineBW, CV_RGB2GRAY);

	cv::HoughLinesP(laserLineBW, lines, 1, CV_PI / 180, 20, 50, 10);

	if (lines.size() == 0) {	//lines not detected
		vector<cv::Vec4i> nullVec(1, 0);
		return nullVec;
	}
	return lines;
}

void MainWindow::framePointsToCloud(cv::Mat &laserOn, cv::Mat &laserOff, int res)//, EndoModel* model)
{
	//check if able to transform all entities into tracker space

	if (!trackReady)
	{
		LOG_ERROR("Unable to transform one or many translations");
		return;
	}

	if (paused) return;
	//get camera centre 
	linalg::EndoPt camera = getCameraPosition();	//updates camera coords

	linalg::EndoPt vectorProj, calcPixel, origin, normal;

	int colorCode;
	//laser plane geometry
	if (usingRedLaser) {
		normal = getNormalPosition();		//updates normal
		origin = getOriginPosition();		//updates origin
		colorCode = 2;
	}

	else {
		normal = getGreenNormalPosition();		//updates normal
		origin = getGreenOriginPosition();		//updates origin
		colorCode = 1;
	}

	

	cv::Mat laserLineImg = subImAlgo(laserOff, laserOn, colorCode);

	int maxIndicies[480] = {};	//initialize as all 0's in order to access laserLineImg.at<uchar>

	for (int row = 0; row < laserOff.rows; row++) {
		for (int col = 0; col < laserOff.cols; col++)
		{
			if (laserLineImg.at<uchar>(row, col) > laserLineImg.at<uchar>(row, maxIndicies[row]))		//find col of highest intensity per row	
				maxIndicies[row] = col;
		}
	}

	cv::Mat result(480, 640, CV_8U, cv::Scalar(0));	//fill with 0's for black and white img 

	//Following is simply for visualization, nothing more
	for (int iterator = 0; iterator < laserOff.rows; iterator++)
	{
		if (maxIndicies[iterator] > 0)												//if max col val is 0, not part of laser
			result.at<uchar>(iterator, maxIndicies[iterator]) = 255;
	}

	imshow("Result", result);

	//Debugging

	for (int row = 0; row < laserOff.rows; row++)
	{
		if (maxIndicies[row] > 0)															//if max value is located at column = 0, laser line not detected for that row. 
		{
			vectorProj = getPixelDirection(row, maxIndicies[row]);														//returns pixel 2 world coords
			linalg::EndoLine camLine = linalg::linePtVector(camera, vectorProj);						//in world coordinates
			linalg::EndoPlane plane = linalg::MakePlane(normal, origin);
			linalg::EndoPt intersection = linalg::solveIntersection(normal, origin, camLine);		//in world coordinates

			if (intersection.x == 0.0)
			{
				qDebug("No intersection found");
				break;
			}
			else
			{
				//calcPixel = validatePixel(intersection);
				Model->addPointToPointCloud(intersection);				//change after testing

				if (saveDataBool)
					saveData(camera, vectorProj, normal, origin, plane, camLine, maxIndicies[row], row, intersection);

				//saveData(camera, vectorProj, normal, origin, camLine, maxIndicies[row], row, calcPixel, intersection);
			}
		}
	}
}

linalg::EndoPt MainWindow::getPixelDirection(int row, int col)		//returns pixel location in tracker space
{
	linalg::EndoPt direction;
	double pixelPrime[3], normPrime[4], result[4];
	double pixel[3] = { col, row, 1.00 };
	invA->MultiplyPoint(pixel, pixelPrime);

	//normalize and append 0 (for vectors):
	double length = sqrt(pixelPrime[0] * pixelPrime[0] + pixelPrime[1] * pixelPrime[1] + pixelPrime[2] * pixelPrime[2]);

	for (int i = 0; i < 3; i++)
		normPrime[i] = pixelPrime[i] / length;
	normPrime[3] = 0;

	//convert vector direction into world coordinate system
	camera2Tracker->MultiplyPoint(normPrime, result);

	//Copy result to EndoPt data type for handling in linalg::linePtVector
	direction.x = result[0];
	direction.y = result[1];
	direction.z = result[2];
	return direction;
}

linalg::EndoPt MainWindow::validatePixel(linalg::EndoPt point)
{
	double plane2FeaturePt[4], normPlane2Feat[3], world2Feat[3];
	linalg::EndoPt result;
	double pointVector[4] = { point.x, point.y, point.z, 1 };		//point append 1
	tracker2ImagePlane->MultiplyPoint(pointVector, plane2FeaturePt);
	normPlane2Feat[0] = plane2FeaturePt[0] / plane2FeaturePt[3];
	normPlane2Feat[1] = plane2FeaturePt[1] / plane2FeaturePt[3];
	normPlane2Feat[2] = plane2FeaturePt[2] / plane2FeaturePt[3];
	intrinsicsMat->MultiplyPoint(normPlane2Feat, world2Feat);
	result.x = world2Feat[0] / world2Feat[2];
	result.y = world2Feat[1] / world2Feat[2];
	result.z = 0;
	return result;
}

void MainWindow::help()
{
	QMessageBox::about(this, tr("Help Page for Endoscanner"),
		tr("Calibrate: calibrate camera prior to any scan \n\n "
			"Camera: ensure correct camera is selected prior to the scan \n\n"
			"Laser: ensure laser is toggled to the on selection \n\n"
			"Save: once the endoscanner has finished the scan to save video \n\n"
			"3D rendering: after saving the video, click on 3D render"));
}

void MainWindow::about()
{
	QMessageBox::about(this, tr("About Page - Something about the GUI"),
		tr("This is a GUI to control the endoscope scanner \n\n"
			"Developed by: \t\t"
			"Jordan Geurten"));
}


void MainWindow::saveData(linalg::EndoPt cameraPt, linalg::EndoPt pixelPt, linalg::EndoPt normalPt, linalg::EndoPt originPt, linalg::EndoPlane plane, linalg::EndoLine cameraline,
	int col, int row, linalg::EndoPt inter)
{

	//CAM:
	ResultsFile << cameraPt.x << ",";
	ResultsFile << cameraPt.y << ",";
	ResultsFile << cameraPt.z << ",";

	//Pixel:
	ResultsFile << pixelPt.x << ",";
	ResultsFile << pixelPt.y << ",";
	ResultsFile << pixelPt.z << ",";

	//Normal:
	ResultsFile << normalPt.x << ",";
	ResultsFile << normalPt.y << ",";
	ResultsFile << normalPt.z << ",";

	//Origin:
	ResultsFile << originPt.x << ",";
	ResultsFile << originPt.y << ",";
	ResultsFile << originPt.z << ",";

	//Plane:
	ResultsFile << plane.a << ",";
	ResultsFile << plane.b << ",";
	ResultsFile << plane.c << ",";
	ResultsFile << plane.d << ",";

	//Camera Line:
	ResultsFile << cameraline.a.x << ",";
	ResultsFile << cameraline.a.y << ",";
	ResultsFile << cameraline.a.z << ",";
	ResultsFile << cameraline.b.x << ",";
	ResultsFile << cameraline.b.y << ",";
	ResultsFile << cameraline.b.z << ",";


	//Original U and V:
	ResultsFile << col << ",";
	ResultsFile << row << ",";


	//intersection:
	ResultsFile << inter.x << ",";
	ResultsFile << inter.y << ",";
	ResultsFile << inter.z << ",";

	//Laser to tracker transform
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			ResultsFile << rLaser2Tracker->GetElement(i, j) << ",";
		}
	}

	//Camera to tracker transform
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			ResultsFile << camera2Tracker->GetElement(i, j) << ",";
		}
	}
	ResultsFile << endl;
}

void MainWindow::contrastChanged(int sliderPos)
{
	contrast = sliderPos;
}

void MainWindow::brightnessChanged(int sliderPos)
{
	brightness = sliderPos;
}

void MainWindow::useGreenLaser(bool checked)
{
	if (checked)
	{
		usingGreenLaser = true;
		usingRedLaser = false;
		greenLaser->setChecked(true);
		redLaser->setChecked(false);
	}
}

void MainWindow::useRedLaser(bool checked)
{
	if (checked)
	{
		usingGreenLaser = false;
		usingRedLaser = true;
		greenLaser->setChecked(false);
		redLaser->setChecked(true);
	}
}