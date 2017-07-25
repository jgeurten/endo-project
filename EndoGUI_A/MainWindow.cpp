//Local includes
#include "MainWindow.h"
#include "Serial.h"
#include "ControlWidget.h"
#include "qlightwidget.h"
#include "defines.h"
#include <EndoModel.h>
#include <LinAlg.h>
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
// #include <QtSerialPort/QSerialPort>
// #include <QtSerialPort/QSerialPortInfo>

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

	trackReady = false;
	//cv::VideoCapture capture = new cv::VideoCapture();

	createMenus();
	createControlDock();	//create control dock in videowidget
	createStatusBar();
	createVTKObject();
	resize(QDesktopWidget().availableGeometry(this).size()*0.6);
	setWindowTitle(tr("Endo Scanner"));
	size = this->size;

	//Parameters from ./config/LogitechC920_Distortion(or Intrinsics)3.xml
	intrinsics = (cv::Mat1d(3, 3) << 6.21962708e+002, 0, 3.18246521e+002, 0, 6.19908875e+002, 2.36307892e+002, 0, 0, 1);
	distortion = (cv::Mat1d(1, 4) << 8.99827331e-002, -2.04057172e-001, -3.27174924e-003, -2.31121108e-003);
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

	//connect(trackTimer, SIGNAL(timeout()), this, SLOT(updateTracker()));

}

void MainWindow::checkComPort()
{
	//QSerialPort serial;
	//serial.setPortName("COM6");
	//serial.open(QIODevice::ReadWrite);
	//serial.setBaudRate(QSerialPort::Baud115200);
	//serial.setDataBits(QSerialPort::Data8);
	//serial.setParity(QSerialPort::NoParity);
	//serial.setStopBits(QSerialPort::OneStop);
	//serial.setFlowControl(QSerialPort::NoFlowControl);
	//
	//if (serial.isOpen() && serial.isWritable())
	//{
	//
	//	QByteArray ba("11");
	//	serial.write(ba);
	//	serial.flush();
	//	qDebug() << "data has been send" << endl;
	//	serial.close();
	//}
	//
	//else
	//{
	//	qDebug() << "An error occured" << endl;
	//}
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

	intrinsicsMat->SetElement(0, 0, 6.21962708e+002);
	intrinsicsMat->SetElement(0, 1, 0);
	intrinsicsMat->SetElement(0, 2, 3.18246521e+002);
	intrinsicsMat->SetElement(1, 0, 0);
	intrinsicsMat->SetElement(1, 1, 6.19908875e+002);
	intrinsicsMat->SetElement(1, 2, 2.36307892e+002);
	intrinsicsMat->SetElement(2, 0, 0);
	intrinsicsMat->SetElement(2, 1, 0);
	intrinsicsMat->SetElement(2, 2, 1);
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
			<< "Pixel X," << "Pixel Y," << "Pixel Z,"
			<< "Plane Normal X," << "Plane Normal Y," << "Plane Normal Z,"
			<< "Laser Origin X," << "Laser Origin Y," << "Laser Origin Z,"
			<< "CamLine.A.X," << "CamLine.A.Y," << "CamLine.A.Z,"
			<< "CamLine.B.X," << "CamLine.B.Y," << "CamLine.B.Z,"
			<< "U," << "V," << "Calc U," << "Calc V,"
			<< "Intersection X," << "Y," << "Z" << endl;


	}
	else {
		trackerControl->scanButton->setText(tr("Start Scan"));
		trackerControl->scanButton->setChecked(false);
		scanTimer->stop();

		//Turn off laser if still on at end of scan.
		if (laserOn)
			toggleLaser();

		isScanning = false;
		ResultsFile.close();

		//Remove outliers and down sample point cloud before saving
		int meanNN = 50;
		float StdDev = 1.0;
		statusBar()->showMessage(tr("Filtering Point Cloud. Please wait."), 8000);
		//Want to see raw PC
		Model->removeOutliers(meanNN, StdDev);	//updates pointcloud to point to a filtered pt cloud

		string filename = savePointCloud();
		//convert to surface mesh and save, if enabled
		if (saveAsMesh) {
			statusBar()->showMessage(tr("Converting Point Cloud to Surface Mesh. This can take awhile."), 15000);
			Model->convertCloudToSurface();
			string st = filename.substr(0, filename.size() - 3);
			string filenameOBJ = st + "OBJ";
			Model->saveMesh(filenameOBJ);
		}
	}
}

string MainWindow::savePointCloud()
{

	QString filename = QFileDialog::getSaveFileName(this, "Save File", tr("./Results"), "PCD (*.pcd) ;; PLY (*.ply)");

	if (filename.isEmpty()) return "";
	qDebug() << filename;
	if (filename.endsWith(".pcd", Qt::CaseInsensitive)) {
		qDebug() << "Save as pcd file.";
		Model->savePointCloudAsPCD(filename.toStdString());
		return filename.toStdString();
	}
	else if (filename.endsWith(".ply", Qt::CaseInsensitive)) {
		qDebug() << "Save as ply file.";
		Model->savePointCloudAsPLY(filename.toStdString());
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
		framePointsToCloud(laserOffImg, laserOnImg, 3);// , model);
	}
}

void MainWindow::updateTracker()
{
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

		bool isToolMatrixValid = false;
		bool isCameraMatrixValid = false;

		if (repository->GetTransform(laser2TrackerName, laser2Tracker, &isToolMatrixValid) == PLUS_SUCCESS && isToolMatrixValid)
			trackerControl->lightWidgets[1]->setGreen();
		else
			trackerControl->lightWidgets[1]->setRed();

		if (repository->GetTransform(camera2TrackerName, camera2Tracker, &isCameraMatrixValid) == PLUS_SUCCESS && isCameraMatrixValid)
			trackerControl->lightWidgets[0]->setGreen();

		else
			trackerControl->lightWidgets[0]->setRed();


		if (repository->GetTransform(normal2TrackerName, normal2Tracker, &isValid) != PLUS_SUCCESS || !isValid) {
			LOG_ERROR("Unable to successfully transform plane normal to tracker");
			trackReady = false;
			return;
		}
		if (repository->GetTransform(origin2TrackerName, origin2Tracker, &isValid) != PLUS_SUCCESS || !isValid) {
			LOG_ERROR("Unable to successfully transform plane origin to tracker");
			trackReady = false;
			return;
		}

		if (repository->GetTransform(imagePlane2TrackerName, imagePlane2Tracker, &isValid) != PLUS_SUCCESS || !isValid)
		{
			LOG_ERROR("Unable to successfully transform image plane to tracker");
			trackReady = false;
			return;
		}
		/*
		if (repository->GetTransform(camera2ImageName, camera2Image, &isValid) != PLUS_SUCCESS || !isValid)
		{
		LOG_ERROR("Unable to successfully transform camera to image");
		trackReady = false;
		return;
		}
		*/

		if (repository->GetTransform(tracker2ImagePlaneName, tracker2ImagePlane, &isValid) != PLUS_SUCCESS || !isValid)
		{
			LOG_ERROR("Unable to successfully transform tracker 2 image plane");
			trackReady = false;
			return;
		}

		trackReady = true;
	}
}

void MainWindow::getProjectionPosition()
{
	camera.x = imagePlane2Tracker->GetElement(0, 3);		// get x,y,z of implane2camera*camera2tracker
	camera.y = imagePlane2Tracker->GetElement(1, 3);
	camera.z = imagePlane2Tracker->GetElement(2, 3);

}

void MainWindow::getNormalPosition()
{
	normal.x = normal2Tracker->GetElement(0, 3);
	normal.y = normal2Tracker->GetElement(1, 3);
	normal.z = normal2Tracker->GetElement(2, 3);
}

void MainWindow::getOriginPosition()
{
	origin.x = origin2Tracker->GetElement(0, 3);
	origin.y = origin2Tracker->GetElement(1, 3);
	origin.z = origin2Tracker->GetElement(2, 3);
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
	qDebug() << "Stream Video Thread";
}

void MainWindow::toggleLaser()
{
	mcuControl->laserButton->setChecked(false);
	if (mcuConnected && !laserOn) {
		comPort->write("G21");
		laserOn = true;
		mcuControl->laserButton->setText(tr("Laser Off"));
		mcuControl->laserButton->setChecked(true);
	}
	else if (mcuConnected && laserOn) {
		comPort->write("G32");
		laserOn = false;
		mcuControl->laserButton->setText(tr("Laser On"));
	}
	else {
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
	if (!mcuConnected) {
		bool okay;

		int portnumber = QInputDialog::getInt(this, tr("Connect MCU"), tr("Enter COM Port #:"), 0, 0, 100, 1, &okay);
		portname = "COM" + to_string(portnumber);
		if (okay && portnumber > 0)
			comPort = new Serial(portname);	//call Serial constructor in SerialPort.cpp

		if (comPort->isConnected()) {
			mcuControl->mcuButton->setText(tr("Disconnect MCU"));
			mcuControl->mcuButton->setChecked(true);
			statusBar()->showMessage(tr("MCU Connected"), 2000);
			mcuConnected = true;
		}
		else
			statusBar()->showMessage(tr("Unable to connect MCU at COM ") + QString::number(portnumber), 5000);
	}
	else {
		delete comPort;
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

	if (filename.endsWith(".pcd", Qt::CaseInsensitive))
		EndoModel::viewPointCloud(filename.toStdString(), 1, camera);

	else if (filename.endsWith(".ply", Qt::CaseInsensitive))
		EndoModel::viewPointCloud(filename.toStdString(), 2, camera);

	else
		EndoModel::viewPointCloud(filename.toStdString(), 3, camera);
}

/*

void MainWindow::viewCloudClicked()
{
	QVTKWidget widget;
	widget.resize(512, 256);

	trackerControl->viewCloud->setChecked(false);
	if (isScanning)
		return;

	QString filename = QFileDialog::getOpenFileName(this, tr("Open File"),
		"./Results", tr("3D Scan Files (*.pcd *.ply *.OBJ)"));

	if (filename.isEmpty())
		return;

	if (filename.endsWith(".pcd", Qt::CaseInsensitive) || filename.endsWith(".ply", Qt::CaseInsensitive))
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		if (filename.endsWith(".pcd", Qt::CaseInsensitive))
			pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud);
		else
			pcl::io::loadPLYFile<pcl::PointXYZ>(filename, cloud);

		pcl::visualization::PCLVisualizer pviz("test_viz");

		pviz.addPointCloud<pcl::PointXYZ>(cloud);
		pviz.setBackgroundColor(0, 0, 0.1);

		vtkSmartPointer<vtkRenderWindow> renderWindow = pviz.getRenderWindow();
		widget.SetRenderWindow(renderWindow);
	}
}
*/
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

	//get camera centre 
	getProjectionPosition();	//updates camera coords

								//laser plane geometry								
	getNormalPosition();		//updates normal
	getOriginPosition();		//updates origin

	cv::Mat laserLineImg = subtractLaser(laserOff, laserOn);
	linalg::EndoPt pixel, calcPixel;

	for (int row = VERTICAL_OFFSET; row < laserLineImg.rows - VERTICAL_OFFSET; row += res) {
		for (int col = HORIZONTAL_OFFSET; col < laserLineImg.cols - HORIZONTAL_OFFSET; col += res) {
			if (laserLineImg.at<uchar>(row, col) == 255) {

				pixel = getPixelPosition(row, col);														//returns pixel 2 world coords
				linalg::EndoLine camLine = linalg::lineFromPoints(camera, pixel);						//in world coordinates
				linalg::EndoPt intersection = linalg::solveIntersection(normal, origin, camLine);		//in world coordinates

				if (intersection.x == 0.0) {
					qDebug("No intersection found");
					break;
				}
				else {
					calcPixel = validatePixel(intersection);
					Model->addPointToPointCloud(intersection);
					if (saveDataBool)
						saveData(camera, pixel, normal, origin, camLine, col, row, calcPixel, intersection);
				}
			}
		}
	}
}

linalg::EndoPt MainWindow::getPixelPosition(int row, int col)		//returns pixel location in tracker space
{
	linalg::EndoPt result;
	double plane2World[4];
	double plane2FeaturePoint[4] = { col - 3.18246521e+002, row - 2.36307892e+002, 6.21962708e+002, 1 }; //homogenous transform from config file
	imagePlane2Tracker->MultiplyPoint(plane2FeaturePoint, plane2World);
	//Now have featurePoint 2 world: divide by 4th element in plane2World

	result.x = plane2World[0] / plane2World[3];
	result.y = plane2World[1] / plane2World[3];
	result.z = plane2World[2] / plane2World[3];
	return result;
}

linalg::EndoPt MainWindow::validatePixel(linalg::EndoPt point)
{
	double plane2FeaturePt[4], normPlane2Feat[3], world2Feat[3];
	linalg::EndoPt result;
	double pointVector[4] = { point.x, point.y, point.z, 1 };		//unclear if last value = 1 or 0
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


void MainWindow::saveData(linalg::EndoPt cameraPt, linalg::EndoPt pixelPt, linalg::EndoPt normalPt, linalg::EndoPt originPt, linalg::EndoLine cameraline,
	int col, int row, linalg::EndoPt calcPixel, linalg::EndoPt inter)
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

	//Calculated U and V:
	ResultsFile << calcPixel.x << ",";
	ResultsFile << calcPixel.y << ",";

	//intersection:
	ResultsFile << inter.x << ",";
	ResultsFile << inter.y << ",";
	ResultsFile << inter.z << endl;
}

void MainWindow::contrastChanged(int sliderPos)
{
	contrast = sliderPos;
}

void MainWindow::brightnessChanged(int sliderPos)
{
	brightness = sliderPos;
}