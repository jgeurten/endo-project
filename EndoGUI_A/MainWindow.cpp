//Local includes
#include "MainWindow.h"
#include "Serial.h"
#include "ControlWidget.h"
#include "qlightwidget.h"
#include "defines.h"
#include <EndoModel.h>
#include <LinAlg.h>


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
	controlDock = new QDockWidget(tr("Control Dock"), this);
	controlDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
	controlDock->setFeatures(QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable);
	addDockWidget(Qt::LeftDockWidgetArea, controlDock);
	controlDock->setMinimumWidth(180);
	controlDock->setMaximumWidth(300);// (round(size->width()*0.33));

	QFrame* mainFrame = new QFrame;
	mainFrame->setFrameStyle(QFrame::WinPanel | QFrame::Sunken);
	mainFrame->setLineWidth(2);

	QGridLayout* controlsLayout = new QGridLayout;
	controlsLayout->setMargin(0);
	controlsLayout->setSpacing(10);
	controlsLayout->setAlignment(Qt::AlignTop);
	mainFrame->setLayout(controlsLayout);

	controlDock->setWidget(mainFrame);

	controlWidget = new ControlWidget(this);
	controlsLayout->addWidget(controlWidget);

	connect(controlWidget->streamButton, SIGNAL(clicked()), this, SLOT(camera_button_clicked()));
	connect(controlWidget->saveButton, SIGNAL(clicked()), this, SLOT(saveButtonPressed()));
	connect(controlWidget->mcuButton, SIGNAL(clicked()), this, SLOT(connectMCU()));
	connect(controlWidget->laserButton, SIGNAL(clicked()), this, SLOT(toggleLaser()));
	connect(controlWidget->trackerButton, SIGNAL(clicked()), this, SLOT(startTracker()));
	connect(controlWidget->scanButton, SIGNAL(clicked()), this, SLOT(scanButtonPress()));

	//create trackTimer to refresh the image every x milliseconds depending on the framerate of the camera
	trackTimer = new QTimer(this);
	connect(trackTimer, SIGNAL(timeout()), this, SLOT(updateTracker()));
}

void MainWindow::startTracker()
{
	controlWidget->trackerButton->setChecked(false);
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
		controlWidget->trackerButton->setText(tr("Stop Tracking"));
		controlWidget->trackerButton->setChecked(true);
	}

	else  //already initalized - unitialize...
	{
		trackTimer->stop();
		trackerInit = false;
		controlWidget->trackerButton->setText(tr("Start Tracking"));
		controlWidget->trackerButton->setChecked(false);
		statusBar()->showMessage(tr("Stopping Tracking"), 2000);
		controlWidget->lightWidgets[0]->setBlue();
		controlWidget->lightWidgets[1]->setBlue();
	}

}

void MainWindow::createVTKObject()
{
	//not sure this is used :

	//intrinsicsFile = "./config/calibration.xml";

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
	//6.21962708e+002, 0, 3.18246521e+002, 0, 6.19908875e+002, 2.36307892e+002, 0, 0, 1);
	//Set point 2 image plane transform explicitly:
	
	point2ImagePlane->SetElement(0, 0, 1);
	point2ImagePlane->SetElement(0, 1, 0);
	point2ImagePlane->SetElement(0, 2, 0);
	point2ImagePlane->SetElement(0, 3, -3.18246521e+002);		//-cx
	point2ImagePlane->SetElement(1, 0, 0);
	point2ImagePlane->SetElement(1, 1, 1);
	point2ImagePlane->SetElement(1, 2, 0);
	point2ImagePlane->SetElement(1, 3, -2.36307892e+002);		//-cy
	point2ImagePlane->SetElement(2, 0, 0);
	point2ImagePlane->SetElement(2, 1, 0);
	point2ImagePlane->SetElement(2, 2, 6.21962708e+002);			//fx
	point2ImagePlane->SetElement(2, 3, 0);
	point2ImagePlane->SetElement(3, 0, 0);
	point2ImagePlane->SetElement(3, 1, 0);
	point2ImagePlane->SetElement(3, 2, 0);
	point2ImagePlane->SetElement(3, 3, 1);

	vtkMatrix4x4::Invert(point2ImagePlane, imagePlane2Point);	//camera plane to pixel
	//Handle vtk transform point 2 tracker:
	point2Tracker->PostMultiply();
	
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
			controlWidget->streamButton->setText(tr("Stop Stream"));

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
		controlWidget->streamButton->setText(tr("Stream Video"));
		statusBar()->showMessage(tr("Ready"), 7000);
		playing = false;
		trackTimer->stop();
	}
}


void MainWindow::camera_button_clicked()
{
	if (!playing)
	{
		if (!trackerInit)
			statusBar()->showMessage(tr("Connect Tracker First"), 5000);
		else
		{
			//Frame:
			mixer->GetChannel()->GetTrackedFrame(mixerFrame);
			//Images:
			vtkImageData *image = mixerFrame.GetImageData()->GetImage();
			int dimensions[3];
			image->GetDimensions(dimensions);

			distStreamImg = cv::Mat(dimensions[1], dimensions[0], CV_8UC3, image->GetScalarPointer(0, 0, 0));
			cv::undistort(distStreamImg, streamImg, intrinsics, distortion);
			cv::initUndistortRectifyMap(intrinsics, distortion, cv::Mat(), intrinsics, cv::Size(distStreamImg.cols, distStreamImg.rows),
				CV_32FC1, map1, map2);
		}
	}
	else
	{
		controlWidget->streamButton->setText(tr("Stream Video"));
		statusBar()->showMessage(tr("Ready"), 7000);
		playing = false;
		trackTimer->stop();
	}
}

void MainWindow::update_image()
{


	if (capture.isOpened())
	{
		cv::namedWindow("Control", CV_WINDOW_NORMAL);
		cvCreateTrackbar("Brightness", "Control", &brightness, 100);
		cvCreateTrackbar("Contrast", "Control", &contrast, 100);
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
		controlWidget->scanButton->setText(tr("Stop Scan"));
		controlWidget->scanButton->setChecked(true);
		scanTimer->start(30);
		isScanning = true;

		//model = new EndoModel();

		/*ofstream myfile("./Data/Scan.csv");
		myfile << "Cam X," << "Cam Y," << "Cam Z,"
		<< "Tool X," << "Tool Y," << "Tool Z,"
		<< "Laser X," << "Laser Y," << "Laser Z" << endl;
		*/

	}
	else {
		controlWidget->scanButton->setText(tr("Start Scan"));
		controlWidget->scanButton->setChecked(false);
		scanTimer->stop();
		savePointCloud();
		isScanning = false;
	}
}

void MainWindow::savePointCloud()
{

	QString filename = QFileDialog::getSaveFileName(this, "Save File", "", "PCD (*.pcd) ;; PLY (*.ply)");

	if (filename.isEmpty()) return;
	qDebug() << filename;
	if (filename.endsWith(".pcd", Qt::CaseInsensitive)) {
		qDebug() << "Save as pcd file.";
		myfile.close();
		//model->savePointCloudAsPCD(filename.toStdString());
	}
	else if (filename.endsWith(".ply", Qt::CaseInsensitive)) {
		qDebug() << "Save as ply file.";
		//model->savePointCloudAsPLY(filename.toStdString());
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
		cv::imshow("Laser On", laserOnImg);
		cv::imshow("Laser Off", laserOffImg);
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
			controlWidget->lightWidgets[1]->setGreen();
		else
			controlWidget->lightWidgets[1]->setRed();
		
		if (repository->GetTransform(camera2TrackerName, camera2Tracker, &isCameraMatrixValid) == PLUS_SUCCESS && isCameraMatrixValid) 
			controlWidget->lightWidgets[0]->setGreen();
			
		else 
			controlWidget->lightWidgets[0]->setRed();
			

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

		if (repository->GetTransform(camera2ImageName, camera2Image, &isValid) != PLUS_SUCCESS || !isValid)
		{
			LOG_ERROR("Unable to successfully transform camera to image");
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

void MainWindow::getProjectionPosition()		//get projection centre wrt tracker
{
	camera.x = imagePlane2Tracker->GetElement(0, 3);
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
				controlWidget->saveButton->setText("End Saving Video");
				isReadyToSave = false;
				isSaving = true;
			}
		}
		else
			statusBar()->showMessage(tr("Unable to save video"));
	}
	else
	{
		controlWidget->saveButton->setText("Save Video");
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
	if (mcuConnected && !laserOn) {
		comPort->write("G21");
		laserOn = true;
		controlWidget->laserButton->setText(tr("Laser Off"));
	}
	else if (mcuConnected && laserOn) {
		comPort->write("G32");
		laserOn = false;
		controlWidget->laserButton->setText(tr("Laser On"));
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

	controlWidget->mcuButton->setChecked(false);
	if (!mcuConnected) {
		bool okay;
		int portnumber = -1;
		portnumber = QInputDialog::getInt(this, tr("Connect MCU"), tr("Enter COM Port #:"), 0, 0, 100, 1, &okay);
		portname = "COM" + to_string(portnumber);
		if (okay && portnumber > 0)
			comPort = new Serial(portname);	//call Serial constructor in Serial.cpp

		if (comPort->isConnected()) {
			controlWidget->mcuButton->setText(tr("Disconnect MCU"));
			controlWidget->mcuButton->setChecked(true);
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
		controlWidget->mcuButton->setText(tr("Connect MCU"));
	}
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
	cv::imshow("subimbw", subImg);

	cv::threshold(subImgBW, threshImg, 10, 255, CV_THRESH_TOZERO);
	cv::imshow("thres", threshImg);
	cv::blur(threshImg, threshImg, cv::Size(5, 5));
	cv::imshow("BLUR", threshImg);

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
	cv::imshow("lineImg", lineImg);
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
	//EndoModel* model = new EndoModel(); 
	//check if able to transform all entities into tracker space
	if (!trackReady)
	{
		LOG_ERROR("Unable to transform one or many translations");
		return;
	}

	//get camera centre 
	getProjectionPosition();	
	//laser plane geometry								
	getNormalPosition();
	getOriginPosition();

	cv::Mat laserLineImg = subtractLaser(laserOff, laserOn);
	linalg::EndoPt pixel, newPixel, newestPixel;
	float render[4]; 
	float calcPixel[4];

	for (int row = VERTICAL_OFFSET; row < laserLineImg.rows - VERTICAL_OFFSET; row+= res) {
		for (int col = HORIZONTAL_OFFSET; col < laserLineImg.cols - HORIZONTAL_OFFSET; col+= res) {
			if (laserLineImg.at<uchar>(row, col) == 255) {

				pixel = getPixelPosition(row, col);
				newPixel = getNewPixelPosition(row, col);

				if (col > 318)	//cx
					newestPixel.x = camera.x + col;
				else
					newestPixel.x = camera.x - col; 

				if (row > 236)	//cy
					newestPixel.y = camera.y + row;
				else
					newestPixel.y = camera.y - row; 

				newestPixel.z = camera.z + 621; //focus

				linalg::EndoLine camLine = linalg::lineFromPoints(camera, pixel);				//in world coordinates
				linalg::EndoPt intersection = linalg::solveIntersection(normal, origin, camLine);		//in world coordinates

					//validated intersection math using MATLAB and http://www.ambrsoft.com/TrigoCalc/Plan3D/PlaneLineIntersection_.htm
					//July 2017

				if (intersection.x == 0.0) {
					qDebug("No intersection found");
					break;
				}
				else {
					//render 3D point intersection -> image plane space
					render[0] = intersection.x; 
					render[1] = intersection.y; 
					render[2] = intersection.z; 
					render[3] = 1; 
					vtkMatrix4x4::Multiply4x4(tracker2ImagePlane, imagePlane2Point, tracker2Point);
					tracker2Point->MultiplyPoint(render, calcPixel);
					saveData(camLine, col, row, normal, origin, calcPixel, intersection);
				}
			}
		}
	}
}

linalg::EndoPt MainWindow::getPixelPosition(int row, int col)		//returns pixel location in tracker space
{
	const float pix[4] = { col, row, 1, 1 };
	float result[4];
	linalg::EndoPt pt;
	//point2Tracker->SetMatrix(point2ImagePlane);
	//point2Tracker->Concatenate(imagePlane2Tracker);
	//point2Tracker->MultiplyPoint(pix, result);	//put point pix into projection space

	vtkMatrix4x4::Multiply4x4(point2ImagePlane,imagePlane2Tracker,   pixel2Tracker);
	pixel2Tracker->MultiplyPoint(pix, result);
	pt.x = result[0];
	pt.y = result[1];
	pt.z = result[2];
	return pt;
}

linalg::EndoPt MainWindow::getNewPixelPosition(int row, int col)		//returns pixel location in tracker space
{
	const float pix[4] = { col, row, 1, 1 };
	float result[4];
	linalg::EndoPt pt;
	bool isValid(false);

	if (repository->GetTransform(tracker2PixelName, tracker2Pixel, &isValid) != PLUS_SUCCESS || !isValid)
	{
		LOG_ERROR("Unable to successfully transform tracker 2 image plane");
		return linalg::MakePoint(0,0,0);
	}
	
	tracker2Pixel->MultiplyPoint(pix, result);

	pt.x = result[0];
	pt.y = result[1];
	pt.z = result[2];
	return pt;
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


void MainWindow::saveData(linalg::EndoLine line, int col, int row, linalg::EndoPt normal, linalg::EndoPt origin, float calc[4], linalg::EndoPt inter)
{
	scanNumber++;
	string name = "./Results/scan" + to_string(scanNumber);
	string fullname = name + ".csv";
	ofstream myfile(fullname);

	myfile << "Cam X," << "Cam Y," << "Cam Z,"
		<< "Plane Normal X," << "Plane Normal Y," << "Plane Normal Z,"
		<< "Laser Origin X," << "Laser Origin Y," << "Laser Origin Z,"
		<< "U," << "V," << "Calc U," << "Calc V," << "Calc W," << "Calc Z,"
		<< "Intersection X," << "Y," << "Z" << endl;

		//CAM:
		myfile << line.a.x << ",";
		myfile << line.a.y << ",";
		myfile << line.a.z << ",";

		//Normal:
		myfile << normal.x << ",";
		myfile << normal.y << ",";
		myfile << normal.z << ",";

		//Origin:
		myfile << origin.x << ",";
		myfile << origin.y << ",";
		myfile << origin.z << ",";

		//Original U and V:
		myfile << col<< ",";
		myfile << row << ",";
		
		//Calculated U and V:
		myfile << calc[0] << ",";
		myfile << calc[1]<< ",";

		myfile << calc[2] << ",";
		myfile << calc[3] << ",";

		//intersection:
		myfile << inter.x << ",";
		myfile << inter.y << ",";
		myfile << inter.z << ",";
}

