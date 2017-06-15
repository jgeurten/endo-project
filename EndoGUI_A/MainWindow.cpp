//Local includes
#include "MainWindow.h"
#include "Serial.h"
#include "Vision.h"
#include "ControlWidget.h"
#include "qlightwidget.h"
#include "C:\Users\jgeurten\Documents\endo-project\endo-project\EndoScannerArduinoFirmware\Laser.h"
#include "C:\Users\jgeurten\Documents\endo-project\endo-project\EndoScannerArduinoFirmware\configuration.h"
#include "C:\Users\jgeurten\Documents\endo-project\endo-project\EndoScannerArduinoFirmware\GCodeInterpreter.h"

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

//MSDN includes
#include <Windows.h>
#include <WinBase.h>
#include <synchapi.h>
#include <string>

using namespace std;
//using namespace cv;
//constructor

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
	//cv::VideoCapture capture = new cv::VideoCapture();

	createMenus();
	createControlDock();	//create control dock in videowidget
	createStatusBar();
	createVTKObject();
	resize(QDesktopWidget().availableGeometry(this).size()*0.6);
	setWindowTitle(tr("Endo Scanner"));
	size = this->size; 
}

//destructor

MainWindow::~MainWindow()
{
	if (capture.isOpened())
		capture.release();
}

void MainWindow::createMenus()
{
	saveAct = new QAction(tr("&Save"), this);
	saveAct->setShortcuts(QKeySequence::Save);
	saveAct->setStatusTip(tr("Save video file"));
	connect(saveAct, SIGNAL(triggered()), this, SLOT(save()));

	openAct = new QAction(tr("&Open"), this);
	openAct->setShortcuts(QKeySequence::Open);
	openAct->setStatusTip(tr("Open a saved video file"));
	connect(openAct, SIGNAL(triggered()), this, SLOT(open()));

	exitAct = new QAction(tr("&Exit"), this);
	exitAct->setShortcuts(QKeySequence::Quit);
	exitAct->setStatusTip(tr("Exit application"));
	connect(exitAct, SIGNAL(triggered()), this, SLOT(exit()));

	// File Menu
	fileMenu = menuBar()->addMenu(tr("&File"));
	fileMenu->addMenu(tr("&File"));
	fileMenu->addAction(openAct);
	fileMenu->addAction(saveAct);
	fileMenu->addSeparator();
	fileMenu->addAction(exitAct);

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
	
	controlsWidget = new ControlWidget(this);
	controlsLayout->addWidget(controlsWidget);

	connect(controlsWidget->streamButton, SIGNAL(clicked()), this, SLOT(camera_button_clicked()));
	connect(controlsWidget->saveButton, SIGNAL(clicked()), this,   SLOT(saveButtonPressed()));
	connect(controlsWidget->mcuButton, SIGNAL(clicked()), this,    SLOT(connectMCU()));
	connect(controlsWidget->laserButton, SIGNAL(clicked()), this,  SLOT(toggleLaser()));
	connect(controlsWidget->trackerButton, SIGNAL(clicked()), this,SLOT(startTracker()));

	//create timer to refresh the image every x milliseconds depending on the framerate of the camera
	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(update_image()));
}

void MainWindow::startTracker()
{
	if (!trackerInit)
	{

	}
	else  //already initalized - unitialize...
	{
		trackerTimer->stop();
		controlsWidget->

	}
}

bool MainWindow::createVTKObject()
{
	configFile = "./config/configEndoscope.xml";
	intrinsicsFile = "./config/calibration.xml";

	
}

void MainWindow::camera_button_clicked()
{
	if (!playing) {
		capture = cv::VideoCapture(0);
		capture.open(0);
		if (capture.isOpened()) {
			capture.set(CV_CAP_PROP_FPS, 30);
			capture.set(CV_CAP_PROP_AUTOFOCUS, 0);
			frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);
			frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
			framePd = 1000 / (int)capture.get(CV_CAP_PROP_FPS);
			playing = true;
			pushButton->setText(tr("Stop Video"));
			timer->start(framePd);	//need to fix framePd and override 34

		}
		else
			statusBar()->showMessage(tr("Unable to Detect Camera"), 3000);
	}

	else {
		pushButton->setText(tr("Play Video"));
		statusBar()->showMessage(tr("Ready"), 7000);
		playing = false;
		timer->stop();
	}
}



void MainWindow::update_image()
{

  	if (capture.isOpened())
	{
		
		cv::namedWindow("Control", CV_WINDOW_NORMAL);
		cvCreateTrackbar("Brightness", "Control", &brightness, 40);
		cvCreateTrackbar("Contrast", "Control", &contrast, 40); 
		capture.set(CV_CAP_PROP_CONTRAST, (double)contrast);
		capture.set(CV_CAP_PROP_BRIGHTNESS, (double)brightness);
		toggleLaser(); //ON
		Sleep(200);
		capture >> laserOnImg;
		
		toggleLaser();	//TURN OFF
		Sleep(200);
		capture >> laserOffImg;

		lines = Vision::detectLaserLine(laserOnImg, laserOffImg);
		for (int index = 0; index < lines.size(); index++)
		{
			point1.x = lines[index][0];
			point1.y = lines[index][1];
			point2.x = lines[index][2];
			point2.y = lines[index][3];

			cv::line(laserOffImg, point1, point2, cv::Scalar(0, 255, 0), 4);
		}

		cv::Size s = laserOffImg.size();
		image = QImage((const unsigned char*)(laserOffImg.data), laserOffImg.cols, laserOffImg.rows, laserOffImg.cols*laserOffImg.channels(), QImage::Format_RGB888).rgbSwapped();

		switch (laserOffImg.type())	//CONVERT MAT TO QIMAGE
		{
		case CV_8UC4:
		{
			image = QImage((const unsigned char*)(laserOffImg.data), laserOffImg.cols, laserOffImg.rows, laserOffImg.cols*laserOffImg.channels(), QImage::Format_ARGB32);
			break;
		}

		case CV_8UC3:
		{
			image = QImage((const unsigned char*)(laserOffImg.data), laserOffImg.cols, laserOffImg.rows, laserOffImg.cols*laserOffImg.channels(), QImage::Format_RGB888).rgbSwapped();
			break;
		}

		default:
			qWarning() << "Type Not Handled";
			break;
		}

		repaint();

		if (isSaving)
			saveVideo();
	}

	else
		statusBar()->showMessage(tr("Unable to Detect Camera"), 5000);
}

void MainWindow::saveButtonPressed()
{
	if (isReadyToSave && capture.isOpened()) {

		int format = capture.get(CV_CAP_PROP_FORMAT);
		int mode = capture.get(CV_CAP_PROP_MODE);

		QString filename = QFileDialog::getSaveFileName(this, tr("Save File"),
			"C:/users/jgeurten/Videos/untitled",
			tr("AWI File (*.avi);; All Files(*.)"));

		if (!filename.isEmpty())
		{
			string fileName = filename.toStdString();
			int fps = (int)capture.get(CV_CAP_PROP_FPS);
			cv::Size lVideoSize = cv::Size(frameWidth, frameHeight);

			gVideoWrite = cv::VideoWriter(fileName, CV_FOURCC('D', 'I', 'V', 'X'), fps / 2, lVideoSize, true);	//video is sped up by a factor of 2

			if (gVideoWrite.isOpened())
			{
				pushButton_2->setText("End Saving Video");
				isReadyToSave = false;
				isSaving = true;
			}
		}
		else
			statusBar()->showMessage(tr("Unable to save video"));
	}
	else
	{
		pushButton_2->setText("Save Video");
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
	}
	else if (mcuConnected && laserOn){
		comPort->write("G32");
		laserOn = false;
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

	if (!mcuConnected) {
		bool okay;
		int portnumber = -1;
		portnumber = QInputDialog::getInt(this, tr("Connect MCU"), tr("Enter COM Port #:"), 0, 0, 100, 1, &okay);
		portname = "COM" + to_string(portnumber);
		if (okay && portnumber > 0)
			comPort = new Serial(portname);	//call Serial constructor in Serial.cpp

		if (comPort->isConnected()) {
			pushButton_3->setText(tr("Disconnect MCU"));
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
		pushButton_3->setText(tr("Connect MCU"));
	}
}

void MainWindow::load_button_clicked()
{
	//
}
void MainWindow::run()
{

}

void MainWindow::showImage(const cv::Mat& image)
{

}


void MainWindow::stop()
{

}

void MainWindow::toggleScan()
{

}

void MainWindow::toggleCamera()
{

}
void MainWindow::calibrateCamera()
{

}

void MainWindow::open()
{

}
void MainWindow::exit()
{

}
void MainWindow::save()
{

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