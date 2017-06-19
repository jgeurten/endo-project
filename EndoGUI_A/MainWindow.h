#pragma once
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

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

//Local includes
#include "Serial.h"
#include "Vision.h"
#include "qlightwidget.h"
#include "ControlWidget.h"

//opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//using namespace cv;
using namespace std;

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
	int framePd;	// period of frame rate
	bool isReadyToSave, isSaving, playing, mcuConnected, laserOn, trackerInit, scanningStatus;
	
	vector<cv::Vec4i> lines;
	cv::Point point1, point2;

	string portname;
	string configFile, intrinsicsFile, resultsDir, calibDir; 

	Serial *comPort; 
	cv::Mat streamImg, laserOnImg, laserOffImg;
	cv::Mat frame;
	cv::Mat savingMat;
	string fileName;
	cv::VideoCapture capture;
	cv::VideoWriter gVideoWrite;
	int frameWidth, frameHeight;
	
	int scancount = 0; 
	int togglecount = 0; 
	int brightness = 6;
	int contrast = 18;
	
	void createMenus(); 
	void createControlDock();
	void createStatusBar();
	bool createVTKObject();

	QMenu		*fileMenu;
	QMenu		*helpMenu;
	QAction		*openAct;
	QAction		*saveAct;
	QAction		*exitAct;
	QAction		*aboutAct;
	QAction		*helpAct;

	QWidget		*videoWidget;
	QPushButton *pushButton;
	QPushButton *pushButton_2;
	QPushButton *pushButton_3;
	QPushButton	*pushButton_4;

	QMediaPlayer *player;
	QTimer		*timer;
	QTimer		*scanTimer; 
	QDockWidget *controlDock; 
	ControlWidget *controlsWidget;
	QImage		image;
	QPixmap		pixLabel;
	QLabel		*videoLabel;
	QSize		*size;
	QThread		*streamThread; 

	public slots:

	void toggleLaser();
	void toggleScan();
	void toggleCamera();
	void calibrateCamera();
		
	void camera_button_clicked();
	void showImage(const cv::Mat& image);
	void load_button_clicked();
	void saveButtonPressed();
	void connectMCU();
	void startTracker();
	void scanButtonPress();
	void scan();

	void update_image();
	void saveVideo();
	void run();
	void open();
	void stop();
	void exit();
	void save();
	void help();
	void about();

	QImage mat_to_qimage(cv::Mat frame, QImage::Format format);

protected:
	
	void paintEvent(QPaintEvent* event);

	QImage _qimage;
	cv::Mat _tmp;
	cv::Mat* imagerd;
};
#endif // MAINWINDOW_H
