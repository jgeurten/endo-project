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

//Local includes
#include "Serial.h"

//opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//using namespace cv;
using namespace std;

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
	bool isReadyToSave;
	bool isSaving;
	bool playing;
	bool mcuConnected;
	bool laserOn;

	string portname; 
	Serial *comPort; 
	cv::Mat frame;
	cv::Mat savingMat;
	string fileName;
	cv::VideoCapture capture;
	cv::VideoWriter gVideoWrite;
	int frameWidth;
	int frameHeight;
	
	void createMenus(); 
	void createVideoWidget();
	void createStatusBar();

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
	QTimer		*savingTimer;
	
	
	QImage		image;
	QPixmap		pixLabel;
	QLabel		*videoLabel;
	

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
