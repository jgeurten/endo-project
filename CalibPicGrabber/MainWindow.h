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
#include <qslider.h>
#include <qcamerafocus.h>
#include <qcamera.h>

//Local include

//opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/*#include <ks.h>
#include <ksmedia.h>	//ksproperty - focus
#include <ksproxy.h>
*/
#include <dshow.h>

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
	void createVideoWidget();
	void createTimer();
	void intitCamera();
	void createStatusBar();
	void setCameraControl();

	QWidget *videoWidget;
	QPushButton *capture_button;
	QTimer *timer;

	cv::VideoCapture capture;
	cv::Mat feedImg;
	QImage image;
	QSlider *brightSlider;
	QSlider *contrastSlider;

	int brightness = 5;
	int contrast = 5;
	int exposure = 5;
	int focus = 150;
	//QCameraFocus *focus;
	QCamera *camera;

public slots:

	void capture_button_pressed();
	void updateFeed();


protected:

	void paintEvent(QPaintEvent* event);
};


#endif // !MAINWINDOW_H
