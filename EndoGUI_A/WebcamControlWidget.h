#pragma once

#ifndef WEBCAMCONTROLWIDGET_H
#define WEBCAMCONTROLWIDGET_H

//QT
#include <QFrame>
#include <QSlider>


// C++ Includes
#include <vector>

// Local Includes
#include "qlightwidget.h"
#include <qcheckbox.h>

class QLabel;
class QPushButton;
class QSlider; 

class WebcamControlWidget : public QFrame {

	Q_OBJECT

public:
	WebcamControlWidget(QWidget *parent = Q_NULLPTR);
	~WebcamControlWidget() {};

public:
	
	QPushButton		*streamButton;
	QSlider			*brightSlider; 
	QSlider			*contrastSlider;

};
#endif // !WEBCAMCONTROLWIDGET_H
