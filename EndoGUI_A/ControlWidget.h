#pragma once
#ifndef CONTROLWIDGET_H
#define CONTROLWIDGET_H

//QT
#include <QFrame>

// C++ Includes
#include <vector>

// Local Includes
#include "qlightwidget.h"
#include <qcheckbox.h>

class QLabel; 
class QPushButton;

class ControlWidget : public QFrame {

	Q_OBJECT

public:
	ControlWidget(QWidget *parent = Q_NULLPTR);
	~ControlWidget() {};

public:
	std::vector< QLightWidget *>			lightWidgets;
	std::vector< QLabel *>					labelWidgets;
	QPushButton								*streamButton;
	QPushButton								*saveButton;
	QPushButton								*mcuButton;
	QPushButton								*laserButton;
	QPushButton								*trackerButton;

};
#endif // !CONTROLWIDGET_H
