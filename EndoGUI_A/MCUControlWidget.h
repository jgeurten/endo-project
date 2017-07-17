#pragma once
#ifndef MCUCONTROLWIDGET_H
#define MCUCONTROLWIDGET_H
				
//QT
#include <QFrame>
#include <QLabel>
#include <QPushButton>

// C++ Includes
#include <vector>

#include <qcheckbox.h>
#include "MainWindow.h"

class QLabel;
class QPushButton;

class MCUControlWidget : public QFrame {

	Q_OBJECT

public:
	MCUControlWidget(QWidget *parent = Q_NULLPTR);
	~MCUControlWidget() {};

public:
	QPushButton								*mcuButton;
	QPushButton								*laserButton;
};
#endif // !MCUCONTROLWIDGET_H
#pragma once
