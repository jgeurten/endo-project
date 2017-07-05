#pragma once
#ifndef QLIGHTWIDGET_H
#define QLIGHTWIDGET_H

#include <QWidget>

class QColor; 

class QLightWidget : public QWidget
{
	Q_OBJECT

public:
	QLightWidget(QWidget *parent = Q_NULLPTR);
	void SetColor(QColor &c) { this->currColor = c; };

protected:
	virtual void paintEvent(QPaintEvent *);
	QColor currColor; 

	public slots:
	void setGreen();
	void setRed();
	void setBlue();
};


#endif // !QLIGHTWIDGET_H
