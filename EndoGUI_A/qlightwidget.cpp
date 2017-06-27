#include "qlightwidget.h"
#include <QColor>
#include <QPainter>

QLightWidget::QLightWidget(QWidget *parent)
	: QWidget(parent)
{
	currColor = Qt::blue;				//default = "not tracked"
	setFixedSize(20, 20);
}

void QLightWidget::setGreen()
{
	currColor = Qt::green;
	update();
}

void QLightWidget::setRed()
{
	currColor = Qt::red;
	update();
}

void QLightWidget::setBlue()
{
	currColor = Qt::blue;
	update();
}

void QLightWidget::paintEvent(QPaintEvent *e)
{
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);

	painter.setPen(Qt::NoPen);
	painter.setBrush(Qt::darkGray);
	painter.drawEllipse(0, 0, width(), height());

	currColor.setAlpha(255);

	painter.setPen(Qt::NoPen);
	painter.setBrush(QBrush(currColor, Qt::Dense4Pattern));
	painter.drawEllipse(1, 1, width() - 1, height() - 1);
}