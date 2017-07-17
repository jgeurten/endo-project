// Main app file
#include "MainWindow.h"
#include "Serial.h"

#include <QApplication>
#include <qdialog.h>
#include <qmainwindow.h>
#include "qstylefactory.h"

int main(int argc, char* argv[])
{
	QApplication app(argc, argv);
	QApplication::setStyle(QStyleFactory::create("fusion"));
	app.setOrganizationName("Robarts Research Institute, Canada");
	app.setApplicationName("EndoScanner");

	MainWindow MainWin;
	MainWin.show();

	return app.exec();
}
