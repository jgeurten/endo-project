// Main app file
#include "MainWindow.h"
#include "Serial.h"

#include <QApplication>
#include <qdialog.h>
#include <qmainwindow.h>

int main(int argc, char* argv[])
{
	QApplication app(argc, argv);
	app.setOrganizationName("Robarts Research Institute, Canada");
	app.setApplicationName("EndoScanner");

	MainWindow MainWin;
	MainWin.show();

	return app.exec();
}
