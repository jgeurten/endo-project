PROJECT( EndoScanner )

CMAKE_MINIMUM_REQUIRED( VERSION 3.0 )

IF( COMMAND CMAKE_POLICY )
	CMAKE_POLICY( SET CMP0003 NEW )
ENDIF( COMMAND CMAKE_POLICY)

set( CMAKE_INCLUDE_CURRENT_DIR ON )

FIND_PACKAGE(Qt5 REQUIRED COMPONENTS Widgets OpenGL Core Gui Multimedia MultimediaWidgets)
SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)



FIND_PACKAGE(OpenCV REQUIRED NO_MODULE)

file(GLOB EndoScannerArduinoFirmware_headers RELATIVE "EndoScannerArduinoFirmware" "*.h")
file(GLOB EndoScannerArduinoFirmware_Sources RELATIVE "EndoScannerArduinoFirmware" "*.cpp")

ADD_EXECUTABLE(EndoScanner
	
	#Source files
	main.cpp
	MainWindow.cpp
	Serial.cpp
	${EndoScannerArduinoFirmware_sources}

	#Header files
	MainWindow.h
	Serial.h
	${EndoScannerArduinoFirmware_headers}
	)

TARGET_LINK_LIBRARIES (EndoScanner
	Qt5::Widgets
	Qt5::OpenGL
	Qt5::Core
	Qt5::Gui
	Qt5::MultimediaWidgets
	Qt5::Multimedia
	${OpenCV_LIBS}
)