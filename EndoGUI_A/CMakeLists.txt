PROJECT(EndoScanner)

CMAKE_MINIMUM_REQUIRED(VERSION 3.7.2)

IF(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
ENDIF()

SET(${PROJECT_NAME}_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  )

FIND_PACKAGE(Qt5 REQUIRED COMPONENTS Widgets OpenGL Core Gui Multimedia MultimediaWidgets)
SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)

FIND_PACKAGE(OpenCV REQUIRED NO_MODULE)
FIND_PACKAGE(PlusLib REQUIRED NO_MODULE)
FIND_PACKAGE(PCL REQUIRED NO_MODULE)
FIND_PACKAGE(vtk REQUIRED)
INCLUDE(${VTK_USE_FILE})

file(GLOB EndoScannerArduinoFirmware_headers RELATIVE "EndoScannerArduinoFirmware" "*.h")
file(GLOB EndoScannerArduinoFirmware_Sources RELATIVE "EndoScannerArduinoFirmware" "*.cpp")

ADD_EXECUTABLE(EndoScanner
  #Source files
  main.cpp
  MainWindow.cpp
  Serial.cpp
  Vision.cpp
  qlightwidget.cpp
  Controlwidget.cpp
  EndoModel.cpp

  ${EndoScannerArduinoFirmware_Sources}

  #Header files
  MainWindow.h
  Serial.h
  Vision.h
  qlightwidget.h
  Controlwidget.h
  EndoModel.h
  defines.h
  LinAlg.h
  ${EndoScannerArduinoFirmware_headers}
  )

target_include_directories(EndoScanner PUBLIC 
  ${PCL_INCLUDE_DIRS}
  ${${PROJECT_NAME}_INCLUDE_DIRS}
  )
TARGET_LINK_LIBRARIES(EndoScanner PUBLIC
  Qt5::Widgets
  Qt5::OpenGL
  Qt5::Core
  Qt5::Gui
  Qt5::MultimediaWidgets
  Qt5::Multimedia
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  vtksys
  vtkglew
  vtkRendering
  vtkPlusCalibration
  vtkPlusCommon
  vtkPlusDataCollection
  vtkPlusHaptics
  vtkPlusImageProcessing
  vtkPlusOpenIGTLink
  vtkPlusRendering
  vtkPlusServer
)
target_compile_definitions(EndoScanner PUBLIC ${PCL_DEFINITIONS})