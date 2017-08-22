CMAKE_MINIMUM_REQUIRED(VERSION 3.7.2)

PROJECT(EndoScanner)


FIND_PACKAGE(PCL REQUIRED NO_MODULE)
#COMPONENTS common io features recognition)

FIND_PACKAGE(OpenCV REQUIRED NO_MODULE)
FIND_PACKAGE(PlusLib REQUIRED NO_MODULE)

IF(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
ENDIF()

SET(${PROJECT_NAME}_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PCL_INCLUDE_DIRS}
  )


 # message("${PCL_IO_LIBRARIES}")

FIND_PACKAGE(Qt5 REQUIRED COMPONENTS Widgets OpenGL Core Gui Multimedia MultimediaWidgets SerialPort)
SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)

SET(${PROJECT_NAME}_HDRS
  MainWindow.h
  qlightwidget.h
  Controlwidget.h
  EndoModel.h
  defines.h
  LinAlg.h
  ScanClass.h
  MCUControlWidget.h
  WebcamControlWidget.h
  matrixUtil.h
  matrix.h
  imageUtil.h
  #SerialComPort.h
  )

SET(${PROJECT_NAME}_SRCS
  main.cpp
  MainWindow.cpp
  qlightwidget.cpp
  Controlwidget.cpp
  EndoModel.cpp
  ScanClass.cpp
  MCUControlWidget.cpp
  WebcamControlWidget.cpp
  #SerialComPort.cpp
  )

ADD_EXECUTABLE(EndoScanner ${${PROJECT_NAME}_HDRS} ${${PROJECT_NAME}_SRCS})

target_include_directories(EndoScanner PUBLIC 
  ${${PROJECT_NAME}_INCLUDE_DIRS}

  )

TARGET_LINK_LIBRARIES(EndoScanner PUBLIC
  Qt5::Widgets
  Qt5::OpenGL
  Qt5::Core
  Qt5::Gui
  Qt5::MultimediaWidgets
  Qt5::Multimedia
  Qt5::SerialPort

  ${OpenCV_LIBS}

  vtkPlusCommon
  vtkPlusDataCollection
  vtkPlusHaptics
  vtkPlusImageProcessing
  vtkPlusOpenIGTLink
  vtkPlusRendering
  #vtkPlusServer

  ${PCL_LIBRARIES}

  
)