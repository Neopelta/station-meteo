cmake_minimum_required(VERSION 3.19)
project(BLEProject LANGUAGES CXX)

# On précise tous les composants nécessaires
find_package(Qt6 6.5 REQUIRED COMPONENTS
    Core
    Gui
    Widgets
    PrintSupport
    Bluetooth
    LinguistTools
    SerialPort
)

qt_standard_project_setup()

qt_add_executable(BLEProject
    WIN32 MACOSX_BUNDLE
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    qcustomplot.cpp
    qcustomplot.h
)

qt_add_translations(
    TARGETS BLEProject
    TS_FILES BLEProject_fr_FR.ts
)

target_link_libraries(BLEProject
    PRIVATE
        Qt::Core
        Qt::Gui
        Qt::Widgets
        Qt::PrintSupport
        Qt::Bluetooth
        Qt::SerialPort

)

include(GNUInstallDirs)

install(TARGETS BLEProject
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

qt_generate_deploy_app_script(
    TARGET BLEProject
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})
