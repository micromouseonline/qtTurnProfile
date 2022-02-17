cmake_minimum_required(VERSION 3.5)
project(qtTurnProfile)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
#How to find the Qt stuff
if (APPLE)
    set(CMAKE_PREFIX_PATH "/Users/peterharrison/Qt5x/5.15.1/clang_64")
elseif (WIN32)
    set(CMAKE_PREFIX_PATH "C:/Qt/5.12.3/mingw73_64")
elseif (UNIX)
    set(CMAKE_PREFIX_PATH "~/Qt/5.15.2/gcc_64")
endif ()
set(CMAKE_PREFIX_PATH "~/Qt/5.15.2/gcc_64")
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
# moc needs to run for, especially, QObject objects
set(CMAKE_AUTOMOC ON)
# if there are any .qrc files
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        ui_mainwindow.h
        mouseitem.cpp
        mouseitem.h
        main.cpp
        mainwindow.cpp
        mainwindow.h
        qcustomplot.cpp
        qcustomplot.h
        pose.h
        track.h
        track.cpp
        resources.qrc
        pathitem.cpp
        pathitem.h
        robotstate.h
        )

add_executable(qtTurnProfile ${SOURCE_FILES})


#qt5_use_modules(qtTurnProfile Core Widgets Gui PrintSupport)
target_link_libraries(qtTurnProfile PRIVATE
        Qt${QT_VERSION_MAJOR}::Widgets
        Qt${QT_VERSION_MAJOR}::PrintSupport
        )
# generate proper GUI program on specified platform
if (WIN32) # Check if we are on Windows
    set_target_properties(${PROJECT_NAME} PROPERTIES WIN32_EXECUTABLE TRUE)
    if (MSVC) # Check if we are using the Visual Studio compiler
        set_target_properties(${PROJECT_NAME} PROPERTIES
                WIN32_EXECUTABLE YES
                LINK_FLAGS "/ENTRY:mainCRTStartup"
                )
    elseif (CMAKE_COMPILER_IS_GNUCXX)
        # SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mwindows") # Not tested
    else ()
        message(SEND_ERROR "You are using an unsupported Windows compiler! (Not MSVC or GCC)")
    endif (MSVC)
elseif (APPLE)
    set_target_properties(${PROJECT_NAME} PROPERTIES
            MACOSX_BUNDLE YES
            )
elseif (UNIX)
    # Nothing special required
else ()
    message(SEND_ERROR "You are on an unsupported platform! (Not Win32, Mac OS X or Unix)")
endif (WIN32)

#set(DLL_PATH "C:/Qt/5.15.2/mingw81_64/bin")
#set(DLL_PATH "C:/Qt/5.12.3/mingw73_64/bin")

# Windows builds need the DLLs copying into th same directory as the executable.
# The names of these end in d (e.g. Qt5Guid.dll) for the debug build
if (WIN32)
    if (CMAKE_BUILD_TYPE MATCHES "Debug")
        add_custom_command(
                TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DLL_PATH}/libgcc_s_seh-1.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DLL_PATH}/libwinpthread-1.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DLL_PATH}/libstdc++-6.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DLL_PATH}/Qt5Widgetsd.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DLL_PATH}/Qt5Guid.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DLL_PATH}/Qt5Cored.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DLL_PATH}/Qt5PrintSupportd.dll ${CMAKE_CURRENT_BINARY_DIR}
                #                C:/Qt/5.15.2/mingw81_64/plugins/platforms/qwindows.dll ???
        )
    endif ()
    if (CMAKE_BUILD_TYPE MATCHES "Release")
        add_custom_command(
                TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_PREFIX_PATH}/bin/libgcc_s_seh-1.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_PREFIX_PATH}/bin/libwinpthread-1.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_PREFIX_PATH}/bin/libstdc++-6.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_PREFIX_PATH}/bin/Qt5Widgets.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_PREFIX_PATH}/bin/Qt5Gui.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_PREFIX_PATH}/bin/Qt5Core.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_PREFIX_PATH}/bin/Qt5PrintSupport.dll ${CMAKE_CURRENT_BINARY_DIR}
        )
    endif ()
ENDIF ()
