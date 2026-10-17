cmake_minimum_required(VERSION 3.5)
project(qtTurnProfile)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
#How to find the Qt stuff
# Linux uses system Qt6. For other installs pass -DCMAKE_PREFIX_PATH=<Qt6 kit dir>.
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets PrintSupport)
message(STATUS "Using Qt ${Qt6_VERSION}")
# moc needs to run for, especially, QObject objects
set(CMAKE_AUTOMOC ON)
# if there are any .qrc files
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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


target_link_libraries(qtTurnProfile PRIVATE
        Qt6::Widgets
        Qt6::PrintSupport
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
    # cmake --install <build-dir> --prefix ~/.local  -> copies to ~/.local/bin
    install(TARGETS ${PROJECT_NAME}
            RUNTIME DESTINATION bin
    )
    # 'deploy' target: copy release binary directly to ~/.local/bin
    add_custom_target(deploy
            COMMAND ${CMAKE_COMMAND} -E make_directory "$ENV{HOME}/.local/bin"
            COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> "$ENV{HOME}/.local/bin/${PROJECT_NAME}"
            DEPENDS ${PROJECT_NAME}
            COMMENT "Installing ${PROJECT_NAME} to ~/.local/bin"
    )
else ()
    message(SEND_ERROR "You are on an unsupported platform! (Not Win32, Mac OS X or Unix)")
endif (WIN32)


# Windows builds need the DLLs copying into th same directory as the executable.
# The names of these end in d (e.g. Qt6Guid.dll) for the debug build
if (WIN32)
    if (CMAKE_BUILD_TYPE MATCHES "Debug")
        add_custom_command(
                TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DLL_PATH}/libgcc_s_seh-1.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DLL_PATH}/libwinpthread-1.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DLL_PATH}/libstdc++-6.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DLL_PATH}/Qt6Widgetsd.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DLL_PATH}/Qt6Guid.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DLL_PATH}/Qt6Cored.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DLL_PATH}/Qt6PrintSupportd.dll ${CMAKE_CURRENT_BINARY_DIR}
        )
    endif ()
    if (CMAKE_BUILD_TYPE MATCHES "Release")
        add_custom_command(
                TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_PREFIX_PATH}/bin/libgcc_s_seh-1.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_PREFIX_PATH}/bin/libwinpthread-1.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_PREFIX_PATH}/bin/libstdc++-6.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_PREFIX_PATH}/bin/Qt6Widgets.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_PREFIX_PATH}/bin/Qt6Gui.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_PREFIX_PATH}/bin/Qt6Core.dll ${CMAKE_CURRENT_BINARY_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_PREFIX_PATH}/bin/Qt6PrintSupport.dll ${CMAKE_CURRENT_BINARY_DIR}
        )
    endif ()
    # 'deploy' target: run windeployqt to bundle all required DLLs and plugins
    find_program(WINDEPLOYQT windeployqt HINTS "${CMAKE_PREFIX_PATH}/bin")
    if (WINDEPLOYQT)
        add_custom_target(deploy
                COMMAND ${WINDEPLOYQT} --release $<TARGET_FILE:${PROJECT_NAME}>
                DEPENDS ${PROJECT_NAME}
                COMMENT "Running windeployqt..."
        )
    else ()
        message(WARNING "windeployqt not found - 'deploy' target unavailable")
    endif ()
ENDIF ()
