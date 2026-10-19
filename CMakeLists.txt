cmake_minimum_required(VERSION 3.16)
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
set(CMAKE_CXX_EXTENSIONS OFF)

# turn physics, no Qt: shared by the app and the unit tests
set(PHYSICS_SOURCES
        src/physics/pose.h
        src/physics/track.h
        src/physics/track.cpp
        src/physics/robotstate.h
        src/physics/angles.h
        src/physics/turn_profile.h
        src/physics/turn_profile.cpp
        src/physics/segmented_profile.h
        src/physics/segmented_profile.cpp
        src/physics/profile_registry.cpp
        src/physics/profile_sinusoid.cpp
        src/physics/profile_trapezoid.cpp
        src/physics/profile_cubic.cpp
        src/physics/profile_hybrid.cpp
        src/physics/robot_sim.h
        src/physics/robot_sim.cpp
        src/robot/velocity_profile.h
        src/robot/robot_compat.h
        src/robot/cubic_profile.h
        src/robot/hybrid_turn_profile.h
        )
# physics headers by name; robot profiles as "robot/<name>.h"
set(PHYSICS_INCLUDE_DIRS src src/physics)

# vendor plotting widget; not reformatted (third_party/.clang-format)
set(QCUSTOMPLOT_SOURCES
        third_party/qcustomplot/qcustomplot.cpp
        third_party/qcustomplot/qcustomplot.h
        )
# deprecated Qt calls inside qcustomplot: not ours to fix
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set_source_files_properties(third_party/qcustomplot/qcustomplot.cpp
            PROPERTIES COMPILE_OPTIONS "-Wno-deprecated-declarations")
endif ()

set(SOURCE_FILES
        ${PHYSICS_SOURCES}
        ${QCUSTOMPLOT_SOURCES}
        src/ui/main_window_ui.h
        src/ui/main_window_ui.cpp
        src/ui/mouseitem.cpp
        src/ui/mouseitem.h
        src/ui/main.cpp
        src/ui/mainwindow.cpp
        src/ui/mainwindow.h
        assets/resources.qrc
        )

add_executable(qtTurnProfile ${SOURCE_FILES})
target_include_directories(qtTurnProfile PRIVATE ${PHYSICS_INCLUDE_DIRS} src/ui third_party/qcustomplot)

# unit tests and baseline regression for the physics: ctest, or
#   turn_profile_tests tests/baseline.txt [--update]
enable_testing()
add_executable(turn_profile_tests tests/turn_profile_tests.cpp ${PHYSICS_SOURCES})
target_include_directories(turn_profile_tests PRIVATE ${PHYSICS_INCLUDE_DIRS})
set_target_properties(turn_profile_tests PROPERTIES AUTOMOC OFF AUTORCC OFF)
add_test(NAME turn_profile_tests
        COMMAND turn_profile_tests ${CMAKE_SOURCE_DIR}/tests/baseline.txt)


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
    # 'deploy' target: copy binary to ~/.local/bin and add a menu entry
    # (Science & Math) via ~/.local/share/applications/${PROJECT_NAME}.desktop.
    # Also installs ~/.local/bin/${PROJECT_NAME}-uninstall, which the menu
    # entry offers as a right-click "Uninstall" action.
    set(DEPLOY_BIN_DIR "$ENV{HOME}/.local/bin")
    set(DEPLOY_DATA_DIR "$ENV{HOME}/.local/share/${PROJECT_NAME}")
    set(DEPLOY_APPS_DIR "$ENV{HOME}/.local/share/applications")
    set(DEPLOY_ICON "${DEPLOY_DATA_DIR}/${PROJECT_NAME}.png")
    set(DEPLOY_UNINSTALL "${DEPLOY_BIN_DIR}/${PROJECT_NAME}-uninstall")
    configure_file(packaging/qt_turn_profile.desktop.in ${PROJECT_NAME}.desktop @ONLY)
    configure_file(packaging/qt_turn_profile_uninstall.sh.in ${PROJECT_NAME}-uninstall @ONLY)
    # refresh the application menu after deploying; Plasma caches entries (ksycoca)
    find_program(KBUILDSYCOCA NAMES kbuildsycoca6 kbuildsycoca5)
    if (KBUILDSYCOCA)
        set(MENU_REFRESH COMMAND ${KBUILDSYCOCA})
    endif ()
    add_custom_target(deploy
            COMMAND ${CMAKE_COMMAND} -E make_directory "${DEPLOY_BIN_DIR}" "${DEPLOY_DATA_DIR}" "${DEPLOY_APPS_DIR}"
            COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> "${DEPLOY_BIN_DIR}/${PROJECT_NAME}"
            COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_SOURCE_DIR}/assets/qtTurnProfiler.png" "${DEPLOY_ICON}"
            COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.desktop" "${DEPLOY_APPS_DIR}/${PROJECT_NAME}.desktop"
            COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_BINARY_DIR}/${PROJECT_NAME}-uninstall" "${DEPLOY_UNINSTALL}"
            COMMAND chmod 755 "${DEPLOY_UNINSTALL}"
            ${MENU_REFRESH}
            DEPENDS ${PROJECT_NAME}
            COMMENT "Installing ${PROJECT_NAME} to ~/.local/bin with menu entry"
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
