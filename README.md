# qtTurnProfile

A Qt6 application that simulates a number of different turn profile strategies for a micromouse.

## Requirements

- Qt 6 (Core, Gui, Widgets, PrintSupport). Tested with Qt 6.10.2
- CMake 3.16+ (minimum required by Qt6)
- C++17 compiler

## Building

Configure and build from the project root:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

### Locating Qt

On Linux the system Qt6 is found automatically (e.g. Debian/Ubuntu package
`qt6-base-dev`).

For any other Qt6 install (Qt online installer, Windows, macOS) set
`CMAKE_PREFIX_PATH` to the Qt kit directory at configure time:

```bash
cmake -B build -DCMAKE_PREFIX_PATH=/path/to/Qt/6.x.y/gcc_64
```

On Windows the Release build copies Qt DLLs from `${CMAKE_PREFIX_PATH}/bin`, so
`CMAKE_PREFIX_PATH` must be set there.

## Tests

The turn physics has a separate test target (no Qt) that runs property checks
and compares results against a stored baseline:

```bash
cmake --build build --target turn_profile_tests
cd build && ctest
```

After an intended physics change, regenerate the baseline and review the diff:

```bash
./build/turn_profile_tests tests/baseline.txt --update
```

## Deployment

### Linux

The binary dynamically links against Qt. To run it on another machine the Qt
shared libraries must be present. Two options:

**Install to `~/.local/bin` (this machine only):**

```bash
cmake --build build --target deploy
```

This installs:
- `~/.local/bin/qtTurnProfile`
- `~/.local/share/qtTurnProfile/qtTurnProfile.png` (icon)
- `~/.local/share/applications/qtTurnProfile.desktop` (menu entry, Science & Math)
- `~/.local/bin/qtTurnProfile-uninstall`

To uninstall, right-click Turn Profile in the application menu and choose
Uninstall, or run `qtTurnProfile-uninstall`. It asks first, then removes only
the files listed above. The source repository is not needed.

`cmake --install build --prefix ~/.local` installs the binary only (no menu
entry or uninstaller).

**Note:** For distribution to machines without Qt installed, consider packaging
as an AppImage using `linuxdeploy` with its Qt plugin.

### Windows

**Untested since the Qt6 port.**

Run `windeployqt` via the CMake deploy target to copy all required DLLs and
platform plugins into the build directory:

```bash
cmake --build build --target deploy
```

This runs `windeployqt --release` on the executable, which copies all Qt DLLs,
the `platforms/qwindows.dll` plugin, and any other runtime dependencies. The
resulting build directory can then be zipped or wrapped in an installer
(e.g. NSIS or Inno Setup) for distribution.

## Static linking

True static linking is not supported with the standard Qt open-source installer,
which ships only shared libraries. Static Qt builds require either:
- Building Qt from source with `-static` (LGPL obligations apply -- object files
  must be provided to end users), or
- A commercial Qt license.