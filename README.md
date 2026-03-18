# qtTurnProfile

A Qt5 application that simulates a number of different turn profile strategies for a micromouse.

## Requirements

- Qt 5.15.2 (the project expects Qt at the paths listed below)
- CMake 3.5+
- C++11 compiler

## Building

Configure and build from the project root:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

### Qt installation paths

The CMakeLists.txt expects Qt at these default locations:

| Platform | Path |
|----------|------|
| Linux | `~/Qt/5.15.2/gcc_64` |
| Windows | `C:/Qt/5.12.3/mingw73_64` |
| macOS | `/Users/peterharrison/Qt5x/5.15.1/clang_64` |

Override by setting `CMAKE_PREFIX_PATH` at configure time:

```bash
cmake -B build -DCMAKE_PREFIX_PATH=/path/to/Qt/5.15.2/gcc_64
```

## Deployment

### Linux

The binary dynamically links against Qt. To run it on another machine the Qt
shared libraries must be present. Two options:

**Install to `~/.local/bin` (this machine only):**

```bash
cmake --build build --target deploy
```

Or equivalently:

```bash
cmake --install build --prefix ~/.local
```

Ensure `~/.local/bin` is on your `PATH`.

**Note:** For distribution to machines without Qt installed, consider packaging
as an AppImage using `linuxdeployqt`.

### Windows

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