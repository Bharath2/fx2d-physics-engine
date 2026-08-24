# Cross-compile Fx2D for 64-bit ARM with the Debian/Ubuntu aarch64 GCC.
#
# Fx2D is meant to run on ARM as well as x86, and this is how that claim gets tested without ARM
# hardware: build with this toolchain, then run the result under qemu-user. The engine has no
# architecture-specific code, so what this actually checks is that the build flags are portable
# and that the numerical results still land inside the tolerances the test suite pins.
#
#   sudo apt install g++-aarch64-linux-gnu qemu-user
#
# yaml-cpp has to exist for the target too. Build it once, anywhere, and point this at it:
#
#   cmake -S yaml-cpp -B yaml-cpp-arm64 \
#     -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/toolchains/aarch64-linux-gnu.cmake \
#     -DCMAKE_INSTALL_PREFIX=$HOME/arm64-sysroot \
#     -DYAML_BUILD_SHARED_LIBS=OFF -DYAML_CPP_BUILD_TESTS=OFF -DYAML_CPP_BUILD_TOOLS=OFF
#   cmake --build yaml-cpp-arm64 -j && cmake --install yaml-cpp-arm64
#
# Eigen is header-only, so the host copy in /usr/include/eigen3 serves both architectures.
# Then:
#
#   cmake -S . -B build-arm64 -DCMAKE_BUILD_TYPE=Release -DFX2D_HEADLESS=ON \
#     -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/toolchains/aarch64-linux-gnu.cmake \
#     -DCMAKE_PREFIX_PATH=$HOME/arm64-sysroot
#   cmake --build build-arm64 -j --target Fx2DTests
#   qemu-aarch64 -L /usr/aarch64-linux-gnu ./build-arm64/Fx2DTests

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# Look for target libraries under the sysroot and CMAKE_PREFIX_PATH, but keep finding programs
# on the host -- otherwise CMake tries to run aarch64 binaries during configure.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BEFORE)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)
