Real-time IMU Processing from ESP32
===================================

Supporting software to stream IMU (ICM-42688-p) connected to ESP32 via SPI onto desktop computer via serial or WiFi. Serial/UDP IO is handled in C++ and exposed to Python with pybind11.

### Building from Source

<details>
  <summary>Linux / MacOS</summary>

After cloning this project onto your computer:

1. Build C++ driver with Python bindings (requires Boost):
  
  ```bash
  mkdir build
  cd build
  cmake ..
  make
  ```
  
Once the package builds successfully, you can install the `esp32comm` Python package as described below.

</details>

<details>
  <summary>Windows</summary>

Compilation of this package has been tested on a Windows 10 machine. The environment was setup as follows. Explicit versions used are listed, but these steps are expected to work with reasonably recent versions of these tools.

1. Install [git bash](https://git-scm.com/downloads) (*2.30.1*)
2. Install [CMake](https://cmake.org/download/) (*3.19.5*)
3. Install [Python](https://www.python.org/) (*3.9.1*)
4. Install [Visual Studio Community](https://visualstudio.microsoft.com/vs/community/) or Professional (*Pro 2019, v16.8.5* with *MSVC 14.28.29333*)
5. Install [Boost](https://sourceforge.net/projects/boost/files/boost-binaries) (*1.75.0*, [`boost_1_75_0-msvc-14.2-64.exe`](https://sourceforge.net/projects/boost/files/boost-binaries/1.75.0/))

Once the development environment is setup, use `git bash` (or `cmd`) to run the following commands

```bash
$ git clone https://github.com/plusk01/esp32comm # clone this repo in your preferred directory
$ cd esp32comm
$ mkdir build
$ cd build
$ cmake -DBUILD_SHARED_LIBS=OFF ..
$ cmake --build . --target ALL_BUILD --config Release # or open in VS: start esp32comm.sln
$ cmake --build . --target pypkg --config Release # to tar python pkg
```

Once the package builds successfully, you can install the `esp32comm` Python package as described below.

</details>

<details>
  <summary>Installing Python Package from Source</summary>

Install the built-from-source Python package with `pip`:

```bash
cd build
make pip-install
```

</details>

## Setting up the ESP32 firmware

Flash ESP32 with `firmware_serial/firmware_serial.ino` sketch for serial comm and `firmware_udp/firmware_udp.ino` for WiFi/UDP.

## Examples

See an IMU data plot in real-time with `python -m esp32comm.plotimu`. Alternatively, you can see the frequency spectrum with `python -m esp32comm.plotfreq`

## Creating New Messages

To create new or change existing messages, changes need to be made in `esp32comm.h` and in `serial_driver.cpp`. For these changes to appear in Python, `py_teensyimu.cpp` also needs to be changed. For an example of necessary changes, see [this commit](https://github.com/plusk01/teensyimu/commit/2f2101865d4a2deb641b958747ef80e209a2884f) where two new messages (*IMU_NoMag* and *IMU_3DOF*) were added.

If messages are changed, this project must be built from source.
