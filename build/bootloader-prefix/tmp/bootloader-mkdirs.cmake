# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/codec/esp/esp-idf/components/bootloader/subproject"
  "/home/codec/esp/rc-car/build/bootloader"
  "/home/codec/esp/rc-car/build/bootloader-prefix"
  "/home/codec/esp/rc-car/build/bootloader-prefix/tmp"
  "/home/codec/esp/rc-car/build/bootloader-prefix/src/bootloader-stamp"
  "/home/codec/esp/rc-car/build/bootloader-prefix/src"
  "/home/codec/esp/rc-car/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/codec/esp/rc-car/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/codec/esp/rc-car/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
