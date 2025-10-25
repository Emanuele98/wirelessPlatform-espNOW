# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/degan/OneDrive/Desktop/esp-idf-v5.1.1/components/bootloader/subproject"
  "C:/Users/degan/OneDrive/Desktop/2XBUMBLEBEE/wirelessPlatform-espNOW/AUX-CTU/build/bootloader"
  "C:/Users/degan/OneDrive/Desktop/2XBUMBLEBEE/wirelessPlatform-espNOW/AUX-CTU/build/bootloader-prefix"
  "C:/Users/degan/OneDrive/Desktop/2XBUMBLEBEE/wirelessPlatform-espNOW/AUX-CTU/build/bootloader-prefix/tmp"
  "C:/Users/degan/OneDrive/Desktop/2XBUMBLEBEE/wirelessPlatform-espNOW/AUX-CTU/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/degan/OneDrive/Desktop/2XBUMBLEBEE/wirelessPlatform-espNOW/AUX-CTU/build/bootloader-prefix/src"
  "C:/Users/degan/OneDrive/Desktop/2XBUMBLEBEE/wirelessPlatform-espNOW/AUX-CTU/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/degan/OneDrive/Desktop/2XBUMBLEBEE/wirelessPlatform-espNOW/AUX-CTU/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/degan/OneDrive/Desktop/2XBUMBLEBEE/wirelessPlatform-espNOW/AUX-CTU/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
