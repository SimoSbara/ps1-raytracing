# PSn00bSDK example CMake script
# (C) 2021 spicyjpeg - MPL licensed

cmake_minimum_required(VERSION 3.21)

project(
	bench
	LANGUAGES    C CXX ASM
	VERSION      1.0.0
	DESCRIPTION  "PS1 raytracing"
	HOMEPAGE_URL "https://github.com/SimoSbara/ps1-raytracing"
)

file(GLOB _sources *.c)
psn00bsdk_add_executable(rt GPREL ${_sources})

psn00bsdk_add_cd_image(
	iso      # Target name
	rt 	     # Output file name (= rt.bin + rt.cue)
	iso.xml  # Path to config file
	DEPENDS rt system.cnf
)

install(
	FILES
		${PROJECT_BINARY_DIR}/rt.bin
		${PROJECT_BINARY_DIR}/rt.cue
	TYPE BIN
)