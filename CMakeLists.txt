# --------------------------------------------------------------------------
# Dependencies
# --------------------------------------------------------------------------

find_package(yat CONFIG REQUIRED)

# --------------------------------------------------------------------------
# Collect sources and includes
# --------------------------------------------------------------------------

file(GLOB_RECURSE sources src/*.cpp)
set(includedirs
    include/
    slsDetectorPackage/slsDetectorSoftware/commonFiles/
    slsDetectorPackage/slsDetectorSoftware/multiSlsDetector/
    slsDetectorPackage/slsDetectorSoftware/slsDetector/
    slsDetectorPackage/slsDetectorSoftware/slsDetectorAnalysis/
    slsDetectorPackage/slsDetectorSoftware/slsReceiverInterface/
    slsDetectorPackage/slsDetectorSoftware/threadFiles/
    slsDetectorPackage/slsReceiverSoftware/include/
)

# --------------------------------------------------------------------------
# Target
# --------------------------------------------------------------------------

add_library(limaslsjungfrau ${sources})

target_include_directories(limaslsjungfrau
    PUBLIC 
        ${includedirs}
)

add_library(SlsDetector STATIC IMPORTED)
set_target_properties(SlsDetector PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/slsDetectorPackage/bin/libSlsDetector.a"
)

add_library(SlsReceiver STATIC IMPORTED)
set_target_properties(SlsReceiver PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/slsDetectorPackage/bin/libSlsReceiver.a"
)

target_link_libraries(limaslsjungfrau 
    PUBLIC
        limacore
        yat::yat
        SlsDetector
        SlsReceiver
)

if (CONAN_LIBCXX STREQUAL "libstdc++")
    target_compile_features(limaslsjungfrau
        PRIVATE
            cxx_std_11
    )
endif()

limatools_set_library_soversion(limaslsjungfrau "VERSION")

# --------------------------------------------------------------------------
# Compile definitions
# --------------------------------------------------------------------------

file(READ "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" SLSJUNGFRAU_VERSION_RAW)
string(STRIP "${SLSJUNGFRAU_VERSION_RAW}" SLSJUNGFRAU_VERSION)

target_compile_definitions(limaslsjungfrau
    PUBLIC
        SLSJUNGFRAU_NAME=SlsJungfrau
        SLSJUNGFRAU_VERSION=${SLSJUNGFRAU_VERSION}
        SLSJUNGFRAU_ENABLED
)

message(STATUS "Camera enabled: SlsJungfrau ${SLSJUNGFRAU_VERSION}")

# --------------------------------------------------------------------------
# Install
# --------------------------------------------------------------------------

install(
    TARGETS limaslsjungfrau 
    LIBRARY DESTINATION lib
)
