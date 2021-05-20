cmake_minimum_required(VERSION 3.0)


include(${CMAKE_CURRENT_SOURCE_DIR}/../../min-api/script/min-pretarget.cmake)

include_directories( 
	"${C74_INCLUDES}"
)

include_directories(
        "${CMAKE_CURRENT_SOURCE_DIR}/mi"
)

set(CLOUDS_SRC
       mi/clouds/resources.cc
       mi/clouds/dsp/correlator.cc
       mi/clouds/dsp/mu_law.cc
       mi/clouds/dsp/granular_processor.cc
       mi/clouds/dsp/pvoc/frame_transformation.cc
       mi/clouds/dsp/pvoc/phase_vocoder.cc
       mi/clouds/dsp/pvoc/stft.cc 
       )

set(MI_COMMON_SRC
       mi/stmlib/dsp/units.cc
       mi/stmlib/dsp/atan.cc 
       mi/stmlib/utils/random.cc
)

set(HUMOLIB_SRC 
        ${MI_COMMON_SRC}
        ${CLOUDS_SRC}
        )

add_library(HUMOLIB ${HUMOLIB_SRC} )

add_library( 
	${PROJECT_NAME} 
	MODULE
	${PROJECT_NAME}.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC HUMOLIB)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../min-api/script/min-posttarget.cmake)
