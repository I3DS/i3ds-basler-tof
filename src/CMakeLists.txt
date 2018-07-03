find_package (Boost COMPONENTS program_options log REQUIRED)

set (GENTL_ROOT /opt/BaslerToF)
set (GENAPI_POSTFIX gcc_v3_0_Basler_pylon_v5_0)

set (BASLER_TOF_LIBDIR  ${GENTL_ROOT}/lib64)
set (BASLER_TOF_INCLUDES ${GENTL_ROOT}/include)

set (BASLER_TOF_LIBS 
  GenApi_${GENAPI_POSTFIX} 
  GCBase_${GENAPI_POSTFIX} 
  dl 
)

message(STATUS "BASLER_TOF_LIBDIR=${BASLER_TOF_LIBDIR}")

set (BASLER_TOF_LIB_FLAGS
  "-Wl,-enable-new-dtags -Wl,-rpath ${BASLER_TOF_LIBDIR} -Wl,-rpath ${BASLER_TOF_LIBDIR}/gentlproducer/gtl -Wl,-E"
  )

set (BASLER_TOF_CXX_FLAGS 
  -std=c++11 -Wall -Wno-unknown-pragmas
  )

set (SRCS
  basler_tof_camera.cpp
  basler_tof_wrapper.cpp
  )

set (LIBS
  i3ds
  zmq
  pthread
  ${BASLER_TOF_LIBS}
  ${Boost_LIBRARIES}
)

include_directories ("../include/")

add_executable (i3ds-basler-tof i3ds_basler_tof.cpp ${SRCS} )
target_include_directories(i3ds-basler-tof PRIVATE ${BASLER_TOF_INCLUDES})
target_compile_options(i3ds-basler-tof PRIVATE -Wno-unknown-pragmas)
target_link_libraries (i3ds-basler-tof -L${BASLER_TOF_LIBDIR} ${BASLER_TOF_LIB_FLAGS} ${LIBS})

install(TARGETS i3ds-basler-tof DESTINATION bin)
