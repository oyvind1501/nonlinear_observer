
find_package(Eigen3 REQUIRED)

add_library(nonlinear_observer_lib
        common.cpp
        nonlinear_observer.cpp)

target_include_directories(nonlinear_observer_lib
        PUBLIC
        ../include/nonlinear_observer
	${EIGEN3_INCLUDE_DIR}
        PRIVATE
        ../src
        )


#target_compile_features(ESKF_lib PUBLIC cxx_std_11)
