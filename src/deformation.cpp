#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <iostream>
#include <vector>
#include <chrono>

#include <Eigen/Dense>

namespace py = pybind11;

using Vector3d = Eigen::Vector3d;

//for disp
//
//std::vector<std::pair<Vector3d, Vector3d>> DeformationDetection(py::array_t<double> vectorA, py::array_t<double> vectorB) {
//
//
	
std::tuple<double, double> DeformationDetection(py::array_t<double> vectorA, py::array_t<double> vectorB) {

    auto bufferA = vectorA.request();
    auto bufferB = vectorB.request();

    double  *ptrA = (double *) bufferA.ptr,
	    *ptrB = (double *) bufferB.ptr;

    std::vector<long int> shapeA = bufferA.shape,
	                  shapeB = bufferB.shape;

    size_t sizeA = shapeA[0] * shapeA[1],
           sizeB = shapeB[0] * shapeB[1];

    std::vector<Vector3d> BF;
    std::vector<Vector3d> AF;

    for (size_t i = 0; i < sizeA; i+=3 ) {
        Vector3d vectorPoint( static_cast<double>(ptrA[i]), static_cast<double>(ptrA[i + 1]), static_cast<double>(ptrA[i + 2]) );
	//std::cout << "x:"<< ptrA[i ]<< "y:" << ptrA[i+1] << "z:" << ptrA[i+2] <<std::endl;
        BF.push_back(vectorPoint);
    }

    for (size_t i = 0; i < sizeB; i+=3) {
        Vector3d vectorPoint( static_cast<double>(ptrB[i]), static_cast<double>(ptrB[i + 1]), static_cast<double>(ptrB[i + 2]) );
        AF.push_back(vectorPoint);
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    double distance = 0.0;

    //
    //std::vector<std::pair<Vector3d, Vector3d>> corr;
    //


    for (const auto& p : AF) {
	double d_temp = 10000;

	//for display
	//Vector3d closest_q;
	//

        for (const auto& q : BF) {
            double d = (p - q).norm();
            if (d < d_temp) {
                d_temp = d;

		//for display
		//closest_q = q;
		//


            }
        }

        distance += d_temp;

	//
	//corr.push_back(std::make_pair(p,closest_q));
	//
    }



    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    std::cout << "Elapsed Time in seconds: " << elapsed_time.count() / 1000.0 << " s" << std::endl;
    std::cout << "Total Euclidean Distance: " << distance << std::endl;

    return std::make_tuple(distance, elapsed_time.count() / 1000.0);
    //return corr;
}

PYBIND11_MODULE(deformation_detection, m) {
    m.def("DeformationDetection", &DeformationDetection, "Perform Deformation Detection");
}

