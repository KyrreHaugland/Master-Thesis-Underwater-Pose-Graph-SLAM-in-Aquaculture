#include <boost/shared_ptr.hpp>

#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include "gtsam/base/serialization.h"
#include "gtsam/nonlinear/utilities.h"  // for RedirectCout.

#include "cpp/DepthFactor.h"
#include "cpp/TiltFactor.h"
#include "cpp/USBLFactor.h"

#include <boost/serialization/export.hpp>



PYBIND11_DECLARE_HOLDER_TYPE(TYPE_PLACEHOLDER_DONOTUSE, boost::shared_ptr<TYPE_PLACEHOLDER_DONOTUSE>);

//#include "python/preamble.h"

using namespace std;

namespace py = pybind11;

PYBIND11_MODULE(gtsam_absolute_factors, m_) {
    m_.doc() = "pybind11 wrapper of gtsam_absolute_factors";


    py::class_<gtsam_absolute_factors::DepthFactor, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam_absolute_factors::DepthFactor>>(m_, "DepthFactor")
        .def(py::init<size_t, const gtsam::Point3&, boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("poseKey"), py::arg("m"), py::arg("model"));

    py::class_<gtsam_absolute_factors::TiltFactor, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam_absolute_factors::TiltFactor>>(m_, "TiltFactor")
        .def(py::init<size_t, const gtsam::Point3&, boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("poseKey"), py::arg("m"), py::arg("model"));

    py::class_<gtsam_absolute_factors::USBLFactor, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam_absolute_factors::USBLFactor>>(m_, "USBLFactor")
        .def(py::init<size_t, const gtsam::Point2&, boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("poseKey"), py::arg("m"), py::arg("model"));


#include "python/specializations.h"

}

