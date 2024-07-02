#include "NNArchiveBindings.hpp"

#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/functional.h>

// depthai
#include "depthai/nn_archive/NNArchive.hpp"

PYBIND11_MAKE_OPAQUE(std::vector<uint8_t>);

void NNArchiveBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<NNArchive> nnArchive(m, "NNArchive", DOC(dai, NNArchive));

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // Bind NNArchive
    nnArchive
        .def(py::init<const dai::Path&, NNArchiveEntry::Compression>(),
             py::arg("path"),
             py::arg("compression") = NNArchiveEntry::Compression::AUTO,
             DOC(dai, NNArchive, NNArchive))
        .def(py::init<const std::vector<uint8_t>&, NNArchiveEntry::Compression>(),
             py::arg("data"),
             py::arg("compression") = NNArchiveEntry::Compression::AUTO,
             DOC(dai, NNArchive, NNArchive))
        .def(py::init<
                 const std::function<int()>&,
                 const std::function<std::shared_ptr<std::vector<uint8_t>>()>&,
                 const std::function<int64_t(int64_t, NNArchiveEntry::Seek)>&,
                 const std::function<int64_t(int64_t)>&,
                 const std::function<int()>&,
                 NNArchiveEntry::Compression>())
        ;
}
