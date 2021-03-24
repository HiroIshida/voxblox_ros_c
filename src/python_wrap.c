#include <Python.h>
#include "c_api.h"

#if PY_MAJOR_VERSION >= 3
// https://stackoverflow.com/questions/32295927/failed-c-extension-compilation-for-python
#define PY3K
#endif

static PyObject* PY_create_esdf_map(PyObject *self, PyObject *args){

    double esdf_voxel_size, esdf_voxel_per_side;
    PyArg_ParseTuple(args, "di", &esdf_voxel_size, esdf_voxel_per_side);

    void* mapm = C_create_esdf_map(esdf_voxel_size, esdf_voxel_per_side);
    PyObject* py_mapm = (PyObject*)mapm;
    return py_mapm;
}

static PyMethodDef methods[] = {
    {"_voxblox_ros_python", PY_create_esdf_map, METH_VARARGS, "Python interface for fputs C library function"},
    {NULL, NULL, 0, NULL}
};

#ifdef PY3K
static struct PyModuleDef voxblox_ros_python_module = {
    PyModuleDef_HEAD_INIT,
    "_voxblox_ros_python",
    "Python interface for voxblox_ros",
    -1,
    methods
};
#else
PyMODINIT_FUNC PyInit_voxblox_ros_python(){
    Py_InitModule3("_voxblox_ros_python", methods, "doc TODO");
}
#endif
/*

PyMODINIT_FUNC PyInit_voxblox_ros_python(void) {
    return PyModule_Create(&voxblox_ros_python_module);
}
*/
