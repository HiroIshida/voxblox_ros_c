#include <Python.h>
#include "c_api.h"
#include "stdio.h"

#if PY_MAJOR_VERSION >= 3
// https://stackoverflow.com/questions/32295927/failed-c-extension-compilation-for-python
#define PY3K
#endif

static PyObject* py_create_esdf_map(PyObject *self, PyObject *args){

    double esdf_voxel_size = 0.0;
    int esdf_voxel_per_side = 0;
    // https://python.readthedocs.io/en/v2.7.2/c-api/arg.html#arg-parsing
    PyArg_ParseTuple(args, "di", &esdf_voxel_size, &esdf_voxel_per_side);

    void* mapm = C_create_esdf_map(esdf_voxel_size, esdf_voxel_per_side);
    return PyLong_FromVoidPtr(mapm);
}

static PyObject* py_update_esdf_map(PyObject *self, PyObject *args){
    PyObject* mapm_;
    uint8_t* msg_serialized;
    PyArg_ParseTuple(args, "Ob", &mapm_, &msg_serialized);
    void* mapm = PyLong_AsVoidPtr(mapm_);
    C_update_esdf_map(mapm, msg_serialized);
    Py_RETURN_NONE;
}

static PyMethodDef methods[] = {
    {"create_esdf_map", py_create_esdf_map, METH_VARARGS, "create esdf map"},
    {"update_esdf_map", py_update_esdf_map, METH_VARARGS, "update esdf map"},
    {NULL, NULL, 0, NULL}
};

#define INITERROR return
#ifdef PY3K
// TODO
#else
PyMODINIT_FUNC
init_voxblox_ros_python(void){
    PyObject* m;
    m = Py_InitModule3("_voxblox_ros_python", methods, "doc TODO");
    if(m==NULL){
        INITERROR;
    }
}
#endif
