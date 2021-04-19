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
    double fill_val;
    // https://python.readthedocs.io/en/v2.7.2/c-api/arg.html#arg-parsing
    PyArg_ParseTuple(args, "did", &esdf_voxel_size, &esdf_voxel_per_side, &fill_val);

    void* mapm = C_create_esdf_map(esdf_voxel_size, esdf_voxel_per_side, fill_val);
    return PyLong_FromVoidPtr(mapm);
}

static PyObject* py_update_esdf_map(PyObject *self, PyObject *args){
    PyObject* mapm_;
    unsigned char* msg_serialized;
    PyArg_ParseTuple(args, "Os", &mapm_, &msg_serialized);
    void* mapm = PyLong_AsVoidPtr(mapm_);
    C_update_esdf_map(mapm, msg_serialized);
    Py_RETURN_NONE;
}

static PyObject* py_get_dist(PyObject *self, PyObject *args){
    PyObject* mapm_;
    PyObject* py_point;
    PyArg_ParseTuple(args, "OO", &mapm_, &py_point);
    void* mapm = PyLong_AsVoidPtr(mapm_);
    double point[3];
    for(int i=0; i<3; i++){
        point[i] = PyFloat_AsDouble(PyList_GetItem(py_point, i));
    }
    double dist;
    C_get_dist(mapm, point, &dist);
    return PyFloat_FromDouble(dist);
}

static PyObject* py_get_batch_dist(PyObject *self, PyObject *args){
    PyObject* mapm_;
    PyObject* py_point_list;
    PyArg_ParseTuple(args, "OO", &mapm_, &py_point_list);
    void* mapm = PyLong_AsVoidPtr(mapm_);

    int n_point = PySequence_Size(py_point_list);
    PyObject* py_dist_list = PyList_New(n_point);
    double point[3];
    double dist;
    for(int i=0; i<n_point; i++){
      PyObject* py_point = PyList_GetItem(py_point_list, i);
      for(int j=0; j<3; j++){
          point[j] = PyFloat_AsDouble(PyList_GetItem(py_point, j));
      }
      C_get_dist(mapm, point, &dist);
      PyList_SetItem(py_dist_list, i, PyFloat_FromDouble(dist));
    }
    return py_dist_list;
}

static PyObject* py_get_batch_dist_and_grad(PyObject *self, PyObject *args){
    PyObject* mapm_;
    PyObject* py_point_list;
    PyArg_ParseTuple(args, "OO", &mapm_, &py_point_list);
    void* mapm = PyLong_AsVoidPtr(mapm_);

    int n_point = PySequence_Size(py_point_list);
    PyObject* py_dist_list = PyList_New(n_point);
    PyObject* py_grad_list = PyList_New(n_point * 3);
    double point[3];
    double dist;
    double grad[3];
    for(int i=0; i<n_point; i++){
      PyObject* py_point = PyList_GetItem(py_point_list, i);
      for(int j=0; j<3; j++){
          point[j] = PyFloat_AsDouble(PyList_GetItem(py_point, j));
      }
      C_get_dist_and_grad(mapm, point, &dist, grad);
      PyList_SetItem(py_dist_list, i, PyFloat_FromDouble(dist));
      for(int j=0; j<3; j++){
          PyList_SetItem(py_grad_list, 3*i+j, PyFloat_FromDouble(grad[j]));
      }
    }
    PyObject* ret = PyTuple_New(2);
    PyTuple_SetItem(ret, 0, py_dist_list);
    PyTuple_SetItem(ret, 1, py_grad_list);
    return ret;
}

static PyObject* py_debug_dist4000(PyObject *self, PyObject *args){
    PyObject* mapm_;
    PyArg_ParseTuple(args, "O", &mapm_);
    void* mapm = PyLong_AsVoidPtr(mapm_);

    double point[3] = {2.0, 2.0, 1.0};
    double dist;
    for(int i=0; i<400; i++){
        C_get_dist(mapm, point, &dist);
    }
    Py_RETURN_NONE;
}

static PyMethodDef methods[] = {
    {"create_esdf_map", py_create_esdf_map, METH_VARARGS, "create esdf map"},
    {"update_esdf_map", py_update_esdf_map, METH_VARARGS, "update esdf map"},
    {"get_dist", py_get_dist, METH_VARARGS, "get signed distance"},
    {"get_batch_dist", py_get_batch_dist, METH_VARARGS, "get signed distances"},
    {"get_batch_dist_and_grad", py_get_batch_dist_and_grad, METH_VARARGS, "get signed distances"},
    {"debug_dist4000", py_debug_dist4000, METH_VARARGS, "debugging only"},
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
