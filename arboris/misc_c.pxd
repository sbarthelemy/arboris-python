# coding=utf-8

cimport numpy
ctypedef numpy.float_t DTYPE_t

cdef extern from "math.h":
    double cos(double angle)
    double sin(double angle)
    double sqrt(double x)

cdef asym(DTYPE_t v_x, DTYPE_t v_y, DTYPE_t v_z)
cdef norm3(DTYPE_t v_x, DTYPE_t v_y, DTYPE_t v_z)
cdef norm6(DTYPE_t w_x, DTYPE_t w_y, DTYPE_t w_z, \
           DTYPE_t v_x, DTYPE_t v_y, DTYPE_t v_z)

