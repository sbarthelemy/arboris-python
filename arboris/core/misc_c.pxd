# coding=utf-8

cimport numpy
ctypedef numpy.float_t DTYPE_t

cdef extern from "math.h":
    double cos(double angle)
    double sin(double angle)
cdef asym(DTYPE_t v_x, DTYPE_t v_y, DTYPE_t v_z)

