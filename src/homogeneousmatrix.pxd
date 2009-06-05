cimport numpy
ctypedef numpy.float_t DTYPE_t
cpdef transl(DTYPE_t t_z, DTYPE_t t_y, DTYPE_t t_x)
cpdef rotzyx(DTYPE_t angle_z, DTYPE_t angle_y, DTYPE_t angle_x)
cpdef rotzy(DTYPE_t angle_z, DTYPE_t angle_y)
cpdef rotzx(DTYPE_t angle_z, DTYPE_t angle_x)
cpdef rotyx(DTYPE_t angle_y, DTYPE_t angle_x)
cpdef rotx(DTYPE_t angle)
cpdef roty(DTYPE_t angle)
cpdef rotz(DTYPE_t angle)
#...boolean function
#...here too
cpdef inv(numpy.ndarray H)
cpdef adjoint(numpy.ndarray H)
cpdef iadjoint(numpy.ndarray H)
