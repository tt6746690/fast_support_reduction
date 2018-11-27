#ifndef DEFS_H
#define DEFS_H

// Because of Mosek complications, we don't use static library if Mosek is used.
#ifdef LIBIGL_WITH_MOSEK
    #ifdef IGL_STATIC_LIBRARY
        #undef IGL_STATIC_LIBRARY
    #endif
#endif

#ifndef DATA_PATH
    #define DATA_PATH "data/"
#endif

#endif