#pragma once

#include <windows.h>

#define MIN2(a,b) (((a) < (b))?(a):(b))
#define MAX2(a,b) (((a) > (b))?(a):(b))
#define MAX3(a,b,c) ( ( (MAX2(a,b)) > (c) ) ? (MAX2(a,b)) : (c) )
#define MIN3(a,b,c) ( ( (MIN2(a,b)) > (c) ) ? (MIN2(a,b)) : (c) )

#define MYPI 3.14159265359

//Degree / Radian conversions
#define D2R(a) ((a)/180.0*MYPI)
#define R2D(a) ((a)/MYPI*180.0)