# ifndef HEAD_H
# define HEAD_H


/* -------------常用常数定义---------------------------------*/
#define pi              3.14159265357f        //π
#define DEG2RAD         1.74532925199433e-2f  //角度变换成弧度
#define RAD2DEG         5.72957795130823e1f   //弧度变换成角度
#define DEG_TO_RAD(deg) ((deg) * pi / 180.0f) //角度转弧度宏
#define RAD_TO_DEG(rad) ((rad) * 180.0f / pi) //弧度转角度宏
#define FLOAT_ZERO      1e-10f                //极小浮点数的容差
#define G               9.8f                  //重力加速度



/* -----------------头文件------------------------------------*/
//C语言官方的
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>




#endif
