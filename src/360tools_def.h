/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2016, Samsung Electronics Co., Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of Samsung Electronics Co., Ltd. nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef __360TOOLS_DEF_H__4232432432432532432432432432__
#define __360TOOLS_DEF_H__4232432432432532432432432432__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* return values and error code **********************************************/
#define S360_OK                            (0)
#define S360_ERR                           (-1)
#define S360_ERR_INVALID_ARGUMENT          (-101)
#define S360_ERR_OUT_OF_MEMORY             (-102)
#define S360_ERR_REACHED_MAX               (-103)
#define S360_ERR_UNSUPPORTED               (-104)
#define S360_ERR_UNSUPPORTED_COLORSPACE    (-105)
#define S360_ERR_INVALID_DIMENSION         (-106)
#define S360_PROJ_NOT_SUPPORTED            (-107)

/* return value checking *****************************************************/
#define S360_SUCCEEDED(ret)              ((ret) >= 0)
#define S360_FAILED(ret)                 ((ret) < 0)

/* utility macros ************************************************************/
#define S360_MIN(a, b)          ((a) < (b) ? (a) : (b))
#define S360_MAX(a, b)          ((a) > (b) ? (a) : (b))
#define S360_MINMAX(v, a, b)    S360_MAX(S360_MIN((v), (b)), (a))
#define S360_CLIP_S32_TO_U8(x)  ((x) > 255 ? 255 : (x) < 0 ? 0 : (x))
#define S360_CLIP_S32_TO_U10(x) ((x) > 1023 ? 1023 : (x) < 0 ? 0 : (x))
#define S360_ALIGN(val, align)  (((val) + (align) - 1) / (align) * (align))
#define S360_ABS(x)             ((x) >= 0) ? (x) : (-(x))

#ifndef NULL
#define NULL                     (void*)0
#endif

/* TSP macros ****************************************************************/
#define TSPAA_S 5

/* redefined types ***********************************************************/
#if defined(WIN32) || defined(WIN64)
typedef __int8                 int8;
typedef unsigned __int8        uint8;
typedef __int16                int16;
typedef unsigned __int16       uint16;
typedef __int32                int32;
typedef unsigned __int32       uint32;
typedef __int64                int64;
typedef unsigned __int64       uint64;
#elif defined(LINUX)
#include <stdint.h>
typedef int8_t                 int8;
typedef uint8_t                uint8;
typedef int16_t                int16;
typedef uint16_t               uint16;
typedef int32_t                int32;
typedef uint32_t               uint32;
typedef int64_t                int64;
typedef uint64_t               uint64;
#else
typedef signed char            int8;
typedef unsigned char          uint8;
typedef signed short           int16;
typedef unsigned short         uint16;
typedef signed int             int32;
typedef unsigned int           uint32;
#if defined(X86_64) && !defined(_MSC_VER) /* for 64bit-Linux */
typedef signed long            int64;
typedef unsigned long          uint64;
#else
typedef signed long long       int64;
typedef unsigned long long     uint64;
#endif
#endif


/* trace message disable/enable **********************************************/
#ifndef S360_TRACE
#define S360_TRACE               0
#endif

/* assert disable/enable *****************************************************/
#ifndef S360_ASSERT
#define S360_ASSERT              0
#endif

/* memory operations *********************************************************/
#define s360_malloc(size)         malloc((size))
#define s360_mfree(m)             free((m))
#define s360_mcpy(dst,src,size)   memcpy((dst), (src), (size))
#define s360_mset(dst,v,size)     memset((dst), (v), (size))

/* print function ************************************************************/
#if defined(LINUX)
#define s360_print(args...) printf(args)
#else
#define s360_print printf
#endif

/* trace function ************************************************************/
#if S360_TRACE
#define s360_trace s360_print("[%s:%d] ", __FILE__, __LINE__); s360_print
#else
#if defined(LINUX)
#define s360_trace(args...) {}
#else
#define s360_trace(...) {}
#endif
#endif

/* assert function ***********************************************************/
#if S360_ASSERT
#define __S360_ASSERT_MSG(x)  { printf("[%s:%d] assert failed! ("#x")\n", __FILE__, __LINE__); }
#define __S360_MAKE_FAULT()   {*(int*)0=0;}
#else
#define __S360_ASSERT_MSG(x)  {}
#define __S360_MAKE_FAULT()  {}
#endif

#define s360_assert(x) {if(!(x)){__S360_ASSERT_MSG(x); __S360_MAKE_FAULT();}}
#define s360_assert_r(x) {if(!(x)){__S360_ASSERT_MSG(x); __S360_MAKE_FAULT(); return;}}
#define s360_assert_rv(x,r) {if(!(x)){__S360_ASSERT_MSG(x); __S360_MAKE_FAULT(); return (r);}}


/* various color spaces ******************************************************/
#define S360_COLORSPACE_YUV420             (10)
#define S360_COLORSPACE_YUV422             (11)
#define S360_COLORSPACE_YUV444             (13)

#define S360_COLORSPACE_YUV420_10          (20)
#define S360_COLORSPACE_YUV422_10          (21)
#define S360_COLORSPACE_YUV444_10          (23)

#define IS_VALID_CS(cs) (((cs)==S360_COLORSPACE_YUV420)||((cs)==S360_COLORSPACE_YUV420_10))

/* 10-bit Macro. To Be Deleted Once 10-bit content is available **************/
#define INP_10B 0
#define INT_10_BIT_DEPTH 2

/* Resample function *********************************************************/
typedef void(*resample_fn)(void * src, int w_start, int w_end,int h_start, \
	int h_src, int s_src, double x, double y, void * dst, int x_dst);

/* image buffer format *******************************************************/
typedef struct
{
	void              * buf_pad[4]; /* adress of each plane with padding */
	void              * buffer[4]; /* address of each plane */
	int                 stride[4]; /* buffer stride */
	int                 elevation[4]; /* buffer elevation */
	int                 width; /* width */
	int                 height; /* height */
	int                 colorspace; /* color space */
} S360_IMAGE;

/* Spherical PSNR Sampling Location Structure ********************************/
typedef struct
{
	float lat;
	float lng;
}S360_SPH_COORD;

/* Latlong map for Y, chroma layers *****************************************/
typedef struct
{
	S360_SPH_COORD  * layer[2]; /* map for each plane*/
	int		width;
	int		height;
	int		pitch;
	int		yaw;
}S360_MAP;

/* Conversion Formats ******************************************************/
typedef enum
{
	CONV_FMT_ERP_TO_CPP   = 0,
	CONV_FMT_ERP_TO_ISP   = 1,
	CONV_FMT_ISP_TO_ERP,
	CONV_FMT_ERP_TO_CMP,
	CONV_FMT_CMP_TO_ERP,
	CONV_FMT_ERP_TO_OHP,
	CONV_FMT_OHP_TO_ERP,
	CONV_FMT_ERP_TO_TSP,
	CONV_FMT_TSP_TO_ERP,
	CONV_FMT_ERP_TO_SSP,
	CONV_FMT_SSP_TO_ERP,
	CONV_FMT_ISP_TO_RISP  =11,
	CONV_FMT_RISP_TO_ISP,
	CONV_FMT_CMP_TO_RCMP,
	CONV_FMT_RCMP_TO_CMP,
	CONV_FMT_OHP_TO_ROHP,
	CONV_FMT_ROHP_TO_OHP,
	CONV_FMT_ERP_TO_RISP1 = 21,
	CONV_FMT_RISP1_TO_ERP,
	CONV_FMT_ERP_TO_COHP  = 25,
	CONV_FMT_COHP_TO_ERP,
	CONV_FMT_CPP_TO_ERP   = 31,
	CONV_FMT_CPP_TO_ISP,
	CONV_FMT_CPP_TO_CMP,
	CONV_FMT_CPP_TO_OHP,
	CONV_FMT_CPP_TO_TSP,
	CONV_FMT_CPP_TO_SSP,
	CONV_FMT_CPP_BYPASS,
	CONV_FMT_MAX
} CONV_FMT;

/* Projection Formats ******************************************************/
typedef enum
{
	PROJ_FMT_ERP = 1,
	PROJ_FMT_ISP,
	PROJ_FMT_CMP,
	PROJ_FMT_OHP,
	PROJ_FMT_TSP,
	PROJ_FMT_CPP,
	PROJ_FMT_SSP,
	PROJ_FMT_MAX
} PROJ_FMT;

#ifdef __cplusplus
}
#endif

#endif /* __360TOOLS_DEF_H__4232432432432532432432432432__ */

