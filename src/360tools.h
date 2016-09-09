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

#ifndef __360TOOLS_H__218937465289347652834659823659823746592834765__
#define __360TOOLS_H__218937465289347652834659823659823746592834765__


#ifdef __cplusplus
extern "C"
{
#endif

#include <math.h>
#include "360tools_def.h"
#include "360tools_args.h"
#include "360tools_img.h"
#include "360tools_map.h"

/* various options for each projections **************************************/
#define S360_OPT_PAD                    (1<<0)

/* constants *****************************************************************/
#define PI                      (3.14159265358979323846)
#define SIN_60                  (0.86602540378443864676)
#ifndef M_PI_2
#define M_PI_2                  (1.5707963f)  /* pi/2 */
#endif
#define M_2PI                   (6.2831853f)  /* pi*2 */

/* utilities *****************************************************************/
#define DEG2RAD(deg)            ((deg) * PI / 180)
#define RAD2DEG(rad)            ((rad) * 180 / PI)
#define CEILING(x)              ((x) - (int)(x) > 0 ? (int)(x) + 1 : (int)(x))
#define NEAREST_EVEN(x)         ((int)(x) + ((int)(x) & 1))
#define ROUNDUP_EVEN(x)         ((int)((x) + 1) & 0xFFFFFFFE)
#define ROUNDDN_EVEN(x)         ((int)(x) & 0xFFFFFFFE)

/* 3D coordinates to lat and long ********************************************/
void cart_to_sph(double x, double y, double z, S360_SPH_COORD  * coord);

/* resample functions ********************************************************/
#define WIDTH_CPP               3840
#define HEIGHT_CPP              1920
#define USE_LANCZOS             1
#define LANCZOS_TAB_SIZE        3
#define LANCZOS_FAST_MODE       1
#if LANCZOS_FAST_MODE
#define LANCZOS_FAST_SCALE      100
#define LANCZOS_FAST_MAX_SIZE   4096
#endif
void resample_2d(void * src, int w_start, int w_end, int h_src, int s_src, \
	double x, double y, void * dst, int x_dst);
void resample_2d_10b(void * src, int w_start, int w_end, int h_src, int s_src, \
	double x, double y, void * dst, int x_dst);
void resample_tsp_2d(void * src, int w_start, int w_end, int w_src, int h_src, \
	int s_src, void * dst, int w_dst, int i, int j, S360_SPH_COORD * map);
void resample_tsp_2d_10b(void * src, int w_start, int w_end, int w_src, \
	int h_src, int s_src, void * dst, int w_dst, int i, int j, S360_SPH_COORD * map);
resample_fn resample_fp(int cs);

/* padding *******************************************************************/
#define USE_MIRROR_PADDING      1
#define PAD_SIZE                16
#define PAD_ALIGN               3
#define RISP2_PAD               8

void cpp_map_plane(int w_map, int h_map, int s_map, uint8 * map);
void pad_cpp_plane(uint8 * buf, int w, int h, int s, uint8 * map0);
void pad_cpp_plane_10b(uint16 * buf, int w, int h, int s, uint8 * map0);

/* cube face *****************************************************************/
void cmp_plane_offset(int *x, int *y, int squ_idx, int w_squ);
void rcmp_plane_offset(int *x, int *y, int squ_idx, int w_squ);
void v3d_scale_face(double eqn[3], double vec[3]);

/* vector calculation ********************************************************/
#define GET_DIST3D(x, y, z)     sqrt((x)*(x) + (y)*(y) + (z)*(z))
#define v3d_sub(v0, v1, res) \
	(res)[0] = (v0)[0] - (v1)[0]; \
	(res)[1] = (v0)[1] - (v1)[1]; \
	(res)[2] = (v0)[2] - (v1)[2]
#define v3d_scale(v, scale) \
	(v)[0] *= scale; \
	(v)[1] *= scale; \
	(v)[2] *= scale
#define v3d_dot(v0, v1, res) \
	(res)[0] = (v0)[0] * (v1)[0]; \
	(res)[1] = (v0)[1] * (v1)[1]; \
	(res)[2] = (v0)[2] * (v1)[2]
#define v3d_affine(v, scale, trans, res) \
	(res)[0] = (v)[0] * (scale) + (trans)[0]; \
	(res)[1] = (v)[1] * (scale) + (trans)[1]; \
	(res)[2] = (v)[2] * (scale) + (trans)[2]
#define v3d_average(v0, v1, res) \
	(res)[0] = ((v0)[0] + (v1)[0]) / 2; \
	(res)[1] = ((v0)[1] + (v1)[1]) / 2; \
	(res)[2] = ((v0)[2] + (v1)[2]) / 2
double v3d_norm(double v[3]);
double v3d_dot_product(double v1[3], double v2[3]);

/* isp triangle calculations *************************************************/
#define GET_W_TRI_ISP(w_erp)    ((int)((int)((w_erp) / 5.5) / 4) * 4)
#define GET_H_TRI_ISP(w_tri)    NEAREST_EVEN((w_tri) * SIN_60)

/* ohp triangle calculations *************************************************/
#define GET_W_TRI_OHP(w_erp)    ((int)((int)((w_erp) / 4) / 2) * 2)
#define GET_H_TRI_OHP(w_tri)    NEAREST_EVEN((w_tri) * SIN_60)

/* tables ********************************************************************/
extern const double tbl_squ_xyz[8][3];
extern const double tbl_squ_center_xyz[6][3];
extern const int tbl_vidx_erp2cmp[6][4];
extern double tbl_face_eqn[6][4];

extern double tbl_tri_xyz[12][3];
extern const int tbl_vidx_erp2isp[20][3];
extern const int tbl_vidx_isp2erp[20][3];
extern const int tbl_vidx_pad_isp[20][2];

extern double tbl_tri_xyz_ohp[6][3];
extern const int tbl_vidx_erp2ohp[8][3];
extern const int tbl_vidx_ohp2erp[8][3];

/*****************************************************************************
 * interface functions
 *****************************************************************************/
int s360_init(void);
void s360_deinit(void);

#include "360tools_erp.h"
#include "360tools_isp.h"
#include "360tools_cmp.h"
#include "360tools_ohp.h"
#include "360tools_tsp.h"

#ifdef __cplusplus
}
#endif


#endif /* __360TOOLS_H__218937465289347652834659823659823746592834765__ */

