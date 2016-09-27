/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2016, Qualcomm inc.
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
 *  * Neither the name of Qualcomm inc. nor the names of its
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

#include <math.h>
#include "360tools.h"
#include "360tools_tsp.h"

static int erp_to_tsp_plane(void * src, int w_src, int h_src, int s_src, \
	int w_dst, int h_dst, int s_dst, void * dst, int w_squ, int opt, int pad_size, int cs, S360_SPH_COORD  * map)
{
	void(*fn_resample)(void * src, int w_start, int w_end, int h_src, int s_src,
		double x, double y, void * dst, int x_dst);
	void(*fn_resample_tsp)(void * src, int w_start, int w_end, int w_src, int h_src, int s_src,
		void * dst, int w_dst, int x_dst, int y_dst, S360_SPH_COORD  * map);
	int sub = TSPAA_S;
	int num = sub * sub;
	unsigned int map_idx;
	double x, y;
	int i, j;
	int w_start, w_end;

	w_start = opt ? -pad_size : 0;
	w_end = opt ? w_src + pad_size : w_src;

	if (cs == S360_COLORSPACE_YUV420)
	{
		fn_resample = resample_2d;
		fn_resample_tsp = resample_tsp_2d;
	}
	else if (cs == S360_COLORSPACE_YUV420_10)
	{
		fn_resample = resample_2d_10b;
		fn_resample_tsp = resample_tsp_2d_10b;
		s_dst <<= 1;
	}

	for (j = 0; j<h_dst; j++)
	{
		for (i = 0; i<(w_dst>>1); i++)
		{
			map_idx = i * sub;
			if (map[map_idx].lng != -1)
			{
				x = (map[map_idx].lng / 360.0) * w_src;
				y = (map[map_idx].lat / 180.0) * h_src;

				fn_resample(src, w_start, w_end, h_src, s_src, x, y, dst, i);
			}
		}
		dst = (void *)((uint8 *)dst + s_dst);
		map += w_dst * num;
	}
	map -= w_dst * h_dst * num;
	dst = (void *)((uint8 *)dst - s_dst * h_dst);

	for (j = 0; j<h_dst; j++)
	{
		for (i = (w_dst>>1); i<w_dst; i++)
		{
			fn_resample_tsp(src, w_start, w_end, w_src, h_src, s_src, dst, w_dst, i, j, map);
		}
		dst = (void *)((uint8 *)dst + s_dst);
	}
	dst = (void *)((uint8 *)dst - s_dst * h_dst);

	return S360_OK;
}

int s360_erp_to_tsp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;
	int w_squ;

	w_src = img_src->width;
	h_src = img_src->height;

	w_dst = img_dst->width;
	h_dst = img_dst->height;
	w_squ = NEAREST_EVEN(w_dst / 2.0);

	s360_img_reset(img_dst);
	if (opt & S360_OPT_PAD)
		s360_pad_erp(img_src);

	if (IS_VALID_CS(img_src->colorspace))
	{
		erp_to_tsp_plane(img_src->buffer[0], w_src, h_src, img_src->stride[0], w_dst, h_dst, img_dst->stride[0], img_dst->buffer[0], w_squ, opt, PAD_SIZE, img_src->colorspace, map->layer[0]);
		w_squ >>= 1;
		w_src >>= 1;
		h_src >>= 1;
		w_dst >>= 1;
		h_dst >>= 1;
		erp_to_tsp_plane(img_src->buffer[1], w_src, h_src, img_src->stride[1], w_dst, h_dst, img_dst->stride[1], img_dst->buffer[1], w_squ, opt, PAD_SIZE >> 1, img_src->colorspace, map->layer[1]);
		erp_to_tsp_plane(img_src->buffer[2], w_src, h_src, img_src->stride[2], w_dst, h_dst, img_dst->stride[2], img_dst->buffer[2], w_squ, opt, PAD_SIZE >> 1, img_src->colorspace, map->layer[1]);
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}
	return S360_OK;
}

static int get_squ_idx(double x, double y, double z, const double center[6][3]){

	double min_dist, dist, h1, h2, h3;
	int i, idx;

	idx = 0;
	min_dist = 9999.0;

	for (i = 0; i<6; i++)
	{
		h1 = center[i][0] - x;
		h2 = center[i][1] - y;
		h3 = center[i][2] - z;
		dist = (h1 * h1 + h2 * h2 + h3 * h3);
		if (dist < min_dist)
		{
			idx = i;
			min_dist = dist;
		}
	}

	return idx;
}

/*-------------------------------   FROM TSP    -----------------------------*/
static void tsp_to_erp_sph2point(double  lng, double lat, int w_squ, double* x, \
	double* y, double d12, double d13)
{
	double	xyz[3], vec_12[3], vec_13[3], vec_1_xyz[3];
	double	d1_xyz;
	double	dist_ver, dist_hor;
	int     v_1_3d, v_2_3d, v_3_3d, v_4_3d;
	int	    squ_idx;
	double xc, yc, xp, yp;

	xyz[0] = sin(lat) * cos(lng);
	xyz[2] = sin(lat) * sin(lng);
	xyz[1] = cos(lat);

	squ_idx = get_squ_idx(xyz[0], xyz[1], xyz[2], tbl_tsp_center_xyz);
	v_1_3d = tbl_vidx_erp2tsp[squ_idx][0];
	v_2_3d = tbl_vidx_erp2tsp[squ_idx][1];
	v_3_3d = tbl_vidx_erp2tsp[squ_idx][2];
	v_4_3d = tbl_vidx_erp2tsp[squ_idx][3];

	v3d_scale_face(tbl_tspface_eqn[squ_idx], xyz);
	v3d_sub(tbl_squ_xyz[v_2_3d], tbl_squ_xyz[v_1_3d], vec_12);
	v3d_sub(tbl_squ_xyz[v_3_3d], tbl_squ_xyz[v_1_3d], vec_13);
	v3d_sub(xyz, tbl_squ_xyz[v_1_3d], vec_1_xyz);
	d1_xyz = GET_DIST3D(vec_1_xyz[0], vec_1_xyz[1], vec_1_xyz[2]);

	dist_ver = v3d_dot_product(vec_12, vec_1_xyz) / d12;
	dist_hor = v3d_dot_product(vec_13, vec_1_xyz) / d13;

	xc = dist_hor / d13;
	yc = dist_ver / d12;

	switch (squ_idx)
	{
	case 0:
		xp = 0.1875 * yc - 0.375 * xc * yc - 0.125 * xc + 0.8125;
		yp = 0.375 - 0.375 * yc;
		break;
	case 1:
		xp = 0.5 * xc;
		yp = yc;
		break;
	case 2:
		xp = 1.0 - 0.1875 * yc - 0.5 * xc + 0.375 * xc * yc;
		yp = 1.0 - 0.375 * yc;
		break;
	case 3:
		xp = 0.1875 * xc + 0.8125;
		yp = 0.25 * yc + 0.75 * xc * yc - 0.375 * xc + 0.375;
		break;
	case 4:
		xp = 0.1875 * xc + 0.5;
		yp = 0.375 * xc - 0.75 * xc * yc + yc;
		break;
	case 5:
		xp = 0.125 * xc + 0.6875;
		yp = 0.25 * yc + 0.375;
		break;
	}

	*x = xp * 2*w_squ;
	*y = yp * w_squ;
}

static int tsp_to_erp_plane(void * src, int w_src, int h_src, int s_src, \
	int w_dst, int h_dst, int s_dst, void * dst, int w_squ, int opt, int cs)
{
	void(*fn_resample)(void * src, int w_start, int w_end, int h_src, int s_src,
		double x, double y, void * dst, int x_dst);
	double	vec_12[3], vec_13[3];
	double  lng, lat, x, y;
	double	d12, d13;
	int     v_1_3d, v_2_3d, v_3_3d;
	int     i, j;

	if (cs == S360_COLORSPACE_YUV420)
	{
		fn_resample = resample_2d;
	}
	else if (cs == S360_COLORSPACE_YUV420_10)
	{
		fn_resample = resample_2d_10b;
		s_dst <<= 1;
	}

	v_1_3d = tbl_vidx_erp2tsp[0][0];
	v_2_3d = tbl_vidx_erp2tsp[0][1];
	v_3_3d = tbl_vidx_erp2tsp[0][2];

	v3d_sub(tbl_squ_xyz[v_2_3d], tbl_squ_xyz[v_1_3d], vec_12);
	d12 = GET_DIST3D(vec_12[0], vec_12[1], vec_12[2]);
	v3d_sub(tbl_squ_xyz[v_3_3d], tbl_squ_xyz[v_1_3d], vec_13);
	d13 = GET_DIST3D(vec_13[0], vec_13[1], vec_13[2]);

	for (j = 0; j<h_dst; j++)
	{
		for (i = 0; i<w_dst; i++)
		{
			lng = M_2PI * i / w_dst;
			lat = PI * j / h_dst;

			tsp_to_erp_sph2point(lng, lat, w_squ, &x, &y, d12, d13);
			fn_resample(src, 0, w_src, h_src, s_src, x, y, dst, i);
		}
		dst = (void *)((uint8 *)dst + s_dst);
	}

	return S360_OK;
}

int s360_tsp_to_erp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;
	int w_squ;

	w_src = img_src->width;
	h_src = img_src->height;

	w_dst = img_dst->width;
	h_dst = img_dst->height;
	w_squ = NEAREST_EVEN(w_src / 2.0);

	if (IS_VALID_CS(img_src->colorspace))
	{
		tsp_to_erp_plane(img_src->buffer[0], w_src, h_src, img_src->stride[0], w_dst, h_dst, img_dst->stride[0], img_dst->buffer[0], w_squ, opt, img_src->colorspace);
		w_squ >>= 1;
		w_src >>= 1;
		h_src >>= 1;
		w_dst >>= 1;
		h_dst >>= 1;
		tsp_to_erp_plane(img_src->buffer[1], w_src, h_src, img_src->stride[1], w_dst, h_dst, img_dst->stride[1], img_dst->buffer[1], w_squ, opt, img_src->colorspace);
		tsp_to_erp_plane(img_src->buffer[2], w_src, h_src, img_src->stride[2], w_dst, h_dst, img_dst->stride[2], img_dst->buffer[2], w_squ, opt, img_src->colorspace);
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}

	return S360_OK;
}

static int tsp_to_cpp_plane(void * src, int w_src, int h_src, int s_src, \
	int w_dst, int h_dst, int s_dst, void * dst, int w_squ, int opt, int cs)
{
	void(*fn_resample)(void * src, int w_start, int w_end, int h_src, int s_src,
		double x, double y, void * dst, int x_dst);

	double   vec_12[3], vec_13[3];
	double   lng, lat, x, y;
	double   d12, d13;
	int      v_1_3d, v_2_3d, v_3_3d;
	int      i, j;
	uint8  * map, *map0;

	if (cs == S360_COLORSPACE_YUV420)
	{
		fn_resample = resample_2d;
	}
	else if (cs == S360_COLORSPACE_YUV420_10)
	{
		fn_resample = resample_2d_10b;
		s_dst <<= 1;
	}

	map = (uint8 *)s360_malloc(sizeof(uint8) * h_dst * w_dst);
	cpp_map_plane(w_dst, h_dst, s_dst, map);
	map0 = map;

	v_1_3d = tbl_vidx_erp2tsp[0][0];
	v_2_3d = tbl_vidx_erp2tsp[0][1];
	v_3_3d = tbl_vidx_erp2tsp[0][2];

	v3d_sub(tbl_squ_xyz[v_2_3d], tbl_squ_xyz[v_1_3d], vec_12);
	d12 = GET_DIST3D(vec_12[0], vec_12[1], vec_12[2]);
	v3d_sub(tbl_squ_xyz[v_3_3d], tbl_squ_xyz[v_1_3d], vec_13);
	d13 = GET_DIST3D(vec_13[0], vec_13[1], vec_13[2]);

	for (j = 0; j<h_dst; j++)
	{
		for (i = 0; i<w_dst; i++)
		{
			lng = ((double)i / w_dst)*(M_2PI)-PI;
			lat = ((double)j / h_dst)*PI - M_PI_2;

			lat = 3 * asin(lat / PI);
			lng = lng / (2 * cos(2 * lat / 3) - 1);

			lat += M_PI_2;
			lng += PI;

			if (*(map0 + i) != 1)
				continue;

			tsp_to_erp_sph2point(lng, lat, w_squ, &x, &y, d12, d13);
			fn_resample(src, 0, w_src, h_src, s_src, x, y, dst, i);
		}

		dst = (void *)((uint8 *)dst + s_dst);
		map0 += w_dst;
	}
	s360_mfree(map);
	return S360_OK;
}

int s360_tsp_to_cpp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;
	int w_squ;

	w_src = img_src->width;
	h_src = img_src->height;

	w_dst = img_dst->width;
	h_dst = img_dst->height;
	w_squ = NEAREST_EVEN(w_src / 2.0);
	
	s360_img_reset(img_dst);

	if (IS_VALID_CS(img_src->colorspace))
	{
		tsp_to_cpp_plane(img_src->buffer[0], w_src, h_src, img_src->stride[0], w_dst, h_dst, img_dst->stride[0], img_dst->buffer[0], w_squ, opt, img_src->colorspace);
		w_squ >>= 1;
		w_src >>= 1;
		h_src >>= 1;
		w_dst >>= 1;
		h_dst >>= 1;
		tsp_to_cpp_plane(img_src->buffer[1], w_src, h_src, img_src->stride[1], w_dst, h_dst, img_dst->stride[1], img_dst->buffer[1], w_squ, opt, img_src->colorspace);
		tsp_to_cpp_plane(img_src->buffer[2], w_src, h_src, img_src->stride[2], w_dst, h_dst, img_dst->stride[2], img_dst->buffer[2], w_squ, opt, img_src->colorspace);
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}

	return S360_OK;
}
