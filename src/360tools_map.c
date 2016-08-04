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

#include "360tools.h"
#include "360tools_map.h"

static int map_erp_cpp_to_cmp(S360_MAP * map, int opt);
static void init_sph2cmp(S360_SPH_COORD  * map, int w_dst, int h_dst, int w_squ);
static void erp2cmp_squ(int w_dst, int h_dst, int squ_idx, S360_SPH_COORD * map, int w_squ);

static int map_erp_cpp_to_isp(S360_MAP * map, int opt);
static void init_sph2isp_map(S360_SPH_COORD * map, int w_isp, int h_isp, int w_tri, int pad_sz);
static void erp2isp_triangle(int x, int w, int w_tri, int h_tri, \
	int tri_idx, S360_SPH_COORD * map, int pad_sz);
static void erp2isp_triangle_rev(int x, int w, int w_tri, int h_tri, \
	int tri_idx, S360_SPH_COORD * map, int pad_sz);

static int(*map_func)(S360_MAP * map, int opt);
static void map_init(S360_SPH_COORD * map, int w_dst, int h_dst);

S360_MAP * s360_map_create(int w_src, int h_src, int w_dst, int h_dst, int cfmt, int opt)
{
	S360_MAP * map = NULL;
	if (cfmt == CONV_FMT_ERP_TO_CMP || cfmt == CONV_FMT_ERP_TO_ISP)
	{
		map = s360_malloc(sizeof(S360_MAP));
		s360_assert_rv(map, NULL);

		map->width = w_dst;
		map->height = h_dst;

		map->layer[0] = s360_malloc((map->width)*(map->height)*sizeof(S360_SPH_COORD));
		map->layer[1] = s360_malloc((map->width >> 1)*(map->height >> 1)*sizeof(S360_SPH_COORD));

		map_init(map->layer[0], map->width, map->height);
		map_init(map->layer[1], (map->width >> 1), (map->height >> 1));

		switch (cfmt)
		{
		case CONV_FMT_ERP_TO_CMP:
			map_func = map_erp_cpp_to_cmp;
			break;
		case CONV_FMT_ERP_TO_ISP:
			map_func = map_erp_cpp_to_isp;
			break;
		}

		map_func(map, opt);
	}

	return map;
}

void map_init(S360_SPH_COORD * map, int w, int h)
{
	int i, size;

	for (i=0, size=w*h; i<size; i++)
	{
		map[i].lng = -1;
		map[i].lat = -1;
	}
}

void s360_map_delete(S360_MAP * map)
{
	s360_assert_r(map);

	if (map->layer[0]) s360_mfree(map->layer[0]);
	if (map->layer[1]) s360_mfree(map->layer[1]);
	s360_mfree(map);
}

/* -----------------------------------ERP TO ISP----------------------------------------*/
static int map_erp_cpp_to_isp(S360_MAP * map, int opt)
{
	int w_dst, h_dst, w_tri, h_tri, pad_sz;
	w_dst = map->width;
	h_dst = map->height;
	w_tri = GET_W_TRI_ISP(w_dst);
	h_tri = GET_H_TRI_ISP(w_tri);

	s360_assert_rv(w_dst == w_tri * 5.5, S360_ERR_INVALID_DIMENSION);
	s360_assert_rv(h_dst == 3 * h_tri, S360_ERR_INVALID_DIMENSION);

	if (opt & S360_OPT_PAD)
	{
		pad_sz = PAD_SIZE;
	}
	else
	{
		pad_sz = 0;
	}

	init_sph2isp_map(map->layer[0], w_dst, h_dst, w_tri, pad_sz);
	init_sph2isp_map(map->layer[1], w_dst >> 1, h_dst >> 1, w_tri >> 1, pad_sz >> 1);

	return S360_OK;
}

static void init_sph2isp_map(S360_SPH_COORD * map, int w_isp, int h_isp, int w_tri, int pad_sz)
{
	int i, x, y1, y2, h_tri, size;

	for (i = 0, size = w_isp*h_isp; i<size; i++)
	{
		map[i].lng = -1;
		map[i].lat = -1;
	}

	h_tri = (h_isp / 3);
	y1 = h_tri;
	y2 = (h_tri << 1);

	/* first row */
	for (i = 0, x = 0; i<5; i++, x += w_tri)
	{
		erp2isp_triangle(x, w_isp, w_tri, y1, i, map, pad_sz);
	}

	/* second row */
	map += y1 * w_isp;
	for (i = 5, x = (w_tri >> 1); i<10; i++, x += w_tri)
	{
		erp2isp_triangle(x, w_isp, w_tri, h_tri, i, map, pad_sz);
	}
	for (i = 0, x = 0; i<5; i++, x += w_tri)
	{
		erp2isp_triangle_rev(x, w_isp, w_tri, h_tri, i, map, pad_sz);
	}

	/* third row */
	map += (y2 - y1) * w_isp;
	for (i = 5, x = (w_tri >> 1); i<10; i++, x += w_tri)
	{
		erp2isp_triangle_rev(x, w_isp, w_tri, h_tri, i, map, pad_sz);
	}
}

static void erp2isp_triangle(int x, int w, int w_tri, int h_tri, \
	int tri_idx, S360_SPH_COORD * map, int pad_sz)
{
	double  vector_12[3], vector_13[3], point12[3], point13[3];
	double  point_vec[3], xyz[3];
	double  d12, d13, d12_scl, d13_scl, point_d;
	double  off;
	double  d_step;
	double  w_cur, w_start;
	int     v_1_3d, v_2_3d, v_3_3d;
	int     x_map, l;
	int     i, j;

	int     pad_left, pad_rt;
	int     start_w, end_w;
	int     x_start;

	x_start = x;

	x += (w_tri >> 1);

	if (pad_sz != 0)
	{
		pad_left = tbl_vidx_pad_isp[tri_idx][0];
		pad_rt = tbl_vidx_pad_isp[tri_idx][1];
	}

	v_1_3d = tbl_vidx_erp2isp[tri_idx][0];
	v_2_3d = tbl_vidx_erp2isp[tri_idx][1];
	v_3_3d = tbl_vidx_erp2isp[tri_idx][2];

	v3d_sub(tbl_tri_xyz[v_2_3d], tbl_tri_xyz[v_1_3d], vector_12);
	d12 = v3d_norm(vector_12);

	v3d_sub(tbl_tri_xyz[v_3_3d], tbl_tri_xyz[v_1_3d], vector_13);
	d13 = v3d_norm(vector_13);

	for (j = 0; j<h_tri; j++)
	{
		w_cur = (double)(w_tri * j) / h_tri;
		l = CEILING((w_cur - 1) / 2);
		w_start = (x + 0.5 - w_cur / 2);
		off = CEILING(w_start) - w_start;
		x_map = CEILING(w_start) - 1;
		d12_scl = d12 * ((double)j / h_tri);
		d13_scl = d13 * ((double)j / h_tri);

		v3d_affine(vector_12, d12_scl, tbl_tri_xyz[v_1_3d], point12);
		v3d_affine(vector_13, d13_scl, tbl_tri_xyz[v_1_3d], point13);

		v3d_sub(point13, point12, point_vec);

		point_d = v3d_norm(point_vec);
		d_step = (w_cur == 0 ? 0 : point_d / w_cur);
		start_w = 0;
		end_w = (l << 1);

		if (pad_left)
		{
			start_w -= pad_sz;
			start_w = (start_w + x_map < x_start) ? (x_start - x_map) : start_w;
			for (i = start_w; i<0; i++)
			{
				v3d_affine(point_vec, (i - 1) * d_step + d_step / 2, point12, xyz);
				v3d_norm(xyz);
				cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map + i]);
			}
		}


		{
			v3d_affine(point_vec, d_step * off / 2, point12, xyz);
			v3d_norm(xyz);
			cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map]);
			start_w++;
		}


		if (pad_rt)
		{
			end_w += pad_sz;
			if (x_map + end_w > w) end_w = w - x_map;
		}

		v3d_affine(point_vec, d_step * off, point12, point12);
		for (i = 1; i<(l << 1); i++)
		{
			v3d_affine(point_vec, (i - 1) * d_step + d_step / 2, point12, xyz);
			v3d_norm(xyz);
			cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map + i]);
		}

		{
			v3d_affine(point_vec, -d_step * off / 2, point13, xyz);
			v3d_norm(xyz);
			cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map + i]);
		}

		if (pad_rt)
		{
			i++;
			for (; i<end_w; i++)
			{
				v3d_affine(point_vec, (i - 1) * d_step + d_step / 2, point12, xyz);
				v3d_norm(xyz);
				cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map + i]);
			}
		}


		map += w;
	}
}

static void erp2isp_triangle_rev(int x, int w, int w_tri, int h_tri, \
	int tri_idx, S360_SPH_COORD * map, int pad_sz)
{
	double  vector_12[3], vector_13[3], point12[3], point13[3];
	double  point_vec[3], xyz[3];
	double  d12, d13, d12_scl, d13_scl, point_d;
	double  off;
	double  d_step;
	double  w_cur, w_start;
	int     v_1_3d, v_2_3d, v_3_3d;
	int     x_map, l;
	int     i, j;

	int     pad_left, pad_rt;
	int     start_w, end_w;
	int     x_start;

	pad_left = tbl_vidx_pad_isp[tri_idx + 10][0];
	pad_rt = tbl_vidx_pad_isp[tri_idx + 10][1];
	x_start = x;

	v_1_3d = tbl_vidx_erp2isp[tri_idx + 10][0];
	v_2_3d = tbl_vidx_erp2isp[tri_idx + 10][1];
	v_3_3d = tbl_vidx_erp2isp[tri_idx + 10][2];

	v3d_sub(tbl_tri_xyz[v_3_3d], tbl_tri_xyz[v_1_3d], vector_12);
	d12 = v3d_norm(vector_12);

	v3d_sub(tbl_tri_xyz[v_3_3d], tbl_tri_xyz[v_2_3d], vector_13);
	d13 = v3d_norm(vector_13);

	for (i = 0; i<h_tri; i++)
	{
		w_cur = ((double)(h_tri - i) * w_tri) / h_tri;
		l = CEILING((w_cur - 1) / 2);
		w_start = (x + 0.5 - w_cur / 2);
		off = CEILING(w_start) - w_start;
		x_map = CEILING(w_start + (w_tri >> 1)) - 1;
		d12_scl = d12 * ((double)i / h_tri);
		d13_scl = d13 * ((double)i / h_tri);

		v3d_affine(vector_12, d12_scl, tbl_tri_xyz[v_1_3d], point12);
		v3d_affine(vector_13, d13_scl, tbl_tri_xyz[v_2_3d], point13);

		v3d_sub(point13, point12, point_vec);

		point_d = v3d_norm(point_vec);
		d_step = (w_cur == 0 ? 0 : point_d / w_cur);
		start_w = 0;
		end_w = (l << 1);

		if (pad_left)
		{
			start_w -= pad_sz;
			if (tri_idx + 10 != 15)
				start_w = (start_w + x_map < x_start) ? (x_start - x_map) : start_w;
			for (j = start_w; j<0; j++)
			{
				v3d_affine(point_vec, (j - 1) * d_step + d_step / 2, point12, xyz);
				v3d_norm(xyz);
				cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map + j]);
			}

		}

		{
			v3d_affine(point_vec, d_step * off / 2, point12, xyz);
			v3d_norm(xyz);
			cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map]);
			start_w++;
		}

		v3d_affine(point_vec, d_step * off, point12, point12);
		for (j = 1; j<(l << 1); j++)
		{
			v3d_affine(point_vec, (j - 1) * d_step + d_step / 2, point12, xyz);
			v3d_norm(xyz);
			cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map + j]);
		}

		{
			v3d_affine(point_vec, -d_step * off / 2, point13, xyz);
			v3d_norm(xyz);
			cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map + j]);
		}

		if (pad_rt)
		{
			end_w += pad_sz;
			if (x_map + end_w > w) end_w = w - x_map;
			j++;
			for (; j<end_w; j++)
			{
				v3d_affine(point_vec, (j - 1) * d_step + d_step / 2, point12, xyz);
				v3d_norm(xyz);
				cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map + j]);
			}
		}

		map += w;
	}
}

/* -----------------------------------ERP TO CMP----------------------------------------*/
static int map_erp_cpp_to_cmp(S360_MAP * map, int opt)
{
	int w_dst, h_dst, w_squ;

	w_dst = map->width;
	h_dst = map->height;
	w_squ = NEAREST_EVEN(w_dst / 4.0);

	init_sph2cmp(map->layer[0], w_dst, h_dst, w_squ);
	init_sph2cmp(map->layer[1], (w_dst >> 1), (h_dst >> 1), (w_squ >> 1));

	return S360_OK;
}

static void init_sph2cmp(S360_SPH_COORD  * map, int w_dst, int h_dst, int w_squ)
{
	int i;

	for (i = 0; i<6; i++)
	{
		erp2cmp_squ(w_dst, h_dst, i, map, w_squ);
	}
}

static void erp2cmp_squ(int w_dst, int h_dst, int squ_idx, S360_SPH_COORD * map, int w_squ)
{
	double  vector_12[3], vector_13[3];
	double  d12, d13, d12_scl, d13_scl;
	double  xyz[3];
	int     v_1_3d, v_2_3d, v_3_3d, v_4_3d;
	int     i, j, x_init, y_init;

	S360_SPH_COORD  coord;

	v_1_3d = tbl_vidx_erp2cmp[squ_idx][0];
	v_2_3d = tbl_vidx_erp2cmp[squ_idx][1];
	v_3_3d = tbl_vidx_erp2cmp[squ_idx][2];
	v_4_3d = tbl_vidx_erp2cmp[squ_idx][3];

	v3d_sub(tbl_squ_xyz[v_2_3d], tbl_squ_xyz[v_1_3d], vector_12);
	d12 = v3d_norm(vector_12);
	v3d_sub(tbl_squ_xyz[v_3_3d], tbl_squ_xyz[v_1_3d], vector_13);
	d13 = v3d_norm(vector_13);

	cmp_plane_offset(&x_init, &y_init, squ_idx, w_squ);

	map += y_init*w_dst + x_init;

	for (i = 0; i<w_squ; i++)
	{
		for (j = 0; j<w_squ; j++)
		{
			d12_scl = d12 * (i / (double)w_squ);
			d13_scl = d13 * (j / (double)w_squ);
			v3d_affine(vector_12, d12_scl, tbl_squ_xyz[v_1_3d], xyz);
			v3d_affine(vector_13, d13_scl, xyz, xyz);
			v3d_norm(xyz);
			cart_to_sph(xyz[0], xyz[2], xyz[1], &coord);
			map[j] = coord;
		}
		map += w_dst;
	}
}

