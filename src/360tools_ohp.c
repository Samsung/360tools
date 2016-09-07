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


#include <math.h>
#include "360tools.h"
#include "360tools_ohp.h"

static void fill_pad_buf(S360_SPH_COORD * map, uint8 * ohp, int x_dst, uint8 * buf, int x_buf, int dir, int w)
{
	uint8 val;
	int i, pad_size;

	if(dir == 1)
	{
		pad_size = S360_MIN(PAD_SIZE, w - x_dst);
		for(i=0; i<pad_size; i++)
		{
			if(map[x_dst + i].lng != -1)
			{
				val = ohp[x_dst + i];
			}
			buf[x_buf + i] = val;
		}
	}
	else
	{
		pad_size = S360_MIN(PAD_SIZE, x_dst);
		for(i=0; i<pad_size; i++)
		{
			if(map[x_dst - i].lng != -1)
			{
				val = ohp[x_dst - i];
			}
			buf[x_buf + i] = val;
		}
	}
}

static void pad_from_buf(S360_SPH_COORD * map, uint8 * dst, int x_dst, uint8 * buf, int x_buf, int dir, int w)
{
	int i, pad, pad_max;

	pad_max = S360_MIN(x_dst, w - x_dst);
	pad = S360_MIN(PAD_SIZE, pad_max);

	buf += x_buf;

	if(dir == 1)
	{
		for(i=0; i<pad; i++)
		{
			if(map[x_dst + i].lng != -1) break;
			dst[x_dst + i] = buf[i];
		}
	}
	else
	{
		for(i=0; i<pad; i++)
		{
			if(map[x_dst - i].lng != -1) break;
			dst[x_dst - i] = buf[i];
		}
	}
}

static int pad_ohp_plane(uint8 * ohp, int w, int h, int s, S360_SPH_COORD * map)
{
	uint8     * buf0, * buf;
	uint8     * cnt;
	int         i, j, k, x_buf;

	buf0 = (uint8 *)s360_malloc(sizeof(uint8) * h * PAD_SIZE * 8);
	if(buf0 == NULL) return S360_ERR_OUT_OF_MEMORY;

	cnt = (uint8 *)s360_malloc(sizeof(uint8) * h);
	if(cnt == NULL) return S360_ERR_OUT_OF_MEMORY;

	buf = buf0;
	for(j=0; j<h; j++)
	{
		for(i=0, k=0; i<w; i++)
		{
			if(map[i].lng != -1)
			{
				x_buf = (k << 1) * PAD_SIZE;
				fill_pad_buf(map, ohp, i, buf, x_buf, 1, w);

				while(map[i].lng != -1 && i < w) i++;
				fill_pad_buf(map, ohp, i-1, buf, x_buf + PAD_SIZE, -1, w);

				k++;
			}
		}
		cnt[j] = k;

		buf += PAD_SIZE * 8;
		map += w;
		ohp += s;
	}

	ohp -= s * h;
	map -= w * h;
	buf  = buf0;
	for(j=0; j<h; j++)
	{
		i = 0;
		if(cnt[j] == 4)
		{
			for(k=0; k<4; k++)
			{
				while(map[i].lng == -1 && i < w) i++;
				x_buf = ((8 + 2 * k - 1) % 8) * PAD_SIZE;
				pad_from_buf(map, ohp, i-1, buf, x_buf, -1, w);

				while(map[i].lng != -1 && i < w) i++;
				x_buf = ((2 * k + 2) % 8) * PAD_SIZE;
				pad_from_buf(map, ohp, i, buf, x_buf, 1, w);
			}
		}
		else
		{
			while(map[i].lng == -1 && i < w) i++;
			pad_from_buf(map, ohp, i-1, buf, PAD_SIZE, -1, w);

			while(map[i].lng != -1 && i < w) i++;
			pad_from_buf(map, ohp, i, buf, 0, 1, w);
		}

		buf += PAD_SIZE * 8;
		map += w;
		ohp += s;
	}

	s360_mfree(buf0);
	s360_mfree(cnt);

	return S360_OK;
}

static void fill_pad_buf_10b(S360_SPH_COORD * map, uint16 * ohp, int x_dst, uint16 * buf, int x_buf, int dir, int w)
{
	uint16 val;
	int i, pad_size;

	if(dir == 1)
	{
		pad_size = S360_MIN(PAD_SIZE, w - x_dst);
		for(i=0; i<pad_size; i++)
		{
			if(map[x_dst + i].lng != -1)
			{
				val = ohp[x_dst + i];
			}
			buf[x_buf + i] = val;
		}
	}
	else
	{
		pad_size = S360_MIN(PAD_SIZE, x_dst);
		for(i=0; i<pad_size; i++)
		{
			if(map[x_dst - i].lng != -1)
			{
				val = ohp[x_dst - i];
			}
			buf[x_buf + i] = val;
		}
	}
}

static void pad_from_buf_10b(S360_SPH_COORD * map, uint16 * dst, int x_dst, uint16 * buf, int x_buf, int dir, int w)
{
	int i, pad, pad_max;

	pad_max = S360_MIN(x_dst, w - x_dst);
	pad = S360_MIN(PAD_SIZE, pad_max);

	buf += x_buf;

	if(dir == 1)
	{
		for(i=0; i<pad; i++)
		{
			if(map[x_dst + i].lng != -1) break;
			dst[x_dst + i] = buf[i];
		}
	}
	else
	{
		for(i=0; i<pad; i++)
		{
			if(map[x_dst - i].lng != -1) break;
			dst[x_dst - i] = buf[i];
		}
	}
}

static int pad_ohp_plane_10b(uint16 * ohp, int w, int h, int s, S360_SPH_COORD * map)
{
	uint16     * buf0, * buf;
	uint16     * cnt;
	int         i, j, k, x_buf;

	buf0 = (uint16 *)s360_malloc(sizeof(uint16) * h * PAD_SIZE * 8);
	if(buf0 == NULL) return S360_ERR_OUT_OF_MEMORY;

	cnt = (uint16 *)s360_malloc(sizeof(uint16) * h);
	if(cnt == NULL) return S360_ERR_OUT_OF_MEMORY;

	buf = buf0;
	for(j=0; j<h; j++)
	{
		for(i=0, k=0; i<w; i++)
		{
			if(map[i].lng != -1)
			{
				x_buf = (k << 1) * PAD_SIZE;
				fill_pad_buf_10b(map, ohp, i, buf, x_buf, 1, w);

				while(map[i].lng != -1 && i < w) i++;
				fill_pad_buf_10b(map, ohp, i-1, buf, x_buf + PAD_SIZE, -1, w);

				k++;
			}
		}
		cnt[j] = k;

		buf += PAD_SIZE * 8;
		map += w;
		ohp += s;
	}

	ohp -= s * h;
	map -= w * h;
	buf  = buf0;
	for(j=0; j<h; j++)
	{
		i = 0;
		if(cnt[j] == 4)
		{
			for(k=0; k<4; k++)
			{
				while(map[i].lng == -1 && i < w) i++;
				x_buf = ((8 + 2 * k - 1) % 8) * PAD_SIZE;
				pad_from_buf_10b(map, ohp, i-1, buf, x_buf, -1, w);

				while(map[i].lng != -1 && i < w) i++;
				x_buf = ((2 * k + 2) % 8) * PAD_SIZE;
				pad_from_buf_10b(map, ohp, i, buf, x_buf, 1, w);
			}
		}
		else
		{
			while(map[i].lng == -1 && i < w) i++;
			pad_from_buf_10b(map, ohp, i-1, buf, PAD_SIZE, -1, w);

			while(map[i].lng != -1 && i < w) i++;
			pad_from_buf_10b(map, ohp, i, buf, 0, 1, w);
		}

		buf += PAD_SIZE * 8;
		map += w;
		ohp += s;
	}

	s360_mfree(buf0);
	s360_mfree(cnt);

	return S360_OK;
}

static int erp_to_ohp_plane(void * src, int w_src, int h_src, int s_src, \
	int w_dst, int h_dst, int s_dst, void * dst, int w_tri, int opt, int cs, int pad_sz, S360_SPH_COORD * map)
{
    void  (*fn_resample)(void * src, int w_start, int w_end, int h_src, int s_src,
							double x, double y, void * dst, int x_dst);
    double      x, y;
	int         i, j;
	int         w_start, w_end;

	w_start = opt ? -pad_sz : 0;
	w_end = opt ? w_src + pad_sz : w_src;

    if(cs == S360_COLORSPACE_YUV420)
	{
		fn_resample = resample_2d;
	}
	else if(cs == S360_COLORSPACE_YUV420_10)
	{
		fn_resample = resample_2d_10b;
		s_dst <<= 1;
	}

	for(j=0; j<h_dst; j++)
	{
		for(i=0; i<w_dst; i++)
		{
			if(map[i].lng != -1)
			{
				x = (map[i].lng / 360) * w_src;
				y = (map[i].lat / 180) * h_src;

				fn_resample(src, w_start, w_end, h_src, s_src, x, y, dst, i);
			}
		}
		dst = (void *)((uint8 *)dst + s_dst);
		map += w_dst;
	}
	map -= w_dst * h_dst;
	dst = (void *)((uint8 *)dst - s_dst * h_dst);

	if(opt & S360_OPT_PAD)
	{
		if(cs == S360_COLORSPACE_YUV420)
		{
			pad_ohp_plane((uint8*)dst, w_dst, h_dst, s_dst, map);
		}
		else if(cs == S360_COLORSPACE_YUV420_10)
		{
			pad_ohp_plane_10b((uint16*)dst, w_dst, h_dst, (s_dst>>1), map);
		}
	}

	return S360_OK;
}

int s360_erp_to_ohp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;
	int w_tri, h_tri;
	int ret;
	int pad_sz;

	w_src = img_src->width;
	h_src = img_src->height;
	
	w_dst = img_dst->width;
	h_dst = img_dst->height;

	w_tri = GET_W_TRI_OHP(w_dst);
	h_tri = GET_H_TRI_OHP(w_tri);

	s360_assert_rv(w_dst == w_tri * 4, S360_ERR_INVALID_DIMENSION);
	s360_assert_rv(h_dst == 2 * h_tri, S360_ERR_INVALID_DIMENSION);

	if(h_dst > img_dst->height) return S360_ERR_INVALID_ARGUMENT;
	
	ret = s360_img_realloc(img_dst, w_dst, h_dst, opt);
	s360_assert_rv(ret == S360_OK, ret);

	s360_img_reset(img_dst);

	pad_sz = (opt & S360_OPT_PAD)?PAD_SIZE:0;

	if(opt & S360_OPT_PAD)
		s360_pad_erp(img_src);

	if(IS_VALID_CS(img_src->colorspace))
	{
		erp_to_ohp_plane(img_src->buffer[0], w_src, h_src, img_src->stride[0], w_dst, h_dst, \
			img_dst->stride[0], img_dst->buffer[0], w_tri, opt, img_src->colorspace, pad_sz, map->layer[0]);

		w_tri >>= 1;
		w_src >>= 1;
		h_src >>= 1;
		w_dst >>= 1;
		h_dst >>= 1;
		pad_sz >>= 1;
		erp_to_ohp_plane(img_src->buffer[1], w_src, h_src, img_src->stride[1], w_dst, h_dst, \
			img_dst->stride[1], img_dst->buffer[1], w_tri, opt, img_src->colorspace, pad_sz, map->layer[1]);
		erp_to_ohp_plane(img_src->buffer[2], w_src, h_src, img_src->stride[2], w_dst, h_dst, \
			img_dst->stride[2], img_dst->buffer[2], w_tri, opt, img_src->colorspace, pad_sz, map->layer[1]);
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}

	return S360_OK;
}

double tbl_center_xyz_ohp[8][3] =
{
	{   0.333333,	0.333333,   0.333333 },
	{   -0.333333,   0.333333,   0.333333 },
	{   -0.333333,  - 0.333333,   0.333333 },
	{   0.333333,   -0.333333,   0.333333 },
	{   0.333333,   0.333333,   -0.333333 },
	{   -0.333333,   0.333333,   -0.333333 },
	{   -0.333333,   -0.333333,   -0.333333 },
	{   0.333333,   -0.333333,   -0.333333 }
};

static int get_tri_idx(double x, double y, double z, double center[][3])
{
	double min_dist, dist, h1, h2, h3;
	int i, idx;

	idx = 0;
	min_dist = 9999.0;

	for(i=0; i<8; i++)
	{
		h1 = center[i][0] - x;
		h2 = center[i][1] - y;
		h3 = center[i][2] - z;
		dist = (h1 * h1 + h2 * h2 + h3 * h3);
		if(dist < min_dist)
		{
			idx = i;
			min_dist = dist;
		}
	}

	return idx;
}
static int ohp_to_erp_plane(void * src, int w_src, int h_src, int s_src, \
	int w_dst, int h_dst, int s_dst, void * dst, int cs)
{
    void  (*fn_resample)(void * src, int w_start, int w_end, int h_src, int s_src,
							double x, double y, void * dst, int x_dst);
	double  n1[3], n2[3], normal[3], mid[3], xyz[3], t_vec[3];
	double  lng, lat, x, y, u;
	double  h_tri_3d, w_tri_3d;
	double  dist_23, dist_cmp1, dist_cmp2, d_ver, d_hor;
	int     tri, w_tri, h_tri, vertex_x, vertex_y;
	int     y1, y2;
	int     v1, v2, v3;
	int     i, j;

    if(cs == S360_COLORSPACE_YUV420)
	{
		fn_resample = resample_2d;
	}
	else if(cs == S360_COLORSPACE_YUV420_10)
	{
		fn_resample = resample_2d_10b;
		s_dst <<= 1;
	}

	w_tri = (int)(w_src / 4);
	h_tri = (int)(h_src / 2);
	y1    = h_tri;
	y2    = (h_tri * 2);

	v3d_average(tbl_tri_xyz_ohp[1], tbl_tri_xyz_ohp[2], mid);

	v3d_sub(tbl_tri_xyz_ohp[0], mid, t_vec);
	h_tri_3d = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);

	for(j=0; j<h_dst; j++)
	{
		for(i=0; i<w_dst; i++)
		{
			lng = DEG2RAD(360) * i / w_dst;
			lat = DEG2RAD(180) * j / h_dst;
			xyz[0] = sin(lat) * cos(lng);
			xyz[1] = sin(lat) * sin(lng);
			xyz[2] = cos(lat);

			tri = get_tri_idx(xyz[0], xyz[1], xyz[2], tbl_center_xyz_ohp);
			v1 = tbl_vidx_ohp2erp[tri][0];
			v2 = tbl_vidx_ohp2erp[tri][1];
			v3 = tbl_vidx_ohp2erp[tri][2];

			v3d_sub(tbl_tri_xyz_ohp[v1], tbl_tri_xyz_ohp[v2], n1);
			v3d_sub(tbl_tri_xyz_ohp[v1], tbl_tri_xyz_ohp[v3], n2);

			normal[0] = -(n1[1] * n2[2] - n2[1] * n1[2]);
			normal[1] = -(n1[2] * n2[0] - n2[2] * n1[0]);
			normal[2] = -(n1[0] * n2[1] - n2[0] * n1[1]);

			v3d_dot(normal, xyz, n1);
			v3d_dot(normal, tbl_tri_xyz_ohp[v1], n2);

			v3d_scale(xyz, (n2[0] + n2[1] + n2[2]) / (n1[0] + n1[1] + n1[2]));

			v3d_sub(tbl_tri_xyz_ohp[v3], tbl_tri_xyz_ohp[v2], n1);
			dist_23 = n1[0] * n1[0] + n1[1] * n1[1] + n1[2] * n1[2];

			v3d_sub(xyz, tbl_tri_xyz_ohp[v2], t_vec);
			v3d_dot(t_vec, n1, t_vec);
			u = (t_vec[0] + t_vec[1] + t_vec[2]) / dist_23;

			v3d_average(tbl_tri_xyz_ohp[v2], tbl_tri_xyz_ohp[v3], mid);

			v3d_sub(tbl_tri_xyz_ohp[v2], tbl_tri_xyz_ohp[v3], t_vec);
			w_tri_3d = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);
			v3d_sub(tbl_tri_xyz_ohp[v1], mid, t_vec);
			h_tri_3d = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);

			v3d_affine(n1, u, tbl_tri_xyz_ohp[v2], n2);

			v3d_sub(mid, n2, t_vec);
			d_hor = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);
			v3d_sub(xyz, n2, t_vec);
			d_ver = h_tri_3d - GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);

			x = (w_tri / w_tri_3d) * d_hor;
			y = (h_tri / h_tri_3d) * d_ver;

			v3d_sub(tbl_tri_xyz_ohp[v2], n2, t_vec);
			dist_cmp1 = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);
			v3d_sub(tbl_tri_xyz_ohp[v3], n2, t_vec);
			dist_cmp2 = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);

			switch(tri)
			{
			case 0:
				vertex_x = (w_tri >> 1);
				vertex_y = 0;
				break;
			case 1:
				vertex_x = (w_tri >> 1) + w_tri;
				vertex_y = 0;
				break;
			case 2:
				vertex_x = (w_tri >> 1) + (w_tri << 1);
				vertex_y = 0;
				break;
			case 3:
				vertex_x = (w_tri >> 1) + 3 * w_tri;
				vertex_y = 0;
				break;

			case 4:
				vertex_x = (w_tri >> 1);
				vertex_y = y2;
				break;
			case 5:
				vertex_x = (w_tri >> 1) + w_tri;
				vertex_y = y2;
				break;
			case 6:
				vertex_x = (w_tri >> 1) + (w_tri << 1);
				vertex_y = y2;
				break;
			case 7:
				vertex_x = (w_tri >> 1) + 3 * w_tri;
				vertex_y = y2;
				break;
			
			}
			if(vertex_y >= y2)
			{
				y = -y;
			}

			if(dist_cmp1 > dist_cmp2)
			{
				x = vertex_x + x;
				y = vertex_y + y;
			}
			else
			{
				x = vertex_x - x;
				y = vertex_y + y;
			}
			fn_resample(src, 0, w_src, h_src, s_src, x, y, dst, i);
		}
		dst = (void *)((uint8 *)dst + s_dst);
	}

	return S360_OK;
}
int s360_ohp_to_erp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;
	int w_tri, h_tri;

	w_src = img_src->width;
	h_src = img_src->height;
	
	w_dst = img_dst->width;
	h_dst = img_dst->height;

	w_tri = GET_W_TRI_OHP(w_src);
	h_tri = GET_H_TRI_OHP(w_tri);

	s360_assert_rv(w_src == w_tri * 4, S360_ERR_INVALID_DIMENSION);
	s360_assert_rv(h_src == 2 * h_tri, S360_ERR_INVALID_DIMENSION);

	if(IS_VALID_CS(img_src->colorspace))
	{
		ohp_to_erp_plane(img_src->buffer[0], w_src, h_src, img_src->stride[0], w_dst, h_dst, img_dst->stride[0], img_dst->buffer[0], img_src->colorspace);

		w_src >>= 1;
		h_src >>= 1;
		w_dst >>= 1;
		h_dst >>= 1;
		ohp_to_erp_plane(img_src->buffer[1], w_src, h_src, img_src->stride[1], w_dst, h_dst, img_dst->stride[1], img_dst->buffer[1], img_src->colorspace);
		ohp_to_erp_plane(img_src->buffer[2], w_src, h_src, img_src->stride[2], w_dst, h_dst, img_dst->stride[2], img_dst->buffer[2], img_src->colorspace);
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}

	return S360_OK;
}

static int ohp_to_rohp_plane(uint8 * src, int w_src, int h_src, int s_src, \
	int w_dst, int h_dst, int s_dst, uint8 * dst, int opt, S360_SPH_COORD * map)
{
	int         w_tri, h_tri, x_off;
	int         i, j;

	w_tri = (w_src / 4);
	h_tri = (h_src / 2);
	x_off = (w_tri / 2);

	for(j=0; j<h_tri; j++)
	{
		for(i=0; i<w_src; i++)
		{
			if(map[i + j * w_src].lng != -1)
			{
				dst[i + j * s_dst] = src[i + j * s_src];
			}
		}
		for(i=x_off; i<w_src-x_off; i++)
		{
			if(map[i + j * w_src].lng == -1)
			{
				dst[i + j * s_dst] = src[i - x_off + (j + h_tri) * s_src];
			}
		}
		for(i=0; i<x_off; i++)
		{
			if(map[i + j * w_src].lng == -1)
			{
				dst[i + j * s_dst] = src[w_src - x_off + i + (j + h_tri) * s_src];
			}
		}
		for(i=w_src-x_off; i<w_src; i++)
		{
			if(map[i + j * w_src].lng == -1)
			{
				dst[i + j * s_dst] = src[i - x_off + (j + h_tri) * s_src];
			}
		}
	}

	return S360_OK;
}

static int ohp_to_rohp_plane_10b(uint16 * src, int w_src, int h_src, int s_src, \
	int w_dst, int h_dst, int s_dst, uint16 * dst, int opt, S360_SPH_COORD * map)
{
	int         w_tri, h_tri, x_off;
	int         i, j;

	w_tri = (w_src / 4);
	h_tri = (h_src / 2);
	x_off = (w_tri / 2);

	for(j=0; j<h_tri; j++)
	{
		for(i=0; i<w_src; i++)
		{
			if(map[i + j * w_src].lng != -1)
			{
				dst[i + j * s_dst] = src[i + j * s_src];
			}
		}
		for(i=x_off; i<w_src-x_off; i++)
		{
			if(map[i + j * w_src].lng == -1)
			{
				dst[i + j * s_dst] = src[i - x_off + (j + h_tri) * s_src];
			}
		}
		for(i=0; i<x_off; i++)
		{
			if(map[i + j * w_src].lng == -1)
			{
				dst[i + j * s_dst] = src[w_src - x_off + i + (j + h_tri) * s_src];
			}
		}
		for(i=w_src-x_off; i<w_src; i++)
		{
			if(map[i + j * w_src].lng == -1)
			{
				dst[i + j * s_dst] = src[i - x_off + (j + h_tri) * s_src];
			}
		}
	}

	return S360_OK;
}

int s360_ohp_to_rohp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;

	w_src = img_src->width;
	h_src = img_src->height;
	w_dst = img_dst->width;
	h_dst = img_dst->height;
	img_dst->height = h_src >> 1;

	if(img_src->colorspace == S360_COLORSPACE_YUV420)
	{
		ohp_to_rohp_plane((uint8*)img_src->buffer[0], w_src, h_src, img_src->stride[0], w_dst, \
			h_dst, img_dst->stride[0], (uint8*)img_dst->buffer[0], opt, map->layer[0]);
		w_src >>= 1;
		h_src >>= 1;
		w_dst >>= 1;
		h_dst >>= 1;
		ohp_to_rohp_plane((uint8*)img_src->buffer[1], w_src, h_src, img_src->stride[1], w_dst, \
			h_dst, img_dst->stride[1], (uint8*)img_dst->buffer[1], opt, map->layer[1]);
		ohp_to_rohp_plane((uint8*)img_src->buffer[2], w_src, h_src, img_src->stride[2], w_dst, \
			h_dst, img_dst->stride[2], (uint8*)img_dst->buffer[2], opt, map->layer[1]);
	}
	else if(img_src->colorspace == S360_COLORSPACE_YUV420_10)
	{
		ohp_to_rohp_plane_10b((uint16*)img_src->buffer[0], w_src, h_src, img_src->stride[0], w_dst, \
			h_dst, img_dst->stride[0], (uint16*)img_dst->buffer[0], opt, map->layer[0]);
		w_src >>= 1;
		h_src >>= 1;
		w_dst >>= 1;
		h_dst >>= 1;
		ohp_to_rohp_plane_10b((uint16*)img_src->buffer[1], w_src, h_src, img_src->stride[1], w_dst, \
			h_dst, img_dst->stride[1], (uint16*)img_dst->buffer[1], opt, map->layer[1]);
		ohp_to_rohp_plane_10b((uint16*)img_src->buffer[2], w_src, h_src, img_src->stride[2], w_dst, \
			h_dst, img_dst->stride[2], (uint16*)img_dst->buffer[2], opt, map->layer[1]);
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}
	return S360_OK;
}

static int rohp_to_ohp_plane(uint8 * src, int w_src, int h_src, int s_src, \
	int w_dst, int h_dst, int s_dst, uint8 * dst, int opt, S360_SPH_COORD * map)
{
	int         w_tri, h_tri, x_off;
	int         i, j;

	w_tri = (w_src / 4);
	h_tri = h_src;
	w_dst = w_src;
	h_dst = h_src * 2;
	x_off = (w_tri / 2);

	for(j=0; j<h_tri; j++)
	{
		for(i=0; i<w_src; i++)
		{
			if(map[i + j * w_dst].lng != -1)
			{
				dst[i + j * s_dst] = src[i + j * s_src];
				map[i + j * w_dst].lat = 9999;
			}
		}
		for(i=x_off; i<w_src-x_off; i++)
		{
			if(map[i + j * w_dst].lng == -1)
			{
				dst[i - x_off + (j + h_tri) * s_dst] = src[i + j * s_src];
				map[i - x_off + (j + h_tri) * w_dst].lat = 9999;
			}
		}
		for(i=0; i<x_off; i++)
		{
			if(map[i + j * w_dst].lng == -1)
			{
				dst[w_src - x_off + i + (j + h_tri) * s_dst] = src[i + j * s_src];
				map[w_src - x_off + i + (j + h_tri) * w_dst].lat = 9999;
			}
		}
		for(i=w_src-(w_tri>>1); i<w_src; i++)
		{
			if(map[i + j * w_dst].lng == -1)
			{
				dst[i - x_off + (j + h_tri) * s_dst] = src[i + j * s_src];
				map[i - x_off + (j + h_tri) * w_dst].lat = 9999;
			}
		}
	}

	/* fill holes */
	for(j=0; j<h_dst; j++)
	{
		for(i=0; i<w_dst; i++)
		{
			if(map[i + j * w_dst].lng != -1 && map[i + j * w_dst].lat != 9999)
			{
				int sum = 0, cnt = 0;
				if(i > 0 && map[i - 1 + j * w_dst].lat == 9999)
				{
					sum += dst[i - 1 + j * s_dst];
					cnt++;
				}
				if(i < w_dst - 1 && map[i + 1 + j * w_dst].lat == 9999)
				{
					sum += dst[i + 1 + j * s_dst];
					cnt++;
				}
				if(j > 0 && map[i + (j - 1) * w_dst].lat == 9999)
				{
					sum += dst[i + (j - 1) * s_dst];
					cnt++;
				}
				if(j < h_dst - 1 && map[i + (j + 1) * w_dst].lat == 9999)
				{
					sum += dst[i + (j + 1) * s_dst];
					cnt++;
				}
				if(cnt > 0)
				{
					dst[i + j * s_dst] = (sum + (cnt>>1)) / cnt;
				}
			}
		}
	}

	if(opt & S360_OPT_PAD)
	{
		pad_ohp_plane(dst, w_dst, h_dst, s_dst, map);
	}

	return S360_OK;
}

static int rohp_to_ohp_plane_10b(uint16 * src, int w_src, int h_src, int s_src, \
	int w_dst, int h_dst, int s_dst, uint16 * dst, int opt, S360_SPH_COORD * map)
{
	
	int         w_tri, h_tri, x_off;
	int         i, j;

	w_tri = (w_src / 4);
	h_tri = h_src;
	w_dst = w_src;
	h_dst = h_src * 2;
	x_off = (w_tri / 2);

	for(j=0; j<h_tri; j++)
	{
		for(i=0; i<w_src; i++)
		{
			if(map[i + j * w_dst].lng != -1)
			{
				dst[i + j * s_dst] = src[i + j * s_src];
				map[i + j * w_dst].lat = 9999;
			}
		}
		for(i=x_off; i<w_src-x_off; i++)
		{
			if(map[i + j * w_dst].lng == -1)
			{
				dst[i - x_off + (j + h_tri) * s_dst] = src[i + j * s_src];
				map[i - x_off + (j + h_tri) * w_dst].lat = 9999;
			}
		}
		for(i=0; i<x_off; i++)
		{
			if(map[i + j * w_dst].lng == -1)
			{
				dst[w_src - x_off + i + (j + h_tri) * s_dst] = src[i + j * s_src];
				map[w_src - x_off + i + (j + h_tri) * w_dst].lat = 9999;
			}
		}
		for(i=w_src-(w_tri>>1); i<w_src; i++)
		{
			if(map[i + j * w_dst].lng == -1)
			{
				dst[i - x_off + (j + h_tri) * s_dst] = src[i + j * s_src];
				map[i - x_off + (j + h_tri) * w_dst].lat = 9999;
			}
		}
	}

	/* fill holes */
	for(j=0; j<h_dst; j++)
	{
		for(i=0; i<w_dst; i++)
		{
			if(map[i + j * w_dst].lng != -1 && map[i + j * w_dst].lat != 9999)
			{
				int sum = 0, cnt = 0;
				if(i > 0 && map[i - 1 + j * w_dst].lat == 9999)
				{
					sum += dst[i - 1 + j * s_dst];
					cnt++;
				}
				if(i < w_dst - 1 && map[i + 1 + j * w_dst].lat == 9999)
				{
					sum += dst[i + 1 + j * s_dst];
					cnt++;
				}
				if(j > 0 && map[i + (j - 1) * w_dst].lat == 9999)
				{
					sum += dst[i + (j - 1) * s_dst];
					cnt++;
				}
				if(j < h_dst - 1 && map[i + (j + 1) * w_dst].lat == 9999)
				{
					sum += dst[i + (j + 1) * s_dst];
					cnt++;
				}
				if(cnt > 0)
				{
					dst[i + j * s_dst] = (sum + (cnt>>1)) / cnt;
				}
			}
		}
	}

	if(opt & S360_OPT_PAD)
	{
		pad_ohp_plane_10b(dst, w_dst, h_dst, (s_dst), map);
	}

	return S360_OK;
}

int s360_rohp_to_ohp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;

	w_src = img_src->width;
	h_src = img_src->height;
	w_dst = img_dst->width;
	h_dst = img_dst->height;

	s360_assert_rv(w_dst >= w_src && h_dst >= h_src * 2, S360_ERR_INVALID_ARGUMENT);
	
	s360_img_reset(img_dst);

	if(img_src->colorspace == S360_COLORSPACE_YUV420)
	{

		rohp_to_ohp_plane((uint8*)img_src->buffer[0], w_src, h_src, img_src->stride[0], w_dst, \
			h_dst, img_dst->stride[0], (uint8*)img_dst->buffer[0], opt, map->layer[0]);
		w_src >>= 1;
		h_src >>= 1;
		w_dst >>= 1;
		h_dst >>= 1;
		rohp_to_ohp_plane((uint8*)img_src->buffer[1], w_src, h_src, img_src->stride[1], w_dst, \
			h_dst, img_dst->stride[1], (uint8*)img_dst->buffer[1], opt, map->layer[1]);
		rohp_to_ohp_plane((uint8*)img_src->buffer[2], w_src, h_src, img_src->stride[2], w_dst, \
			h_dst, img_dst->stride[2], (uint8*)img_dst->buffer[2], opt, map->layer[1]);
	}
	else if(img_src->colorspace == S360_COLORSPACE_YUV420_10)
	{
		rohp_to_ohp_plane_10b((uint16*)img_src->buffer[0], w_src, h_src, img_src->stride[0], w_dst, \
			h_dst, img_dst->stride[0], (uint16*)img_dst->buffer[0], opt, map->layer[0]);
		w_src >>= 1;
		h_src >>= 1;
		w_dst >>= 1;
		h_dst >>= 1;
		rohp_to_ohp_plane_10b((uint16*)img_src->buffer[1], w_src, h_src, img_src->stride[1], w_dst, \
			h_dst, img_dst->stride[1], (uint16*)img_dst->buffer[1], opt, map->layer[1]);
		rohp_to_ohp_plane_10b((uint16*)img_src->buffer[2], w_src, h_src, img_src->stride[2], w_dst, \
			h_dst, img_dst->stride[2], (uint16*)img_dst->buffer[2], opt, map->layer[1]);
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}

	return S360_OK;
}


static int ohp_to_cpp_plane(void * src, int w_src, int h_src, int s_src, \
	int w_dst, int h_dst, int s_dst, void * dst, int cs)
{
    void  (*fn_resample)(void * src, int w_start, int w_end, int h_src, int s_src,
							double x, double y, void * dst, int x_dst);

	double  n1[3], n2[3], normal[3], mid[3], xyz[3], t_vec[3];
	double  lng, lat, x, y, u;
	double  h_tri_3d, w_tri_3d;
	double  dist_23, dist_cmp1, dist_cmp2, d_ver, d_hor;
	int     tri, w_tri, h_tri, vertex_x, vertex_y;
	int     y1, y2;
	int     v1, v2, v3;
	int     i, j;
	int		* map, * map0;

    if(cs == S360_COLORSPACE_YUV420)
	{
		fn_resample = resample_2d;
	}
	else if(cs == S360_COLORSPACE_YUV420_10)
	{
		fn_resample = resample_2d_10b;
		s_dst <<= 1;
	}

    map = (int *)s360_malloc(sizeof(int) * h_dst * w_dst);
	w_tri = (int)(w_src / 4);
	h_tri = (int)(h_src / 2);
	y1    = h_tri;
	y2    = (h_tri * 2);

	v3d_average(tbl_tri_xyz_ohp[1], tbl_tri_xyz_ohp[2], mid);

	v3d_sub(tbl_tri_xyz_ohp[0], mid, t_vec);
	h_tri_3d = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);

	v3d_average(tbl_tri_xyz[1], tbl_tri_xyz[2], mid);

	v3d_sub(tbl_tri_xyz[0], mid, t_vec);
	h_tri_3d = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);

	map0 = map;
	for(j=0; j<h_dst ;j++)
	{
		for(i=0; i<w_dst; i++)
		{
			*(map0+i) = 0;
			x = ((double)i / (w_dst-0)) * M_2PI - PI;
			y = ((double)j / (h_dst-0)) * PI - (M_PI_2);

			lat = 3 * asin(y / PI);
			lng = x / (2 * cos(2 * lat / 3) - 1);

			x = (lng + PI) / 2 / PI * (w_dst - 0);
			y = (lat + (M_PI_2)) / PI * (h_dst - 0);

			if(x>=0 && x <w_dst && y>=0 && y<h_dst)
				*(map0+i) = 1;
		}
		map0 += w_dst;
	}
	map0 = map;

	for(j=0; j<h_dst; j++)
	{
		for(i=0; i<w_dst; i++)
		{
			lng = ((double)i/w_dst)*M_2PI - PI;
			lat = ((double)j/h_dst)*PI - M_PI_2;

			lat = 3 * asin(lat / PI);
			lng = lng / (2 * cos(2 * lat / 3) - 1);

			lat += M_PI_2;
			lng += PI;

			lat = RAD2DEG(lat);
			lng = RAD2DEG(lng);

			x = (int)(lat * (h_dst/180.0));
			y = (int)(lng * (w_dst/360.0));

			if(*(map0+i)==0)
				continue;

			lat = DEG2RAD(lat);
			lng = DEG2RAD(lng);

			xyz[0] = sin(lat) * cos(lng);
			xyz[1] = sin(lat) * sin(lng);
			xyz[2] = cos(lat);

			tri = get_tri_idx(xyz[0], xyz[1], xyz[2], tbl_center_xyz_ohp);
			v1 = tbl_vidx_ohp2erp[tri][0];
			v2 = tbl_vidx_ohp2erp[tri][1];
			v3 = tbl_vidx_ohp2erp[tri][2];

			v3d_sub(tbl_tri_xyz_ohp[v1], tbl_tri_xyz_ohp[v2], n1);
			v3d_sub(tbl_tri_xyz_ohp[v1], tbl_tri_xyz_ohp[v3], n2);

			normal[0] = -(n1[1] * n2[2] - n2[1] * n1[2]);
			normal[1] = -(n1[2] * n2[0] - n2[2] * n1[0]);
			normal[2] = -(n1[0] * n2[1] - n2[0] * n1[1]);

			v3d_dot(normal, xyz, n1);
			v3d_dot(normal, tbl_tri_xyz_ohp[v1], n2);

			v3d_scale(xyz, (n2[0] + n2[1] + n2[2]) / (n1[0] + n1[1] + n1[2]));

			v3d_sub(tbl_tri_xyz_ohp[v3], tbl_tri_xyz_ohp[v2], n1);
			dist_23 = n1[0] * n1[0] + n1[1] * n1[1] + n1[2] * n1[2];

			v3d_sub(xyz, tbl_tri_xyz_ohp[v2], t_vec);
			v3d_dot(t_vec, n1, t_vec);
			u = (t_vec[0] + t_vec[1] + t_vec[2]) / dist_23;

			v3d_average(tbl_tri_xyz_ohp[v2], tbl_tri_xyz_ohp[v3], mid);

			v3d_sub(tbl_tri_xyz_ohp[v2], tbl_tri_xyz_ohp[v3], t_vec);
			w_tri_3d = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);
			v3d_sub(tbl_tri_xyz_ohp[v1], mid, t_vec);
			h_tri_3d = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);

			v3d_affine(n1, u, tbl_tri_xyz_ohp[v2], n2);

			v3d_sub(mid, n2, t_vec);
			d_hor = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);
			v3d_sub(xyz, n2, t_vec);
			d_ver = h_tri_3d - GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);

			x = (w_tri / w_tri_3d) * d_hor;
			y = (h_tri / h_tri_3d) * d_ver;

			v3d_sub(tbl_tri_xyz_ohp[v2], n2, t_vec);
			dist_cmp1 = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);
			v3d_sub(tbl_tri_xyz_ohp[v3], n2, t_vec);
			dist_cmp2 = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);

			switch(tri)
			{
			case 0:
				vertex_x = (w_tri >> 1);
				vertex_y = 0;
				break;
			case 1:
				vertex_x = (w_tri >> 1) + w_tri;
				vertex_y = 0;
				break;
			case 2:
				vertex_x = (w_tri >> 1) + (w_tri << 1);
				vertex_y = 0;
				break;
			case 3:
				vertex_x = (w_tri >> 1) + 3 * w_tri;
				vertex_y = 0;
				break;

			case 4:
				vertex_x = (w_tri >> 1);
				vertex_y = y2;
				break;
			case 5:
				vertex_x = (w_tri >> 1) + w_tri;
				vertex_y = y2;
				break;
			case 6:
				vertex_x = (w_tri >> 1) + (w_tri << 1);
				vertex_y = y2;
				break;
			case 7:
				vertex_x = (w_tri >> 1) + 3 * w_tri;
				vertex_y = y2;
				break;
			
			}
			if(vertex_y >= y2)
			{
				y = -y;
			}

			if(dist_cmp1 > dist_cmp2)
			{
				x = vertex_x + x;
				y = vertex_y + y;
			}
			else
			{
				x = vertex_x - x;
				y = vertex_y + y;
			}
			fn_resample(src, 0, w_src, h_src, s_src, x, y, dst, i);
		}

		dst = (void *)((uint8 *)dst + s_dst);
		map0 += w_dst;
	}
	s360_mfree(map);
	return S360_OK;
}
int s360_ohp_to_cpp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;
	int w_tri, h_tri;

	w_src = img_src->width;
	h_src = img_src->height;
	
	w_dst = img_dst->width;
	h_dst = img_dst->height;

	w_tri = GET_W_TRI_OHP(w_src);
	h_tri = GET_H_TRI_OHP(w_tri);

	s360_assert_rv(w_src == w_tri * 4, S360_ERR_INVALID_DIMENSION);
	s360_assert_rv(h_src == 2 * h_tri, S360_ERR_INVALID_DIMENSION);

	s360_img_reset(img_dst);

	if(IS_VALID_CS(img_src->colorspace))
	{
		ohp_to_cpp_plane(img_src->buffer[0], w_src, h_src, img_src->stride[0], w_dst, h_dst, img_dst->stride[0], img_dst->buffer[0], img_src->colorspace);

		w_src >>= 1;
		h_src >>= 1;
		w_dst >>= 1;
		h_dst >>= 1;
		ohp_to_cpp_plane(img_src->buffer[1], w_src, h_src, img_src->stride[1], w_dst, h_dst, img_dst->stride[1], img_dst->buffer[1], img_src->colorspace);
		ohp_to_cpp_plane(img_src->buffer[2], w_src, h_src, img_src->stride[2], w_dst, h_dst, img_dst->stride[2], img_dst->buffer[2], img_src->colorspace);
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}

	return S360_OK;
}