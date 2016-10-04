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
#include "360tools_isp.h"

#if USE_MIRROR_PADDING
static void fill_pad_buf(S360_SPH_COORD * map, uint8 * isp, int x_dst, \
    uint8 * buf, int x_buf, int dir, int w)
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
				val = isp[x_dst + i];
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
				val = isp[x_dst - i];
			}
			buf[x_buf + i] = val;
		}
	}
}

static void pad_from_buf(S360_SPH_COORD * map, uint8 * dst, int x_dst, \
    uint8 * buf, int x_buf, int dir, int w)
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

static int pad_isp_plane(uint8 * isp, int w, int h, int s, S360_SPH_COORD * map)
{
	uint8     * buf0, * buf;
	uint8     * cnt;
	int         i, j, k, x_buf;

	buf0 = (uint8 *)s360_malloc(sizeof(uint8) * h * PAD_SIZE * 10);
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
				fill_pad_buf(map, isp, i, buf, x_buf, 1, w);

				while(map[i].lng != -1 && i < w) i++;
				fill_pad_buf(map, isp, i-1, buf, x_buf + PAD_SIZE, -1, w);

				k++;
			}
		}
		cnt[j] = k;

		buf += PAD_SIZE * 10;
		map += w;
		isp += s;
	}

	isp -= s * h;
	map -= w * h;
	buf  = buf0;
	for(j=0; j<h; j++)
	{
		i = 0;
		if(cnt[j] == 5)
		{
			for(k=0; k<5; k++)
			{
				while(map[i].lng == -1 && i < w) i++;
				x_buf = ((10 + 2 * k - 1) % 10) * PAD_SIZE;
				pad_from_buf(map, isp, i-1, buf, x_buf, -1, w);

				while(map[i].lng != -1 && i < w) i++;
				x_buf = ((2 * k + 2) % 10) * PAD_SIZE;
				pad_from_buf(map, isp, i, buf, x_buf, 1, w);
			}
		}
		else
		{
			while(map[i].lng == -1 && i < w) i++;
			pad_from_buf(map, isp, i-1, buf, PAD_SIZE, -1, w);

			while(map[i].lng != -1 && i < w) i++;
			pad_from_buf(map, isp, i, buf, 0, 1, w);
		}

		buf += PAD_SIZE * 10;
		map += w;
		isp += s;
	}

	s360_mfree(buf0);
	s360_mfree(cnt);

	return S360_OK;
}

static void fill_pad_buf_10b(S360_SPH_COORD * map, uint16 * isp, int x_dst, \
    uint16 * buf, int x_buf, int dir, int w)
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
				val = isp[x_dst + i];
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
				val = isp[x_dst - i];
			}
			buf[x_buf + i] = val;
		}
	}
}

static void pad_from_buf_10b(S360_SPH_COORD * map, uint16 * dst, int x_dst, \
    uint16 * buf, int x_buf, int dir, int w)
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

static int pad_isp_plane_10b(uint16 * isp, int w, int h, int s, \
    S360_SPH_COORD * map)
{
	uint16     * buf0, * buf;
	uint16     * cnt;
	int         i, j, k, x_buf;

	buf0 = (uint16 *)s360_malloc(sizeof(uint16) * h * PAD_SIZE * 10);
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
				fill_pad_buf_10b(map, isp, i, buf, x_buf, 1, w);

				while(map[i].lng != -1 && i < w) i++;
				fill_pad_buf_10b(map, isp, i-1, buf, x_buf + PAD_SIZE, -1, w);

				k++;
			}
		}
		cnt[j] = k;

		buf += PAD_SIZE * 10;
		map += w;
		isp += s;
	}

	isp -= s * h;
	map -= w * h;
	buf  = buf0;
	for(j=0; j<h; j++)
	{
		i = 0;
		if(cnt[j] == 5)
		{
			for(k=0; k<5; k++)
			{
				while(map[i].lng == -1 && i < w) i++;
				x_buf = ((10 + 2 * k - 1) % 10) * PAD_SIZE;
				pad_from_buf_10b(map, isp, i-1, buf, x_buf, -1, w);

				while(map[i].lng != -1 && i < w) i++;
				x_buf = ((2 * k + 2) % 10) * PAD_SIZE;
				pad_from_buf_10b(map, isp, i, buf, x_buf, 1, w);
			}
		}
		else
		{
			while(map[i].lng == -1 && i < w) i++;
			pad_from_buf_10b(map, isp, i-1, buf, PAD_SIZE, -1, w);

			while(map[i].lng != -1 && i < w) i++;
			pad_from_buf_10b(map, isp, i, buf, 0, 1, w);
		}

		buf += PAD_SIZE * 10;
		map += w;
		isp += s;
	}

	s360_mfree(buf0);
	s360_mfree(cnt);

	return S360_OK;
}
#else
static void pad_isp_plane(uint8 * isp, int w, int h, int s, S360_SPH_COORD * map)
{
	uint8 val;
	int i, j, n;

	for(j=0; j<h; j++)
	{
		for(i=1; i<w; i++)
		{
			if(map[i-1].lng == -1 && map[i].lng != -1)
			{
				val = isp[i];
				for(n=S360_MAX(i-PAD_SIZE, 0); n<i && map[n].lng == -1; n++)
				{
					isp[n] = val;
				}
			}
		}
		map += w;
		isp += s;
	}

	map -= w * h;
	isp -= h * s;
	for(j=0; j<h; j++)
	{
		for(i=1; i<w; i++)
		{
			if(map[i-1].lng != -1 && map[i].lng == -1)
			{
				val = isp[i - 1];
				for(n=i; n<S360_MIN(i+PAD_SIZE, w) && map[n].lng == -1; n++)
				{
					isp[n] = val;
				}
			}
		}
		map += w;
		isp += s;
	}
}
#endif

static int erp_to_isp_plane_strip(int start_h, int end_h, int h_dst, \
	int w_dst, void *src, void *dst, int w_start, int w_end, \
	S360_SPH_COORD * map, int cs, int s_dst, int s_src, int w_src, int h_src)
{
	void  (*fn_resample)(void * src, int w_start, int w_end, int h_start, int h_end,\
		int s_src, double x, double y, void * dst, int x_dst);
	int i, j;
	double x, y;

	if(cs == S360_COLORSPACE_YUV420)
	{
		fn_resample = resample_2d;
	}
	else if(cs == S360_COLORSPACE_YUV420_10)
	{
		fn_resample = resample_2d_10b;
		s_dst <<= 1;
	}

	for(j=0; j<end_h-start_h; j++)
	{
		for(i=0; i<w_dst; i++)
		{
			if(map[i].lng != -1)
			{
				x = (map[i].lng / 360) * w_src;
				y = (map[i].lat / 180) * h_src;

				fn_resample(src, w_start, w_end, start_h, h_src, s_src, x, y, dst, i);
			}
		}
		dst = (void *)((uint8 *)dst + s_dst);
		map += w_dst;
	}
	return S360_OK;
}

static int erp_to_isp_plane(void * src, int w_src, int h_src, int s_src, \
	int w_dst, int h_dst, int s_dst, void * dst, int w_tri, int opt, int cs, \
	int pad_sz, S360_SPH_COORD * map)
{
	int              w_start, w_end;
	int              start_h, end_h;
	void           * dst1;
	S360_SPH_COORD * map1;

	w_start = opt ? -pad_sz : 0;
	w_end = opt ? w_src + pad_sz : w_src;

	start_h = 0;
	end_h	= h_dst;

	dst1 = (void *)((uint8 *)dst + s_dst*start_h);
	map1 = map + w_dst*start_h;

	erp_to_isp_plane_strip(start_h, end_h, h_dst, w_dst, src, dst1, w_start, \
		w_end, map1, cs, s_dst, s_src, w_src, h_src);

	return S360_OK;
}

int s360_erp_to_isp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, \
	S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;
	int w_tri, h_tri;
	int ret;
	int pad_sz;

	w_src = img_src->width;
	h_src = img_src->height;

	w_dst = img_dst->width;
	h_dst = img_dst->height;

	w_tri = GET_W_TRI_ISP(w_dst);
	h_tri = GET_H_TRI_ISP(w_tri);

	s360_assert_rv(w_dst == w_tri * 5.5, S360_ERR_INVALID_DIMENSION);
	s360_assert_rv(h_dst == 3 * h_tri, S360_ERR_INVALID_DIMENSION);

	ret = s360_img_realloc(img_dst, w_dst, h_dst, opt);
	s360_assert_rv(ret == S360_OK, ret);

	s360_img_reset(img_dst);

	pad_sz = (opt & S360_OPT_PAD)?PAD_SIZE:0;

	if(opt & S360_OPT_PAD)
		s360_pad_erp(img_src);

	if(IS_VALID_CS(img_src->colorspace))
	{
		erp_to_isp_plane(img_src->buffer[0], w_src, h_src, \
			img_src->stride[0], w_dst, h_dst, img_dst->stride[0], \
			img_dst->buffer[0], w_tri, opt, img_src->colorspace, pad_sz, \
			map->layer[0]);

		w_tri  >>= 1;
		w_src  >>= 1;
		h_src  >>= 1;
		w_dst  >>= 1;
		h_dst  >>= 1;
		pad_sz >>= 1;
		erp_to_isp_plane(img_src->buffer[1], w_src, h_src, \
			img_src->stride[1], w_dst, h_dst, img_dst->stride[1], \
			img_dst->buffer[1], w_tri, opt, img_src->colorspace, pad_sz, \
			map->layer[1]);
		erp_to_isp_plane(img_src->buffer[2], w_src, h_src, \
			img_src->stride[2], w_dst, h_dst, img_dst->stride[2], \
			img_dst->buffer[2], w_tri, opt, img_src->colorspace, pad_sz, \
			map->layer[1]);
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}

	return S360_OK;
}

static int cpp_to_isp_plane(int w_src, int h_src, int w_dst, int h_dst, \
	void * src, void * img_dst, int s_src, int s_dst, int w_tri, int opt, \
	int cs, int pad_sz, S360_SPH_COORD * map0)
{
	resample_fn fn_resample;
	S360_SPH_COORD * map;
	uint8          * cpp_map;
	double           lon, lat, la_src, lo_src;
	double           x, y;
	void           * dst;
	int              i, j;

	fn_resample = resample_fp(cs);

	cpp_map = (uint8 *)s360_malloc(sizeof(uint8) * w_src * h_src);
	cpp_map_plane(w_src, h_src, s_src, cpp_map);


	if(cs == S360_COLORSPACE_YUV420)
	{
		pad_cpp_plane((uint8 *)(src), w_src, h_src, s_src, cpp_map);
	}
	else if(cs == S360_COLORSPACE_YUV420_10)
	{
		pad_cpp_plane_10b((uint16 *)(src), w_src, h_src, s_src, cpp_map);
		s_dst <<= 1;
	}


	map = map0;
	dst = img_dst;
	for(j=0; j<h_dst; j++)
	{
		for(i=0; i<w_dst; i++)
		{
			lon = map[i].lng;
			lat = map[i].lat;

			if(lon != -1)
			{
				la_src = DEG2RAD(lat) - M_PI_2;
				lo_src = DEG2RAD(lon) - PI;

				x = (lo_src * (2*cos(2*la_src/3) - 1) + PI) * w_src / (2* PI); 
				y = (PI * sin((la_src)/3) + M_PI_2) * h_src / PI;

				fn_resample(src, 0, w_src, 0, h_src, s_src, x, y, dst, i);
			}
		}
		map += w_dst;
		dst = (void *)((uint8 *)dst + s_dst);
	}

#if 0
	/* padding for rendering */
	if(opt & S360_OPT_PAD)
	{
		if(cs == S360_COLORSPACE_YUV420)
		{
			pad_isp_plane(img_dst, w_dst, h_dst, s_dst, map0);
		}
		else if(cs == S360_COLORSPACE_YUV420_10)
		{
			pad_isp_plane_10b(img_dst, w_dst, h_dst, (s_dst>>1), map0);
		}
	}
#endif

	s360_mfree(cpp_map);

	return 1;
}

int s360_cpp_to_isp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, \
	S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;
	int w_tri, h_tri;
	int ret;
	int pad_sz;

	w_src = img_src->width;
	h_src = img_src->height;

	w_dst = img_dst->width;
	h_dst = img_dst->height;

	w_tri = GET_W_TRI_ISP(w_dst);
	h_tri = GET_H_TRI_ISP(w_tri);

	s360_assert_rv(w_dst == (w_tri * 5.5), S360_ERR_INVALID_DIMENSION);
	s360_assert_rv(h_dst == 3 * h_tri, S360_ERR_INVALID_DIMENSION);

	s360_assert_rv(h_dst <= img_dst->height, S360_ERR_INVALID_ARGUMENT);

	ret = s360_img_realloc(img_dst, w_dst, h_dst, opt);
	s360_assert_rv(ret == S360_OK, ret);	

	s360_img_reset(img_dst);

	if(opt & S360_OPT_PAD)
	{
		pad_sz = PAD_SIZE;
	}
	else
	{
		pad_sz = 0;
	}

	if(IS_VALID_CS(img_src->colorspace))
	{
		cpp_to_isp_plane(w_src, h_src, w_dst, h_dst, img_src->buffer[0], \
			img_dst->buffer[0], img_src->stride[0], img_dst->stride[0], w_tri,\
			opt, img_src->colorspace, pad_sz, map->layer[0]);

		w_tri  >>= 1;
		w_src  >>= 1;
		h_src  >>= 1;
		w_dst  >>= 1;
		h_dst  >>= 1;
		pad_sz >>= 1;
		cpp_to_isp_plane(w_src, h_src, w_dst, h_dst, img_src->buffer[1], \
			img_dst->buffer[1], img_src->stride[1], img_dst->stride[1], w_tri,\
			opt, img_src->colorspace, pad_sz, map->layer[1]);
		cpp_to_isp_plane(w_src, h_src, w_dst, h_dst, img_src->buffer[2], \
			img_dst->buffer[2], img_src->stride[2], img_dst->stride[2], w_tri,\
			opt, img_src->colorspace, pad_sz, map->layer[1]);
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}
	return S360_OK;
}

static int get_tri_idx(double x, double y, double z, double center[][3])
{
	double min_dist, dist, h1, h2, h3;
	int i, idx;

	idx = 0;
	min_dist = 9999.0;

	for(i=0; i<20; i++)
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


double tbl_center_xyz[20][3] =
{
	{   0.390273,   0.283550,   0.631476 },
	{  -0.149071,   0.458794,   0.631476 },
	{  -0.482405,   0.000000,   0.631476 },
	{  -0.149071,  -0.458794,   0.631476 },
	{   0.390273,  -0.283550,   0.631476 },
	{   0.780547,   0.000000,  -0.149071 },
	{   0.631476,   0.458794,   0.149071 },
	{   0.241202,   0.742344,  -0.149071 },
	{  -0.241202,   0.742344,   0.149071 },
	{  -0.631476,   0.458794,  -0.149071 },
	{  -0.780547,   0.000000,   0.149071 },
	{  -0.631476,  -0.458794,  -0.149071 },
	{  -0.241202,  -0.742344,   0.149071 },
	{   0.241202,  -0.742344,  -0.149071 },
	{   0.631476,  -0.458794,   0.149071 },
	{   0.482405,   0.000000,  -0.631476 },
	{   0.149071,   0.458794,  -0.631476 },
	{  -0.390273,   0.283550,  -0.631476 },
	{  -0.390273,  -0.283550,  -0.631476 },
	{   0.149071,  -0.458794,  -0.631476 }
};

static int isp_to_erp_plane(void * src, int w_src, int h_src, int s_src, \
	int w_dst, int h_dst, int s_dst, void * dst, int cs)
{
	void    (*fn_resample)(void * src, int w_start, int w_end, int h_start, int h_end,\
		int s_src, double x, double y, void * dst, int x_dst);

	double  n1[3], n2[3], normal[3], mid[3], xyz[3], t_vec[3];
	double  lng, lat, x, y, u;
	double  h_tri_3d, w_tri_3d;
	double  dist_23, dist_cmp1, dist_cmp2, d_ver, d_hor;
	int     tri, w_tri, h_tri, vertex_x, vertex_y;
	int     y1, y2, y3;
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

	w_tri = (int)(w_src / 5.5);
	h_tri = (int)(h_src / 3);
	y1    = h_tri;
	y2    = (h_tri * 2);
	y3    = (h_tri * 3);

	v3d_average(tbl_tri_xyz[1], tbl_tri_xyz[2], mid);

	v3d_sub(tbl_tri_xyz[0], mid, t_vec);
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

			tri = get_tri_idx(xyz[0], xyz[1], xyz[2], tbl_center_xyz);
			v1 = tbl_vidx_isp2erp[tri][0];
			v2 = tbl_vidx_isp2erp[tri][1];
			v3 = tbl_vidx_isp2erp[tri][2];

			v3d_sub(tbl_tri_xyz[v1], tbl_tri_xyz[v2], n1);
			v3d_sub(tbl_tri_xyz[v1], tbl_tri_xyz[v3], n2);

			normal[0] = -(n1[1] * n2[2] - n2[1] * n1[2]);
			normal[1] = -(n1[2] * n2[0] - n2[2] * n1[0]);
			normal[2] = -(n1[0] * n2[1] - n2[0] * n1[1]);

			v3d_dot(normal, xyz, n1);
			v3d_dot(normal, tbl_tri_xyz[v1], n2);

			v3d_scale(xyz, (n2[0] + n2[1] + n2[2]) / (n1[0] + n1[1] + n1[2]));

			v3d_sub(tbl_tri_xyz[v3], tbl_tri_xyz[v2], n1);
			dist_23 = n1[0] * n1[0] + n1[1] * n1[1] + n1[2] * n1[2];

			v3d_sub(xyz, tbl_tri_xyz[v2], t_vec);
			v3d_dot(t_vec, n1, t_vec);
			u = (t_vec[0] + t_vec[1] + t_vec[2]) / dist_23;

			v3d_average(tbl_tri_xyz[v2], tbl_tri_xyz[v3], mid);

			v3d_sub(tbl_tri_xyz[v2], tbl_tri_xyz[v3], t_vec);
			w_tri_3d = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);
			v3d_sub(tbl_tri_xyz[v1], mid, t_vec);
			h_tri_3d = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);

			v3d_affine(n1, u, tbl_tri_xyz[v2], n2);

			v3d_sub(mid, n2, t_vec);
			d_hor = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);
			v3d_sub(xyz, n2, t_vec);
			d_ver = h_tri_3d - GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);

			x = (w_tri / w_tri_3d) * d_hor;
			y = (h_tri / h_tri_3d) * d_ver;

			v3d_sub(tbl_tri_xyz[v2], n2, t_vec);
			dist_cmp1 = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);
			v3d_sub(tbl_tri_xyz[v3], n2, t_vec);
			dist_cmp2 = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);

			switch(tri)
			{
			case 0:
				vertex_x = (w_tri >> 1) + (w_tri << 1);
				vertex_y = 0;
				break;
			case 1:
				vertex_x = (w_tri >> 1) + 3 * w_tri;
				vertex_y = 0;
				break;
			case 2:
				vertex_x = (w_tri >> 1) + (w_tri << 2);
				vertex_y = 0;
				break;
			case 3:
				vertex_x = (w_tri >> 1);
				vertex_y = 0;
				break;
			case 4:
				vertex_x = (w_tri >> 1) + w_tri;
				vertex_y = 0;
				break;
			case 5:
				vertex_x = (w_tri << 1);
				vertex_y = y1;
				break;
			case 7:
				vertex_x = 3 * w_tri;
				vertex_y = y1;
				break;
			case 9:
				vertex_x = (w_tri << 2);
				vertex_y = y1;
				break;
			case 11:
				vertex_x = 5 * w_tri;
				vertex_y = y1;
				break;
			case 13:
				vertex_x = w_tri;
				vertex_y = y1;
				break;
			case 6:
				vertex_x = (w_tri >> 1) + (w_tri << 1);
				vertex_y = y2;
				break;
			case 8:
				vertex_x = (w_tri >> 1) + 3 * w_tri;
				vertex_y = y2;
				break;
			case 10:
				vertex_x = (w_tri >> 1) + (w_tri << 2);
				vertex_y = y2;
				break;
			case 12:
				vertex_x = (w_tri >> 1);
				vertex_y = y2;
				break;
			case 14:
				vertex_x = (w_tri >> 1) + w_tri;
				vertex_y = y2;
				break;
			case 15:
				vertex_x = (w_tri << 1);
				vertex_y = y3;
				break;
			case 16:
				vertex_x = 3 * w_tri;
				vertex_y = y3;
				break;
			case 17:
				vertex_x = (w_tri << 2);
				vertex_y = y3;
				break;
			case 18:
				vertex_x = 5 * w_tri;
				vertex_y = y3;
				break;
			case 19:
				vertex_x = w_tri;
				vertex_y = y3;
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
			fn_resample(src, 0, w_src, 0, h_src, s_src, x, y, dst, i);
		}

		dst = (void *)((uint8 *)dst + s_dst);
	}

	return S360_OK;
}

int s360_isp_to_erp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, \
	S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;
	int w_tri, h_tri;

	w_dst = img_dst->width;
	h_dst = img_dst->height;

	w_src = img_src->width;
	h_src = img_src->height;

	w_tri = GET_W_TRI_ISP(w_src);
	h_tri = GET_H_TRI_ISP(w_tri);

	s360_assert_rv(w_src == (w_tri * 5.5), S360_ERR_INVALID_DIMENSION);
	s360_assert_rv(h_src == 3 * h_tri, S360_ERR_INVALID_DIMENSION);

	if(IS_VALID_CS(img_src->colorspace))
	{
		isp_to_erp_plane(img_src->buffer[0], w_src, h_src, \
			img_src->stride[0], w_dst, h_dst, img_dst->stride[0], \
			img_dst->buffer[0], img_src->colorspace);

		w_src >>= 1;
		h_src >>= 1;
		w_dst >>= 1;
		h_dst >>= 1;
		isp_to_erp_plane(img_src->buffer[1], w_src, h_src, \
			img_src->stride[1], w_dst, h_dst, img_dst->stride[1], \
			img_dst->buffer[1], img_src->colorspace);
		isp_to_erp_plane(img_src->buffer[2], w_src, h_src, \
			img_src->stride[2], w_dst, h_dst, img_dst->stride[2], \
			img_dst->buffer[2], img_src->colorspace);
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}

	return S360_OK;
}
static int isp_to_cpp_plane(void * src, int w_src, int h_src, int s_src, \
	int w_dst, int h_dst, int s_dst, void * dst, int cs)
{
	void    (*fn_resample)(void * src, int w_start, int w_end, int h_start, int h_end,\
		int s_src, double x, double y, void * dst, int x_dst);

	double   n1[3], n2[3], normal[3], mid[3], xyz[3], t_vec[3];
	double   lng, lat, x, y, u;
	double   h_tri_3d, w_tri_3d;
	double   dist_23, dist_cmp1, dist_cmp2, d_ver, d_hor;
	int      tri, w_tri, h_tri, vertex_x, vertex_y;
	int      y1, y2, y3;
	int      v1, v2, v3;
	int      i, j;
	uint8  * map, * map0;

	if(cs == S360_COLORSPACE_YUV420)
	{
		fn_resample = resample_2d;
	}
	else if(cs == S360_COLORSPACE_YUV420_10)
	{
		fn_resample = resample_2d_10b;
		s_dst <<= 1;
	}

	map = (uint8 *)s360_malloc(sizeof(uint8) * h_dst * w_dst);
	cpp_map_plane(w_dst, h_dst, s_dst, map);
	map0 = map;

	w_tri = (int)(w_src / 5.5);
	h_tri = (int)(h_src / 3);
	y1    = h_tri;
	y2    = (h_tri * 2);
	y3    = (h_tri * 3);

	v3d_average(tbl_tri_xyz[1], tbl_tri_xyz[2], mid);

	v3d_sub(tbl_tri_xyz[0], mid, t_vec);
	h_tri_3d = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);
	
	for(j=0; j<h_dst; j++)
	{
		for(i=0; i<w_dst; i++)
		{
			lng = ((double)i/w_dst)*(M_2PI) - PI;
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

			tri = get_tri_idx(xyz[0], xyz[1], xyz[2], tbl_center_xyz);
			v1 = tbl_vidx_isp2erp[tri][0];
			v2 = tbl_vidx_isp2erp[tri][1];
			v3 = tbl_vidx_isp2erp[tri][2];

			v3d_sub(tbl_tri_xyz[v1], tbl_tri_xyz[v2], n1);
			v3d_sub(tbl_tri_xyz[v1], tbl_tri_xyz[v3], n2);

			normal[0] = -(n1[1] * n2[2] - n2[1] * n1[2]);
			normal[1] = -(n1[2] * n2[0] - n2[2] * n1[0]);
			normal[2] = -(n1[0] * n2[1] - n2[0] * n1[1]);

			v3d_dot(normal, xyz, n1);
			v3d_dot(normal, tbl_tri_xyz[v1], n2);

			v3d_scale(xyz, (n2[0] + n2[1] + n2[2]) / (n1[0] + n1[1] + n1[2]));

			v3d_sub(tbl_tri_xyz[v3], tbl_tri_xyz[v2], n1);
			dist_23 = n1[0] * n1[0] + n1[1] * n1[1] + n1[2] * n1[2];

			v3d_sub(xyz, tbl_tri_xyz[v2], t_vec);
			v3d_dot(t_vec, n1, t_vec);
			u = (t_vec[0] + t_vec[1] + t_vec[2]) / dist_23;

			v3d_average(tbl_tri_xyz[v2], tbl_tri_xyz[v3], mid);

			v3d_sub(tbl_tri_xyz[v2], tbl_tri_xyz[v3], t_vec);
			w_tri_3d = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);
			v3d_sub(tbl_tri_xyz[v1], mid, t_vec);
			h_tri_3d = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);

			v3d_affine(n1, u, tbl_tri_xyz[v2], n2);

			v3d_sub(mid, n2, t_vec);
			d_hor = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);
			v3d_sub(xyz, n2, t_vec);
			d_ver = h_tri_3d - GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);

			x = (w_tri / w_tri_3d) * d_hor;
			y = (h_tri / h_tri_3d) * d_ver;

			v3d_sub(tbl_tri_xyz[v2], n2, t_vec);
			dist_cmp1 = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);
			v3d_sub(tbl_tri_xyz[v3], n2, t_vec);
			dist_cmp2 = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);

			switch(tri)
			{
			case 0:
				vertex_x = (w_tri >> 1) + (w_tri << 1);
				vertex_y = 0;
				break;
			case 1:
				vertex_x = (w_tri >> 1) + 3 * w_tri;
				vertex_y = 0;
				break;
			case 2:
				vertex_x = (w_tri >> 1) + (w_tri << 2);
				vertex_y = 0;
				break;
			case 3:
				vertex_x = (w_tri >> 1);
				vertex_y = 0;
				break;
			case 4:
				vertex_x = (w_tri >> 1) + w_tri;
				vertex_y = 0;
				break;
			case 5:
				vertex_x = (w_tri << 1);
				vertex_y = y1;
				break;
			case 7:
				vertex_x = 3 * w_tri;
				vertex_y = y1;
				break;
			case 9:
				vertex_x = (w_tri << 2);
				vertex_y = y1;
				break;
			case 11:
				vertex_x = 5 * w_tri;
				vertex_y = y1;
				break;
			case 13:
				vertex_x = w_tri;
				vertex_y = y1;
				break;
			case 6:
				vertex_x = (w_tri >> 1) + (w_tri << 1);
				vertex_y = y2;
				break;
			case 8:
				vertex_x = (w_tri >> 1) + 3 * w_tri;
				vertex_y = y2;
				break;
			case 10:
				vertex_x = (w_tri >> 1) + (w_tri << 2);
				vertex_y = y2;
				break;
			case 12:
				vertex_x = (w_tri >> 1);
				vertex_y = y2;
				break;
			case 14:
				vertex_x = (w_tri >> 1) + w_tri;
				vertex_y = y2;
				break;
			case 15:
				vertex_x = (w_tri << 1);
				vertex_y = y3;
				break;
			case 16:
				vertex_x = 3 * w_tri;
				vertex_y = y3;
				break;
			case 17:
				vertex_x = (w_tri << 2);
				vertex_y = y3;
				break;
			case 18:
				vertex_x = 5 * w_tri;
				vertex_y = y3;
				break;
			case 19:
				vertex_x = w_tri;
				vertex_y = y3;
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
			fn_resample(src, 0, w_src, 0, h_src, s_src, x, y, dst, i);
		}

		dst = (void *)((uint8 *)dst + s_dst);
		map0 += w_dst;
	}
	s360_mfree(map);
	return S360_OK;
}

int s360_isp_to_cpp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, \
    S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;
	int w_tri, h_tri;

	w_dst = img_dst->width;
	h_dst = img_dst->height;

	w_src = img_src->width;
	h_src = img_src->height;

	w_tri = GET_W_TRI_ISP(w_src);
	h_tri = GET_H_TRI_ISP(w_tri);

	s360_assert_rv(w_src == w_tri * 5.5, S360_ERR_INVALID_DIMENSION);
	s360_assert_rv(h_src == 3 * h_tri, S360_ERR_INVALID_DIMENSION);

	s360_img_reset(img_dst);

	if(IS_VALID_CS(img_src->colorspace))
	{

		isp_to_cpp_plane(img_src->buffer[0], w_src, h_src, \
			img_src->stride[0], w_dst, h_dst, img_dst->stride[0], \
			img_dst->buffer[0], img_src->colorspace);

		w_src >>= 1;
		h_src >>= 1;
		w_dst >>= 1;
		h_dst >>= 1;
		isp_to_cpp_plane(img_src->buffer[1], w_src, h_src, \
			img_src->stride[1], w_dst, h_dst, img_dst->stride[1], \
			img_dst->buffer[1], img_src->colorspace);
		isp_to_cpp_plane(img_src->buffer[2], w_src, h_src, \
			img_src->stride[2], w_dst, h_dst, img_dst->stride[2], \
			img_dst->buffer[2], img_src->colorspace);
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}

	return S360_OK;
}

/***********************************  RISP  **********************************/

static int isp_to_risp_plane(uint8 * src, int w_src, int h_src, int s_src, \
	uint8 * dst, int s_dst, int w_tri, int h_tri, int opt, S360_SPH_COORD * map)
{
	
	int w_dst, h_dst;
	int i, j, y;

	w_dst = w_tri * 5;
	h_dst = h_tri * 2;

	for(j=0; j<h_tri; j++)
	{
		for(i=0; i<w_dst; i++)
		{
			y = (map[i + j * w_src].lng != -1 ? j : j + h_dst);
			dst[i + j * s_dst] = src[i + y * s_src];
		}
		for(i=0; i<(w_tri>>1); i++)
		{
			if(map[i + j * w_src].lng == -1)
			{
				dst[i + j * s_dst] = src[i + w_dst + (j + h_dst) * s_src];
			}
		}
	}

	for(j=h_tri; j<h_dst; j++)
	{
		s360_mcpy(dst + j * s_dst, src + j * s_src, w_dst * sizeof(uint8));
		for(i=0; i<(w_tri>>1); i++)
		{
			if(map[i + j * w_src].lng == -1)
			{
				dst[i + j * s_dst] = src[i + w_dst + j * s_src];
			}
		}
	}

	return S360_OK;
}

static int isp_to_risp_plane_10b(uint16 * src, int w_src, int h_src, int s_src, \
	uint16 * dst, int s_dst, int w_tri, int h_tri, int opt, S360_SPH_COORD * map)
{
	
	int w_dst, h_dst;
	int i, j, y;

	w_dst = w_tri * 5;
	h_dst = h_tri * 2;

	for(j=0; j<h_tri; j++)
	{
		for(i=0; i<w_dst; i++)
		{
			y = (map[i + j * w_src].lng != -1 ? j : j + h_dst);
			dst[i + j * s_dst] = src[i + y * s_src];
		}
		for(i=0; i<(w_tri>>1); i++)
		{
			if(map[i + j * w_src].lng == -1)
			{
				dst[i + j * s_dst] = src[i + w_dst + (j + h_dst) * s_src];
			}
		}
	}

	for(j=h_tri; j<h_dst; j++)
	{
		s360_mcpy(dst + j * s_dst, src + j * s_src, w_dst*sizeof(uint16));
		for(i=0; i<(w_tri>>1); i++)
		{
			if(map[i + j * w_src].lng == -1)
			{
				dst[i + j * s_dst] = src[i + w_dst + j * s_src];
			}
		}
	}

	return S360_OK;
}

int s360_isp_to_risp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;
	int w_tri, h_tri;
	int ret;

	w_src = img_src->width;
	h_src = img_src->height;
	w_tri = GET_W_TRI_ISP(w_src);
	h_tri = GET_H_TRI_ISP(w_tri);
	w_dst = w_tri * 5;
	h_dst = h_tri * 2;

	ret = s360_img_realloc(img_dst, w_dst, h_dst, opt);
	s360_assert_rv(ret == S360_OK, ret);

	if(img_src->colorspace == S360_COLORSPACE_YUV420)
	{
		isp_to_risp_plane((uint8*)img_src->buffer[0], w_src, h_src, img_src->stride[0],\
			(uint8*)img_dst->buffer[0], img_dst->stride[0], w_tri, h_tri, opt, map->layer[0]);

		w_tri >>= 1;
		h_tri >>= 1;
		w_src >>= 1;
		h_src >>= 1;
		isp_to_risp_plane((uint8*)img_src->buffer[1], w_src, h_src, img_src->stride[1],\
			(uint8*)img_dst->buffer[1], img_dst->stride[1], w_tri, h_tri, opt, map->layer[1]);
		isp_to_risp_plane((uint8*)img_src->buffer[2], w_src, h_src, img_src->stride[2],\
			(uint8*)img_dst->buffer[2], img_dst->stride[2], w_tri, h_tri, opt, map->layer[1]);
	}
	else if (img_src->colorspace == S360_COLORSPACE_YUV420_10)
	{
		isp_to_risp_plane_10b((uint16*)img_src->buffer[0], w_src, h_src, img_src->stride[0],\
			(uint16*)img_dst->buffer[0], img_dst->stride[0], w_tri, h_tri, opt, map->layer[0]);

		w_tri >>= 1;
		h_tri >>= 1;
		w_src >>= 1;
		h_src >>= 1;
		isp_to_risp_plane_10b((uint16*)img_src->buffer[1], w_src, h_src, img_src->stride[1],\
			(uint16*)img_dst->buffer[1], img_dst->stride[1], w_tri, h_tri, opt, map->layer[1]);
		isp_to_risp_plane_10b((uint16*)img_src->buffer[2], w_src, h_src, img_src->stride[2],\
			(uint16*)img_dst->buffer[2], img_dst->stride[2], w_tri, h_tri, opt, map->layer[1]);
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}
	return S360_OK;
}

static int risp_to_isp_plane(uint8 * src, int w_src, int h_src, int s_src, \
	uint8 * dst, int s_dst, int w_tri, int h_tri, int opt, S360_SPH_COORD * map)
{
	
	int w_dst, h_dst;
	int i, j;

	w_dst = (int)(w_tri * 5.5);
	h_dst = h_tri * 3;

	for(j=0; j<(h_tri<<1); j++)
	{
		for(i=0; i<w_src; i++)
		{
			if(map[i + j * w_dst].lng != -1)
			{
				dst[i + j * s_dst] = src[i + j * s_src];
				map[i + j * w_dst].lat = 9999;
			}
		}
		for(i=w_src; i<w_dst; i++)
		{
			if(map[i + j * w_dst].lng != -1)
			{
				dst[i + j * s_dst] = src[i - w_src + j * s_src];
				map[i + j * w_dst].lat = 9999;
			}
		}
	}

	for(j=0; j<h_tri; j++)
	{
		for(i=0; i<(w_tri>>1); i++)
		{
			if(map[i + j * w_dst].lng == -1)
			{
				dst[i + w_src + (j + (h_tri<<1)) * s_dst] = src[i + j * s_src];
				map[i + w_src + (j + (h_tri<<1)) * w_dst].lat = 9999;
			}
		}
		for(i=(w_tri>>1); i<w_src; i++)
		{
			if(map[i + j * w_dst].lng == -1)
			{
				dst[i + (j + (h_tri<<1)) * s_dst] = src[i + j * s_src];
				map[i + (j + (h_tri<<1)) * w_dst].lat = 9999;
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
		pad_isp_plane(dst, w_dst, h_dst, s_dst, map);
	}


	return S360_OK;
}

static int risp_to_isp_plane_10b(uint16 * src, int w_src, int h_src, int s_src, \
	uint16 * dst, int s_dst, int w_tri, int h_tri, int opt, S360_SPH_COORD * map)
{
	
	int w_dst, h_dst;
	int i, j;

	w_dst = (int)(w_tri * 5.5);
	h_dst = h_tri * 3;

	for(j=0; j<(h_tri<<1); j++)
	{
		for(i=0; i<w_src; i++)
		{
			if(map[i + j * w_dst].lng != -1)
			{
				dst[i + j * s_dst] = src[i + j * s_src];
				map[i + j * w_dst].lat = 9999;
			}
		}
		for(i=w_src; i<w_dst; i++)
		{
			if(map[i + j * w_dst].lng != -1)
			{
				dst[i + j * s_dst] = src[i - w_src + j * s_src];
				map[i + j * w_dst].lat = 9999;
			}
		}
	}

	for(j=0; j<h_tri; j++)
	{
		for(i=0; i<(w_tri>>1); i++)
		{
			if(map[i + j * w_dst].lng == -1)
			{
				dst[i + w_src + (j + (h_tri<<1)) * s_dst] = src[i + j * s_src];
				map[i + w_src + (j + (h_tri<<1)) * w_dst].lat = 9999;
			}
		}
		for(i=(w_tri>>1); i<w_src; i++)
		{
			if(map[i + j * w_dst].lng == -1)
			{
				dst[i + (j + (h_tri<<1)) * s_dst] = src[i + j * s_src];
				map[i + (j + (h_tri<<1)) * w_dst].lat = 9999;
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
		pad_isp_plane_10b(dst, w_dst, h_dst, s_dst, map);
	}

	return S360_OK;
}

int s360_risp_to_isp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;
	int w_tri, h_tri;

	w_src = img_src->width;
	h_src = img_src->height;
	w_tri = w_src / 5;
	h_tri = h_src / 2;
	w_dst = (int)(w_tri * 5.5);
	h_dst = h_tri * 3;

	s360_assert_rv(img_dst->width >= w_dst && img_dst->height >= h_dst, S360_ERR_INVALID_ARGUMENT);

	s360_img_reset(img_dst);
	
	if(img_src->colorspace == S360_COLORSPACE_YUV420)
	{
		risp_to_isp_plane((uint8*)img_src->buffer[0], w_src, h_src, img_src->stride[0],\
			(uint8*)img_dst->buffer[0], img_dst->stride[0], w_tri, h_tri, opt, map->layer[0]);

		w_tri >>= 1;
		h_tri >>= 1;
		w_src >>= 1;
		h_src >>= 1;
		risp_to_isp_plane((uint8*)img_src->buffer[1], w_src, h_src, img_src->stride[1],\
			(uint8*)img_dst->buffer[1], img_dst->stride[1], w_tri, h_tri, opt, map->layer[1]);
		risp_to_isp_plane((uint8*)img_src->buffer[2], w_src, h_src, img_src->stride[2],\
			(uint8*)img_dst->buffer[2], img_dst->stride[2], w_tri, h_tri, opt, map->layer[1]);
	}
	else if(img_src->colorspace == S360_COLORSPACE_YUV420_10)
	{
		risp_to_isp_plane_10b((uint16*)img_src->buffer[0], w_src, h_src, img_src->stride[0],\
			(uint16*)img_dst->buffer[0], img_dst->stride[0], w_tri, h_tri, opt, map->layer[0]);

		w_tri >>= 1;
		h_tri >>= 1;
		w_src >>= 1;
		h_src >>= 1;
		risp_to_isp_plane_10b((uint16*)img_src->buffer[1], w_src, h_src, img_src->stride[1],\
			(uint16*)img_dst->buffer[1], img_dst->stride[1], w_tri, h_tri, opt, map->layer[1]);
		risp_to_isp_plane_10b((uint16*)img_src->buffer[2], w_src, h_src, img_src->stride[2],\
			(uint16*)img_dst->buffer[2], img_dst->stride[2], w_tri, h_tri, opt, map->layer[1]);
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}
	return S360_OK;
}

/*********************************  RISP V2  *********************************/

/* ISP to RISP2 plane function */

static void copy_rect(S360_SPH_COORD * map, void * src, void * dst, \
	int w_map, int x_src, int y_src, int x_dst, int y_dst, int w, int h,	\
	int s_src, int s_dst, int cs, uint8 * map_dst)
{
	int      i, j;
	uint8  * src_8, * dst_8;
	uint16 * src_16, * dst_16;

	map_dst = map_dst + ((y_dst*s_dst) + x_dst);
	if(cs == S360_COLORSPACE_YUV420_10)
	{
		src_16 = (uint16 *)src + ((y_src*s_src) + x_src);
		dst_16 = (uint16 *)dst + ((y_dst*s_dst) + x_dst);
		
		for(j=0;j<h;j++)
		{
			for(i=0;i<w;i++)
			{
				if(map[j*w_map + i].lat != -1)
				{
					dst_16[i] = src_16[i];
					map_dst[j*s_dst + i] = 1;
				}
			}
			dst_16 += s_dst;
			src_16 += s_src;
		}
	}
	else if(cs == S360_COLORSPACE_YUV420)
	{
		src_8 = (uint8 *)src + ((y_src*s_src) + x_src);
		dst_8 = (uint8 *)dst + ((y_dst*s_dst) + x_dst);

		for(j=0;j<h;j++)
		{
			for(i=0;i<w;i++)
			{
				if(map[j*w_map + i].lat != -1)
				{
					dst_8[i] = src_8[i];
					map_dst[j*s_dst + i] = 1;
				}
			}
			dst_8 += s_dst;
			src_8 += s_src;
		}
	}
}

static void flip_rect_hor_to_isp(S360_SPH_COORD * map, void * src, void * dst, \
	int w_map, int x_src, int y_src, int x_dst, int y_dst, int w, int h,	\
	int s_src, int s_dst, int cs, uint8 *map_dst)
{
	int      i, j;
	uint8  * src_8, * dst_8;
	uint16 * src_16, * dst_16;

	map_dst = map_dst + ((y_dst*s_dst) + x_dst) + (h-1)*s_dst;

	if(cs == S360_COLORSPACE_YUV420_10)
	{
		src_16  = (uint16 *)src + ((y_src*s_src) + x_src);
		dst_16  = (uint16 *)dst + ((y_dst*s_dst) + x_dst) + (h-1)*s_dst;
		map    += (h-1)*w_map;

		for(j=0;j<h;j++)
		{
			for(i=0;i<w;i++)
			{
				if(map[i].lat != -1)
				{
					dst_16[i]  = src_16[i];
					map_dst[i] = 1;
				}
			}
			map     -= w_map;
			map_dst -= s_dst;
			dst_16  -= s_dst;
			src_16  += s_src;
		}
	}
	else if(cs == S360_COLORSPACE_YUV420)
	{
		src_8   = (uint8 *)src + ((y_src*s_src) + x_src);
		dst_8   = (uint8 *)dst + ((y_dst*s_dst) + x_dst) + (h-1)*s_dst;
		map    += (h-1)*w_map;

		for(j=0;j<h;j++)
		{
			for(i=0;i<w;i++)
			{
				if(map[i].lat != -1)
				{
					dst_8[i]   = src_8[i];
					map_dst[i] = 1;
				}
			}
			map     -= w_map;
			map_dst -= s_dst;
			dst_8   -= s_dst;
			src_8   += s_src;
		}
	}
}

static void flip_rect_hor(S360_SPH_COORD * map, void * src, void * dst, \
	int w_map, int x_src, int y_src, int x_dst, int y_dst, int w, int h,	\
	int s_src, int s_dst, int cs, uint8 *map_dst)
{
	int      i, j;
	uint8  * src_8, * dst_8;
	uint16 * src_16, * dst_16;

	map_dst = map_dst + ((y_dst*s_dst) + x_dst) + (h-1)*s_dst;
	if(cs == S360_COLORSPACE_YUV420_10)
	{
		src_16  = (uint16 *)src + ((y_src*s_src) + x_src);
		dst_16  = (uint16 *)dst + ((y_dst*s_dst) + x_dst) + (h-1)*s_dst;

		for(j=0;j<h;j++)
		{
			for(i=0;i<w;i++)
			{
				if(map[j*w_map + i].lat != -1)
				{
					dst_16[i] = src_16[i];
					map_dst[i] = 1;
				}
			}
			dst_16  -= s_dst;
			map_dst -= s_dst;
			src_16  += s_src;
		}
	}
	else if(cs == S360_COLORSPACE_YUV420)
	{
		src_8 = (uint8 *)src + ((y_src*s_src) + x_src);
		dst_8 = (uint8 *)dst + ((y_dst*s_dst) + x_dst) + (h-1)*s_dst;

		for(j=0;j<h;j++)
		{
			for(i=0;i<w;i++)
			{
				if(map[j*w_map + i].lat != -1)
				{
					dst_8[i] = src_8[i];
					map_dst[i] = 1;
				}
			}
			map_dst -= s_dst;
			dst_8   -= s_dst;
			src_8   += s_src;
		}
	}
}

static void fill_risp2_hole(void * risp, int w, int h, int s, uint8 * map, int cs)
{
	int      i, j, k;
	int      dist;
	uint8  * p8;
	uint16 * p16; 
	uint8    val_8, val_8_end;
	uint16   val_16, val_16_end;

	if(cs == S360_COLORSPACE_YUV420_10)
	{
		p16 = (uint16 *)risp;
		for(i=0;i<w;i++)
		{
			for(j=0;j<h;j++)
			{
				if(map[i+j*s] == 0)
				{
					dist = 1;
					
					while(map[i + (j+dist)*s] == 0) dist++;
					
					val_16_end = p16[i + (j+dist)*s];

					for(k=0;k<dist;k++)
					{
						p16[i+(j+k)*s] = (uint16)((k*val_16_end + (dist-k)*val_16)/\
							(float)dist);
					}
					j += dist;
				}
				val_16 = p16[i+j*s];
			}
		}
	}
	else if(cs == S360_COLORSPACE_YUV420)
	{
		p8 = (uint8 *)risp;
		for(i=0;i<w;i++)
		{
			for(j=0;j<h;j++)
			{
				if(map[i+j*s] == 0)
				{
					dist = 1;
					
					while(map[i + (j+dist)*s] == 0) dist++;
					val_8_end = p8[i + (j+dist)*s];
					for(k=0;k<dist;k++)
					{
						p8[i+(j+k)*s] = (uint8)((k*val_8_end + (dist-k)*val_8)/\
							(float)dist);
					}
					j += dist;
				}
				val_8 = p8[i+j*s];
			}
		}
	}
}

/* ISP to RISP2 plane function */
static int isp_to_risp2_plane(void *src, int w_src, int h_src, int s_src, \
	void *dst, int s_dst, int w_tri, int h_tri, int is_luma, \
	S360_SPH_COORD * map, int cs)
{
	int w_dst, h_dst;
	int offset, pad;
	uint8 * map_dst;

	w_dst = (int)(w_tri * 2.5);
	h_dst = h_tri * 4;
	pad   = (is_luma)?RISP2_PAD:(RISP2_PAD>>1);

	map_dst = (uint8 *)s360_malloc(s_dst*(h_dst+2*pad)*sizeof(uint8));
	s360_mset(map_dst, 0, (s_dst*(h_dst+2*pad)*sizeof(uint8)));

	// Top Triangles
	offset = (h_tri<<1)*w_src + (w_tri*5);
	copy_rect(map + offset, src, dst, w_src, w_tri*5, h_tri<<1, 0, 0, \
		w_tri>>1, h_tri, s_src, s_dst, cs, map_dst);

	offset = (h_tri<<1)*w_src + (w_tri>>1);
	copy_rect(map + offset, src, dst, w_src, (w_tri>>1), h_tri<<1, \
		(w_tri>>1), 0, w_tri<<1, h_tri, s_src, s_dst, cs, map_dst);
	
	// Centre-Block
	offset = w_dst;
	flip_rect_hor(map + offset, src, dst, w_src, w_dst, 0, 0, pad, w_dst, \
		h_tri*3, s_src, s_dst, cs, map_dst);
	
	//Bottom Triangles
	offset = 0;
	copy_rect(map + offset, src, dst, w_src, 0, 0, 0, (h_tri + pad)<<1, w_dst,\
		h_tri<<1, s_src, s_dst, cs, map_dst);
	
	offset = h_tri*w_src + 5*w_tri;
	copy_rect(map + offset, src, dst, w_src, 5*w_tri, h_tri, 0, \
		3*h_tri+(pad<<1), w_tri>>1, h_tri, s_src, s_dst, cs, map_dst);

	// Fill Pad
	fill_risp2_hole(dst, w_dst, h_dst, s_dst, map_dst, cs);

	s360_mfree(map_dst);
	
	return S360_OK;
}

/* ISP to RISPv2 main function */
int s360_isp_to_risp2(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, \
	S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;
	int w_tri, h_tri;
	int ret;

	if(!IS_VALID_CS(img_src->colorspace))
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}

	w_src = img_src->width;
	h_src = img_src->height;
	w_tri = GET_W_TRI_ISP(w_src);
	h_tri = GET_H_TRI_ISP(w_tri);
	w_dst = (int)(w_tri * 2.5);
	h_dst = h_tri * 4 + (RISP2_PAD<<1);

	ret = s360_img_realloc(img_dst, w_dst, h_dst, opt);
	s360_assert_rv(ret == S360_OK, ret);

	s360_img_reset(img_dst);

	isp_to_risp2_plane(img_src->buffer[0], w_src, h_src, img_src->stride[0],\
		img_dst->buffer[0], img_dst->stride[0], w_tri, h_tri, 1, \
		(S360_SPH_COORD *)map->layer[0], img_src->colorspace);
	w_tri >>= 1;
	h_tri >>= 1;
	w_src >>= 1;
	h_src >>= 1;
	isp_to_risp2_plane(img_src->buffer[1], w_src, h_src, img_src->stride[1],\
		img_dst->buffer[1], img_dst->stride[1], w_tri, h_tri, 0, \
		(S360_SPH_COORD *)map->layer[1], img_src->colorspace);
	isp_to_risp2_plane(img_src->buffer[2], w_src, h_src, img_src->stride[2],\
		img_dst->buffer[2], img_dst->stride[2], w_tri, h_tri, 0, \
		(S360_SPH_COORD *)map->layer[1], img_src->colorspace);
	
	return S360_OK;
}

/* RISP2 to ISP plane function */
static int risp2_to_isp_plane(void *src, int w_src, int h_src, int s_src, \
	void *dst, int s_dst, int w_tri, int h_tri, int opt, S360_SPH_COORD * map, \
	int cs, unsigned int is_luma)
{
	int w_dst, h_dst;
	int offset;
	int pad; 
	uint8 * map_dst;

	w_dst = (int)(w_tri * 5.5);
	h_dst = h_tri * 3;
	pad   = (is_luma)?RISP2_PAD:(RISP2_PAD>>1);

	map_dst = (uint8 *)s360_malloc(s_dst*(h_dst+(pad<<1))*sizeof(uint8));
	s360_mset(map_dst, 0, (s_dst*(h_dst+(pad<<1))*sizeof(uint8)));

	offset = (w_tri>>1) + (h_tri<<1)*w_dst;
	copy_rect(map+offset, src, dst, w_dst, (w_tri>>1), 0, (w_tri>>1), \
		(h_tri<<1), (w_tri<<1), h_tri, s_src, s_dst, cs, map_dst);

	offset = (h_tri<<1)*w_dst + w_tri*5;
	copy_rect(map+offset, src, dst, w_dst, 0, 0, w_tri*5, h_tri<<1, \
		w_tri>>1, h_tri, s_src, s_dst, cs, map_dst);

	offset = w_src;
	flip_rect_hor_to_isp(map+offset, src, dst, w_dst, 0, pad, w_src, \
		0, w_src, h_tri*3, s_src, s_dst, cs, map_dst);

	offset = 0;
	copy_rect(map+offset, src, dst, w_dst, 0, (h_tri<<1) + (pad<<1), 0, 0, \
		w_src, (h_tri<<1), s_src, s_dst, cs, map_dst);

	offset = h_tri*w_dst + w_tri*5;
	copy_rect(map+offset, src, dst, w_dst, 0, h_tri*3 + (pad<<1), w_tri*5, \
		h_tri, w_tri>>1, h_tri, s_src, s_dst, cs, map_dst);

	//Padding
	if(opt & S360_OPT_PAD)
	{
		if(cs == S360_COLORSPACE_YUV420_10)
		{
			pad_isp_plane_10b((uint16 *)dst, w_dst, h_dst, s_dst, map);
		}
		else if(cs == S360_COLORSPACE_YUV420)
		{
			pad_isp_plane((uint8 *)dst, w_dst, h_dst, s_dst, map);
		}
	}

	free(map_dst);

	return S360_OK;
}

/* RISPv2 to ISP main function */
int s360_risp2_to_isp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, \
    S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;
	int w_tri, h_tri;

	if(!IS_VALID_CS(img_src->colorspace))
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}

	w_src = img_src->width;
	h_src = img_src->height;
	w_tri = (int)(w_src / 2.5);
	h_tri = (h_src-(RISP2_PAD<<1)) / 4;
	w_dst = (int)(w_tri * 5.5);
	h_dst = h_tri * 3;

	s360_assert_rv(img_dst->width >= w_dst && img_dst->height >= h_dst, \
		S360_ERR_INVALID_ARGUMENT);

	s360_img_reset(img_dst);

	risp2_to_isp_plane(img_src->buffer[0], w_src, h_src, img_src->stride[0],\
		img_dst->buffer[0], img_dst->stride[0], w_tri, h_tri, opt, \
		(S360_SPH_COORD *)map->layer[0], img_src->colorspace, 1);

	w_tri >>= 1;
	h_tri >>= 1;
	w_src >>= 1;
	h_src >>= 1;

	risp2_to_isp_plane(img_src->buffer[1], w_src, h_src, img_src->stride[1],\
		img_dst->buffer[1], img_dst->stride[1], w_tri, h_tri, opt, \
		(S360_SPH_COORD *)map->layer[1], img_src->colorspace, 0);
	risp2_to_isp_plane(img_src->buffer[2], w_src, h_src, img_src->stride[2],\
		img_dst->buffer[2], img_dst->stride[2], w_tri, h_tri, opt, \
		(S360_SPH_COORD *)map->layer[1], img_src->colorspace, 0);

	return S360_OK;
}

/***********************  RISP V1 direct transformation  *********************/
static int risp1_to_erp_plane(void * src, int w_src, int h_src, int s_src, \
	int w_dst, int h_dst, int s_dst, void * dst, int cs, int pad_gap)
{
	resample_fn fn_resample;
	double  n1[3], n2[3], normal[3], mid[3], xyz[3], t_vec[3];
	double  lng, lat, x, y, u;
	double  h_tri_3d, w_tri_3d;
	double  dist_23, dist_cmp1, dist_cmp2, d_ver, d_hor;
	int     tri, w_tri, h_tri, vertex_x, vertex_y;
	int     y1, y2, y3;
	int     v1, v2, v3;
	int     i, j;	

	fn_resample = resample_fp(cs);

	if(cs == S360_COLORSPACE_YUV420_10)
	{
		s_dst <<= 1;
	}

	w_tri = (int)((w_src-5*pad_gap)/ 5);
	h_tri = (int)(h_src / 2);
	y1    = h_tri;
	y2    = (h_tri * 2);
	y3    = (h_tri * 3);

	v3d_average(tbl_tri_xyz[1], tbl_tri_xyz[2], mid);

	v3d_sub(tbl_tri_xyz[0], mid, t_vec);
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

			tri = get_tri_idx(xyz[0], xyz[1], xyz[2], tbl_center_xyz);
			v1 = tbl_vidx_isp2erp[tri][0];
			v2 = tbl_vidx_isp2erp[tri][1];
			v3 = tbl_vidx_isp2erp[tri][2];

			v3d_sub(tbl_tri_xyz[v1], tbl_tri_xyz[v2], n1);
			v3d_sub(tbl_tri_xyz[v1], tbl_tri_xyz[v3], n2);

			normal[0] = -(n1[1] * n2[2] - n2[1] * n1[2]);
			normal[1] = -(n1[2] * n2[0] - n2[2] * n1[0]);
			normal[2] = -(n1[0] * n2[1] - n2[0] * n1[1]);

			v3d_dot(normal, xyz, n1);
			v3d_dot(normal, tbl_tri_xyz[v1], n2);

			v3d_scale(xyz, (n2[0] + n2[1] + n2[2]) / (n1[0] + n1[1] + n1[2]));

			v3d_sub(tbl_tri_xyz[v3], tbl_tri_xyz[v2], n1);
			dist_23 = n1[0] * n1[0] + n1[1] * n1[1] + n1[2] * n1[2];

			v3d_sub(xyz, tbl_tri_xyz[v2], t_vec);
			v3d_dot(t_vec, n1, t_vec);
			u = (t_vec[0] + t_vec[1] + t_vec[2]) / dist_23;

			v3d_average(tbl_tri_xyz[v2], tbl_tri_xyz[v3], mid);

			v3d_sub(tbl_tri_xyz[v2], tbl_tri_xyz[v3], t_vec);
			w_tri_3d = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);
			v3d_sub(tbl_tri_xyz[v1], mid, t_vec);
			h_tri_3d = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);

			v3d_affine(n1, u, tbl_tri_xyz[v2], n2);

			v3d_sub(mid, n2, t_vec);
			d_hor = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);
			v3d_sub(xyz, n2, t_vec);
			d_ver = h_tri_3d - GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);

			x = (w_tri / w_tri_3d) * d_hor;
			y = (h_tri / h_tri_3d) * d_ver;

			v3d_sub(tbl_tri_xyz[v2], n2, t_vec);
			dist_cmp1 = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);
			v3d_sub(tbl_tri_xyz[v3], n2, t_vec);
			dist_cmp2 = GET_DIST3D(t_vec[0], t_vec[1], t_vec[2]);
			switch(tri)
			{
			case 0:
				vertex_x = (w_tri >> 1) + (w_tri << 1) + (5*pad_gap>>1);
				vertex_y = 0;
				break;
			case 1:
				vertex_x = (w_tri >> 1) + 3 * w_tri + (7*pad_gap>>1);
				vertex_y = 0;
				break;
			case 2:
				vertex_x = (w_tri >> 1) + (w_tri << 2) + (9*pad_gap>>1);
				vertex_y = 0;
				break;
			case 3:
				vertex_x = (w_tri >> 1) + (pad_gap>>1);
				vertex_y = 0;
				break;
			case 4:
				vertex_x = (w_tri >> 1) + w_tri + (3*pad_gap>>1);
				vertex_y = 0;
				break;
			case 5:
				vertex_x = (w_tri << 1) + 2*pad_gap;
				vertex_y = y1;
				break;
			case 7:
				vertex_x = 3 * w_tri + 3*pad_gap;
				vertex_y = y1;
				break;
			case 9:
				vertex_x = (w_tri << 2) + 4*pad_gap;
				vertex_y = y1;
				break;
			case 11:
				vertex_x = 5 * w_tri + 5*pad_gap;
				vertex_y = y1;
				break;
			case 13:
				vertex_x = w_tri + pad_gap;
				vertex_y = y1;
				break;
			case 6:
				vertex_x = (w_tri >> 1) + (w_tri << 1) + (5*pad_gap>>1);
				vertex_y = y2;
				break;
			case 8:
				vertex_x = (w_tri >> 1) + 3 * w_tri + (7*pad_gap>>1);
				vertex_y = y2;
				break;
			case 10:
				vertex_x = (w_tri >> 1) + (w_tri << 2) + (9*pad_gap>>1);
				vertex_y = y2;
				break;
			case 12:
				vertex_x = (w_tri >> 1) + (pad_gap>>1);
				vertex_y = y2;
				break;
			case 14:
				vertex_x = (w_tri >> 1) + w_tri + (3*pad_gap>>1);
				vertex_y = y2;
				break;

			case 15:
				vertex_x = (w_tri << 1) + 2*pad_gap;
				vertex_y = y1;
				break;
			case 16:
				vertex_x = 3 * w_tri + 3*pad_gap;
				vertex_y = y1;
				break;
			case 17:
				vertex_x = (w_tri << 2) + 4*pad_gap;
				vertex_y = y1;
				break;
			case 18:
				vertex_x = 5 * w_tri + 5*pad_gap;
				vertex_y = y1;
				break;
			case 19:
				vertex_x = w_tri + pad_gap;
				vertex_y = y1;
				break;
			}
			if(vertex_y >= y2 || tri > 14)
			{
				y = -y;
			}
			if(dist_cmp1 > dist_cmp2)
			{
				x = vertex_x + x;
				y = vertex_y + y;

				if(x>=w_src)
					x = x - w_src;
				if(x<0)
					x = w_src+x;
			}
			else
			{
				x = vertex_x - x;
				y = vertex_y + y;
				
				if(x>=w_src)
					x = x - w_src;
				if(x<0)
					x = w_src+x;
			}
			fn_resample(src, 0, w_src, 0, h_src, s_src, x, y, dst, i);
		}

		dst = (void *)((uint8 *)dst + s_dst);
	}

	return S360_OK;
}

int s360_risp1_to_erp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;
	int w_tri, h_tri;
	int pad_sz;
	
	w_dst = img_dst->width;
	h_dst = img_dst->height;

	w_src = img_src->width;
	h_src = img_src->height;

	if (opt & S360_OPT_PAD)
	{
		pad_sz = PAD_SIZE_RISP/4;
	}
	else
	{
		pad_sz = 0;
	}
	w_tri = ((w_src-20*pad_sz)/5);
	h_tri = (h_src/2);

	s360_assert_rv((w_src-20*pad_sz) == (w_tri * 5), S360_ERR_INVALID_DIMENSION);
	s360_assert_rv(h_src == 2 * h_tri, S360_ERR_INVALID_DIMENSION);

	if(IS_VALID_CS(img_src->colorspace))
	{
		risp1_to_erp_plane(img_src->buffer[0], w_src, h_src, img_src->stride[0], w_dst, h_dst, img_dst->stride[0], img_dst->buffer[0], img_src->colorspace,pad_sz<<2);

		w_src >>= 1;
		h_src >>= 1;
		w_dst >>= 1;
		h_dst >>= 1;
		risp1_to_erp_plane(img_src->buffer[1], w_src, h_src, img_src->stride[1], w_dst, h_dst, img_dst->stride[1], img_dst->buffer[1], img_src->colorspace,pad_sz<<1);
		risp1_to_erp_plane(img_src->buffer[2], w_src, h_src, img_src->stride[2], w_dst, h_dst, img_dst->stride[2], img_dst->buffer[2], img_src->colorspace,pad_sz<<1);
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}

	return S360_OK;
}

int s360_erp_to_risp1(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;
	int w_tri, h_tri;
	int ret;
	int pad_sz;

	w_src = img_src->width;
	h_src = img_src->height;

	w_dst = img_dst->width;
	h_dst = img_dst->height;

	w_tri = (w_dst/5);
	h_tri = (h_dst/3);

	ret = s360_img_realloc(img_dst, w_dst, h_dst, opt);
	s360_assert_rv(ret == S360_OK, ret);

	s360_img_reset(img_dst);

	pad_sz = (opt & S360_OPT_PAD)?PAD_SIZE:0;

	if(opt & S360_OPT_PAD)
		s360_pad_erp(img_src);

	if(IS_VALID_CS(img_src->colorspace))
	{
		erp_to_isp_plane(img_src->buffer[0], w_src, h_src, img_src->stride[0], w_dst, h_dst, img_dst->stride[0], img_dst->buffer[0], w_tri, opt, img_src->colorspace, pad_sz, map->layer[0]);	

		w_tri  >>= 1;
		w_src  >>= 1;
		h_src  >>= 1;
		w_dst  >>= 1;
		h_dst  >>= 1;
		pad_sz >>= 1;
		erp_to_isp_plane(img_src->buffer[1], w_src, h_src, img_src->stride[1], w_dst, h_dst, img_dst->stride[1], img_dst->buffer[1], w_tri, opt, img_src->colorspace, pad_sz, map->layer[1]);
		erp_to_isp_plane(img_src->buffer[2], w_src, h_src, img_src->stride[2], w_dst, h_dst, img_dst->stride[2], img_dst->buffer[2], w_tri, opt, img_src->colorspace, pad_sz, map->layer[1]);
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}

	return S360_OK;
}