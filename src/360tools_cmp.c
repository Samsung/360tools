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
#include "360tools_cmp.h"

#if USE_MIRROR_PADDING
static void pad_cmp_plane(uint8 *dst, int w_dst, int h_dst, int s_dst, int w_squ, int pad_size)
{
	uint8 *buf, *temp_dst;
	int i, j;

	temp_dst = dst;
	buf = dst;
	dst += (w_squ - 1) * s_dst;
	buf += w_squ;
	for (j = 0; j < pad_size; j++)
	{
		for (i = 0; i < w_squ; i++)
		{
			dst[i] = buf[i * s_dst + j];
		}
		dst -= s_dst;
	}

	dst = temp_dst;
	buf = temp_dst;
	dst += w_squ - 1;
	buf += w_squ*s_dst;
	for (j = 0; j < pad_size; j++)
	{
		for (i = 0; i < w_squ; i++)
		{
			dst[i * s_dst - j] = buf[i];
		}
		buf += s_dst;
	}

	dst = temp_dst;
	buf = temp_dst;
	dst += 2 * w_squ * s_dst;
	buf += w_squ + (3 * w_squ - 1)*s_dst;
	for (j = 0; j < pad_size; j++)
	{
		for (i = 0; i < w_squ; i++)
		{
			dst[i] = buf[j - i * s_dst];
		}
		dst += s_dst;
	}

	dst = temp_dst;
	buf = temp_dst;
	dst += w_squ - 1 + (3 * w_squ - 1) * s_dst;
	buf += (2 * w_squ - 1)*s_dst;
	for (j = 0; j < pad_size; j++)
	{
		for (i = 0; i < w_squ; i++)
		{
			dst[(-i) * s_dst - j] = buf[i];
		}
		buf -= s_dst;
	}

	dst = temp_dst;
	buf = temp_dst;
	dst += (w_squ - 1) * s_dst + 2 * w_squ;
	buf += 2 * w_squ - 1 + (w_squ - 1) * s_dst;
	for (j = 0; j < pad_size; j++)
	{
		for (i = 0; i < w_squ; i++)
		{
			dst[i] = buf[(-i) * s_dst - j];
		}
		dst -= s_dst;
	}

	dst = temp_dst;
	buf = temp_dst;
	dst += 2 * w_squ + (w_squ - 1) * s_dst;
	buf += w_squ * s_dst + 2 * w_squ;
	for (j = 0; j < pad_size; j++)
	{
		for (i = 0; i < w_squ; i++)
		{
			dst[(-i) * s_dst + j] = buf[i];
		}
		buf += s_dst;
	}

	dst = temp_dst;
	buf = temp_dst;
	dst += 2 * w_squ * s_dst + 3 * w_squ - 1;
	buf += 2 * w_squ - 1 + (3 * w_squ - 1) * s_dst;
	for (j = 0; j < pad_size; j++)
	{
		for (i = 0; i < w_squ; i++)
		{
			dst[-i] = buf[(-i) * s_dst - j];
		}
		dst += s_dst;
	}

	dst = temp_dst;
	buf = temp_dst;
	dst += 2 * w_squ + (3 * w_squ - 1) * s_dst;
	buf += 3 * w_squ - 1 + (2 * w_squ - 1)*s_dst;
	for (j = 0; j < pad_size; j++)
	{
		for (i = 0; i < w_squ; i++)
		{
			dst[(-i) * s_dst + j] = buf[-i];
		}
		buf -= s_dst;
	}

	dst = temp_dst;
	buf = temp_dst;
	dst += 4 * w_squ - 1 + (w_squ - 1)  *s_dst;
	buf += w_squ;
	for (j = 0; j < pad_size; j++)
	{
		for (i = 0; i < w_squ; i++)
		{
			dst[-i] = buf[i + (s_dst)* j];
		}
		dst -= s_dst;
		buf += s_dst;
	}

	dst = temp_dst;
	buf = temp_dst;
	dst += 4 * w_squ - 1 + (2 * w_squ) * s_dst;
	buf += w_squ + (3 * w_squ - 1) * s_dst;
	for (j = 0; j < pad_size; j++)
	{
		for (i = 0; i < w_squ; i++)
		{
			dst[-i] = buf[i];
		}
		dst += s_dst;
		buf -= s_dst;
	}

}

static void pad_cmp_plane_10b(uint16 *dst, int w_dst, int h_dst, int s_dst, int w_squ, int pad_size)
{
	uint16 *buf, *temp_dst;
	int i, j;

	temp_dst = dst;
	buf = dst;
	dst += (w_squ - 1) * s_dst;
	buf += w_squ;
	for (j = 0; j < pad_size; j++)
	{
		for (i = 0; i < w_squ; i++)
		{
			dst[i] = buf[i * s_dst + j];
		}
		dst -= s_dst;
	}

	dst = temp_dst;
	buf = temp_dst;
	dst += w_squ - 1;
	buf += w_squ*s_dst;
	for (j = 0; j < pad_size; j++)
	{
		for (i = 0; i < w_squ; i++)
		{
			dst[i * s_dst - j] = buf[i];
		}
		buf += s_dst;
	}

	dst = temp_dst;
	buf = temp_dst;
	dst += 2 * w_squ * s_dst;
	buf += w_squ + (3 * w_squ - 1)*s_dst;
	for (j = 0; j < pad_size; j++)
	{
		for (i = 0; i < w_squ; i++)
		{
			dst[i] = buf[j - i * s_dst];
		}
		dst += s_dst;
	}

	dst = temp_dst;
	buf = temp_dst;
	dst += w_squ - 1 + (3 * w_squ - 1) * s_dst;
	buf += (2 * w_squ - 1)*s_dst;
	for (j = 0; j < pad_size; j++)
	{
		for (i = 0; i < w_squ; i++)
		{
			dst[(-i) * s_dst - j] = buf[i];
		}
		buf -= s_dst;
	}

	dst = temp_dst;
	buf = temp_dst;
	dst += (w_squ - 1) * s_dst + 2 * w_squ;
	buf += 2 * w_squ - 1 + (w_squ - 1) * s_dst;
	for (j = 0; j < pad_size; j++)
	{
		for (i = 0; i < w_squ; i++)
		{
			dst[i] = buf[(-i) * s_dst - j];
		}
		dst -= s_dst;
	}

	dst = temp_dst;
	buf = temp_dst;
	dst += 2 * w_squ + (w_squ - 1) * s_dst;
	buf += w_squ * s_dst + 2 * w_squ;
	for (j = 0; j < pad_size; j++)
	{
		for (i = 0; i < w_squ; i++)
		{
			dst[(-i) * s_dst + j] = buf[i];
		}
		buf += s_dst;
	}

	dst = temp_dst;
	buf = temp_dst;
	dst += 2 * w_squ * s_dst + 3 * w_squ - 1;
	buf += 2 * w_squ - 1 + (3 * w_squ - 1) * s_dst;
	for (j = 0; j < pad_size; j++)
	{
		for (i = 0; i < w_squ; i++)
		{
			dst[-i] = buf[(-i) * s_dst - j];
		}
		dst += s_dst;
	}

	dst = temp_dst;
	buf = temp_dst;
	dst += 2 * w_squ + (3 * w_squ - 1) * s_dst;
	buf += 3 * w_squ - 1 + (2 * w_squ - 1)*s_dst;
	for (j = 0; j < pad_size; j++)
	{
		for (i = 0; i < w_squ; i++)
		{
			dst[(-i) * s_dst + j] = buf[-i];
		}
		buf -= s_dst;
	}

	dst = temp_dst;
	buf = temp_dst;
	dst += 4 * w_squ - 1 + (w_squ - 1)  *s_dst;
	buf += w_squ;
	for (j = 0; j < pad_size; j++)
	{
		for (i = 0; i < w_squ; i++)
		{
			dst[-i] = buf[i + (s_dst)* j];
		}
		dst -= s_dst;
		buf += s_dst;
	}

	dst = temp_dst;
	buf = temp_dst;
	dst += 4 * w_squ - 1 + (2 * w_squ) * s_dst;
	buf += w_squ + (3 * w_squ - 1) * s_dst;
	for (j = 0; j < pad_size; j++)
	{
		for (i = 0; i < w_squ; i++)
		{
			dst[-i] = buf[i];
		}
		dst += s_dst;
		buf -= s_dst;
	}

}
#else
static void pad_cmp_plane(uint8 *dst, int w_dst, int h_dst, int s_dst, int w_squ, int pad_size)
{
	int i, j;

	for (j = 0; j<pad_size; j++){
		for (i = 0; i<w_squ; i++)
		{
			/* -------------------  vertical padding -------------------------------*/
			dst[(w_squ - j)*s_dst + i] = dst[w_squ*s_dst + i];
			dst[(w_squ - j)*s_dst + i + 2 * w_squ] = dst[w_squ*s_dst + i + 2 * w_squ];
			dst[(w_squ - j)*s_dst + i + 3 * w_squ] = dst[w_squ*s_dst + i + 3 * w_squ];
			dst[(2 * w_squ + j)*s_dst + i] = dst[(2 * w_squ - 1)*s_dst + i];
			dst[(2 * w_squ + j)*s_dst + i + 2 * w_squ] = dst[(2 * w_squ - 1)*s_dst + i + 2 * w_squ];
			dst[(2 * w_squ + j)*s_dst + i + 3 * w_squ] = dst[(2 * w_squ - 1)*s_dst + i + 3 * w_squ];
			/* -------------------  horizontal padding -------------------------------*/
			dst[w_squ - j + s_dst*i] = dst[s_dst*i + w_squ];
			dst[2 * w_squ + j + s_dst*i] = dst[s_dst*i + 2 * w_squ - 1];
			dst[w_squ - j + s_dst*i + 2 * w_squ*s_dst] = dst[s_dst*i + w_squ + 2 * w_squ*s_dst];
			dst[2 * w_squ + j + s_dst*i + 2 * w_squ*s_dst] = dst[s_dst*i + 2 * w_squ - 1 + 2 * w_squ*s_dst];
		}
	}
}
#endif

static int erp_to_cmp_plane(void * src, int w_src, int h_src, int s_src, \
	int w_dst, int h_dst, int s_dst, void * dst, int w_squ, int opt, int pad_size, int cs, S360_SPH_COORD  * map)
{
	void(*fn_resample)(void * src, int w_start, int w_end, int h_src, int s_src,
		double x, double y, void * dst, int x_dst);

	double      x, y;
	int         i, j;
	int			w_start, w_end;

	w_start = opt ? -pad_size : 0;
	w_end = opt ? w_src + pad_size : w_src;

	if (cs == S360_COLORSPACE_YUV420)
	{
		fn_resample = resample_2d;
	}
	else if (cs == S360_COLORSPACE_YUV420_10)
	{
		fn_resample = resample_2d_10b;
		s_dst <<= 1;
	}

	for (j = 0; j<h_dst; j++)
	{
		for (i = 0; i<w_dst; i++)
		{
			if (map[i].lng != -1)
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

	if (opt & S360_OPT_PAD)
	{
		if (cs == S360_COLORSPACE_YUV420)
		{
			pad_cmp_plane(dst, w_dst, h_dst, s_dst, w_squ, pad_size);
		}
		else if (cs == S360_COLORSPACE_YUV420_10)
		{
			pad_cmp_plane_10b(dst, w_dst, h_dst, (s_dst >> 1), w_squ, pad_size);
		}
	}

	return S360_OK;
}

int s360_erp_to_cmp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;
	int w_squ;

	w_src = img_src->width;
	h_src = img_src->height;

	w_dst = img_dst->width;
	h_dst = img_dst->height;
	w_squ = NEAREST_EVEN(w_dst / 4.0);

	s360_img_reset(img_dst);
	if (opt & S360_OPT_PAD)
		s360_pad_erp(img_src);

	if (IS_VALID_CS(img_src->colorspace))
	{
		erp_to_cmp_plane(img_src->buffer[0], w_src, h_src, img_src->stride[0], w_dst, h_dst, img_dst->stride[0], img_dst->buffer[0], w_squ, opt, PAD_SIZE, img_src->colorspace, map->layer[0]);
		w_squ >>= 1;
		w_src >>= 1;
		h_src >>= 1;
		w_dst >>= 1;
		h_dst >>= 1;
		erp_to_cmp_plane(img_src->buffer[1], w_src, h_src, img_src->stride[1], w_dst, h_dst, img_dst->stride[1], img_dst->buffer[1], w_squ, opt, PAD_SIZE >> 1, img_src->colorspace, map->layer[1]);
		erp_to_cmp_plane(img_src->buffer[2], w_src, h_src, img_src->stride[2], w_dst, h_dst, img_dst->stride[2], img_dst->buffer[2], w_squ, opt, PAD_SIZE >> 1, img_src->colorspace, map->layer[1]);
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}
	return S360_OK;
}

/*-------------------------------   FROM CMP    -----------------------------*/
static int get_squ_idx(double x, double y, double z, const double center[6][3])
{
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

static void cmp_to_erp_sph2point(double  lng, double lat, int w_squ, double* x, \
	double* y, double d12, double d13)
{
	double	xyz[3], vec_12[3], vec_13[3], vec_1_xyz[3];
	double	d1_xyz;
	double	dist_ver, dist_hor;
	int     v_1_3d, v_2_3d, v_3_3d, v_4_3d;
	int	    x_init, y_init;
	int	    squ_idx;

	xyz[0] = sin(lat) * cos(lng);
	xyz[2] = sin(lat) * sin(lng);
	xyz[1] = cos(lat);

	squ_idx = get_squ_idx(xyz[0], xyz[1], xyz[2], tbl_squ_center_xyz);
	v_1_3d = tbl_vidx_erp2cmp[squ_idx][0];
	v_2_3d = tbl_vidx_erp2cmp[squ_idx][1];
	v_3_3d = tbl_vidx_erp2cmp[squ_idx][2];
	v_4_3d = tbl_vidx_erp2cmp[squ_idx][3];

	v3d_scale_face(tbl_face_eqn[squ_idx], xyz);
	v3d_sub(tbl_squ_xyz[v_2_3d], tbl_squ_xyz[v_1_3d], vec_12);
	v3d_sub(tbl_squ_xyz[v_3_3d], tbl_squ_xyz[v_1_3d], vec_13);
	v3d_sub(xyz, tbl_squ_xyz[v_1_3d], vec_1_xyz);
	d1_xyz = GET_DIST3D(vec_1_xyz[0], vec_1_xyz[1], vec_1_xyz[2]);

	dist_ver = v3d_dot_product(vec_12, vec_1_xyz) / d12;
	dist_hor = v3d_dot_product(vec_13, vec_1_xyz) / d13;

	*x = dist_hor * w_squ / d13;
	*y = dist_ver * w_squ / d12;

	cmp_plane_offset(&x_init, &y_init, squ_idx, w_squ);

	*x += x_init;
	*y += y_init;
}

static int cmp_to_erp_plane(void * src, int w_src, int h_src, int s_src, \
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

	v_1_3d = tbl_vidx_erp2cmp[0][0];
	v_2_3d = tbl_vidx_erp2cmp[0][1];
	v_3_3d = tbl_vidx_erp2cmp[0][2];

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

			cmp_to_erp_sph2point(lng, lat, w_squ, &x, &y, d12, d13);
			fn_resample(src, 0, w_src, h_src, s_src, x, y, dst, i);
		}
		dst = (void *)((uint8 *)dst + s_dst);
	}

	return S360_OK;
}

int s360_cmp_to_erp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;
	int w_squ;

	w_src = img_src->width;
	h_src = img_src->height;

	w_dst = img_dst->width;
	h_dst = img_dst->height;
	w_squ = NEAREST_EVEN(w_src / 4.0);

	if (IS_VALID_CS(img_src->colorspace))
	{
		cmp_to_erp_plane(img_src->buffer[0], w_src, h_src, img_src->stride[0], w_dst, h_dst, img_dst->stride[0], img_dst->buffer[0], w_squ, opt, img_src->colorspace);
		w_squ >>= 1;
		w_src >>= 1;
		h_src >>= 1;
		w_dst >>= 1;
		h_dst >>= 1;
		cmp_to_erp_plane(img_src->buffer[1], w_src, h_src, img_src->stride[1], w_dst, h_dst, img_dst->stride[1], img_dst->buffer[1], w_squ, opt, img_src->colorspace);
		cmp_to_erp_plane(img_src->buffer[2], w_src, h_src, img_src->stride[2], w_dst, h_dst, img_dst->stride[2], img_dst->buffer[2], w_squ, opt, img_src->colorspace);
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}

	return S360_OK;
}

static int cmp_to_cpp_plane(void * src, int w_src, int h_src, int s_src, \
	int w_dst, int h_dst, int s_dst, void * dst, int w_squ, int opt, int cs)
{
	void(*fn_resample)(void * src, int w_start, int w_end, int h_src, int s_src,
		double x, double y, void * dst, int x_dst);

	double	vec_12[3], vec_13[3];
	double  lng, lat, x, y;
	double	d12, d13;
	int     v_1_3d, v_2_3d, v_3_3d;
	int     i, j;
	int		* map, *map0;


	if (cs == S360_COLORSPACE_YUV420)
	{
		fn_resample = resample_2d;
	}
	else if (cs == S360_COLORSPACE_YUV420_10)
	{
		fn_resample = resample_2d_10b;
		s_dst <<= 1;
	}

	map = (int *)s360_malloc(sizeof(int)* h_dst * w_dst);
	v_1_3d = tbl_vidx_erp2cmp[0][0];
	v_2_3d = tbl_vidx_erp2cmp[0][1];
	v_3_3d = tbl_vidx_erp2cmp[0][2];

	v3d_sub(tbl_squ_xyz[v_2_3d], tbl_squ_xyz[v_1_3d], vec_12);
	d12 = GET_DIST3D(vec_12[0], vec_12[1], vec_12[2]);
	v3d_sub(tbl_squ_xyz[v_3_3d], tbl_squ_xyz[v_1_3d], vec_13);
	d13 = GET_DIST3D(vec_13[0], vec_13[1], vec_13[2]);

	map0 = map;
	for (j = 0; j<h_dst; j++)
	{
		for (i = 0; i<w_dst; i++)
		{
			*(map0 + i) = 0;
			x = ((double)i / (w_dst - 0)) * (M_2PI)-PI;
			y = ((double)j / (h_dst - 0)) * PI - (M_PI_2);

			lat = 3 * asin(y / PI);
			lng = x / (2 * cos(2 * lat / 3) - 1);

			x = (lng + PI) / 2 / PI * (w_dst - 0);
			y = (lat + (M_PI_2)) / PI * (h_dst - 0);

			if (x >= 0 && x <w_dst && y >= 0 && y<h_dst)
				*(map0 + i) = 1;
		}
		map0 += w_dst;
	}
	map0 = map;

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

			if (*(map0 + i) == 0)
				continue;

			cmp_to_erp_sph2point(lng, lat, w_squ, &x, &y, d12, d13);
			fn_resample(src, 0, w_src, h_src, s_src, x, y, dst, i);
		}

		dst = (void *)((uint8 *)dst + s_dst);
		map0 += w_dst;
	}
	s360_mfree(map);
	return S360_OK;
}

int s360_cmp_to_cpp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map)
{
	int w_src, h_src, w_dst, h_dst;
	int w_squ;

	w_src = img_src->width;
	h_src = img_src->height;

	w_dst = img_dst->width;
	h_dst = img_dst->height;
	w_squ = NEAREST_EVEN(w_src / 4.0);

	s360_img_reset(img_dst);

	if (IS_VALID_CS(img_dst->colorspace))
	{
		cmp_to_cpp_plane(img_src->buffer[0], w_src, h_src, img_src->stride[0], w_dst, h_dst, img_dst->stride[0], img_dst->buffer[0], w_squ, opt, img_dst->colorspace);
		w_squ >>= 1;
		w_src >>= 1;
		h_src >>= 1;
		w_dst >>= 1;
		h_dst >>= 1;
		cmp_to_cpp_plane(img_src->buffer[1], w_src, h_src, img_src->stride[1], w_dst, h_dst, img_dst->stride[1], img_dst->buffer[1], w_squ, opt, img_dst->colorspace);
		cmp_to_cpp_plane(img_src->buffer[2], w_src, h_src, img_src->stride[2], w_dst, h_dst, img_dst->stride[2], img_dst->buffer[2], w_squ, opt, img_dst->colorspace);
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}

	return S360_OK;
}