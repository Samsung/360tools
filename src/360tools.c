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
#include <math.h>
#include "360tools_def.h"
#include "360tools_img.h"

#if LANCZOS_FAST_MODE
static double tbl_lanczos_coef[LANCZOS_FAST_MAX_SIZE * LANCZOS_FAST_SCALE];
#endif

double v3d_norm(double v[3])
{
	double d = GET_DIST3D(v[0], v[1], v[2]);
	if (d != 0)
	{
		v[0] /= d;
		v[1] /= d;
		v[2] /= d;
	}
	return d;
}

double v3d_dot_product(double v1[3], double v2[3])
{
	return S360_ABS((v1[0] * v2[0]) + (v1[1] * v2[1]) + (v1[2] * v2[2]));
}

void cart_to_sph(double x, double y, double z, S360_SPH_COORD  * coord)
{
	coord->lat = (float)RAD2DEG(acos(z));
	if (coord->lat > 180) coord->lat = 180;

	if (x < 0)
	{
		coord->lng = 180.0f - (float)RAD2DEG(-atan(y / x));
	}
	else if (x == 0)
	{
		if (y < 0)
		{
			coord->lng = 270;
		}
		else if (y == 0)
		{
			coord->lng = (z < 0 ? 270.0f : 90.0f);
		}
		else
		{
			coord->lng = 90;
		}
	}
	else /* x>0 */
	{
		coord->lng = (float)RAD2DEG(atan(y / x));
		if (coord->lng < 0) coord->lng += 360;
	}

	if (coord->lng == 0) coord->lng = 0.01f;
	if (coord->lng == 360) coord->lng = 359.99f;
}

void v3d_scale_face(double eqn[3], double vec[3])
{
	double scale = (eqn[3]) / (eqn[0] * vec[0] + eqn[1] * vec[1] + eqn[2] * vec[2]);
	vec[0] = (vec[0] * scale);
	vec[1] = (vec[1] * scale);
	vec[2] = (vec[2] * scale);
}

void cmp_plane_offset(int *x, int *y, int squ_idx, int w_squ)
{
	switch (squ_idx)
	{
	case 0:
		*x = w_squ;
		*y = 0;
		break;
	case 1:
		*x = w_squ;
		*y = w_squ;
		break;
	case 2:
		*x = w_squ;
		*y = 2 * w_squ;
		break;
	case 3:
		*x = 0;
		*y = w_squ;
		break;
	case 4:
		*x = 2 * w_squ;
		*y = w_squ;
		break;
	case 5:
		*x = 3 * w_squ;
		*y = w_squ;
		break;
	default:
		break;
	}
}

void rcmp_plane_offset(int *x, int *y, int squ_idx, int w_squ)
{
	switch (squ_idx)
	{
	case 0:
		// 180 deg rotate
		*x = 2*w_squ;
		*y = w_squ;
		break;
	case 1:
		*x = 2*w_squ;
		*y = 0;
		break;
	case 2:
		*x = 0;    
		*y = w_squ;
		break;
	case 3:
		*x = w_squ;
		*y = 0;
		break;
	case 4:
		// 90 deg rotate
		*x = w_squ;
		*y = w_squ;
		break;
	case 5:
		*x = 0;
		*y = 0;
		break;
	default:
		break;
	}
 
}

void cpp_map_plane(int w_map, int h_map, int s_map, uint8 * map)
{
	int    i, j;
	int    idx_x, idx_y;
	double x, y, phi, lambda;

	s360_mset(map, 0, w_map * h_map);

	for(j=0;j<h_map;j++)
	{
		for(i=0;i<w_map;i++)
		{
			phi = 3 *  asin((double)j/h_map-0.5);
			lambda = (2 * PI * (double)i/w_map - PI) / (2 * cos(2 * phi/3) - 1);

			x = w_map * (lambda + PI)/(2*PI);
			y = h_map * (phi + PI/2)/PI;

			idx_x = (int)((x < 0) ? x - 0.5 : x + 0.5);
			idx_y = (int)((y < 0) ? y - 0.5 : y + 0.5);

			if(idx_y >= 0 && idx_x >= 0 && idx_x < w_map && idx_y < h_map)
			{
				map[i+j*w_map] = 1;
			}
		}
	}
}

void pad_cpp_plane(uint8 * buf, int w, int h, int s, uint8 * map0)
{
	uint8 * map = map0;
	int i, j, cnt;

	for(j=0; j<h; j++)
	{
		for(i=0; i<w; i++)
		{
			/* circular padding: L <--> R */
			if(map[i] == 1)
			{
				cnt = S360_MIN(PAD_SIZE, i);
				s360_mset(buf + i - cnt, buf[i], cnt);

				while(map[i] == 1 && i < w) i++;
				cnt = S360_MIN(PAD_SIZE, w - i);
				s360_mset(buf + i, buf[i-1], cnt);
				break;
			}
		}
		buf += s;
		map += w;
	}
}

void pad_cpp_plane_10b(uint16 * buf, int w, int h, int s, uint8 * map0)
{
	uint8 * map = map0;
	int i, j, k, cnt;

	for(j=0; j<h; j++)
	{
		for(i=0; i<w; i++)
		{
			/* circular padding: L <--> R */
			if(map[i] == 1)
			{
				cnt = S360_MIN(PAD_SIZE, i);
				for(k=1;k<=cnt;k++)
				{
					buf[i-k] = buf[i];
				}

				while(map[i] == 1 && i < w) i++;
				cnt = S360_MIN(PAD_SIZE, w - i);
				for(k=0;k<cnt;k++)
				{
					buf[i+k] = buf[i-1];
				}

				break;
			}
		}
		buf += s;
		map += w;
	}
}

static double sinc(double x)
{
	x *= PI;
	if(x < 0.01 && x > -0.01)
	{
		double x2 = x * x;
		return 1.0f + x2 * (-1.0 / 6.0 + x2 / 120.0);
	}
	else
	{
		return sin(x) / x;
	}
}

#if !LANCZOS_FAST_MODE
static double lanczos_coef(double x)
{
	return sinc(x) * sinc(x / LANCZOS_TAB_SIZE);
}
#else
#define lanczos_coef(x) tbl_lanczos_coef[(int)(fabs(x) * LANCZOS_FAST_SCALE + 0.5)]
#endif

int s360_init(void)
{
#if LANCZOS_FAST_MODE
	int i;
	for(i=0; i<LANCZOS_FAST_MAX_SIZE*LANCZOS_FAST_SCALE; i++)
	{
		float x = (float)i / LANCZOS_FAST_SCALE;
		tbl_lanczos_coef[i] = sinc(x) * sinc(x / LANCZOS_TAB_SIZE);
	}
#endif
	return S360_OK;
}

void s360_deinit(void)
{
    /*...*/
}

/* lanczos */
void resample_2d(void * src, int w_start, int w_end, int h_start, int h_end,\
	int s_src, double x, double y, void * dst, int x_dst)
{
	uint8 * src_8;
	uint8 * dst_8;
	float coef, sum = 0, res = 0;
	int i, j, idx_x, idx_y;

	src_8 = (uint8 *)src;
	dst_8 = (uint8 *)dst;

	for(j=-LANCZOS_TAB_SIZE; j<LANCZOS_TAB_SIZE; j++)
	{
		for(i=-LANCZOS_TAB_SIZE; i<LANCZOS_TAB_SIZE; i++)
		{
			idx_x = (int)x + i + 1;
			idx_y = (int)y + j + 1;
			if(idx_x >= w_start && idx_y >= h_start && idx_x < w_end && idx_y < h_end)
			{
				coef = (float)(lanczos_coef(x - idx_x) * lanczos_coef(y - idx_y));
				res += src_8[idx_x + idx_y * s_src] * coef;
				sum += coef;
			}
		}
	}
	if(sum != 0)
	{
		i = (int)(res / sum + 0.5);
		dst_8[x_dst] = S360_CLIP_S32_TO_U8(i);
	}
}

/* lanczos for 10-bit*/
void resample_2d_10b(void * src, int w_start, int w_end, int h_start, int h_end,\
	int s_src, double x, double y, void * dst, int x_dst)
{
	double coef, sum = 0, res = 0;
	int i, j, idx_x, idx_y;
	uint16 * src_16;
	uint16 * dst_16;

	src_16 = (uint16 *)src;
	dst_16 = (uint16 *)dst;

	for(j=-LANCZOS_TAB_SIZE; j<LANCZOS_TAB_SIZE; j++)
	{
		for(i=-LANCZOS_TAB_SIZE; i<LANCZOS_TAB_SIZE; i++)
		{
			idx_x = (int)x + i + 1;
			idx_y = (int)y + j + 1;
			if(idx_x >= w_start && idx_y >= h_start && idx_x < w_end && idx_y < h_end)
			{
				coef = lanczos_coef(x - idx_x) * lanczos_coef(y - idx_y);
				res += src_16[idx_x + idx_y * s_src] * coef;
				sum += coef;
			}
		}
	}
	if(sum != 0)
	{
		i = (int)(res / sum + 0.5);
		dst_16[x_dst] = S360_CLIP_S32_TO_U10(i);
	}
}

/* bi-linear */
void resample_bl_2d(void * src, int w_start, int w_end, int h_start, int h_end,\
	int s_src, double x, double y, void * dst, int x_dst)
{
	int val;
	int w_src;
	uint8 * src_8;
	uint8 * dst_8;

	src_8 = (uint8 *)src;
	dst_8 = (uint8 *)dst;

	w_src = w_end - w_start;

	if(x > w_start-1 && y > w_end-1 && x < w_src && y < h_end)
	{
		double dx = x - (int)x;
		double dy = y - (int)y;
		double w0 = (1 - dx) * (1 - dy);
		double w1 = dx * (1 - dy);
		double w2 = (1 - dx) * dy;
		double w3 = dx * dy;

		double s0, s1, s2, s3;

		int ox = ((int)x + 1 < w_src ? 1 : 0);
		int oy = ((int)y + 1 < h_end ? s_src : 0);
		int off0 = ((int)x + (int)y * s_src);
		int off1 = off0 + ox;
		int off2 = off0 + oy;
		int off3 = off1 + oy;

		s0 = src_8[off0];
		s1 = src_8[off1];
		s2 = src_8[off2];
		s3 = src_8[off3];
		val = (int)(w0 * s0 + w1 * s1 + w2 * s2 + w3 * s3 + 0.5);
		dst_8[x_dst] = S360_CLIP_S32_TO_U8(val);
	}
}

/* bi-linear */
void resample_bl_2d_10b(void * src, int w_start, int w_end, int h_start, int h_end,\
	int s_src, double x, double y, void * dst, int x_dst)
{
	int val;
	int w_src;
	uint16 * src_16;
	uint16 * dst_16;

	src_16 = (uint16 *)src;
	dst_16 = (uint16 *)dst;

	w_src = w_end - w_start;

	if(x > -1 && y > -1 && x < w_src && y < h_end)
	{
		double dx = x - (int)x;
		double dy = y - (int)y;
		double w0 = (1 - dx) * (1 - dy);
		double w1 = dx * (1 - dy);
		double w2 = (1 - dx) * dy;
		double w3 = dx * dy;

		double s0, s1, s2, s3;

		int ox = ((int)x + 1 < w_src ? 1 : 0);
		int oy = ((int)y + 1 < h_end ? s_src : 0);
		int off0 = ((int)x + (int)y * s_src);
		int off1 = off0 + ox;
		int off2 = off0 + oy;
		int off3 = off1 + oy;

		s0 = src_16[off0];
		s1 = src_16[off1];
		s2 = src_16[off2];
		s3 = src_16[off3];
		val = (int)(w0 * s0 + w1 * s1 + w2 * s2 + w3 * s3 + 0.5);
		dst_16[x_dst] = S360_CLIP_S32_TO_U10(val);
	}
}

/* TSP related */

void resample_tsp_2d(void * src, int w_start, int w_end, int w_src, int h_src, int s_src, \
	void * dst, int w_dst, int i, int j, S360_SPH_COORD  * map)
{
	int x, y, m, n;
	uint8 * src_8;
	uint8 * dst_8;
	int sub = TSPAA_S;
	int num = 0;
	int subdiv2 = (sub>>1);
	uint32 map_idx = 0, sum = 0, a = 0;

	src_8 = (uint8 *)src;
	dst_8 = (uint8 *)dst;

	for (n = -subdiv2; n <= subdiv2; n++)
	{
		for (m = -subdiv2; m <= subdiv2; m++)
		{
			int idx_x = i * sub + m;
			int idx_y = j * sub + n;
			if (idx_x < 0) idx_x = 0;
			if (idx_y < 0) idx_y = 0;
			map_idx = idx_y * w_dst * sub + idx_x;
			if (map[map_idx].lng != -1)
			{
				x = (int)(map[map_idx].lng * w_src / 360.0 + 0.5);
				y = (int)(map[map_idx].lat * h_src / 180.0 + 0.5);

				if(y >= 0 && x >= w_start && x < w_end && y < h_src)
				{
					sum += src_8[x + y * s_src];
					num++;
				}
			}
		}
	}
	if (num > 0)
	{
		a = (sum + (num>>1))/num;
	}
	dst_8[i] = S360_CLIP_S32_TO_U8((int)a);
}

void resample_tsp_2d_10b(void * src, int w_start, int w_end, int w_src, int h_src, int s_src, \
	void * dst, int w_dst, int i, int j, S360_SPH_COORD  * map)
{
	int x, y, m, n;
	uint16 * src_16;
	uint16 * dst_16;
	int sub = TSPAA_S;
	int num = 0;
	int subdiv2 = (sub>>1);
	uint32 map_idx = 0, sum = 0, a = 0;

	src_16 = (uint16 *)src;
	dst_16 = (uint16 *)dst;

	for (n = -subdiv2; n <= subdiv2; n++)
	{
		for (m = -subdiv2; m <= subdiv2; m++)
		{
			int idx_x = i * sub + m;
			int idx_y = j * sub + n;
			if (idx_x < 0) idx_x = 0;
			if (idx_y < 0) idx_y = 0;
			map_idx = idx_y * w_dst * sub + idx_x;
			if (map[map_idx].lng != -1)
			{
				x = (int)(map[map_idx].lng * w_src / 360.0 + 0.5);
				y = (int)(map[map_idx].lat * h_src / 180.0 + 0.5);

				if(y >= 0 && x >= w_start && x < w_end && y < h_src)
				{
					sum += src_16[x + y * s_src];
					num++;
				}
			}
		}
	}
	if (num > 0)
	{
		a = (sum + (num>>1))/num;
	}
	dst_16[i] = S360_CLIP_S32_TO_U10((int)a);
}

resample_fn resample_fp(int cs)
{
	if (cs == S360_COLORSPACE_YUV420)
	{
			return resample_2d;
	}
	else if (cs == S360_COLORSPACE_YUV420_10)
	{
			return resample_2d_10b;
	}
	else
	{
		return resample_2d_10b;
	}
}
