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

#define _CRT_SECURE_NO_WARNINGS



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "360tools.h"

static char fname_org[256] = "\0";
static char fname_rec[256] = "\0";
static char fname_sph[256] = "\0";
static char fname_cfg[256] = "\0";
static int  w_org = 0;
static int  h_org = 0;
static int  w_rec = 0;
static int  h_rec = 0;
static int  qmetric = 0;
static int  frm_num = 0;
static int  sph_num = 0;
static int  cs_org = 0;
static int  cs_rec = 0;
static int  verb = 0;
static int  pfmt_org = 0;
static int  pfmt_rec = 0;

static S360_SPH_COORD * sph_pts = NULL;

typedef enum
{
	OPT_METRIC_PSNR = 1,
	OPT_METRIC_SPSNR,
	OPT_METRIC_WSPSNR,
	OPT_METRIC_CPPPSNR,
	OPT_METRIC_NUM,
} OPT_METRIC;

int opt_flag[CMD_FLAG_METRIC_MAX] = {0};

static S360_ARGS_OPT argopt[] = \
{
	{
		'c', "config", S360_ARGS_VAL_TYPE_STRING,
		&opt_flag[CMD_FLAG_METRIC_CONFIG], fname_cfg,
		"config file name"
	},
	{
		'o', "original", S360_ARGS_VAL_TYPE_STRING|S360_ARGS_VAL_TYPE_MANDATORY,
		&opt_flag[CMD_FLAG_METRIC_FNAME_ORG], fname_org,
		"original file name"
	},
	{
		'r', "reconstruct", S360_ARGS_VAL_TYPE_STRING|S360_ARGS_VAL_TYPE_MANDATORY,
		&opt_flag[CMD_FLAG_METRIC_FNAME_REC], fname_rec,
		"output file name"
	},
	{
		'w', "original_width", S360_ARGS_VAL_TYPE_INTEGER|S360_ARGS_VAL_TYPE_MANDATORY,
		&opt_flag[CMD_FLAG_METRIC_WIDTH_ORG], &w_org,
		"width of original image"
	},
	{
		'h', "original_height", S360_ARGS_VAL_TYPE_INTEGER|S360_ARGS_VAL_TYPE_MANDATORY,
		&opt_flag[CMD_FLAG_METRIC_HEIGHT_ORG], &h_org,
		"height of original image"
	},
	{
		'l', "reconstructed_width", S360_ARGS_VAL_TYPE_INTEGER,
		&opt_flag[CMD_FLAG_METRIC_WIDTH_ORG], &w_rec,
		"width of reconstructed image"
	},
	{
		'm', "reconstructed_height", S360_ARGS_VAL_TYPE_INTEGER,
		&opt_flag[CMD_FLAG_METRIC_HEIGHT_ORG], &h_rec,
		"height of reconstruncted image"
	},
	{
		'n',  "num_frame", S360_ARGS_VAL_TYPE_INTEGER,
		&opt_flag[CMD_FLAG_METRIC_FRM_NUM], &frm_num,
		"number of frames to be processed"
	},
	{
		'q',  "qmetric", S360_ARGS_VAL_TYPE_INTEGER|S360_ARGS_VAL_TYPE_MANDATORY,
		&opt_flag[CMD_FLAG_METRIC_QMETRIC], &qmetric,
		"quality metric \n\t1: PSNR\n\t2: sperical PSNR(S-PSNR)\n\t"
        "3: weighted sperical PSNR(WS-PSNR)\n\t"
        "4: craster parabolic projection PSNR(CPP-PSNR)"
	},
	{
		's',  "sphere_file", S360_ARGS_VAL_TYPE_STRING,
		&opt_flag[CMD_FLAG_METRIC_FNAME_SPH], fname_sph,
		"sphere file; used for calculation of S-PSNR"
	},
	{
		'f',  "original_proj_format", S360_ARGS_VAL_TYPE_INTEGER,
		&opt_flag[CMD_FLAG_METRIC_PFMT_ORG], &pfmt_org,
		"original projection format\n\t "
		"1: ERP\n\t 2: ISP\n\t 3: CMP\n\t"
	},
	{
		't',  "reconstructed_proj_format", S360_ARGS_VAL_TYPE_INTEGER,
		&opt_flag[CMD_FLAG_METRIC_PFMT_REC], &pfmt_rec,
		"reconstructed projection format\n\t "
		"1: ERP\n\t 2: ISP\n\t 3: CMP\n\t"
	},
	{
		'x',  "color_space_orig", S360_ARGS_VAL_TYPE_INTEGER,
		&opt_flag[CMD_FLAG_METRIC_CS_ORG], &cs_org,
		"color space\n\t 1: YUV420 8-bit\n\t 2: YUV420 10-bit\n\t"
	},
	{
		'y',  "color_space_ref", S360_ARGS_VAL_TYPE_INTEGER,
		&opt_flag[CMD_FLAG_METRIC_CS_REC], &cs_rec,
		"color space\n\t 1: YUV420 8-bit\n\t 2: YUV420 10-bit\n\t"
	},
	{
		'v', "verbose", S360_ARGS_VAL_TYPE_NONE,
		&opt_flag[CMD_FLAG_METRIC_VERB], &verb,
		"verbose output"
	},
	{ 0, "", S360_ARGS_VAL_TYPE_NONE, NULL, NULL, ""} /* termination */
};

#define NUM_ARGOPT   ((int)(sizeof(argopt)/sizeof(argopt[0]))-1)


static void print_usage(void)
{
	int i;
	char str[512];

	s360_print("< Usage >\n");

	for(i=0; i<NUM_ARGOPT; i++)
	{
		if(s360_args_get_help(argopt, i, str) < 0) return;
		s360_print("%s\n", str);
	}
}

/* Weighted Spherical PSNR ***************************************************/
/* “WS-PSNR for 360 video quality evaluation”, Yule Sun, Ang Lu, Lu Yu,      */
/* ISO/IEC JTC1/SC29/WG11 MPEG2016/M38551, May 2016, Geneva                  */
static double ws_psnr(int w, int h, void * psrc, void * pdst, int colorSpace)
{
	int i, j;
	double diff;
	double pixel_weight;
	double sum, w_sum = 0;

	if (colorSpace == S360_COLORSPACE_YUV420)
	{
		uint8 *src = (uint8 *)psrc;
		uint8 *dst = (uint8 *)pdst;

		sum = 0;
		for(i=0;i<h;i++)
		{
			for(j=0;j<w;j++)
			{
				pixel_weight = cos ((i - (h/2)) * PI/h);
				diff = (pixel_weight)*(((double)src[i*w + j]) - ((double)dst[i*w + j]));
				diff = S360_ABS(diff);
				sum += diff * diff;
                w_sum += pixel_weight;
			}
		}
		sum = sum/(w_sum);
		if(sum == 0) sum = 100; /* to avoid fault */
		else sum = 10*log10(255*255/sum);
	}
	else if (colorSpace == S360_COLORSPACE_YUV420_10)
	{
		uint16 *src = (uint16 *)psrc;
		uint16 *dst = (uint16 *)pdst;

		sum = 0;
		for(i=0;i<h;i++)
		{
			for(j=0;j<w;j++)
			{
				pixel_weight = cos ((i - (h/2)) * PI/h);
				diff = (pixel_weight)*(((double)src[i*w + j]) - ((double)dst[i*w + j]));
				diff = S360_ABS(diff);
				sum += diff * diff;
                w_sum += pixel_weight;
			}
		}
		sum = sum/(w_sum);
		if(sum == 0) sum = 100; /* to avoid fault */
		else sum = 10*log10(1023*1023/sum);
	}

	return sum;
}

/* Spherical PSNR  ***********************************************************/
/* Matt Yu, Haricharan Lakshman, and Bernd Girod, “A frame-work to evaluate  */
/* omnidirectional video coding schemes,” in Mixed and Augmented Reality     */
/* (ISMAR), 2015 IEEE International Symposium on. IEEE, 2015, pp. 31–36.     */
static S360_SPH_COORD  * read_sph(FILE * fp_sph, int sph_cnt)
{
	S360_SPH_COORD * sph_coord;
	int i;

	sph_coord = (S360_SPH_COORD *)s360_malloc(sph_cnt*sizeof(S360_SPH_COORD));

	for (i = 0; i < sph_cnt; i++)
	{
		if(2 != fscanf(fp_sph, "%f %f", &sph_coord[i].lat, &sph_coord[i].lng))
		{
			goto ERR;
		}
	}

	return sph_coord;

ERR:
	s360_mfree(sph_coord);
	return NULL;
}

static void conv_sph_to_cart(S360_SPH_COORD sph, float *cart)
{
	float lat = DEG2RAD(sph.lat);
	float lon = DEG2RAD(sph.lng);

	cart[0] = sinf(lon) * cosf(lat);
	cart[1] = sinf(lat);
	cart[2] = -cosf(lon) * cosf(lat);
}

static void conv_cart_to_rect(float* x_out, float* y_out, int w, int h, const float* cart){
	float phi, theta;
	float x, y, z;

	x = cart[0]; y = cart[1]; z = cart[2];

	phi = acosf(y);
	theta = atan2f(x, z);

	*x_out = w * (0.5f + theta / M_2PI);
	*y_out = h * (phi / PI);
}

static __inline float linearinterpolation(float a, float b, float k)
{
	return a * (1.0f - k) + b * k;
}

static __inline float cubicinterpolation(float aZ, float a0, float a1, float a2, float t)
{
	float cZ = 2 * a0;
	float c0 = -aZ + a1;
	float c1 = 2 * aZ - 5 * a0 + 4 * a1 - a2;
	float c2 = -aZ + 3 * a0 - 3 * a1 + a2;

	float t2 = t * t;
	float t3 = t2 * t;

	float v = 0.5f*(cZ + c0*t + c1*t2 + c2*t3);
	return S360_MINMAX(v, 0, 255.0f);
}

static __inline float cubicinterpolation_420_10(float aZ, float a0, float a1, float a2, float t)
{
	float cZ = 2 * a0;
	float c0 = -aZ + a1;
	float c1 = 2 * aZ - 5 * a0 + 4 * a1 - a2;
	float c2 = -aZ + 3 * a0 - 3 * a1 + a2;

	float t2 = t * t;
	float t3 = t2 * t;

	float v = 0.5f*(cZ + c0*t + c1*t2 + c2*t3);
	return S360_MINMAX(v, 0, 1023.0f);
}

static float linear_interp(uint8 * src, float x, float y, int s_src)
{
	int x_lo, y_lo;
	int x_hi, y_hi;
	float val, val1, val2;

	x_lo = (int)(x);
	y_lo = (int)(y);
	x_hi = (int)CEILING(x);
	y_hi = (int)CEILING(y);

	val1 = linearinterpolation(src[x_lo + y_lo*s_src],
											src[x_hi + y_lo*s_src], x-x_lo);
	val2 = linearinterpolation(src[x_lo + y_hi*s_src],
											src[x_hi + y_hi*s_src ], x-x_lo);
	val = linearinterpolation(val1, val2, y - y_lo);

	return val;
}

static float cubic_interp(uint8 * src, float x, float y, int s_src)
{
	float val[4];
	float filt_val;
	int i;
	long x_idx[4], y_idx[4];

	x_idx[0] = (long)(x - 1);
	x_idx[1] = x_idx[0] + 1;
	x_idx[2] = x_idx[1] + 1;
	x_idx[3] = x_idx[2] + 1;
	y_idx[0] = (long)(y - 1);
	y_idx[1] = y_idx[0] + 1;
	y_idx[2] = y_idx[1] + 1;
	y_idx[3] = y_idx[2] + 1;


	for(i=0;i<4;i++)
	{
		val[i] = cubicinterpolation(
								*(src + y_idx[i] * s_src + x_idx[0]),
								*(src + y_idx[i] * s_src + x_idx[1]),
								*(src + y_idx[i] * s_src + x_idx[2]),
								*(src + y_idx[i] * s_src + x_idx[3]),
								x - x_idx[1]);
	}
	filt_val = cubicinterpolation(val[0], val[1], val[2], val[3], y - y_idx[1])+ 0.5f;

	return filt_val;
}

static float linear_interp_420_10(uint16 * src, float x, float y, int s_src)
{
	int x_lo, y_lo;
	int x_hi, y_hi;
	float val, val1, val2;

	x_lo = (int)(x);
	y_lo = (int)(y);
	x_hi = (int)CEILING(x);
	y_hi = (int)CEILING(y);

	val1 = linearinterpolation(src[x_lo + y_lo*s_src],
		src[x_hi + y_lo*s_src], x-x_lo);
	val2 = linearinterpolation(src[x_lo + y_hi*s_src],
		src[x_hi + y_hi*s_src ], x-x_lo);
	val = linearinterpolation(val1, val2, y - y_lo);

	return val;
}

static float cubic_interp_420_10(uint16 * src, float x, float y, int s_src)
{
	float val[4];
	float filt_val;
	int i;
	long x_idx[4], y_idx[4];

	x_idx[0] = (long)(x - 1);
	x_idx[1] = x_idx[0] + 1;
	x_idx[2] = x_idx[1] + 1;
	x_idx[3] = x_idx[2] + 1;
	y_idx[0] = (long)(y - 1);
	y_idx[1] = y_idx[0] + 1;
	y_idx[2] = y_idx[1] + 1;
	y_idx[3] = y_idx[2] + 1;


	for(i=0;i<4;i++)
	{
		val[i] = cubicinterpolation_420_10(
			*(src + y_idx[i] * s_src + x_idx[0]),
			*(src + y_idx[i] * s_src + x_idx[1]),
			*(src + y_idx[i] * s_src + x_idx[2]),
			*(src + y_idx[i] * s_src + x_idx[3]),
			x - x_idx[1]);
	}
	filt_val = cubicinterpolation_420_10(val[0], val[1], val[2], val[3], y - y_idx[1])+ 0.5f;

	return filt_val;
}

static float filter(uint8 * src, float x, float y, int w, int h, int s_src)
{
	const int width_LB = 1;
	const int width_HB = w - 2;
	const int height_LB = 1;
	const int height_HB = h - 2;
	float filt_val;

	if ((y <= height_LB) || (y >= height_HB) || (x <= width_LB) || (x >= width_HB))
	{
		x = S360_MINMAX(x, 0.0f, w-1.0f);
		y = S360_MINMAX(y, 0.0f, h-1.0f);

		filt_val = linear_interp(src, x, y, s_src);
	}
	else
	{
		filt_val = cubic_interp(src, x, y, s_src);
	}
	return filt_val;
}

static float filter_420_10(uint16 * src, float x, float y, int w, int h, int s_src)
{
	const int width_LB = 1;
	const int width_HB = w - 2;
	const int height_LB = 1;
	const int height_HB = h - 2;
	float filt_val;

	if ((y <= height_LB) || (y >= height_HB) || (x <= width_LB) || (x >= width_HB))
	{
		x = S360_MINMAX(x, 0.0f, w-1.0f);
		y = S360_MINMAX(y, 0.0f, h-1.0f);

		filt_val = linear_interp_420_10(src, x, y, s_src);
	}
	else
	{
		filt_val = cubic_interp_420_10(src, x, y, s_src);
	}
	return filt_val;
}

static double s_psnr(int w, int h, void * psrc, void * pdst, int colorSpace)
{
	int               i;
	float             cart[3];
	float             x, y;
	double            val1, val2;
	double            diff;
	double            sum = 0.0f;

	if(colorSpace == S360_COLORSPACE_YUV420)
	{
		uint8 *src = (uint8 *)psrc;
		uint8 *dst = (uint8 *)pdst;

		sum = 0.f;
		for (i = 0; i < sph_num; i++)
		{
			conv_sph_to_cart(sph_pts[i], cart);
			conv_cart_to_rect(&x, &y, w, h, cart);

			x -= 0.5f;
			y -= 0.5f;

			val1 = filter(src, x, y, w, h, w);
			val2 = filter(dst, x, y, w, h, w);
			diff = val1 - val2;
			diff = S360_ABS(diff);
			sum += diff*diff;
		}

		sum = sum/(sph_num);
		if(sum == 0) sum = 100; /* to avoid fault */
		else sum = 10*log10(255*255/sum);
	}
	else if (colorSpace == S360_COLORSPACE_YUV420_10)
	{
		uint16 *src = (uint16 *)psrc;
		uint16 *dst = (uint16 *)pdst;

		sum = 0.f;
		for (i = 0; i < sph_num; i++)
		{
			conv_sph_to_cart(sph_pts[i], cart);
			conv_cart_to_rect(&x, &y, w, h, cart);

			x -= 0.5f;
			y -= 0.5f;

			val1 = filter_420_10(src, x, y, w, h, w);
			val2 = filter_420_10(dst, x, y, w, h, w);
			diff = val1 - val2;
			diff = S360_ABS(diff);
			sum += diff*diff;
		}

		sum = sum/(sph_num);
		if(sum == 0) sum = 100; /* to avoid fault */
		else sum = 10*log10(1023*1023/sum);
	}

	return sum;

}

/* PSNR **********************************************************************/
static double psnr(int w, int h, void * psrc, void * pdst, int colorSpace)
{
	int i, j;
	int diff;
	double sum;

	if (colorSpace == S360_COLORSPACE_YUV420)
	{
		uint8 *src = (uint8 *)psrc;
		uint8 *dst = (uint8 *)pdst;

		sum = 0;
		for(i=0;i<h;i++)
		{
			for(j=0;j<w;j++)
			{
				diff = (int)src[i*w + j] - (int)dst[i*w + j];
				diff = S360_ABS(diff);
				sum += diff * diff;
			}
		}
		sum = sum/(w*h);
		if(sum == 0) sum = 100; /* to avoid fault */
		else sum = 10*log10(255*255/sum);
	}
	else if (colorSpace == S360_COLORSPACE_YUV420_10)
	{
		uint16 *src = (uint16 *)psrc;
		uint16 *dst = (uint16 *)pdst;

		sum = 0;
		for(i=0;i<h;i++)
		{
			for(j=0;j<w;j++)
			{
				diff = (int)src[i*w + j] - (int)dst[i*w + j];
				diff = S360_ABS(diff);
				sum += diff * diff;
			}
		}
		sum = sum/(w*h);
		if(sum == 0) sum = 100; /* to avoid fault */
		else sum = 10*log10(1023*1023/sum);
	}

	return sum;
}

/* CPP-PSNR ********************************************************************/
/* V. Zakharchenko, K.P. Choi, J.H. Park, Video quality metric for spherical   */
/* panoramic video,  Optics and Photonics, SPIE, 9970, 2016                    */
static int map_plane(int w, int h, uint8 * map)
{
	int    i, j;
	int    idx_x, idx_y;
	int    size;
	double x, y, phi, lambda;

	size = 0;

	for(j=0;j<h;j++)
	{
		for(i=0;i<w;i++)
		{
			x = ((double)i / (w)) * (2 * PI) - PI;
			y = ((double)j / (h)) * PI - (PI / 2);

			phi = 3 * asin(y / PI);
			lambda = x / (2 * cos(2 * phi / 3) - 1);

			x = (lambda + PI) / 2 / PI * (w);
			y = (phi + (PI / 2)) / PI *  (h);

			idx_x = (int)((x < 0) ? x - 0.5 : x + 0.5);
			idx_y = (int)((y < 0) ? y - 0.5 : y + 0.5);

			if(idx_y >= 0 && idx_x >= 0 && idx_x < w && idx_y < h)
			{
				map[i+j*w] = 1;
				size++;
			}
		}
	}
	return size;
}

static double cpp_psnr(int w, int h, void * psrc, void * pdst, int colorSpace)
{
	int i, j;
	int diff;
	int size = 0;
	double sum;

	uint8 * map;
	map = (uint8 *)s360_malloc(sizeof(uint8) * w * h);
	s360_mset(map, 0, w * h);

	size = map_plane(w, h, map);

	if (colorSpace == S360_COLORSPACE_YUV420)
	{
		uint8 *src = (uint8 *)psrc;
		uint8 *dst = (uint8 *)pdst;

		sum = 0;
		for(i=0;i<h;i++)
		{
			for(j=0;j<w;j++)
			{
				if(map[i*w + j] == 1)
				{
					diff = (int)src[i*w + j] - (int)dst[i*w + j];
					diff = S360_ABS(diff);
					sum += diff * diff;
				}
			}
		}

		sum = sum/(size);
		if(sum == 0) sum = 100; /* to avoid fault */
		else sum = 10*log10(255*255/sum);
	}
	else if (colorSpace == S360_COLORSPACE_YUV420_10)
	{
		uint16 *src = (uint16 *)psrc;
		uint16 *dst = (uint16 *)pdst;

		sum = 0;
		for(i=0;i<h;i++)
		{
			for(j=0;j<w;j++)
			{
				if(map[i*w + j] == 1)
				{
					diff = (int)src[i*w + j] - (int)dst[i*w + j];
					diff = S360_ABS(diff);
					sum += diff * diff;
				}
			}
		}

		sum = sum/(size);
		if(sum == 0) sum = 100; /* to avoid fault */
		else sum = 10*log10(1023*1023/sum);
	}

	s360_mfree(map);
	return sum;
}

/*****************************************************************************/
int main(int argc, const char * argv[])
{
	S360_IMAGE      * img_org = NULL;
	S360_IMAGE      * img_rec = NULL;
	S360_IMAGE      * img_a   = NULL;
	S360_IMAGE      * img_b   = NULL;
	FILE            * fp_org  = NULL;
	FILE            * fp_rec  = NULL;
	FILE            * fp_sph  = NULL;

	double         (* fn_qmetric)(int w, int h, void * src, void * dst, int cs);
	int            (* fn_conv_org)(S360_IMAGE * imgi, S360_IMAGE * imgo, int opt, S360_MAP * map);
	int            (* fn_conv_rec)(S360_IMAGE * imgi, S360_IMAGE * imgo, int opt, S360_MAP * map);
	int               i, ret = 0;
	int               pic_cnt = 0;
	int               h_in = 0;
	int               w_in = 0;
    int               cs_int = S360_COLORSPACE_YUV420_10;
	double            qual[4] = {0,};
	double            qual_sum[4] = {0,};

	ret = s360_args_parse_all(argc, argv, argopt);
	if(ret != 0)
	{
		if(ret > 0) s360_print("-%c argument should be set\n", ret);
		print_usage();
		return 0;
	}
	fp_org = fopen(fname_org, "rb");
	if(fp_org == NULL)
	{
		s360_print("failed to open original file: %s\n", fname_org);
		print_usage(); ret = -1; goto ERR;
	}

	fp_rec = fopen(fname_rec, "rb");
	if(fp_rec == NULL)
	{
		s360_print("failed to open reconstructed file: %s\n", fname_rec);
		print_usage(); ret = -1; goto ERR;
	}

	if(!opt_flag[CMD_FLAG_METRIC_CS_ORG])
		cs_org = 1;
	if(cs_org == 1)
		cs_org = S360_COLORSPACE_YUV420;
	else if (cs_org == 2)
		cs_org = S360_COLORSPACE_YUV420_10;
	else
	{
		s360_print("Invalid input option for color space\n");
		ret = S360_ERR_UNSUPPORTED_COLORSPACE;
		print_usage();
		goto ERR;
	}

	if(!opt_flag[CMD_FLAG_METRIC_CS_REC])
		cs_rec = cs_org;
	if(cs_rec == 1)
		cs_rec = S360_COLORSPACE_YUV420;
	else if (cs_rec == 2)
		cs_rec = S360_COLORSPACE_YUV420_10;
	else
	{
		s360_print("Invalid input option for color space\n");
		ret = S360_ERR_UNSUPPORTED_COLORSPACE;
		print_usage();
		goto ERR;
	}

	w_in = w_org;
	h_in = h_org;

	if(w_in <= 0 || h_in <= 0)
	{
		s360_print("Invalid input resolution: %dx%d\n", w_in, h_in);
		print_usage(); ret = -1; goto ERR;
	}

	if(w_rec <= 0 || h_rec <= 0 )
	{
		w_rec = w_org;
		h_rec = h_org;
	}

	switch(qmetric)
	{
	case OPT_METRIC_PSNR: /* PSNR */
		fn_qmetric = psnr;
		break;
	case OPT_METRIC_SPSNR: /* Spherical PSNR (S-PSNR) */
		if(!opt_flag[CMD_FLAG_METRIC_FNAME_SPH])
		{
			s360_print("sphere file should be set for S-PSNR\n");
			print_usage(); ret = -1; goto ERR;
		}
		fp_sph = fopen(fname_sph, "r");
		if(fp_sph == NULL)
		{
			s360_print("failed to open sphere file: %s\n", fname_sph);
			goto ERR;
		}
		if(1 != fscanf(fp_sph, "%d", &sph_num))
		{
			s360_print("failed to read sphere file: %s\n", fname_sph);
			goto ERR;
		}
		sph_pts = read_sph(fp_sph, sph_num);
		if(sph_pts == NULL)
		{
			s360_print("Failed to create Sphere Co-ordinates\n");
			goto ERR;
		}

		fn_qmetric = s_psnr;
		break;
	case OPT_METRIC_WSPSNR: /* Weighted Spherical PSNR (WS-PSNR) */
		fn_qmetric = ws_psnr;
		break;
	case OPT_METRIC_CPPPSNR: /* Craster Parabolic Projection PSNR (CPP-PSNR) */
        if(!opt_flag[CMD_FLAG_METRIC_PFMT_ORG])
		{
			s360_print("projection type should be set for CPP-PSNR\n");
			print_usage(); ret = -1; goto ERR;
		}
        if(!pfmt_rec) pfmt_rec = pfmt_org;
		w_in = WIDTH_CPP;
		h_in = HEIGHT_CPP;
        switch(pfmt_org)
        {
        case PROJ_FMT_ERP:
            fn_conv_org = s360_erp_to_cpp;
			w_in = w_org;
            h_in = h_org;
            break;
        case PROJ_FMT_ISP:
            fn_conv_org = s360_isp_to_cpp;
            break;
        case PROJ_FMT_CMP:
            fn_conv_org = s360_cmp_to_cpp;
            break;
        default:
            s360_print("Unsupprted input format\n");
			print_usage();
			return -1;
		}

        switch(pfmt_rec)
        {
        case PROJ_FMT_ERP:
            fn_conv_rec = s360_erp_to_cpp;
			w_in = w_rec;
            h_in = h_rec;
            break;
        case PROJ_FMT_ISP:
            fn_conv_rec = s360_isp_to_cpp;
            break;
        case PROJ_FMT_CMP:
            fn_conv_rec = s360_cmp_to_cpp;
            break;
        default:
            s360_print("Unsupprted input format\n");
			print_usage();
			return -1;
		}

		ret = s360_init();
		if(S360_FAILED(ret))
		{
			s360_print("failed to initialize 360tools (ret=%d)\n", ret);
			print_usage();
			goto ERR;
		}
		img_a = s360_img_create(w_in, h_in, cs_int, 0);
		img_b = s360_img_create(w_in, h_in, cs_int, 0);
		if(img_a == NULL || img_b == NULL)
		{
			s360_print("Failed to create image buffer\n");
			print_usage(); ret = -1; goto ERR;
		}
		fn_qmetric = cpp_psnr;
		break;

	default:
		s360_print("Unsupported quality metric\n");
		print_usage(); ret = -1; goto ERR;
		return -1;
	}

	img_org = s360_img_create(w_org, h_org, cs_int, 0);
	img_rec = s360_img_create(w_rec, h_rec, cs_int, 0);
	if(img_org == NULL || img_rec == NULL)
	{
		s360_print("Failed to create image buffer\n");
		print_usage(); ret = -1; goto ERR;
	}

	while(1)
	{
        S360_IMAGE      * img_org_m = NULL;
        S360_IMAGE      * img_rec_m = NULL;

		if(s360_img_read(fp_org, img_org, cs_org) < 0) break;
		if(s360_img_read(fp_rec, img_rec, cs_rec) < 0) break;

		/* YCbCr 4:2:0 for now */
        if (qmetric == OPT_METRIC_CPPPSNR)
        {
            ret = fn_conv_org(img_org, img_a, 0, NULL);
            ret = fn_conv_rec(img_rec, img_b, 0, NULL);
            img_org_m = img_a;
            img_rec_m = img_b;
        }
        else
        {
            img_org_m = img_org;
            img_rec_m = img_rec;
        }

        qual[0] = fn_qmetric(w_in, h_in, img_org_m->buffer[0], img_rec_m->buffer[0], cs_int);
		for(i=1; i<3; i++)
		{
			qual[i] = fn_qmetric(w_in>>1, h_in>>1, img_org_m->buffer[i], img_rec_m->buffer[i], cs_int);
		}

		if (verb)
			s360_print("[%4d]-frame quality: %5.4f %5.4f %5.4f\n", pic_cnt++,
				qual[0], qual[1], qual[2]);
		else
			pic_cnt++;

		for(i=0; i<3; i++)
		{
			qual_sum[i] += qual[i];
		}

		if(frm_num > 0 && pic_cnt >= frm_num) break;
	}

	for(i=0; i<3; i++)
	{
		qual_sum[i] /= pic_cnt;
	}

	s360_print("Total %d frames quality:\n\t QM_ID:%d\tY\t%5.4f\tCb\t%5.4f\tCr\t%5.4f\n",
		pic_cnt, qmetric, (float)qual_sum[0], (float)qual_sum[1], (float)qual_sum[2]);

ERR:
	if(fp_org) fclose(fp_org);
	if(fp_rec) fclose(fp_rec);
	if(fp_sph) fclose(fp_sph);

	if(sph_pts) s360_mfree(sph_pts);

	if(img_org) s360_img_delete(img_org);
	if(img_rec) s360_img_delete(img_rec);
	if(img_a) s360_img_delete(img_a);
	if(img_b) s360_img_delete(img_b);

	return ret;
}