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
#include "360tools.h"

static char fname_inp[256] = "\0";
static char fname_out[256] = "\0";
static char fname_cfg[256] = "\0";
static int w_in = 0;
static int h_in = 0;
static int w_out = 0;
static int h_out = 0;
static int cfmt = 0;
static int frm_num = 0;
static int align = 0;
static int no_pad = 0;
static int cs_in = 0;
static int cs_out = 0;
static int cs_int = INT_10_BIT_DEPTH;

int opt_flag[CMD_FLAG_CONV_MAX] = {0};

static S360_ARGS_OPT argopt[] = \
{
	{
		'c', "config", S360_ARGS_VAL_TYPE_STRING,
		&opt_flag[CMD_FLAG_CONV_CONFIG], fname_cfg,
		"config file name"
	},
	{
		'i', "input", S360_ARGS_VAL_TYPE_STRING|S360_ARGS_VAL_TYPE_MANDATORY,
		&opt_flag[CMD_FLAG_CONV_FNAME_INP], fname_inp,
		"input file name"
	},
	{
		'o', "output", S360_ARGS_VAL_TYPE_STRING|S360_ARGS_VAL_TYPE_MANDATORY,
		&opt_flag[CMD_FLAG_CONV_FNAME_OUT], fname_out,
		"output file name"
	},
	{
		'w', "width", S360_ARGS_VAL_TYPE_INTEGER|S360_ARGS_VAL_TYPE_MANDATORY,
		&opt_flag[CMD_FLAG_CONV_WIDTH], &w_in,
		"width of input image"
	},
	{
		'h', "height", S360_ARGS_VAL_TYPE_INTEGER|S360_ARGS_VAL_TYPE_MANDATORY,
		&opt_flag[CMD_FLAG_CONV_HEIGHT], &h_in,
		"height of input image"
	},
	{
		'f',  "convfmt", S360_ARGS_VAL_TYPE_INTEGER | S360_ARGS_VAL_TYPE_MANDATORY,
		&opt_flag[CMD_FLAG_CONV_CFMT], &cfmt,
		"converting format\n\t"
		"1:  ERP to ISP\n\t2:  ISP to ERP\n\t"
		"3:  ERP to CMP\n\t4:  CMP to ERP\n\t"
	},
	{
		'l', "out_width", S360_ARGS_VAL_TYPE_INTEGER|S360_ARGS_VAL_TYPE_MANDATORY,
		&opt_flag[CMD_FLAG_CONV_OWIDTH], &w_out,
		"width of output image. if not set, this is same value with width of input "
		"image"
	},
	{
		'm', "out_height", S360_ARGS_VAL_TYPE_INTEGER|S360_ARGS_VAL_TYPE_MANDATORY,
		&opt_flag[CMD_FLAG_CONV_OHEIGHT], &h_out,
		"height of output image.  if not set, this is same value with height"
		" of input image"
	},

	{
		'n',  "frmnum", S360_ARGS_VAL_TYPE_INTEGER,
		&opt_flag[CMD_FLAG_CONV_FRM_NUM], &frm_num,
		"number of frames to be converted"
	},
	{
		'a',  "align", S360_ARGS_VAL_TYPE_INTEGER,
		&opt_flag[CMD_FLAG_CONV_ALIGN], &align,
		"vertical align"
	},
	{
		'u',  "no_pad", S360_ARGS_VAL_TYPE_NONE,
		&opt_flag[CMD_FLAG_CONV_ALIGN], &no_pad,
		"turn off padding"
	},
	{
		'x',  "cs_in", S360_ARGS_VAL_TYPE_INTEGER,
		&opt_flag[CMD_FLAG_CONV_CS_IN], &cs_in,
		"Input Color Space\n\t 1: YUV420 8-bit\n\t 2: YUV420 10-bit"
	},
	{
		'y',  "cs_out", S360_ARGS_VAL_TYPE_INTEGER,
		&opt_flag[CMD_FLAG_CONV_CS_OUT], &cs_out,
		"Output Color Space\n\t 1: YUV420 8-bit\n\t 2: YUV420 10-bit"
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

int main(int argc, const char * argv[])
{
	S360_IMAGE     * imgi = NULL;
	S360_IMAGE     * imgo = NULL;
	S360_MAP	    * map = NULL;
	FILE            * fpi = NULL;
	FILE            * fpo = NULL;
	int            (* fn_conv)(S360_IMAGE * imgi, S360_IMAGE * imgo, int opt, S360_MAP * map);
	int               opt = 0;
	int               ret, pic_cnt = 0;

	ret = s360_args_parse_all(argc, argv, argopt);
	if(ret != 0)
	{
		if(ret > 0) s360_print("-%c argument should be set\n", ret);
		print_usage();
		return 0;
	}

	fpi = fopen(fname_inp, "rb");
	if(fpi == NULL)
	{
		s360_print("No input file: %s\n", fname_inp);
		print_usage();
		goto END;
	}

	fpo = fopen(fname_out, "wb");
	if(fpo == NULL)
	{
		s360_print("Failed to create output file: %s\n", fname_out);
		print_usage();
		goto END;
	}
	fclose(fpo);

	if(w_in <= 0 || h_in <= 0)
	{
		s360_print("Invalid input resolution: %dx%d\n", w_in, h_in);
		print_usage();
		goto END;
	}

	if((cfmt == CONV_FMT_ERP_TO_CMP) &&
        ((w_out%4 != 0 || h_out%4 != 0) || (w_out*3 != h_out*4)))
	{
		s360_print("Invalid output resolution for cubemap, suugested aspect "
			"ratio 4:3 and must be multiple of 4: %dx%d\n", w_out, h_out);
		s360_print("Suggested sample dimension: %dx%d\n", w_out, w_out*3/4);
		print_usage();
		goto END;
	}

	if((cfmt == CONV_FMT_ERP_TO_ISP) && (((w_out<<1)%11 != 0 ||
        h_out != 3*NEAREST_EVEN((((int)((int)((w_out)/5.5)/4)*4))*SIN_60))))
	{
		/* some suggested dimensions */
		int w_tri, h_tri;
		w_tri = ((int)((int)((w_out)/5.5)/4)*4);
		h_tri = NEAREST_EVEN((w_tri)*SIN_60);
		w_tri = w_tri*5.5;
		h_tri = h_tri*3;
		s360_print("Invalid output resolution %dx%d, ISP recommended aspect "
			" ratio: 88:42\n",	w_out, h_out);
		s360_print("Suggested sample dimension: %dx%d\n", w_tri, h_tri);
		print_usage();
		goto END;
	}

	if(!opt_flag[CMD_FLAG_CONV_OWIDTH]) w_out = w_in;
	if(!opt_flag[CMD_FLAG_CONV_OHEIGHT]) h_out = h_in;

	cs_in  = (cs_in  == 1)?S360_COLORSPACE_YUV420:S360_COLORSPACE_YUV420_10;
	cs_out = (cs_out == 1)?S360_COLORSPACE_YUV420:S360_COLORSPACE_YUV420_10;
	cs_int = (cs_int == 1)?S360_COLORSPACE_YUV420:S360_COLORSPACE_YUV420_10;

	if(!opt_flag[CMD_FLAG_CONV_CS_IN])  cs_in  = S360_COLORSPACE_YUV420;
	if(!opt_flag[CMD_FLAG_CONV_CS_OUT]) cs_out = S360_COLORSPACE_YUV420;
	//Internal Bitdepth for 360Tools. Default 10-bit
	if(!opt_flag[CMD_FLAG_CONV_CS_INT]) cs_int = S360_COLORSPACE_YUV420_10;

	if(align > 0)
	{
		w_out = S360_ALIGN(w_out, align);
		h_out = S360_ALIGN(h_out, align);
	}

	if(!no_pad)
	{
		opt |= S360_OPT_PAD;
	}

	imgi = s360_img_create(w_in, h_in, cs_int, opt);
	imgo = s360_img_create(w_out, h_out, cs_int, opt);
	if(imgi == NULL || imgo == NULL)
	{
		s360_print("Failed to create image buffer\n");
		goto END;
	}

	switch(cfmt)
	{
	case CONV_FMT_ERP_TO_ISP:
		fn_conv = s360_erp_to_isp;
		break;
	case CONV_FMT_ISP_TO_ERP:
		fn_conv = s360_isp_to_erp;
		break;
	case CONV_FMT_ERP_TO_CMP:
		fn_conv = s360_erp_to_cmp;
		break;
	case CONV_FMT_CMP_TO_ERP:
		fn_conv = s360_cmp_to_erp;
		break;
	default:
		s360_print("Unsupprted converting format\n");
		print_usage();
		return -1;
	}

	ret = s360_init();
	if(S360_FAILED(ret))
	{
		s360_print("failed to initialize 360tools (ret=%d)\n", ret);
		print_usage();
		goto END;
	}

	map = s360_map_create(w_in, h_in, w_out, h_out, cfmt, opt);

	while(s360_img_read(fpi, imgi, cs_in) == 0)
	{
		s360_print("[%4d]-frame: ", pic_cnt++);

		ret = fn_conv(imgi, imgo, opt, map);
		if(S360_SUCCEEDED(ret) && align != 0 && \
			(imgi->width != imgo->width || imgi->height != imgo->height))
		{
			s360_img_align(imgo, align);
			s360_print("%dx%d --> ", imgo->width, imgo->height);
			imgo->width = S360_ALIGN(imgo->width, align);
			imgo->height = S360_ALIGN(imgo->height, align);
		}

		if(S360_SUCCEEDED(ret))
		{
			s360_print("convert succeeded: resolution= %dx%d\n", imgo->width, imgo->height);
		}
		else
		{
			s360_print("convert failed: ret = %d\n", ret);
			goto END;
		}

		fpo = fopen(fname_out, "ab");
		if(fpo)
		{
			s360_img_write(fpo, imgo, cs_out);
			fclose(fpo);
		}

		if(frm_num > 0 && pic_cnt >= frm_num) break;
	}

END:
	if(fpi) fclose(fpi);
	if(imgi) s360_img_delete(imgi);
	if(imgo) s360_img_delete(imgo);
	if (map) s360_map_delete(map);

	s360_deinit();

	return 0;
}

