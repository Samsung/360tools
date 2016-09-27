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
#include "360tools_erp.h"

static void erp_to_cpp_plane(int w_src, int h_src, int s_src, void * src,
    void * dst, int s_dst, int cs, int opt, int pad_sz)
{
	void    (* fn_resample)(void * src, int w_start, int w_end, int h_src,
        int s_src, double x, double y, void * dst, int x_dst);
	double lambda, phi, x, y;
	int offset;
	int i, j, w_dst, h_dst;
	void * dst0;
	uint8 * map;
	int	w_start, w_end;

	w_start = opt ? -pad_sz : 0;
	w_end = opt ? w_src + pad_sz : w_src;
	w_dst = w_src;
	h_dst = h_src;

	map = (uint8 *)s360_malloc(sizeof(uint8) * w_dst * h_dst);
	cpp_map_plane(w_dst, h_dst, s_dst, map);

	dst0 = dst;

	if(cs == S360_COLORSPACE_YUV420)
	{
		fn_resample = resample_2d;
	}
	else if(cs == S360_COLORSPACE_YUV420_10)
	{
		fn_resample = resample_2d_10b;
		s_dst <<= 1;
	}

	offset = 0;
	for(j=0; j<h_src; j++)
	{
		for(i=0; i<w_src; i++)
		{
			phi = 3 *  asin((double)j/h_src-0.5);
			lambda = (2 * PI * (double)i/w_src - PI) / (2 * cos(2 * phi/3) - 1);

			x = w_dst * (lambda + PI)/(2*PI);
			y = h_dst * (phi + PI/2)/PI;

			if(map[i+j*w_src] != 0)
			{
				fn_resample(src, w_start, w_end, h_src, s_src, x, y, dst, i);
			}
		}
		dst = (void *)((uint8 *)dst + s_dst);
	}
	if(opt & S360_OPT_PAD)
	{
		if(cs == S360_COLORSPACE_YUV420)
		{
			pad_cpp_plane((uint8 *)dst0, w_src, h_src, s_src, map);
		}
		else if(cs == S360_COLORSPACE_YUV420_10)
		{
			pad_cpp_plane_10b((uint16 *)dst0, w_src, h_src, s_src, map);
		}
	}

	s360_mfree(map);
}

int s360_erp_to_cpp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map)
{
	int i, w_src, w_dst, h_src, pad_sz;

	s360_img_reset(img_dst);
	if (opt & S360_OPT_PAD)
		s360_pad_erp(img_src);

	if((img_src->colorspace == S360_COLORSPACE_YUV420) || ((img_src->colorspace == S360_COLORSPACE_YUV420_10)))
	{
		for(i=0; i<3; i++)
		{
			if(i == 0)
			{
				w_src = w_dst = img_src->width;
				h_src = img_src->height;
				pad_sz = (opt & S360_OPT_PAD) ? PAD_SIZE : 0;
			}
			else
			{
				w_src = w_dst = (img_src->width + 1)>> 1;
				h_src = (img_src->height + 1) >> 1;
				pad_sz = (opt & S360_OPT_PAD) ? PAD_SIZE >> 1 : 0;
			}
			erp_to_cpp_plane(w_src, h_src, img_src->stride[i], img_src->buffer[i], img_dst->buffer[i], img_dst->stride[i], img_src->colorspace, opt, pad_sz);
		}
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}
	return S360_OK;
}

static void cpp_to_erp_plane(int w_src, int h_src, int s_src, void * src, void * dst, int s_dst, int cs)
{
	resample_fn fn_resample;
	uint8 * map;
	int i, j;
	double phi;
	double x, y, h_dst, w_dst;
	
	fn_resample = resample_fp(cs);
	h_dst = h_src;
	w_dst = w_src;

	map = (uint8 *)s360_malloc(sizeof(uint8) * w_src * h_src);
	cpp_map_plane(w_src, h_src, s_src, map);

	if(cs == S360_COLORSPACE_YUV420)
	{
		pad_cpp_plane((uint8 *)(src), w_src, h_src, s_src, map);
	}
	else if(cs == S360_COLORSPACE_YUV420_10)
	{
		pad_cpp_plane_10b((uint16 *)(src), w_src, h_src, s_src, map);
		s_dst <<= 1;
	}

	for(j=0; j<h_src; j++)
	{
		for(i=0; i<w_src; i++)
		{
			y    = h_dst * (.5 + sin(PI / 3 * ((double)j/h_src -.5f)));
			phi  = 3 * asin(y/h_src-.5);
			x    = (w_dst/2) * (1 + ((4 * cos(2* phi/3)) - 2) * ((double)i/w_src -.5));

			fn_resample(src, 0, w_src, h_src, s_src, x, y, dst, i);
		}
		dst = (void *)((uint8 *)dst + s_dst);
	}
	s360_mfree(map);
}

int s360_cpp_to_erp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map)
{
	int i, w_src, w_dst, h_src;

	if(IS_VALID_CS(img_src->colorspace))
	{
		for(i=0; i<3; i++)
		{
			if(i == 0)
			{
				w_src = w_dst = img_src->width;
				h_src = img_src->height;
			}
			else
			{
				w_src = w_dst = img_src->width >> 1;
				h_src = img_src->height >> 1;
			}
			cpp_to_erp_plane(w_src, h_src, img_src->stride[i], img_src->buffer[i], img_dst->buffer[i], img_dst->stride[i], img_src->colorspace);
		}
		return S360_OK;
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}
}

static void pad_erp_plane(uint8 * buf, int w, int h, int s, int pad_sz)
{
	uint8     * src;
	uint8     * dst;
	int         i;

	// Pad Left
	src = buf + w - pad_sz;
	dst = buf - pad_sz;
	for(i=0;i<h;i++)
	{
		memcpy(dst, src, pad_sz*sizeof(uint8));
		dst += s;
		src += s;
	}

	// Pad Right
	src = buf;
	dst = buf + w;

	for(i=0;i<h;i++)
	{
		memcpy(dst, src, pad_sz*sizeof(uint8));
		dst += s;
		src += s;
	}
}

static void pad_erp_plane_10b(uint16 * buf, int w, int h, int s, int pad_sz)
{
	uint16     * src;
	uint16     * dst;
	int         i;

	// Pad Left
	src = buf + w - pad_sz;
	dst = buf - pad_sz;
	for (i=0;i<h;i++)
	{
		memcpy(dst, src, pad_sz*sizeof(uint16));
		dst += s;
		src += s;
	}

	// Pad Right
	src = buf;
	dst = buf + w;

	for (i=0;i<h;i++)
	{
		memcpy(dst, src, pad_sz*sizeof(uint16));
		dst += s;
		src += s;
	}
}

int s360_pad_erp(S360_IMAGE * img)
{
	int i, w, h;
	int pad_sz;

	if(IS_VALID_CS(img->colorspace))
	{
		for(i=0;i<3;i++)
		{
			if(i == 0)
			{
				w = img->width;
				h = img->height;
				pad_sz = PAD_SIZE;
			}
			else
			{
				w = img->width >> 1;
				h = img->height >> 1;
				pad_sz = PAD_SIZE >> 1;
			}

			if (img->colorspace == S360_COLORSPACE_YUV420)
			{
				pad_erp_plane((uint8 *)(img->buffer[i]), w, h, img->stride[i], pad_sz);
			}
			else if (img->colorspace == S360_COLORSPACE_YUV420_10)
			{
				pad_erp_plane_10b((uint16 *)(img->buffer[i]), w, h, img->stride[i], pad_sz);
			}
		}
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}
	return S360_OK;
}