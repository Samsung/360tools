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
#include "360tools_cpp.h"

static void cpp_bypass_plane(int w_src, int h_src, int s_src, void * src,
    void * dst, int s_dst, int cs, int opt, int pad_sz)
{
	int       i, j, size;
	int       w_dst, h_dst;
	uint8   * map;

	w_dst = w_src;
	h_dst = h_src;
	size  = ((cs == S360_COLORSPACE_YUV420) ? sizeof(uint8) : sizeof(uint16));
	
	map = (uint8 *)s360_malloc(sizeof(uint8) * w_dst * h_dst);
	cpp_map_plane(w_dst, h_dst, s_dst, map);

	for(j=0; j<h_src; j++)
	{
		for(i=0; i<s_dst; i++)
		{
			if (map[i+j*w_src] != 0)
			s360_mcpy(dst, src, size);

			if(cs == S360_COLORSPACE_YUV420)
			{
				dst = (void *)((uint8 *)dst + 1);
				src = (void *)((uint8 *)src + 1);
			}
			if(cs == S360_COLORSPACE_YUV420_10)
			{
				dst = (void *)((uint16 *)dst + 1);
				src = (void *)((uint16 *)src + 1);
			}
		}
	}

	s360_mfree(map);
}

int s360_cpp_bypass(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map)
{
	int i, w_src, w_dst, h_src, pad_sz;

	s360_img_reset(img_dst);

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
			cpp_bypass_plane(w_src, h_src, img_src->stride[i], img_src->buffer[i], img_dst->buffer[i], img_dst->stride[i], img_src->colorspace, opt, pad_sz);
		}
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}
	return S360_OK;
}