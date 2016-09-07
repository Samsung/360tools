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
#include "360tools_img.h"

S360_IMAGE * s360_img_create(int w, int h, int cs, int opt)
{
	S360_IMAGE * img;
	int pad_sz;


	img = (S360_IMAGE *)s360_malloc(sizeof(S360_IMAGE));
	s360_assert_rv(img, NULL);
	s360_mset(img, 0, sizeof(S360_IMAGE));
	img->colorspace = cs;

	pad_sz = (opt & S360_OPT_PAD)?(PAD_SIZE<<1):0;

	if(img->colorspace == S360_COLORSPACE_YUV420)
	{
		img->width = w;
		img->height = h;
		img->stride[0] = (((w + 15) >> 4) << 4) + pad_sz;
		img->stride[1] = img->stride[2] = ((img->stride[0] + 1) >> 1);

		img->elevation[0] = ((h + 15) >> 4) << 4;
		img->elevation[1] = img->elevation[2] = (img->elevation[0] + 1) >> 1;
		img->buf_pad[0] = s360_malloc(img->stride[0] * img->elevation[0] * sizeof(uint8));
		img->buf_pad[1] = s360_malloc(img->stride[1] * img->elevation[1] * sizeof(uint8));
		img->buf_pad[2] = s360_malloc(img->stride[2] * img->elevation[2] * sizeof(uint8));

		img->buffer[0] = (uint8 *)img->buf_pad[0] + (pad_sz >> 1);
		img->buffer[1] = (uint8 *)img->buf_pad[1] + (pad_sz >> 2);
		img->buffer[2] = (uint8 *)img->buf_pad[2] + (pad_sz >> 2);
	}
	else if(img->colorspace == S360_COLORSPACE_YUV420_10)
	{
		img->width = w;
		img->height = h;
		img->stride[0] = (((w + 15) >> 4) << 4) + pad_sz;
		img->stride[1] = img->stride[2] = ((img->stride[0] + 1) >> 1);

		img->elevation[0] = ((h + 15) >> 4) << 4;
		img->elevation[1] = img->elevation[2] = (img->elevation[0] + 1) >> 1;
		img->buf_pad[0] = s360_malloc(img->stride[0] * img->elevation[0] * sizeof(uint16));
		img->buf_pad[1] = s360_malloc(img->stride[1] * img->elevation[1] * sizeof(uint16));
		img->buf_pad[2] = s360_malloc(img->stride[2] * img->elevation[2] * sizeof(uint16));

		img->buffer[0] = (uint16 *)img->buf_pad[0] + (pad_sz >> 1);
		img->buffer[1] = (uint16 *)img->buf_pad[1] + (pad_sz >> 2);
		img->buffer[2] = (uint16 *)img->buf_pad[2] + (pad_sz >> 2);
	}
	else
	{
		printf("unsupported color space\n");
		s360_mfree(img);
		return NULL;
	}
	return img;
}

void s360_img_delete(S360_IMAGE * img)
{
	s360_assert_r(img);

	if (img->buf_pad[0]) s360_mfree(img->buf_pad[0]);
	if (img->buf_pad[1]) s360_mfree(img->buf_pad[1]);
	if (img->buf_pad[2]) s360_mfree(img->buf_pad[2]);
	if (img->buf_pad[3]) s360_mfree(img->buf_pad[3]);

	s360_mfree(img);
}

void s360_img_reset(S360_IMAGE * img)
{
	int i;

	if(img->colorspace == S360_COLORSPACE_YUV420)
	{

		s360_mset(img->buf_pad[0],   0, img->stride[0] * img->elevation[0] * sizeof(uint8));
		s360_mset(img->buf_pad[1], 128, img->stride[1] * img->elevation[1] * sizeof(uint8));
		s360_mset(img->buf_pad[2], 128, img->stride[2] * img->elevation[2] * sizeof(uint8));

	}
	else if(img->colorspace == S360_COLORSPACE_YUV420_10)
	{
		uint16 * buf_u, * buf_v;

		buf_u = (uint16 *)img->buf_pad[1];
		buf_v = (uint16 *)img->buf_pad[2];

		s360_mset(img->buf_pad[0], 0, img->stride[0] * img->elevation[0] * sizeof(uint16));

		for(i = 0;i<img->stride[1] * img->elevation[1];i++)
		{
			buf_u[i] = 512;
		}
		for(i = 0;i<img->stride[2] * img->elevation[2];i++)
		{
			buf_v[i] = 512;
		}
	}
}

int s360_img_read(FILE * fp, S360_IMAGE * img, int cs_int)
{
	uint8     * p8;
	uint16    * p16;
	uint16      val16;
	int         i, j, k, w;

	if((cs_int == S360_COLORSPACE_YUV420) && (img->colorspace == S360_COLORSPACE_YUV420_10))
	{

		/* luma */
		p16 = (uint16 *)img->buffer[0];
		w = img->width;
		for(j=0; j<img->height; j++)
		{
			memset(p16, 0, w*sizeof(uint16));
			for(k=0;k<w;k++)
			{
				if(fread((p16+k), sizeof(uint8), 1, fp) != (unsigned)1) return -1;
				p16[k] = (p16[k])<<2;
			}

			p16 += img->stride[0];
		}

		/* chroma */
		for(i=1; i<3; i++)
		{
			p16 = (uint16 *)img->buffer[i];
			w = (img->width + 1) >> 1;
			for(j=0; j<((img->height+1)>>1); j++)
			{
				memset(p16, 0, w*sizeof(uint16));
				for(k=0;k<(w);k++)
				{
					if(fread(p16+k, sizeof(uint8), 1, fp) != (unsigned)1) return -1;
					p16[k] = (p16[k])<<2;
				}
				p16 += img->stride[i];
			}
		}
	}
	else if((cs_int == S360_COLORSPACE_YUV420) && (img->colorspace == S360_COLORSPACE_YUV420))
	{
		/* luma */
		p8 = (uint8 *)img->buffer[0];
		w = img->width;
		for(j=0; j<img->height; j++)
		{
			if(fread(p8, sizeof(uint8), w, fp) != (unsigned)w) return -1;
			p8 += img->stride[0];
		}

		/* chroma */
		for(i=1; i<3; i++)
		{
			p8 = (uint8 *)img->buffer[i];
			w = (img->width + 1) >> 1;
			for(j=0; j<((img->height+1)>>1); j++)
			{
				if(fread(p8, sizeof(uint8), w, fp) != (unsigned)w) return -1;
				p8 += img->stride[i];
			}
		}
	}
	else if((cs_int == S360_COLORSPACE_YUV420_10) && (img->colorspace == S360_COLORSPACE_YUV420_10))
	{
		/* luma */
		p16 = (uint16 *)img->buffer[0];
		w = img->width;
		for(j=0; j<img->height; j++)
		{
			if(fread(p16, sizeof(uint16), w, fp) != (unsigned)w) return -1;
			p16 += img->stride[0];
		}

		/* chroma */
		for(i=1; i<3; i++)
		{
			p16 = (uint16 *)img->buffer[i];
			w = (img->width + 1) >> 1;
			for(j=0; j<((img->height+1)>>1); j++)
			{
				if(fread(p16, sizeof(uint16), w, fp) != (unsigned)w) return -1;
				p16 += img->stride[i];
			}
		}
	}
	else if((cs_int == S360_COLORSPACE_YUV420_10) && (img->colorspace == S360_COLORSPACE_YUV420))
	{
		/* luma */
		p8 = (uint8 *)img->buffer[0];
		w = img->width;
		for(j=0; j<img->height; j++)
		{
			for(k=0;k<w;k++)
			{
				if(fread(&val16, sizeof(uint16), 1, fp) != (unsigned)1) return -1;
				p8[k] = S360_CLIP_S32_TO_U8((val16 + 2)>>2);
			}
			p8 += img->stride[0];
		}

		/* chroma */
		for(i=1; i<3; i++)
		{
			p8 = (uint8 *)img->buffer[i];
			w = (img->width + 1) >> 1;
			for(j=0; j<((img->height+1)>>1); j++)
			{
				for(k=0;k<w;k++)
				{
					if(fread(&val16, sizeof(uint16), 1, fp) != (unsigned)1) return -1;
					p8[k] = S360_CLIP_S32_TO_U8((val16 + 2)>>2);
				}
				p8 += img->stride[i];
			}
		}
	}
	else
	{
		s360_trace("not supported color space\n");
		return -1;
	}

	return 0;
}

int s360_img_write(FILE * fp, S360_IMAGE * img, int cs)
{
	uint8     * p;
	uint16    * p16;
	uint16      val16;
	int         i, j, k;

	if(img->colorspace == S360_COLORSPACE_YUV420_10)
	{
		if(cs == S360_COLORSPACE_YUV420)
		{
			/* luma */
			p16 = (uint16 *)img->buffer[0];
			for(j=0; j<img->height; j++)
			{
				for(k = 0;k<img->width;k++)
				{
					p16[k] = S360_CLIP_S32_TO_U8((p16[k]+2)>>2);
					fwrite(p16+k, sizeof(uint8), 1, fp);
				}

				p16 += img->stride[0];
			}

			/* chroma */
			for(i=1; i<3; i++)
			{
				p16 = (uint16 *)img->buffer[i];
				for(j=0; j<((img->height+1)>>1); j++)
				{
					for(k = 0;k<((img->width+1)>>1);k++)
					{
						p16[k] = S360_CLIP_S32_TO_U8((p16[k]+2)>>2);
						fwrite(p16+k, sizeof(uint8), 1, fp);
					}
					p16 += img->stride[i];
				}
			}
		}
		if(cs == S360_COLORSPACE_YUV420_10)
		{
			/* luma */
			p16 = (uint16 *)img->buffer[0];
			for(j=0; j<img->height; j++)
			{
				for(k = 0;k<((img->width));k++)
				{
					fwrite((p16+k), sizeof(uint16), 1, fp);
				}
				p16 += img->stride[0];
			}

			/* chroma */
			for(i=1; i<3; i++)
			{
				p16 = (uint16 *)img->buffer[i];
				for(j=0; j<((img->height+1)>>1); j++)
				{
					for(k = 0;k<((img->width+1)>>1);k++)
					{
						fwrite(p16+k, sizeof(uint16), 1, fp);
					}
					p16 += img->stride[i];
				}
			}
		}
	}
	else if(img->colorspace == S360_COLORSPACE_YUV420)
	{
		if(cs == S360_COLORSPACE_YUV420)
		{
			/* luma */
			p = (uint8 *)img->buffer[0];
			
			for(j=0; j<img->height; j++)
			{
				for(k = 0;k<img->width;k++)
				{
					fwrite(p+k, sizeof(uint8), 1, fp);
				}

				p += img->stride[0];
			}

			/* chroma */
			for(i=1; i<3; i++)
			{
				p = (uint8 *)img->buffer[i];
				for(j=0; j<((img->height+1)>>1); j++)
				{
					for(k = 0;k<((img->width+1)>>1);k++)
					{
						fwrite(p+k, sizeof(uint8), 1, fp);
					}
					p += img->stride[i];
				}
			}
		}
		if(cs == S360_COLORSPACE_YUV420_10)
		{
			
			/* luma */
			p = (uint8 *)img->buffer[0];
			for(j=0; j<img->height; j++)
			{
				for(k = 0;k<((img->width));k++)
				{
					val16 = (p[k] << 2);
					fwrite(&val16, sizeof(uint16), 1, fp);
				}
				p += img->stride[0];
			}

			/* chroma */
			for(i=1; i<3; i++)
			{
				p = (uint8 *)img->buffer[i];
				for(j=0; j<((img->height+1)>>1); j++)
				{
					for(k = 0;k<((img->width+1)>>1);k++)
					{
						val16 = (p[k] << 2);
						fwrite(&val16, sizeof(uint16), 1, fp);
					}
					p += img->stride[i];
				}
			}
		}
	}
	else
	{
		s360_trace("cannot support the color space\n");
		return -1;
	}

	return S360_OK;
}


static void img_align_plane(uint8 * buf, int w, int h, int s, int align)
{
	uint8     * src, * dst;
	int         w_dst, h_dst;
	int         j, size;

	w_dst = S360_ALIGN(w, align);
	h_dst = S360_ALIGN(h, align);

	if(w < w_dst)
	{
		dst = buf + w;
		for(j=0; j<h; j++)
		{
			size = w_dst - w;
			s360_mset(dst, dst[-1], size);
			dst += s;
		}
	}

	if(h < h_dst)
	{
		src = buf + (h - 1) * s;
		dst = src + s;
		for(j=h; j<h_dst; j++)
		{
			s360_mcpy(dst, src, w_dst);
			dst += s;
		}
	}
}

int s360_img_realloc(S360_IMAGE * img, int w, int h, int opt)
{
	S360_IMAGE * img_new;
	int i;

	if(w > img->width || h > img->height)
	{
		img_new = s360_img_create(w, h, img->colorspace, opt);
		s360_assert_rv(img_new, S360_ERR_OUT_OF_MEMORY);

		for(i=0; i<4; i++)
		{
			if (img->buf_pad[i]) s360_mfree(img->buf_pad[i]);
			img->buf_pad[i] = img_new->buf_pad[i];
			img->buffer[i] = img_new->buffer[i];
			img->stride[i] = img_new->stride[i];
			img->elevation[i] = img_new->elevation[i];
		}

		s360_mfree(img_new);
	}

	img->width = w;
	img->height = h;

	return S360_OK;
}

static void img_align_plane_10b(uint16 * buf, int w, int h, int s, int align)
{
	uint16    * src, * dst;
	int         w_dst, h_dst;
	int         i,j, size;

	w_dst = S360_ALIGN(w, align);
	h_dst = S360_ALIGN(h, align);

	if(w < w_dst)
	{
		dst = buf + w;
		for(j=0; j<h; j++)
		{
			size = w_dst - w;
			for(i=0;i<size;i++)
			{
				dst[i] = dst[-1];
			}
			dst += s;
		}
	}

	if(h < h_dst)
	{
		src = buf + (h - 1) * s;
		dst = src + s;
		for(j=h; j<h_dst; j++)
		{
			s360_mcpy(dst, src, w_dst*sizeof(uint16));
			dst += s;
		}
	}
}

int s360_img_align(S360_IMAGE * img, int align)
{
	if(img->colorspace == S360_COLORSPACE_YUV420)
	{
		img_align_plane(img->buffer[0], img->width, img->height, img->stride[0], align);
		img_align_plane(img->buffer[1], img->width>>1, img->height>>1, img->stride[1], align>>1);
		img_align_plane(img->buffer[2], img->width>>1, img->height>>1, img->stride[2], align>>1);
		return S360_OK;
	}
	if(img->colorspace == S360_COLORSPACE_YUV420_10)
	{
		img_align_plane_10b(img->buffer[0], img->width, img->height, img->stride[0], align);
		img_align_plane_10b(img->buffer[1], img->width>>1, img->height>>1, img->stride[1], align>>1);
		img_align_plane_10b(img->buffer[2], img->width>>1, img->height>>1, img->stride[2], align>>1);
		return S360_OK;
	}
	else
	{
		return S360_ERR_UNSUPPORTED_COLORSPACE;
	}
}

