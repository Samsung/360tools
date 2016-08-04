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

#ifndef __360TOOLS_IMGB_H__19834765298347652938745629384562938562938456__
#define __360TOOLS_IMGB_H__19834765298347652938745629384562938562938456__


#ifdef __cplusplus
extern "C"
{
#endif


#include "360tools_def.h"


#define S360_ALIGN(val, align)       (((val) + (align) - 1) / (align) * (align))

/* create image buffer */
S360_IMAGE * s360_img_create(int w, int h, int cs, int opt);
/* delete image buffer */
void s360_img_delete(S360_IMAGE * img);

/* read an image from file */
int s360_img_read(FILE * fp, S360_IMAGE * img, int cs);
/* write an image to file */
int s360_img_write(FILE * fp, S360_IMAGE * img, int cs);

/* change width and height of the image */
int s360_img_realloc(S360_IMAGE * img, int w, int h, int opt);

/* align an image's width and height */
int s360_img_align(S360_IMAGE * img, int align);

/* Reset Image Buffer */
void s360_img_reset(S360_IMAGE * img);

#ifdef __cplusplus
}
#endif


#endif /* __360TOOLS_IMGB_H__19834765298347652938745629384562938562938456__ */

