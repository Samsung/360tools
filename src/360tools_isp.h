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

#ifndef __360TOOLS_ISP_H__2346758963478562398457629384562938576234__
#define __360TOOLS_ISP_H__2346758963478562398457629384562938576234__


#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* to ISP */
int s360_erp_to_isp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map);
int s360_risp2_to_isp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map);
int s360_risp_to_isp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map);
int s360_erp_to_risp1(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map);

/* from ISP */
int s360_isp_to_erp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map);
int s360_isp_to_cpp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map);
int s360_isp_to_risp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map);
int s360_isp_to_risp2(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map);
int s360_risp1_to_erp(S360_IMAGE * img_src, S360_IMAGE * img_dst, int opt, S360_MAP * map);

#ifdef __cplusplus
}
#endif


#endif /* __360TOOLS_ISP_H__2346758963478562398457629384562938576234__ */


