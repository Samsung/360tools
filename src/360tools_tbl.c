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

double tbl_tri_xyz[12][3] = \
{
	{  0,            0,            1           },
	{  0.894427191,  0,            0.447213595 },
	{  0.276393202,  0.850650808,  0.447213595 },
	{ -0.723606798,  0.525731112,  0.447213595 },
	{ -0.723606798, -0.525731112,  0.447213595 },
	{  0.276393202, -0.850650808,  0.447213595 },
	{  0.723606798,  0.525731112, -0.447213595 },
	{ -0.276393202,  0.850650808, -0.447213595 },
	{ -0.894427191,  0,           -0.447213595 },
	{ -0.276393202, -0.850650808, -0.447213595 },
	{  0.723606798, -0.525731112, -0.447213595 },
	{  0,            0,           -1           }
};

const int tbl_vidx_erp2cmp[6][4] = // Change cmp_plane_offset if needed
{
	{ 7, 0, 4, 1 }, // Top
	{ 0, 2, 1, 3 }, // Back
	{ 2, 6, 3, 5 },	// Bottom
	{ 7, 6, 0, 2 },	// Left
	{ 1, 3, 4, 5 },	// Right
	{ 4, 5, 7, 6 },	// Front
};

const double tbl_squ_center_xyz[6][3] = \
{
	{  0.0,            0.57735026919,  0.0           },
	{  0.0,            0.0,           -0.57735026919 },
	{  0.0,           -0.57735026919,  0.0           },
	{ -0.57735026919,  0.0,            0.0           },
	{  0.57735026919,  0.0,            0.0           },
	{  0.0,            0.0,            0.57735026919 }
};

const double tbl_squ_xyz[8][3] = \
{
	{ -0.57735026919,  0.57735026919, -0.57735026919 },
	{ 0.57735026919,   0.57735026919, -0.57735026919 },
	{ -0.57735026919, -0.57735026919, -0.57735026919 },
	{ 0.57735026919,  -0.57735026919, -0.57735026919 },
	{ 0.57735026919,   0.57735026919,  0.57735026919 },
	{ 0.57735026919,  -0.57735026919,  0.57735026919 },
	{ -0.57735026919, -0.57735026919,  0.57735026919 },
	{ -0.57735026919,  0.57735026919,  0.57735026919 }

};

double tbl_face_eqn[6][4] = \
{
	/* ax + by + cz = d */
	{ 0.0, 1.0, 0.0,  0.57735026919 },
	{ 0.0, 0.0, 1.0, -0.57735026919 },
	{ 0.0, 1.0, 0.0, -0.57735026919 },
	{ 1.0, 0.0, 0.0, -0.57735026919 },
	{ 1.0, 0.0, 0.0,  0.57735026919 },
	{ 0.0, 0.0, 1.0,  0.57735026919 }
};

const int tbl_vidx_erp2isp[20][3] =
{
	{ 0, 4,  5  },
	{ 0, 5,  1  },
	{ 0, 1,  2  },
	{ 0, 2,  3  },
	{ 0, 3,  4  },
	{ 5, 9,  10 },
	{ 1, 10, 6  },
	{ 2, 6,  7  },
	{ 3, 7,  8  },
	{ 4, 8,  9  },
	/* upside-down */
	{ 4,  5,  9  },
	{ 5,  1,  10 },
	{ 1,  2,  6  },
	{ 2,  3,  7  },
	{ 3,  4,  8  },
	{ 9,  10, 11 },
	{ 10, 6,  11 },
	{ 6,  7,  11 },
	{ 7,  8,  11 },
	{ 8,  9,  11 }
};

const int tbl_vidx_isp2erp[20][3] = \
{
	{ 0,  1,  2  },
	{ 0,  2,  3  },
	{ 0,  3,  4  },
	{ 0,  4,  5  },
	{ 0,  5,  1  },
	{ 1,  10, 6  },
	{ 6,  1,  2  },
	{ 2,  6,  7  },
	{ 7,  2,  3  },
	{ 3,  7,  8  },
	{ 8,  3,  4  },
	{ 4,  8,  9  },
	{ 9,  4,  5  },
	{ 5,  9,  10 },
	{ 10, 5,  1  },
	{ 11, 10, 6  },
	{ 11, 6,  7  },
	{ 11, 7,  8  },
	{ 11, 8,  9  },
	{ 11, 9,  10 }
};

const int tbl_vidx_pad_isp[20][2] = \
{
	{ 1, 1 },			//1
	{ 1, 1 },
	{ 1, 1 },
	{ 1, 1 },
	{ 1, 1 },			//5
	{ 0, 0 },
	{ 0, 0 },
	{ 0, 0 },
	{ 0, 0 },
	{ 0, 1 },			//10
	{ 1, 0 },
	{ 0, 0 },
	{ 0, 0 },
	{ 0, 0 },
	{ 0, 0 },			//15
	{ 1, 1 },
	{ 1, 1 },
	{ 1, 1 },
	{ 1, 1 },
	{ 1, 1 }			//20
};