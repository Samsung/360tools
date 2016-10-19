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
#include "360tools_map.h"

void map_init(S360_SPH_COORD * map, int w, int h)
{
	int i, size;

	for (i=0, size=w*h; i<size; i++)
	{
		map[i].lng = -1;
		map[i].lat = -1;
	}
}

void s360_map_delete(S360_MAP * map)
{
	s360_assert_r(map);

	if (map->layer[0]) s360_mfree(map->layer[0]);
	if (map->layer[1]) s360_mfree(map->layer[1]);
	s360_mfree(map);
}

static void erp2ohp_triangle(int x, int w, double h_tri, int tri_idx, \
	S360_SPH_COORD * map)
{
	double  vector_12[3], vector_13[3], point12[3], point13[3];
	double  point_vec[3], xyz[3];
	double  d12, d13, d12_scl, d13_scl, point_d;
	double  off;
	double  d_step;
	double  w_tri, w_start;
	int     v_1_3d, v_2_3d, v_3_3d;
	int     x_map, l;
	int     i, j, side;

	side = w / 4;
	x += (side >> 1);

	v_1_3d = tbl_vidx_erp2ohp[tri_idx][0];
	v_2_3d = tbl_vidx_erp2ohp[tri_idx][1];
	v_3_3d = tbl_vidx_erp2ohp[tri_idx][2];

	v3d_sub(tbl_tri_xyz_ohp[v_2_3d], tbl_tri_xyz_ohp[v_1_3d], vector_12);
	d12 = v3d_norm(vector_12);

	v3d_sub(tbl_tri_xyz_ohp[v_3_3d], tbl_tri_xyz_ohp[v_1_3d], vector_13);
	d13 = v3d_norm(vector_13);

	for (j = 0; j<h_tri; j++)
	{
		w_tri = (side * j) / h_tri;
		l = CEILING((w_tri - 1) / 2);
		w_start = (x + 0.5 - w_tri / 2);
		off = CEILING(w_start) - w_start;
		x_map = CEILING(w_start) - 1;
		d12_scl = d12 * (j / h_tri);
		d13_scl = d13 * (j / h_tri);

		v3d_affine(vector_12, d12_scl, tbl_tri_xyz_ohp[v_1_3d], point12);
		v3d_affine(vector_13, d13_scl, tbl_tri_xyz_ohp[v_1_3d], point13);

		v3d_sub(point13, point12, point_vec);

		point_d = v3d_norm(point_vec);
		d_step = (w_tri == 0 ? 0 : point_d / w_tri);

		v3d_affine(point_vec, d_step * off / 2, point12, xyz);
		v3d_norm(xyz);
		cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map]);

		v3d_affine(point_vec, d_step * off, point12, point12);
		for (i = 1; i<(l << 1); i++)
		{
			v3d_affine(point_vec, (i - 1) * d_step + d_step / 2, point12, xyz);
			v3d_norm(xyz);
			cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map + i]);
		}

		v3d_affine(point_vec, -d_step * off / 2, point13, xyz);
		v3d_norm(xyz);
		cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map + i]);

		map += w;
	}
}

static void erp2ohp_triangle_rev(int x, int w, double h_tri, int tri_idx, \
	S360_SPH_COORD * map)
{
	double  vector_12[3], vector_13[3], point12[3], point13[3];
	double  point_vec[3], xyz[3];
	double  d12, d13, d12_scl, d13_scl, point_d;
	double  off;
	double  d_step;
	double  w_tri, w_start;
	int     v_1_3d, v_2_3d, v_3_3d;
	int     x_map, l;
	int     i, j, side;

	side = w / 4;

	v_1_3d = tbl_vidx_erp2ohp[tri_idx + 4][0];
	v_2_3d = tbl_vidx_erp2ohp[tri_idx + 4][1];
	v_3_3d = tbl_vidx_erp2ohp[tri_idx + 4][2];

	v3d_sub(tbl_tri_xyz_ohp[v_3_3d], tbl_tri_xyz_ohp[v_1_3d], vector_12);
	d12 = v3d_norm(vector_12);

	v3d_sub(tbl_tri_xyz_ohp[v_3_3d], tbl_tri_xyz_ohp[v_2_3d], vector_13);
	d13 = v3d_norm(vector_13);

	for (i = 0; i<h_tri; i++)
	{
		w_tri = ((h_tri - i) * side) / h_tri;
		l = CEILING((w_tri - 1) / 2);
		w_start = (x + 0.5 - w_tri / 2);
		off = CEILING(w_start) - w_start;
		x_map = CEILING(w_start + (side >> 1)) - 1;
		d12_scl = d12 * (i / h_tri);
		d13_scl = d13 * (i / h_tri);

		v3d_affine(vector_12, d12_scl, tbl_tri_xyz_ohp[v_1_3d], point12);
		v3d_affine(vector_13, d13_scl, tbl_tri_xyz_ohp[v_2_3d], point13);

		v3d_sub(point13, point12, point_vec);

		point_d = v3d_norm(point_vec);
		d_step = (w_tri == 0 ? 0 : point_d / w_tri);

		v3d_affine(point_vec, d_step * off / 2, point12, xyz);
		v3d_norm(xyz);
		cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map]);

		v3d_affine(point_vec, d_step * off, point12, point12);
		for (j = 1; j<(l << 1); j++)
		{
			v3d_affine(point_vec, (j - 1) * d_step + d_step / 2, point12, xyz);
			v3d_norm(xyz);
			cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map + j]);
		}

		v3d_affine(point_vec, -d_step * off / 2, point13, xyz);
		v3d_norm(xyz);
		cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map + j]);

		map += w;
	}
}

void general_mapping(double input_x, double j, S360_SPH_COORD * map2_coord, \
	int x, int side, int h_tri, double vector_12[3], double vector_13[3], \
	int v_1_3d, double d12, double d13)
{
	double  point12[3], point13[3];
	double  point_vec[3], xyz[3];
	double  d12_scl, d13_scl, point_d;
	double  off;
	double  d_step;
	double  w_tri, w_start;
	double  x_map, l, i;

	w_tri = (side * j) / h_tri;

	l = CEILING((w_tri - 1) / 2);
	w_start = (x + 0.5 - w_tri / 2);
	off = CEILING(w_start) - w_start;
	x_map = CEILING(w_start) - 1;

	d12_scl = d12 * (j / h_tri);
	d13_scl = d13 * (j / h_tri);

	v3d_affine(vector_12, d12_scl, tbl_tri_xyz_ohp[v_1_3d], point12);
	v3d_affine(vector_13, d13_scl, tbl_tri_xyz_ohp[v_1_3d], point13);

	v3d_sub(point13, point12, point_vec);

	point_d = v3d_norm(point_vec);
	d_step = (w_tri == 0 ? 0 : point_d / w_tri);

	v3d_affine(point_vec, d_step * off / 2, point12, xyz);
	v3d_norm(xyz);

	if (input_x<x_map+1)
	{
	    cart_to_sph(xyz[0], xyz[1], xyz[2], &map2_coord[0]);
	}
	else if ((x_map+1<=input_x) && (input_x<x_map+2*l)) 
	{
		v3d_affine(point_vec, d_step * off, point12, point12);
		i = input_x-x_map;
		v3d_affine(point_vec, (i - 1) * d_step + d_step / 2, point12, xyz);
		v3d_norm(xyz);
		cart_to_sph(xyz[0], xyz[1], xyz[2], &map2_coord[0]);
	} 
	else if (input_x >= x_map+2*l) 
	{
		v3d_affine(point_vec, -d_step * off / 2, point13, xyz);
		v3d_norm(xyz);
		cart_to_sph(xyz[0], xyz[1], xyz[2], &map2_coord[0]);
	}
}

void general_mapping_rev(double input_x, double j, S360_SPH_COORD * map2_coord, \
	int x, int side, int h_tri, double vector_12[3], double vector_13[3], \
	int v_1_3d, int v_2_3d, double d12, double d13)
{
	double  point12[3], point13[3];
	double  point_vec[3], xyz[3];
	double  d12_scl, d13_scl, point_d;
	double  off;
	double  d_step;
	double  w_tri, w_start;
	double  x_map, l, i;

	w_tri = (h_tri-j) * side/ h_tri;
	l = CEILING((w_tri - 1) / 2);
	w_start = (x + 0.5 - w_tri / 2);
	off = CEILING(w_start) - w_start;
	x_map = CEILING(w_start + (side >> 1)) - 1;
	d12_scl = d12 * (j / h_tri);
	d13_scl = d13 * (j / h_tri);

	v3d_affine(vector_12, d12_scl, tbl_tri_xyz_ohp[v_1_3d], point12);
	v3d_affine(vector_13, d13_scl, tbl_tri_xyz_ohp[v_2_3d], point13);

	v3d_sub(point13, point12, point_vec);

	point_d = v3d_norm(point_vec);
	d_step = (w_tri == 0 ? 0 : point_d / w_tri);

	v3d_affine(point_vec, d_step * off / 2, point12, xyz);
	v3d_norm(xyz);

	if (input_x >= x_map-1 && input_x<x_map+1)
	{
	    cart_to_sph(xyz[0], xyz[1], xyz[2], &map2_coord[0]);
	}
	else if ((x_map+1<=input_x) && (input_x<x_map+2*l)) 
	{
		v3d_affine(point_vec, d_step * off, point12, point12);
		i = input_x-x_map;
		v3d_affine(point_vec, (i - 1) * d_step + d_step / 2, point12, xyz);
		v3d_norm(xyz);
		cart_to_sph(xyz[0], xyz[1], xyz[2], &map2_coord[0]);
	} 
	else if (input_x >= x_map+2*l && input_x<=x_map+2*l+1) 
	{
		v3d_affine(point_vec, -d_step * off / 2, point13, xyz);
		v3d_norm(xyz);
		cart_to_sph(xyz[0], xyz[1], xyz[2], &map2_coord[0]);
	}
}

static void erp2cohp_triangle(int x, int w, int h_tri, int tri_idx, S360_SPH_COORD * map2_coord)
{
	double  vector_12[3], vector_13[3];
	double  d12, d13;
	int     v_1_3d, v_2_3d, v_3_3d;
	int     i, j, side;
	double  input_x, input_y;

	side = w / 4;
	x += (side >> 1);

	v_1_3d = tbl_vidx_erp2ohp[tri_idx][0];
	v_2_3d = tbl_vidx_erp2ohp[tri_idx][1];
	v_3_3d = tbl_vidx_erp2ohp[tri_idx][2];

	v3d_sub(tbl_tri_xyz_ohp[v_2_3d], tbl_tri_xyz_ohp[v_1_3d], vector_12);
	d12 = v3d_norm(vector_12);

	v3d_sub(tbl_tri_xyz_ohp[v_3_3d], tbl_tri_xyz_ohp[v_1_3d], vector_13);
	d13 = v3d_norm(vector_13);

	if (tri_idx==0) 
	{
		for (j=0; j<h_tri; j++) 
		{
			for (i=side/2; i<side*3/2; i++) 
			{
				if ((i>=side && i-(side)<=CEILING((double)(side)/2*(h_tri-0.5-j)/(h_tri-0.5))) || \
					(i<side && side-i<=CEILING((double)(side)/2*(h_tri-0.5-j)/(h_tri-0.5)))) 
				{
					input_x = (i-side+0.5)/2-SIN_60*(h_tri-j-0.5) + side-0.5;
					input_y = h_tri-0.5 - (SIN_60*(i-side+0.5)+(h_tri-0.5-j)/2);

					general_mapping(input_x, input_y, &map2_coord[j*w+i], x, side, h_tri, \
						vector_12, vector_13, v_1_3d, d12, d13);
				}
			}
		}

		//padding
		for (j=0; j<h_tri; j++) 
		{
			for (i=side/2; i<side/2+side/2*(double)j/h_tri; i++)
			{
				map2_coord[j*w+i] = map2_coord[j*w+side/2+(int)(side/2*j/h_tri)];
			}
		}
	} 
	else if (tri_idx==1) 
	{
		for (j=0; j<h_tri; j++)
			for (i=0; i<side; i++) 
			{
				if ((i>=side/2 && i-(side/2)<=CEILING((double)(side)/2*j/(h_tri-0.5))) || \
					(i<side/2 && (side/2)-i<=CEILING((double)(side)/2*j/(h_tri-0.5)))) 
				{
					input_x = x+i-(side>>1);
					input_y = j;
					general_mapping(input_x, input_y, &map2_coord[j*w+i+side], x, side, \
						h_tri, vector_12, vector_13, v_1_3d, d12, d13);
				}
			}	
	} 
	else if (tri_idx==2) 
	{
		for (j=0; j<h_tri; j++) 
		{
			for (i=(int)(1.5*side); i<(int)(2.5*side); i++) 
			{
				if ((i>=side*2 && i-side*2<=CEILING((double)side/2*(h_tri-0.5-j)/(h_tri-0.5))) || \
					(i<side*2 && side*2-i<=CEILING((double)side/2*(h_tri-0.5-j)/(h_tri-0.5)))) 
				{
					input_x = (i-2*side-0.5)/2+SIN_60*(h_tri-j-0.5) + 2*side+0.5;
					input_y = h_tri-0.5 - (-SIN_60*(i-2*side-0.5)+(h_tri-j-0.5)/2);
					general_mapping(input_x, input_y, &map2_coord[j*w+i], x, side, h_tri, \
						vector_12, vector_13, v_1_3d, d12, d13);
				}
			}
		}

		//padding
		for (j=0; j<h_tri; j++) 
		{
			for (i=(int)(side*2.5-(double)j/h_tri*side/2); i<side*2.5; i++)
			{
				map2_coord[j*w+i] = map2_coord[j*w+side*5/2-(int)(side/2*j/h_tri)];
			}
		}
	} 
	else if (tri_idx==3) 
	{
		for (j=0; j<h_tri; j++)
		{
			for (i=3*side; i<3.5*side; i++) 
			{
				if (i<side*7/2 && side*7/2-i<=CEILING((double)side/2*j/(h_tri-0.5))) 
				{
					input_x = i;
					input_y = j;

					general_mapping(input_x, input_y, &map2_coord[j*w+i], x, side, h_tri, \
						vector_12, vector_13, v_1_3d, d12, d13);
				}
			}
		}
		for (j=0; j<h_tri; j++)
		{
			for (i=(int)(3.5*side); i<4*side; i++) 
			{
				if (i>=side*7/2 && i-side*7/2<=CEILING((double)side/2*j/(h_tri-0.5))) 
				{
					input_x = i;
					input_y = j;

					general_mapping(input_x, input_y, &map2_coord[j*w+i], x, side, h_tri, \
						vector_12, vector_13, v_1_3d, d12, d13);
				}
			}	
		}

		//padding
		for (j=0; j<h_tri; j++) 
		{
		    for (i=side*3; i<3*side+side/2*(double)(h_tri-j)/h_tri; i++)
			{
				map2_coord[j*w+i] = map2_coord[j*w+3*side+(int)(side/2*(h_tri-j)/h_tri)];
			}
			for (i=(int)(side*3.5+side/2*(double)(j)/h_tri); i<side*4; i++)
			{
				map2_coord[j*w+i] = map2_coord[j*w+(int)(side*3.5)+(int)(side/2*(j)/h_tri)];
			}
		}

	}
}

static void erp2cohp_triangle_rev(int x, int w, int h_tri, int tri_idx, S360_SPH_COORD * map2_coord)
{
	double  vector_12[3], vector_13[3];
	double  d12, d13;
	int     v_1_3d, v_2_3d, v_3_3d;
	int     i, j, side;
	double  input_x, input_y;

	side = w / 4;

	v_1_3d = tbl_vidx_erp2ohp[tri_idx + 4][0];
	v_2_3d = tbl_vidx_erp2ohp[tri_idx + 4][1];
	v_3_3d = tbl_vidx_erp2ohp[tri_idx + 4][2];

	v3d_sub(tbl_tri_xyz_ohp[v_3_3d], tbl_tri_xyz_ohp[v_1_3d], vector_12);
	d12 = v3d_norm(vector_12);

	v3d_sub(tbl_tri_xyz_ohp[v_3_3d], tbl_tri_xyz_ohp[v_2_3d], vector_13);
	d13 = v3d_norm(vector_13);

	if (tri_idx==0) 
	{
		for (j=0; j<h_tri; j++)
			for (i=(int)(0.5*side); i<(int)(1.5*side); i++) 
			{
				if ((i>=side && i-(side)<=CEILING((double)(side)/2*j/(h_tri+0.5))) ||
					(i<side && side-i<=CEILING((double)(side)/2*j/(h_tri+0.5)))) 
				{

					input_x = (i-side+0.5)/2 + SIN_60*(-j+1.5) + side-0.5;
					input_y = -(-SIN_60*(i-side+0.5)+(-j+1.5)/2) +1.5;
					general_mapping_rev(input_x, input_y, &map2_coord[j*w+i + w*(int)h_tri], x, side, h_tri, \
						vector_12, vector_13, v_1_3d, v_2_3d, d12, d13);
				}

			}

		//padding
		for (j=0; j<h_tri; j++) 
		{
		    for (i=(int)(side*0.5); i<(int)(side*0.5+(double)(h_tri-j)/h_tri*side/2); i++)
			{
				map2_coord[j*w+i+w*(int)h_tri] = map2_coord[j*w+w*(int)h_tri+side/2+(int)(side/2*(h_tri-j)/h_tri)];
			}
		}
	} 
	else if (tri_idx==1) 
	{
		for (j=0; j<h_tri; j++)
		{
			for (i=0; i<side; i++) 
			{
				if ((i>=side/2 && i-(side/2)<=CEILING((double)(side/2)*(h_tri+0.5-j)/(h_tri+0.5))) ||
					(i<side/2 && side/2-i<=CEILING((double)(side/2)*(h_tri+0.5-j)/(h_tri+0.5)))) 
				{
					input_x = x+i;
					input_y = j;
					general_mapping_rev(input_x, input_y, &map2_coord[j*w+i+w*(int)h_tri+side], x, side, h_tri, \
						vector_12, vector_13, v_1_3d, v_2_3d, d12, d13);
				}
			}	
		}
	} 
	else if (tri_idx==2) 
	{
		for (j=0; j<h_tri; j++)
		{
			for (i=(int)(1.5*side); i<(int)(2.5*side); i++) 
			{
				if ((i>=side*2 && i-(side*2)<=CEILING((double)(side)/2*j/(h_tri+0.5))) ||
					(i<side*2 && side*2-i<=CEILING((double)(side)/2*j/(h_tri+0.5)))) 
				{
					input_x = (i-2*side-0.5)/2-SIN_60*(-j+1.5) + 2*side+0.5;
					input_y =  - (SIN_60*(i-2*side-0.5)+(-j+1.5)/2) +1.5;
					general_mapping_rev(input_x, input_y, &map2_coord[j*w+i + w*(int)h_tri], x, side, h_tri, 
						vector_12, vector_13, v_1_3d, v_2_3d, d12, d13);
				}
			}
		}

		//padding
		for (j=0; j<h_tri; j++) 
		{
		    for (i=(int)(side*2.5-(double)(h_tri-j)/h_tri*side/2); i<(int)(side*2.5); i++)
			{
				map2_coord[j*w+i+w*(int)h_tri] = map2_coord[j*w+w*(int)h_tri+side*5/2-(int)(side/2*(h_tri-j)/h_tri)];
			}
		}
	} 
	else if (tri_idx==3) 
	{
		for (j=0; j<h_tri; j++)
		{
			for (i=3*side; i<3.5*side; i++) 
			{
				if (i<side*7/2 && side*7/2-i<=CEILING((double)side/2*(h_tri+0.5-j)/(h_tri+0.5))) 
				{
					input_x = i;
					input_y = j;

					general_mapping_rev(input_x, input_y, &map2_coord[j*w+i+w*(int)h_tri], x, side, h_tri, \
						vector_12, vector_13, v_1_3d, v_2_3d, d12, d13);
				}
			}
		}
		for (j=0; j<h_tri; j++)
		{
			for (i=(int)(3.5*side); i<4*side; i++) 
			{
				if (i>=side*7/2 && i-side*7/2<=CEILING((double)side/2*(h_tri+0.5-j)/(h_tri+0.5))) 
				{
					input_x = i;
					input_y = j;

					general_mapping_rev(input_x, input_y, &map2_coord[j*w+i+w*(int)h_tri], x, side, h_tri, \
						vector_12, vector_13, v_1_3d, v_2_3d, d12, d13);
				}

			}	
		}

		//padding
		for (j=0; j<h_tri; j++) 
		{
		    for (i=side*3; i<(int)(3*side+side/2*(double)(j)/h_tri); i++)
			{
				map2_coord[j*w+i+w*(int)h_tri] = map2_coord[j*w+w*(int)h_tri+3*side+(int)(side/2*(j)/h_tri)];
			}
		    for (i=(int)(side*3.5+side/2*(double)(h_tri-j)/h_tri); i<side*4; i++)
			{
				map2_coord[j*w+i+w*(int)h_tri] = map2_coord[j*w+w*(int)h_tri+(int)(side*3.5)+(int)(side/2*(h_tri-j)/h_tri)];
			}
		}
	}
}

static void init_sph2ohp_map(S360_SPH_COORD * map, int w_ohp, int h_ohp, int w_tri)
{
	int i, x, y1, y2, h_tri, size;

	for (i = 0, size = w_ohp*h_ohp; i<size; i++)
	{
		map[i].lng = -1;
		map[i].lat = -1;
	}

	h_tri = (h_ohp / 2);
	y1 = h_tri;
	y2 = (h_tri << 1);

	/* first row */
	for (i = 0, x = 0; i<4; i++, x += w_tri)
	{
		erp2ohp_triangle(x, w_ohp, y1, i, map);
	}
	/* second row */
	map += y1 * w_ohp;
	for (i = 0, x = 0; i<4; i++, x += w_tri)
	{
		erp2ohp_triangle_rev(x, w_ohp, h_tri, i, map);
	}
}

static void init_sph2cohp_map(S360_SPH_COORD * map, int w_ohp, int h_ohp, int w_tri, int layer_num)
{
	int i, x, y1, y2, h_tri, size;
	S360_SPH_COORD *map2_coord = map;

	for (i = 0, size = w_ohp*h_ohp; i<size; i++)
	{
		map[i].lng = -1;
		map[i].lat = -1;
	}

	h_tri = (h_ohp / 2);
	y1 = h_tri;
	y2 = (h_tri << 1);

	/* first row */
	for (i = 0, x = 0; i<4; i++, x += w_tri)
	{
		erp2cohp_triangle(x, w_ohp, y1, i, map2_coord);
	}
	/* second row */
	map += y1 * w_ohp;
	for (i = 0, x = 0; i<4; i++, x += w_tri)
	{
		erp2cohp_triangle_rev(x, w_ohp, h_tri, i, map2_coord);
	}
}

/*------------------------------------ERP TO OHP----------------------------------------*/
static int map_to_ohp(S360_MAP * map, int opt)
{
	int w, h, w_tri, h_tri;
	w = map->width;
	h = map->height;
	w_tri = GET_W_TRI_OHP(w);
	h_tri = GET_H_TRI_OHP(w_tri);

	s360_assert_rv(w == w_tri * 4, S360_ERR_INVALID_DIMENSION);
	s360_assert_rv(h == 2 * h_tri, S360_ERR_INVALID_DIMENSION);

	init_sph2ohp_map(map->layer[0], w, h, w_tri);
	init_sph2ohp_map(map->layer[1], (w >> 1), (h >> 1), (w_tri >> 1));

	return S360_OK;
}

static int map_erp_to_cohp(S360_MAP * map, int opt)
{
	int w, h, w_tri, h_tri;
	w = map->width;
	h = map->height;
	w_tri = GET_W_TRI_OHP(w);
	h_tri = GET_H_TRI_OHP(w_tri);

	s360_assert_rv(w == w_tri * 4, S360_ERR_INVALID_DIMENSION);
	s360_assert_rv(h == 2 * h_tri, S360_ERR_INVALID_DIMENSION);

	init_sph2cohp_map(map->layer[0], w, h, w_tri, 0);
	init_sph2cohp_map(map->layer[1], (w >> 1), (h >> 1), (w_tri >> 1), 1);

	return S360_OK;
}

static void erp2isp_triangle(int x, int w, int w_tri, int h_tri, \
	int tri_idx, S360_SPH_COORD * map, int pad_sz, int align_sz, int is_risp)
{
	double  vector_12[3], vector_13[3], point12[3], point13[3];
	double  point_vec[3], xyz[3];
	double  d12, d13, d12_scl, d13_scl, point_d;
	double  off;
	double  d_step;
	double  w_cur, w_start;
	int     v_1_3d, v_2_3d, v_3_3d;
	int     x_map, l;
	int     i, j;

	int     pad_left, pad_rt;
	int     start_w, end_w;
	int     x_start;
	int     y_start;
	int     prev_align_left, prev_align_rt;
	int     align;
	
	x_start = x;

	x += (w_tri >> 1);

	if(pad_sz != 0)
	{
		pad_left = tbl_vidx_pad_isp[tri_idx][0];
		pad_rt = tbl_vidx_pad_isp[tri_idx][1];
		if((is_risp == 2) && (tri_idx == 9))
		{
			pad_rt = 0;
		}
	}
	else
	{
		pad_left = pad_rt = 0;
	}
	if(is_risp == 1)
	{
		pad_left = pad_rt = 1;
	}

	if(tri_idx<5) y_start = 0;
	else y_start = h_tri;
	
	v_1_3d = tbl_vidx_erp2isp[tri_idx][0];
	v_2_3d = tbl_vidx_erp2isp[tri_idx][1];
	v_3_3d = tbl_vidx_erp2isp[tri_idx][2];

	v3d_sub(tbl_tri_xyz[v_2_3d], tbl_tri_xyz[v_1_3d], vector_12);
	d12 = v3d_norm(vector_12);

	v3d_sub(tbl_tri_xyz[v_3_3d], tbl_tri_xyz[v_1_3d], vector_13);
	d13 = v3d_norm(vector_13);

	for (j = 0; j<h_tri; j++)
	{
		w_cur = (double)(w_tri * j) / h_tri;
		l = CEILING((w_cur - 1) / 2);
		w_start = (x + 0.5 - w_cur / 2);
		off = CEILING(w_start) - w_start;
		x_map = CEILING(w_start) - 1;
		d12_scl = d12 * ((double)j / h_tri);
		d13_scl = d13 * ((double)j / h_tri);

		v3d_affine(vector_12, d12_scl, tbl_tri_xyz[v_1_3d], point12);
		v3d_affine(vector_13, d13_scl, tbl_tri_xyz[v_1_3d], point13);

		v3d_sub(point13, point12, point_vec);

		point_d = v3d_norm(point_vec);
		d_step = (w_cur == 0 ? 0 : point_d / w_cur);
		start_w = 0;
		end_w = (l << 1);

		if (pad_left)
		{
			start_w -= pad_sz;

			if(((j+y_start) & ((1<<align_sz)-1)) == 0)
			{
				align = ((start_w + x_map)>>align_sz)<<align_sz;
				prev_align_left = align;
			}
			else
			{
				align = prev_align_left;
			}
			start_w = align - x_map;
			start_w = (start_w + x_map < x_start) ? (x_start - x_map) : start_w;
			if(is_risp == 1)
				start_w = -pad_sz;

			for (i = start_w; i<0; i++)
			{
				v3d_affine(point_vec, (i - 1) * d_step + d_step / 2, point12, xyz);
				v3d_norm(xyz);
				cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map + i]);
			}
		}

		{
			v3d_affine(point_vec, d_step * off / 2, point12, xyz);
			v3d_norm(xyz);
			cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map]);
			start_w++;
		}

		v3d_affine(point_vec, d_step * off, point12, point12);
		for (i = 1; i<(l << 1); i++)
		{
			v3d_affine(point_vec, (i - 1) * d_step + d_step / 2, point12, xyz);
			v3d_norm(xyz);
			cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map + i]);
		}

		{
			v3d_affine(point_vec, -d_step * off / 2, point13, xyz);
			v3d_norm(xyz);
			cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map + i]);
		}

		if (pad_rt)
		{
			i++;

			end_w += pad_sz;

			if(((j+y_start) & ((1<<align_sz)-1)) == 0)
			{
				align = ((end_w + x_map)>>align_sz)<<align_sz;
				prev_align_rt= align;
			}
			else
			{
				align = prev_align_rt;
			}

			end_w = align - x_map;

			if (x_map + end_w > w) end_w = w - x_map;
			
			if(is_risp == 1)
				end_w = i + pad_sz;
			for (; i<end_w; i++)
			{
				v3d_affine(point_vec, (i - 1) * d_step + d_step / 2, point12, xyz);
				v3d_norm(xyz);
				cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map + i]);
			}
		}

		map += w;
	}
}

static void erp2isp_triangle_rev(int x, int w, int w_tri, int h_tri, \
	int tri_idx, S360_SPH_COORD * map, int pad_sz, int align_sz, int is_risp)
{
	double  vector_12[3], vector_13[3], point12[3], point13[3];
	double  point_vec[3], xyz[3];
	double  d12, d13, d12_scl, d13_scl, point_d;
	double  off;
	double  d_step;
	double  w_cur, w_start;
	int     v_1_3d, v_2_3d, v_3_3d;
	int     x_map, l;
	int     i, j;

	int     pad_left, pad_rt;
	int     start_w, end_w;
	int     x_start, y_start;
	int     prev_align_left, prev_align_rt;
	int     align;

	prev_align_left = prev_align_rt = 0;

	if(tri_idx<5) y_start = h_tri;
	else y_start = h_tri*2;

	if(is_risp == 1)
	{
		pad_left = pad_rt = 1;
	}
	else if(pad_sz != 0)
	{
		pad_left = tbl_vidx_pad_isp[tri_idx + 10][0];
		pad_rt = tbl_vidx_pad_isp[tri_idx + 10][1];
		if((is_risp == 2) && (tri_idx == 9))
		{
			pad_left = 0;
		}
	}
	else
	{
		pad_left = pad_rt = 0;
	}
	
	x_start = x;

	v_1_3d = tbl_vidx_erp2isp[tri_idx + 10][0];
	v_2_3d = tbl_vidx_erp2isp[tri_idx + 10][1];
	v_3_3d = tbl_vidx_erp2isp[tri_idx + 10][2];

	v3d_sub(tbl_tri_xyz[v_3_3d], tbl_tri_xyz[v_1_3d], vector_12);
	d12 = v3d_norm(vector_12);

	v3d_sub(tbl_tri_xyz[v_3_3d], tbl_tri_xyz[v_2_3d], vector_13);
	d13 = v3d_norm(vector_13);

	for (i = 0; i<h_tri; i++)
	{
		w_cur = ((double)(h_tri - i) * w_tri) / h_tri;
		l = CEILING((w_cur - 1) / 2);
		w_start = (x + 0.5 - w_cur / 2);
		off = CEILING(w_start) - w_start;
		x_map = CEILING(w_start + (w_tri >> 1)) - 1;
		d12_scl = d12 * ((double)i / h_tri);
		d13_scl = d13 * ((double)i / h_tri);

		v3d_affine(vector_12, d12_scl, tbl_tri_xyz[v_1_3d], point12);
		v3d_affine(vector_13, d13_scl, tbl_tri_xyz[v_2_3d], point13);

		v3d_sub(point13, point12, point_vec);

		point_d = v3d_norm(point_vec);
		d_step = (w_cur == 0 ? 0 : point_d / w_cur);
		start_w = 0;
		end_w = (l << 1);

		if (pad_left)
		{
			start_w -= pad_sz;

			if(((i+y_start) & ((1<<align_sz)-1)) == 0)
			{
				align = ((start_w + x_map)>>align_sz)<<align_sz;
				prev_align_left = align;
			}
			else
			{
				align = prev_align_left;
			}
			start_w = align - x_map;

			if (tri_idx + 10 != 15)
				start_w = (start_w + x_map < x_start) ? (x_start - x_map) : start_w;
			if(is_risp == 1)
				start_w = -pad_sz;
			for (j = start_w; j<0; j++)
			{
				v3d_affine(point_vec, (j - 1) * d_step + d_step / 2, point12, xyz);
				v3d_norm(xyz);
				cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map + j]);
			}

		}

		{
			v3d_affine(point_vec, d_step * off / 2, point12, xyz);
			v3d_norm(xyz);
			cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map]);
			start_w++;
		}

		v3d_affine(point_vec, d_step * off, point12, point12);
		for (j = 1; j<(l << 1); j++)
		{
			v3d_affine(point_vec, (j - 1) * d_step + d_step / 2, point12, xyz);
			v3d_norm(xyz);
			cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map + j]);
		}

		{
			v3d_affine(point_vec, -d_step * off / 2, point13, xyz);
			v3d_norm(xyz);
			cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map + j]);
		}

		if (pad_rt)
		{
			end_w += pad_sz;

			if(((i+y_start) & ((1<<align_sz)-1)) == 0)
			{
				align = ((end_w + x_map)>>align_sz)<<align_sz;
				prev_align_rt= align;
			}
			else
			{
				align = prev_align_rt;
			}

			end_w = align - x_map;

			if (x_map + end_w > w) end_w = w - x_map;
			j++;
			if(is_risp == 1)
				end_w = j + pad_sz;
			for (; j<end_w; j++)
			{
				v3d_affine(point_vec, (j - 1) * d_step + d_step / 2, point12, xyz);
				v3d_norm(xyz);
				cart_to_sph(xyz[0], xyz[1], xyz[2], &map[x_map + j]);
			}
		}

		map += w;
	}
}

static void init_sph2isp_map(S360_SPH_COORD * map, int w_isp, int h_isp, int w_tri, int pad_sz, int plane, int is_risp)
{
	int i, x, y1, y2, h_tri, size;
	int align_sz;

	for (i = 0, size = w_isp*h_isp; i<size; i++)
	{
		map[i].lng = -1;
		map[i].lat = -1;
	}

	h_tri = (h_isp / 3);
	y1 = h_tri;
	y2 = (h_tri << 1);
	
	align_sz = (plane==0)?PAD_ALIGN:PAD_ALIGN-1;

	/* first row */
	for (i = 0, x = 0; i<5; i++, x += w_tri)
	{
		erp2isp_triangle(x, w_isp, w_tri, y1, i, map, pad_sz, align_sz, is_risp);
	}

	/* second row */
	map += y1 * w_isp;
	for (i = 5, x = (w_tri >> 1); i<10; i++, x += w_tri)
	{
		erp2isp_triangle(x, w_isp, w_tri, h_tri, i, map, pad_sz, align_sz, is_risp);
	}
	for (i = 0, x = 0; i<5; i++, x += w_tri)
	{
		erp2isp_triangle_rev(x, w_isp, w_tri, h_tri, i, map, pad_sz, align_sz, is_risp);
	}

	/* third row */
	map += (y2 - y1) * w_isp;
	for (i = 5, x = (w_tri >> 1); i<10; i++, x += w_tri)
	{
		erp2isp_triangle_rev(x, w_isp, w_tri, h_tri, i, map, pad_sz, align_sz, is_risp);
	}
}

static void erp2isp_triangle_half(int x, int w, int w_tri, int h_tri, \
	int tri_idx, S360_SPH_COORD * map, int pad_sz, int align_sz, int plane)
{
	double  vector_12[3], vector_13[3], point12[3], point13[3];
	double  point_vec[3], xyz[3];
	double  d12, d13, d12_scl, d13_scl, point_d;
	double  off;
	double  d_step;
	double  w_cur, w_start;
	int     v_1_3d, v_2_3d, v_3_3d;
	int     x_map, l;
	int     i, j;

	int     pad_left, pad_rt;
	int     start_w, end_w;
	int     x_start;
	int     y_start;
	int	    prev_align_left, prev_align_rt;
	int     align;
	int		risp_idx;
	
	x_start = x;

	x += (w_tri >> 1);

	if(pad_sz != 0)
	{
		pad_left = pad_rt = 1;
	}
	else
	{
		pad_left = pad_rt = 0;
	}

	if(tri_idx<5) y_start = 0;
	else y_start = h_tri;

	v_1_3d = tbl_vidx_erp2isp[tri_idx][0];
	v_2_3d = tbl_vidx_erp2isp[tri_idx][1];
	v_3_3d = tbl_vidx_erp2isp[tri_idx][2];

	v3d_sub(tbl_tri_xyz[v_2_3d], tbl_tri_xyz[v_1_3d], vector_12);
	d12 = v3d_norm(vector_12);

	v3d_sub(tbl_tri_xyz[v_3_3d], tbl_tri_xyz[v_1_3d], vector_13);
	d13 = v3d_norm(vector_13);

	for (j = 0; j<h_tri; j++)
	{
		w_cur = (double)(w_tri * j) / h_tri;
		l = CEILING((w_cur - 1) / 2);
		w_start = (x + 0.5 - w_cur / 2);
		off = CEILING(w_start) - w_start;
		x_map = CEILING(w_start) - 1;
		d12_scl = d12 * ((double)j / h_tri);
		d13_scl = d13 * ((double)j / h_tri);

		v3d_affine(vector_12, d12_scl, tbl_tri_xyz[v_1_3d], point12);
		v3d_affine(vector_13, d13_scl, tbl_tri_xyz[v_1_3d], point13);

		v3d_sub(point13, point12, point_vec);

		point_d = v3d_norm(point_vec);
		d_step = (w_cur == 0 ? 0 : point_d / w_cur);
		start_w = 0;
		end_w = (l << 1);

		if (pad_left)
		{
			start_w -= pad_sz;

			if(((j+y_start) & ((1<<align_sz)-1)) == 0)
			{
				align = ((start_w + x_map)>>align_sz)<<align_sz;
				prev_align_left = align;
			}
			else
			{
				align = prev_align_left;
			}
			start_w = align - x_map;
			start_w = (start_w + x_map < x_start) ? (x_start - x_map) : start_w;
			for (i = start_w; i<0; i++)
			{
				v3d_affine(point_vec, (i - 1) * d_step + d_step / 2, point12, xyz);
				v3d_norm(xyz);
				risp_idx = x_map+i;
				risp_idx = risp_idx >= w ? risp_idx-w : risp_idx;
				cart_to_sph(xyz[0], xyz[1], xyz[2], &map[risp_idx]);
			}
		}

		{
			v3d_affine(point_vec, d_step * off / 2, point12, xyz);
			v3d_norm(xyz);
			risp_idx = x_map;
			risp_idx = risp_idx >= w ? risp_idx-w : risp_idx;
			cart_to_sph(xyz[0], xyz[1], xyz[2], &map[risp_idx]);
			start_w++;
		}

		v3d_affine(point_vec, d_step * off, point12, point12);
		for (i = 1; i<(l << 1); i++)
		{
			v3d_affine(point_vec, (i - 1) * d_step + d_step / 2, point12, xyz);
			v3d_norm(xyz);
			risp_idx = x_map+i;
			risp_idx = risp_idx >= w ? risp_idx-w : risp_idx;
			cart_to_sph(xyz[0], xyz[1], xyz[2], &map[risp_idx]);
		}

		{
			v3d_affine(point_vec, -d_step * off / 2, point13, xyz);
			v3d_norm(xyz);
			risp_idx = x_map+i;
			risp_idx = risp_idx >= w ? risp_idx-w : risp_idx;
			cart_to_sph(xyz[0], xyz[1], xyz[2], &map[risp_idx]);
		}

		if (pad_rt)
		{
			i++;
			end_w += pad_sz;
			if(((j+y_start) & ((1<<align_sz)-1)) == 0)
			{
				align = ((end_w + x_map)>>align_sz)<<align_sz;
				prev_align_rt= align;
			}
			else
			{
				align = prev_align_rt;
			}

			end_w = align - x_map;

			if (x_map + end_w > w) end_w = w - x_map;
			
			end_w = i+2*plane;
			for (; i<end_w; i++)
			{
				v3d_affine(point_vec, (i - 1) * d_step + d_step / 2, point12, xyz);
				v3d_norm(xyz);
				risp_idx = x_map+i;
				risp_idx = risp_idx >= w ? risp_idx-w : risp_idx;
				cart_to_sph(xyz[0], xyz[1], xyz[2], &map[risp_idx]);
			}
		}

		map += w;
	}
}

static void erp2isp_triangle_rev_half(int x, int w, int w_tri, int h_tri, \
	int tri_idx, S360_SPH_COORD * map, int pad_sz, int align_sz, int plane)
{
	double  vector_12[3], vector_13[3], point12[3], point13[3];
	double  point_vec[3], xyz[3];
	double  d12, d13, d12_scl, d13_scl, point_d;
	double  off;
	double  d_step;
	double  w_cur, w_start;
	int     v_1_3d, v_2_3d, v_3_3d;
	int     x_map, l;
	int     i, j;

	int     pad_left, pad_rt;
	int     start_w, end_w;
	int     x_start, y_start;
	int     prev_align_left, prev_align_rt;
	int     align;
	int     risp_idx;

	prev_align_left = prev_align_rt = 0;

	if(tri_idx<5) y_start = h_tri;
	else y_start = h_tri*2;

	if(pad_sz != 0)
	{
		pad_left = pad_rt = 1;
	}
	else
	{
		pad_left = pad_rt = 0;
	}
	x_start = x;

	v_1_3d = tbl_vidx_erp2isp[tri_idx + 10][0];
	v_2_3d = tbl_vidx_erp2isp[tri_idx + 10][1];
	v_3_3d = tbl_vidx_erp2isp[tri_idx + 10][2];

	v3d_sub(tbl_tri_xyz[v_3_3d], tbl_tri_xyz[v_1_3d], vector_12);
	d12 = v3d_norm(vector_12);

	v3d_sub(tbl_tri_xyz[v_3_3d], tbl_tri_xyz[v_2_3d], vector_13);
	d13 = v3d_norm(vector_13);

	for (i = 0; i<h_tri; i++)
	{
		w_cur = ((double)(h_tri - i) * w_tri) / h_tri;
		l = CEILING((w_cur - 1) / 2);
		w_start = (x + 0.5 - w_cur / 2);
		off = CEILING(w_start) - w_start;
		x_map = CEILING(w_start + (w_tri >> 1)) - 1;
		d12_scl = d12 * ((double)i / h_tri);
		d13_scl = d13 * ((double)i / h_tri);

		v3d_affine(vector_12, d12_scl, tbl_tri_xyz[v_1_3d], point12);
		v3d_affine(vector_13, d13_scl, tbl_tri_xyz[v_2_3d], point13);

		v3d_sub(point13, point12, point_vec);

		point_d = v3d_norm(point_vec);
		d_step = (w_cur == 0 ? 0 : point_d / w_cur);
		start_w = 0;
		end_w = (l << 1);

		if (pad_left)
		{
			start_w -= pad_sz;

			if(((i+y_start) & ((1<<align_sz)-1)) == 0)
			{
				align = ((start_w + x_map)>>align_sz)<<align_sz;
				prev_align_left = align;
			}
			else
			{
				align = prev_align_left;
			}
			start_w = align - x_map;

			if (tri_idx + 10 != 15)
				start_w = (start_w + x_map < x_start) ? (x_start - x_map) : start_w;
			for (j = start_w; j<0; j++)
			{
				v3d_affine(point_vec, (j - 1) * d_step + d_step / 2, point12, xyz);
				v3d_norm(xyz);
				risp_idx = x_map+j;
				risp_idx = risp_idx >= w ? risp_idx-w : risp_idx;
				cart_to_sph(xyz[0], xyz[1], xyz[2], &map[risp_idx]);
			}

		}

		{
			v3d_affine(point_vec, d_step * off / 2, point12, xyz);
			v3d_norm(xyz);
			risp_idx = x_map;
			risp_idx = risp_idx >= w ? risp_idx-w : risp_idx;
			cart_to_sph(xyz[0], xyz[1], xyz[2], &map[risp_idx]);
			start_w++;
		}

		v3d_affine(point_vec, d_step * off, point12, point12);
		for (j = 1; j<(l << 1); j++)
		{
			v3d_affine(point_vec, (j - 1) * d_step + d_step / 2, point12, xyz);
			v3d_norm(xyz);
			risp_idx = x_map+j;
			risp_idx = risp_idx >= w ? risp_idx-w : risp_idx;
			cart_to_sph(xyz[0], xyz[1], xyz[2], &map[risp_idx]);
		}

		{
			v3d_affine(point_vec, -d_step * off / 2, point13, xyz);
			v3d_norm(xyz);
			risp_idx = x_map+j;
			risp_idx = risp_idx >= w ? risp_idx-w : risp_idx;
			cart_to_sph(xyz[0], xyz[1], xyz[2], &map[risp_idx]);
		}

		if (pad_rt)
		{
			end_w += pad_sz;

			if(((i+y_start) & ((1<<align_sz)-1)) == 0)
			{
				align = ((end_w + x_map)>>align_sz)<<align_sz;
				prev_align_rt= align;
			}
			else
			{
				align = prev_align_rt;
			}

			end_w = align - x_map;

			if (x_map + end_w > w) end_w = w - x_map;
			j++;
			end_w = j+2*plane;
			for (; j<end_w; j++)
			{
				v3d_affine(point_vec, (j - 1) * d_step + d_step / 2, point12, xyz);
				v3d_norm(xyz);
				risp_idx = x_map+j;
				risp_idx = risp_idx >= w ? risp_idx-w : risp_idx;
				cart_to_sph(xyz[0], xyz[1], xyz[2], &map[risp_idx]);
			}
		}

		map += w;
	}
}

static void init_sph_to_risp1_map(S360_SPH_COORD * map, int w_isp, int h_isp, int w_tri, int pad_sz, int plane)
{
	int i, x, y1, y2, h_tri, size;
	S360_SPH_COORD * map1;
	int pad_gap, dist;
	int align_sz;

	pad_gap = pad_sz*(2<<plane);
	align_sz = 0;
	dist = 2>>plane;

	for (i = 0, size = w_isp*h_isp; i<size; i++)
	{
		map[i].lng = -1;
		map[i].lat = -1;
	}

	h_tri = (h_isp / 2);
	y1 = h_tri;
	y2 = (h_tri << 1);

	map1 = map;
	map = map1;
	for (i = 5, x = (w_tri >> 1) + pad_gap*dist; i<9; i++, x += (pad_gap*dist + w_tri))
	{
		erp2isp_triangle_rev(x, w_isp, w_tri, h_tri, i, map, pad_sz,align_sz,1);
	}
	erp2isp_triangle_rev_half(x, w_isp, w_tri, h_tri, i, map, pad_sz,align_sz,dist);

	map = map1;
	for (i = 0, x = (pad_gap/2)*dist; i<5; i++, x += (pad_gap*dist + w_tri))
	{
		erp2isp_triangle(x, w_isp, w_tri, y1, i, map, pad_sz,align_sz,1);
	}
	map += y1 * w_isp;
	for (i = 5, x = (w_tri >> 1) + (pad_gap)*dist; i<9; i++, x += (pad_gap*dist + w_tri))
	{
		erp2isp_triangle(x, w_isp, w_tri, h_tri, i, map, pad_sz,align_sz,1);
	}
	erp2isp_triangle_half(x, w_isp, w_tri, h_tri, i, map, pad_sz,align_sz,dist);
	map = map1;
	map += y1 * w_isp;
	for (i = 0, x = (pad_gap/2)*dist; i<5; i++, x += (pad_gap*dist + w_tri))
	{
		erp2isp_triangle_rev(x, w_isp, w_tri, h_tri, i, map, pad_sz,align_sz,1);
	}
}

/*------------------------------------RISP2---------------------------------------*/
static int map_risp2(S360_MAP * map, int opt)
{
	int w, h, w_tri, h_tri;
	int pad_sz;
	
	w      = map->width;
	h      = map->height;
	w_tri  = GET_W_TRI_ISP(w);
	h_tri  = GET_H_TRI_ISP(w_tri);

	pad_sz = (opt & S360_OPT_PAD)? PAD_SIZE:0;
	init_sph2isp_map(map->layer[0], w, h, w_tri, pad_sz, 0, 2);
	init_sph2isp_map(map->layer[1], (w >> 1), (h>> 1), (w_tri >> 1), pad_sz>>1, 1, 2);

	return S360_OK;
}

/* -----------------------------------ERP TO ISP----------------------------------------*/
static int map_erp_cpp_to_isp(S360_MAP * map, int opt)
{
	int w_dst, h_dst, w_tri, h_tri, pad_sz;
	w_dst = map->width;
	h_dst = map->height;
	w_tri = GET_W_TRI_ISP(w_dst);
	h_tri = GET_H_TRI_ISP(w_tri);

	s360_assert_rv(w_dst == w_tri * 5.5, S360_ERR_INVALID_DIMENSION);
	s360_assert_rv(h_dst == 3 * h_tri, S360_ERR_INVALID_DIMENSION);

	if (opt & S360_OPT_PAD)
	{
		pad_sz = PAD_SIZE;
	}
	else
	{
		pad_sz = 0;
	}

	init_sph2isp_map(map->layer[0], w_dst, h_dst, w_tri, pad_sz, 0, 0);
	init_sph2isp_map(map->layer[1], w_dst >> 1, h_dst >> 1, w_tri >> 1, pad_sz >> 1, 1, 0);

	return S360_OK;
}

static void erp2cmp_squ(int w_dst, int h_dst, int squ_idx, S360_SPH_COORD * map, int w_squ)
{
	double  vector_12[3], vector_13[3];
	double  d12, d13, d12_scl, d13_scl;
	double  xyz[3];
	int     v_1_3d, v_2_3d, v_3_3d, v_4_3d;
	int     i, j, x_init, y_init;

	S360_SPH_COORD  coord;

	v_1_3d = tbl_vidx_erp2cmp[squ_idx][0];
	v_2_3d = tbl_vidx_erp2cmp[squ_idx][1];
	v_3_3d = tbl_vidx_erp2cmp[squ_idx][2];
	v_4_3d = tbl_vidx_erp2cmp[squ_idx][3];

	v3d_sub(tbl_squ_xyz[v_2_3d], tbl_squ_xyz[v_1_3d], vector_12);
	d12 = v3d_norm(vector_12);
	v3d_sub(tbl_squ_xyz[v_3_3d], tbl_squ_xyz[v_1_3d], vector_13);
	d13 = v3d_norm(vector_13);

	cmp_plane_offset(&x_init, &y_init, squ_idx, w_squ);

	map += y_init*w_dst + x_init;

	for (i = 0; i<w_squ; i++)
	{
		for (j = 0; j<w_squ; j++)
		{
			d12_scl = d12 * (i / (double)w_squ);
			d13_scl = d13 * (j / (double)w_squ);
			v3d_affine(vector_12, d12_scl, tbl_squ_xyz[v_1_3d], xyz);
			v3d_affine(vector_13, d13_scl, xyz, xyz);
			v3d_norm(xyz);
			cart_to_sph(xyz[0], xyz[2], xyz[1], &coord);
			map[j] = coord;
		}
		map += w_dst;
	}
}

static void init_sph2cmp(S360_SPH_COORD  * map, int w_dst, int h_dst, int w_squ)
{
	int i;

	for (i = 0; i<6; i++)
	{
		erp2cmp_squ(w_dst, h_dst, i, map, w_squ);
	}
}

/* -----------------------------------ERP TO RISP1----------------------------------------*/
static int map_erp_to_risp1(S360_MAP * map, int opt)
{
	int w_dst, h_dst, w_tri, h_tri, pad_sz;
	w_dst = map->width;
	h_dst = map->height;

	if (opt & S360_OPT_PAD)
	{
		pad_sz = PAD_SIZE_RISP/4;
	}
	else
	{
		pad_sz = 0;
	}
	w_tri = ((w_dst-20*pad_sz)/5);
	h_tri = GET_H_TRI_ISP(w_tri);

	s360_assert_rv((w_dst-20*pad_sz) == w_tri * 5, S360_ERR_INVALID_DIMENSION);
	s360_assert_rv(h_dst == 2 * h_tri, S360_ERR_INVALID_DIMENSION);

	init_sph_to_risp1_map(map->layer[0], w_dst, h_dst, w_tri, pad_sz,0);
	init_sph_to_risp1_map(map->layer[1], w_dst >> 1, h_dst >> 1, w_tri >> 1, pad_sz >> 1,1);

	return S360_OK;
}

/* -----------------------------------ERP TO CMP----------------------------------------*/
static int map_erp_cpp_to_cmp(S360_MAP * map, int opt)
{
	int w_dst, h_dst, w_squ;

	w_dst = map->width;
	h_dst = map->height;
	w_squ = NEAREST_EVEN(w_dst / 4.0);

	init_sph2cmp(map->layer[0], w_dst, h_dst, w_squ);
	init_sph2cmp(map->layer[1], (w_dst >> 1), (h_dst >> 1), (w_squ >> 1));

	return S360_OK;
}

/* -----------------------------------ERP TO TSP----------------------------------------*/
static void rotate_sample(double *v, int pitch, int yaw)
{
	double sy, sp, cy, cp, vx, vy, vz;

	pitch = 90 - pitch;
	sy = sin((double)yaw * PI / 180.0);
	sp = sin((double)pitch * PI / 180.0);
	cy = cos((double)yaw * PI / 180.0);
	cp = cos((double)pitch * PI / 180.0);
	vx = v[0];
	vy = v[1];
	vz = v[2];

	v[0] = vx * cy * cp - vy * cy * sp + vz * sy;
	v[1] = vx * sp + vy * cp;
	v[2] = vx * (-sy) * cp + vy * sy * sp + vz * cy;
}

static void erp2tsp_sample(int squ_idx, S360_SPH_COORD * map, double j, double i, int pitch, int yaw)
{
	double  vector_12[3], vector_13[3];
	double  d12, d13, d12_scl, d13_scl;
	double  xyz[3];
	int     v_1_3d, v_2_3d, v_3_3d, v_4_3d;
	S360_SPH_COORD  coord;

	v_1_3d = tbl_vidx_erp2tsp[squ_idx][0];
	v_2_3d = tbl_vidx_erp2tsp[squ_idx][1];
	v_3_3d = tbl_vidx_erp2tsp[squ_idx][2];
	v_4_3d = tbl_vidx_erp2tsp[squ_idx][3];

	v3d_sub(tbl_squ_xyz[v_2_3d], tbl_squ_xyz[v_1_3d], vector_12);
	d12 = v3d_norm(vector_12);
	v3d_sub(tbl_squ_xyz[v_3_3d], tbl_squ_xyz[v_1_3d], vector_13);
	d13 = v3d_norm(vector_13);
	d12_scl = d12 * i;
	d13_scl = d13 * j;
	v3d_affine(vector_12, d12_scl, tbl_squ_xyz[v_1_3d], xyz);
	v3d_affine(vector_13, d13_scl, xyz, xyz);
	rotate_sample(xyz, pitch, yaw);
	v3d_norm(xyz);
	cart_to_sph(xyz[0], xyz[2], xyz[1], &coord);
	*map = coord;
}

static void init_sph2tsp(S360_SPH_COORD  * map, int w_dst, int h_dst, int w_squ, int pitch, int yaw)
{
	int i;
	int mx, my;
	double x, y, x0, y0;

	for (my = 0; my < w_squ; my++)
	{
		for (mx = 0; mx < 2*w_squ; mx++)
		{
			x = (double)mx / (2.0 * w_squ);
			y = (double)my / w_squ;

			if (0.0 <= x && x < 0.5)
			{
				i = 1;
				x = x/0.5;
			}
			else if (0.6875 <= x && x < 0.8125 && 0.375 <= y && y < 0.625)
			{
				i = 5;
				x = (x - 0.6875)/0.125;
				y = (y - 0.375)/0.25;
			}
			else if (0.5 <= x && x < 0.6875 && 
				((0.0 <= y && y < 0.375 && y >= 2.0*(x - 0.5)) ||
				(0.375 <= y && y < 0.625) ||
				(0.625 <= y && y < 1.0 && y <= 2.0*(1.0 - x) )))
			{
				i = 4;
				x0 = x;
				x = (x - 0.5)/0.1875;
				y = (y - 2.0*x0 + 1.0)/(3.0 - 4.0*x0);
			}
			else if (0.8125 <= x && x < 1.0 && 
				((0.0 <= y && y < 0.375 && x >= (1.0 - y/2.0)) ||
				(0.375 <= y && y < 0.625) ||
				(0.625 <= y && y < 1.0 && y <= (2.0*x - 1.0) )))
			{
				i = 3;
				x0 = x;
				x = (x - 0.8125)/0.1875;
				y = (y + 2.0*x0 - 2.0)/(4.0*x0 - 3.0);
			}
			else if (0.0 <= y && y < 0.375 &&
				((0.5 <= x && x < 0.8125 && y < 2.0*(x - 0.5)) ||
				(0.6875 <= x && x < 0.8125) ||
				(0.8125 <= x && x < 1.0 && x < (1.0 - y/2.0) )))
			{
				i = 0;
				y0 = y;
				y = (0.375 - y)/0.375;
				x = (1.0 - x - 0.5*y0)/(0.5 - y0);
			}
			else
			{
				i = 2;
				y0 = y;
				y = (1.0 - y)/0.375;
				x = (0.5 - x + 0.5*y0)/(y0 - 0.5);
			}
			erp2tsp_sample(i, &(map[mx]), x, y, pitch, yaw);
		}
		map += w_dst;
	}
}

static int map_erp_to_tsp(S360_MAP * map, int opt)
{
	int w_dst, h_dst, w_squ;

	w_dst = map->width;
	h_dst = map->height;
	w_squ = NEAREST_EVEN(w_dst / 2.0);

	init_sph2tsp(map->layer[0], w_dst, h_dst, w_squ, map->pitch, map->yaw);
	init_sph2tsp(map->layer[1], (w_dst >> 1), (h_dst >> 1), (w_squ >> 1), map->pitch, map->yaw);

	return S360_OK;
}

S360_MAP * s360_map_create(int w_src, int h_src, int w_dst, int h_dst, int cfmt, int opt, int pitch, int yaw)
{
	S360_MAP * map = NULL;
	int(*fn_map)(S360_MAP * map, int opt) = NULL;

	if (cfmt == CONV_FMT_ERP_TO_CMP  || cfmt == CONV_FMT_ERP_TO_ISP  || \
		cfmt == CONV_FMT_ISP_TO_RISP || cfmt == CONV_FMT_RISP_TO_ISP || \
		cfmt == CONV_FMT_ERP_TO_TSP  || cfmt == CONV_FMT_ERP_TO_OHP  || \
		cfmt == CONV_FMT_OHP_TO_ROHP || cfmt == CONV_FMT_ROHP_TO_OHP || \
		cfmt == CONV_FMT_CPP_TO_CMP  || cfmt == CONV_FMT_CPP_TO_ISP  || \
		cfmt == CONV_FMT_CPP_TO_OHP  || cfmt == CONV_FMT_ERP_TO_COHP)
	{
		map = (S360_MAP*)s360_malloc(sizeof(S360_MAP));
		s360_assert_rv(map, NULL);
		
		if (cfmt == CONV_FMT_ERP_TO_TSP)
		{
			map->width = w_dst * TSPAA_S;
			map->height = h_dst * TSPAA_S;
			map->pitch = pitch;
			map->yaw = yaw;
		}
		else if (cfmt == CONV_FMT_ISP_TO_RISP || cfmt == CONV_FMT_OHP_TO_ROHP)
		{
			map->width = w_src;
			map->height = h_src;
			map->pitch = 0;
			map->yaw = 0;
		}
		else
		{
			map->width = w_dst;
			map->height = h_dst;
			map->pitch = 0;
			map->yaw = 0;
		}

		map->layer[0] = (S360_SPH_COORD*)s360_malloc((map->width)*\
			(map->height)*sizeof(S360_SPH_COORD));
		map->layer[1] = (S360_SPH_COORD*)s360_malloc((map->width >> 1)*\
			(map->height >> 1)*sizeof(S360_SPH_COORD));

		map_init(map->layer[0], map->width, map->height);
		map_init(map->layer[1], (map->width >> 1), (map->height >> 1));

		if((cfmt == CONV_FMT_ISP_TO_RISP) || (cfmt == CONV_FMT_RISP_TO_ISP))
		{
			opt &= ~(S360_OPT_PAD);
		}

		switch (cfmt)
		{
		case CONV_FMT_ERP_TO_CMP:
		case CONV_FMT_CPP_TO_CMP:
			fn_map = map_erp_cpp_to_cmp;
			break;
		case CONV_FMT_ERP_TO_ISP:
		case CONV_FMT_CPP_TO_ISP:
			fn_map = map_erp_cpp_to_isp;
			break;
		case CONV_FMT_ERP_TO_TSP:
			fn_map = map_erp_to_tsp;
			break;
		case CONV_FMT_ISP_TO_RISP:
		case CONV_FMT_RISP_TO_ISP:
			fn_map = map_risp2;
			break;
		case CONV_FMT_ERP_TO_OHP:
		case CONV_FMT_OHP_TO_ROHP:
		case CONV_FMT_ROHP_TO_OHP:
		case CONV_FMT_CPP_TO_OHP:
			fn_map = map_to_ohp;
			break;
		case CONV_FMT_ERP_TO_RISP1:
			fn_map = map_erp_to_risp1;
			break;
		case CONV_FMT_ERP_TO_COHP:
			fn_map = map_erp_to_cohp;
		break;
		}

		if(fn_map != NULL)
			fn_map(map, opt);
	}

	return map;
}