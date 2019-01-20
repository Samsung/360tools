# 360tools  [![Travis Build Status](https://travis-ci.org/Samsung/360tools.svg?branch=master)](https://travis-ci.org/Samsung/360tools)

	Projection and quality evaluation tools for VR video compression exploration experiments

## Description
	+ 360tools_conv is used for conversion between different projection formats
	+ 360tools_metric implements various quality metrics for vR video quality evaluation
	
	The 360tools is software for VR video processing experiments.
	The software includes both 360 video conversion and quality metrics
	functionality
	
	360tools software is useful in aiding users of a 360 VR video coding
	experiments to establish and test various processing techniques for VR video
	content.
	360tools is useful to educate users. For these purposes, the software is
	provided as an aid for the study and implementation of 360 VR video coding and
	may eventually be formally published as reference software, e.g., by
	ITU-T and ISO/IEC.
	
	This software may be subject to other third party and contributor rights,
	including patent rights and no such rights are granted.

## Building

Checkout the source for 360 Tools

	+ Windows
	In 360tools/build/x86_windows/
	select corresponding *.sln file
	build and run with preferred visual studio 2008 or 2010
	Executable files are stored to 360tools/bin/ directory
	
	+ Linux
	In 360tools/build/x86_linux/
	execute make command 
	Executable files are stored at 360tools/bin/ directory

## Example usage for conversion tool

	./360tools_conv -i [file] -o [file] -f [int] -w [int] -h [int] -l [int] -m [int] -x [int] -y [int]
	./360tools_conv -i glacier_vr_24p_3840x1920.yuv -w 3840 -h 1920 -x 1 -o glacier_vr_24p_isp_4268x2016.yuv -l 4268 -m 2016 -y 1 -f 5 -n 10
	
## Example usage for quality metrics

	./360tools_metrics -w [int] -h [int] -f [int] -o [file] -q [int] -l [int] -m [int] -t [int] -r [file] -n [int] -x [int]
	./360tools_metrics -w 4096 -h 2048 -f 1 -o glacier_vr_24p_3840x1920.yuv -q 4 -l 4268 -m 2016 -t 2 -r glacier_vr_24p_isp_4268x2016.yuv -n 7 -x 1 
	
## Supported formats
	+	Equirectangular projection (ERP)
	+	Icosahedral projection (ISP)
	+	Octahedron projection (OHP)
	+	Cubemap projection (CMP)
	+	Truncated Square Pyramid projection (TSP)
	+	Segmented Sphere Projection (SSP)
	+	Reshaped Icosahedral projection (RISP)
	+	Reshaped Octahedron projection (ROHP)
	+	Reshaped Cubemap projection (RCMP)
	
## Supported quality metrics
	+	PSNR - conventional Peak Signal to Noise Ratio quality metrics
	+	S-PSNR - spherical PSNR (requires sphere_655362.txt file with point coordinates)
	+	WS-PSNR - weighted Spherical PSNR (for equirectangular projection only)
	+	CPP-PSNR - equal area common projection PSNR
	
## Conversion parameters
	+	Mandatory
		-i	input file name
		-o	output file name
		-w	input image width
		-h	input image height
		-l	output image width
		-m	output image height
		-x	input image colorspace
		-y	output image colorspace
		-f	conversion format
		
	+	Optional
		-n	number of converted frames
		-a	align to multiple of size
		-u	disable padding
		-c	config file
		-p	pitch angle
		-y	yaw angle
		
## Quality metrics parameters
	+	Mandatory
		-o	original file
		-r	reconstructed file
		-w	width original
		-h	height original
		-q	metrics type
		-x	original image colorspace
		-y	reconstructed image colorspace
		
	+	Optinal
		-l	width reconstructed
		-m	height reconstructed
		-n	number of frames
		-s	spheric coordinates file, mandatory for S-PSNR
		-f	projection format original, mandatory for CPP-PSNR
		-t	projection format reconstructed for CPP-PSNR
		-c	config file
		
## Recommended equal spatial resolution: number of pixels in each projection is constant 
	(S_ERP == S_ISP == S_CPP); Value is suggested for HM and JEM sw
	+	ERP 		3840x1920
		- ISP		4928x2328
		- OHP		5984x2592
		- CMP		3840x2880
		- TSP		1920x960
		- SSP		3840x2880
		
	+	ERP 		4096x2048
		- ISP		4928x2328
		- OHP		6208x2688
		- CMP		4096x3072
		- TSP		2048x1024
		- SSP		4096x3072
