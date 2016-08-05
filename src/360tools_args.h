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

#ifndef __360TOOLS_ARGS_H__4324324324324324324324324324242432432453424324__
#define __360TOOLS_ARGS_H__4324324324324324324324324324242432432453424324__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define S360_ARGS_VAL_TYPE_MANDATORY       (1<<0) /* mandatory or not */
#define S360_ARGS_VAL_TYPE_NONE            (0<<1) /* no value */
#define S360_ARGS_VAL_TYPE_INTEGER         (10<<1) /* integer type value */
#define S360_ARGS_VAL_TYPE_STRING          (20<<1) /* string type value */

#define S360_ARGS_GET_CMD_OPT_VAL_TYPE(x)  ((x) & ~S360_ARGS_VAL_TYPE_MANDATORY)
#define S360_CMD_FLAG_CFG                  0     /* Config File Param Index */

typedef struct
{
	char   key; /* option keyword. ex) -f */
	char   key_long[32]; /* option long keyword, ex) --file */
	int    val_type; /* value type */
	int  * flag; /* flag to setting or not */
	void * val; /* actual value */
	char   desc[512]; /* description of option */
} S360_ARGS_OPT;

typedef enum
{
	CMD_FLAG_CONV_CONFIG = S360_CMD_FLAG_CFG,
	CMD_FLAG_CONV_FNAME_INP,
	CMD_FLAG_CONV_FNAME_OUT,
	CMD_FLAG_CONV_WIDTH,
	CMD_FLAG_CONV_HEIGHT,
	CMD_FLAG_CONV_CFMT,
	CMD_FLAG_CONV_OWIDTH,
	CMD_FLAG_CONV_OHEIGHT,
	CMD_FLAG_CONV_FRM_NUM,
	CMD_FLAG_CONV_ALIGN,
	CMD_FLAG_CONV_CS_IN,
	CMD_FLAG_CONV_CS_OUT,
	CMD_FLAG_CONV_CS_INT,
	CMD_FLAG_CONV_MAX
} CMD_FLAG_CONV;

typedef enum
{
	CMD_FLAG_METRIC_CONFIG = S360_CMD_FLAG_CFG,
	CMD_FLAG_METRIC_FNAME_ORG,
	CMD_FLAG_METRIC_FNAME_REC,
	CMD_FLAG_METRIC_FNAME_SPH,
	CMD_FLAG_METRIC_WIDTH_ORG,
	CMD_FLAG_METRIC_HEIGHT_ORG,
	CMD_FLAG_METRIC_WIDTH_REC,
	CMD_FLAG_METRIC_HEIGHT_REC,
	CMD_FLAG_METRIC_FRM_NUM,
	CMD_FLAG_METRIC_QMETRIC,
	CMD_FLAG_METRIC_CS_ORG,
	CMD_FLAG_METRIC_CS_REC,
	CMD_FLAG_METRIC_PFMT_ORG,
	CMD_FLAG_METRIC_PFMT_REC,
	CMD_FLAG_METRIC_VERB,
	CMD_FLAG_METRIC_MAX
} CMD_FLAG_METRIC;


static int s360_args_search_long_arg(S360_ARGS_OPT * opts, const char * argv)
{
	int oidx = 0;
	S360_ARGS_OPT * o;

	o = opts;

	while(o->key != 0)
	{
		if(!strcmp(argv, o->key_long))
		{
			return oidx;
		}
		oidx++;
		o++;
	}
	return -1;
}


static int s360_args_search_short_arg(S360_ARGS_OPT * opts, const char argv)
{
	int oidx = 0;
	S360_ARGS_OPT * o;

	o = opts;

	while(o->key != 0)
	{
		if(o->key == argv)
		{
			return oidx;
		}
		oidx++;
		o++;
	}
	return -1;
}

static int s360_args_read_value(S360_ARGS_OPT * opts, const char * argv)
{
	if(argv == NULL) return -1;
	if(argv[0] == '-') return -1;

	switch(S360_ARGS_GET_CMD_OPT_VAL_TYPE(opts->val_type))
	{
	case S360_ARGS_VAL_TYPE_INTEGER:
		*((int*)opts->val) = atoi(argv);
		break;

	case S360_ARGS_VAL_TYPE_STRING:
		strcpy((char*)opts->val, argv);
		break;

	default:
		return -1;
	}
	return 0;
}

static int s360_args_get_help(S360_ARGS_OPT * opts, int idx, char * help)
{
	int optional = 0;
	char vtype[32];
	S360_ARGS_OPT * o = opts + idx;

	switch(S360_ARGS_GET_CMD_OPT_VAL_TYPE(o->val_type))
	{
	case S360_ARGS_VAL_TYPE_INTEGER:
		strcpy(vtype, "INTEGER");
		break;
	case S360_ARGS_VAL_TYPE_STRING:
		strcpy(vtype, "STRING");
		break;
	case S360_ARGS_VAL_TYPE_NONE:
	default:
		strcpy(vtype, "FLAG");
		break;
	}
	optional = !(o->val_type & S360_ARGS_VAL_TYPE_MANDATORY);

	sprintf(help, "  -%c, --%s [%s]%s\n    : %s", o->key, o->key_long,
		vtype, (optional)? " (optional)":"", o->desc);

	return 0;
}

static int s360_args_get_arg(S360_ARGS_OPT * opts, int idx, char * result)
{
	char vtype[32];
	char value[512];
	S360_ARGS_OPT * o = opts + idx;

	switch(S360_ARGS_GET_CMD_OPT_VAL_TYPE(o->val_type))
	{
	case S360_ARGS_VAL_TYPE_INTEGER:
		strcpy(vtype, "INTEGER");
		sprintf(value, "%d", *((int*)o->val));
		break;

	case S360_ARGS_VAL_TYPE_STRING:
		strcpy(vtype, "STRING");
		sprintf(value, "%s", (char*)o->val);
		break;

	case S360_ARGS_VAL_TYPE_NONE:
	default:
		strcpy(vtype, "FLAG");
		sprintf(value, "%d", *((int*)o->val));
		break;
	}

	if(o->flag != NULL && (*o->flag))
	{
		strcat(value, " (SET)");
	}
	else
	{
		strcat(value, " (DEFAULT)");
	}

	sprintf(result, "  -%c(--%s) = %s\n    : %s", o->key, o->key_long,
		value, o->desc);

	return 0;

}

static int s360_parse_cfg(S360_ARGS_OPT * opts)
{
	char * fname_cfg, * parser;
	FILE * fp_cfg;
	char line[256] = "", tag[50] = "", val[256] = "";
	int oidx;

	fname_cfg = (char*)opts[S360_CMD_FLAG_CFG].val;
	fp_cfg = fopen(fname_cfg, "r");

	while (fgets(line, sizeof(line), fp_cfg))
	{
		parser = strtok(line, "= \t");
		strcpy(tag, parser);

		parser = strtok(NULL, "= \t\n");
		strcpy(val, parser);

		if (strlen(tag) == 1)
		{
			oidx = s360_args_search_short_arg(opts, tag[0]);
			if (oidx < 0) continue;
		}
		else
		{
			oidx = s360_args_search_long_arg(opts, tag);
			if (oidx < 0) continue;
		}

		if (S360_ARGS_GET_CMD_OPT_VAL_TYPE(opts[oidx].val_type) != S360_ARGS_VAL_TYPE_NONE)
		{
			if (s360_args_read_value(opts + oidx, val)) continue;
		}
		else
		{
			*((int*)opts[oidx].val) = 1;
		}
		*opts[oidx].flag = 1;
	}

	fclose(fp_cfg);
	return 0;
}

static int s360_parse_cmd(int argc, const char * argv[], S360_ARGS_OPT * opts,
	int * idx)
{
	int    aidx; /* arg index */
	int    oidx; /* option index */

	aidx = *idx + 1;

	if(aidx >= argc || argv[aidx] == NULL) goto NO_MORE;
	if(argv[aidx][0] != '-') goto ERR;

	if(argv[aidx][1] == '-')
	{
		/* long option */
		oidx = s360_args_search_long_arg(opts, argv[aidx] + 2);
		if(oidx < 0) goto ERR;
	}
	else if(strlen(argv[aidx]) == 2)
	{
		/* short option */
		oidx = s360_args_search_short_arg(opts, argv[aidx][1]);
		if(oidx < 0) goto ERR;
	}
	else
	{
		goto ERR;
	}

	if(S360_ARGS_GET_CMD_OPT_VAL_TYPE(opts[oidx].val_type) != S360_ARGS_VAL_TYPE_NONE)
	{
		if(aidx + 1 >= argc) goto ERR;
		if(s360_args_read_value(opts + oidx, argv[aidx + 1])) goto ERR;
		if (oidx == S360_CMD_FLAG_CFG) { s360_parse_cfg(opts); }
		*idx = *idx + 1;
	}
	else
	{
		*((int*)opts[oidx].val) = 1;
	}
	*opts[oidx].flag = 1;
	*idx = *idx + 1;

	return opts[oidx].key;


NO_MORE:
	return 0;

ERR:
	return -1;
}

static int s360_args_parse_all(int argc, const char * argv[], S360_ARGS_OPT * opts)
{
	int ret = 0, idx = 0;
	S360_ARGS_OPT * o;

	while(1)
	{
		ret = s360_parse_cmd(argc, argv, opts, &idx);
		if(ret <= 0) break;
	}

	/* check mandatory argument */
	o = opts;

	while(o->key != 0)
	{
		if(o->val_type & S360_ARGS_VAL_TYPE_MANDATORY)
		{
			if(*o->flag == 0)
			{
				/* not filled all mandatory argument */
				return o->key;
			}
		}
		o++;
	}
	return ret;
}


static int s360_args_parse_int_x_int(char * str, int * num0, int * num1)
{
	char str0_t[64];
	int i, cnt0, cnt1;
	char * str0, * str1 = NULL;

	str0 = str;
	cnt1 = strlen(str);

	/* find 'x' */
	for(i=0; i<(int)strlen(str); i++)
	{
		if(str[i] == 'x' || str[i] == 'X')
		{
			str1 = str + i + 1;
			cnt0 = i;
			cnt1 = cnt1 - cnt0 - 1;
			break;
		}
	}

	/* check malformed data */
	if(str1 == NULL || cnt0 == 0 || cnt1 == 0) return -1;

	for(i=0; i<cnt0; i++)
	{
		if(str0[i] < 0x30 || str0[i] > 0x39) return -1; /* not a number */
	}
	for(i=0; i<cnt1; i++)
	{
		if(str1[i] < 0x30 || str1[i] > 0x39) return -1; /* not a number */
	}


	strncpy(str0_t, str0, cnt0);
	str0_t[cnt0] = '\0';

	*num0 = atoi(str0_t);
	*num1 = atoi(str1);

	return 0;
}


#ifdef __cplusplus
}
#endif


#endif /*__360TOOLS_ARGS_H__4324324324324324324324324324242432432453424324__ */

