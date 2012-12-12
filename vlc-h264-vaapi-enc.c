/*****************************************************************************
 * Copyright (C) 2012
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 *****************************************************************************/

/*****************************************************************************
 * Preamble
 *****************************************************************************/

#include <vlc_common.h>
#include <vlc_plugin.h>
#include <vlc_sout.h>
#include <vlc_codec.h>
#include <vlc_charset.h>
#include <vlc_cpu.h>
#include <math.h>

#include <va/va.h>
#include <va/va_x11.h>

static int OpenEncoder( vlc_object_t * );
static void CloseEncoder( vlc_object_t * );

vlc_module_begin ()
	set_shortname( "h264-vaapi-enc" )
	set_description( "h264 encoder using vaapi for hardware accelerated endocing" )
	set_capability( "encoder", 0 )
	set_callbacks( OpenEncoder, CloseEncoder )
	add_shortcut( "h264-vaapi-enc" )
vlc_module_end ()


/*****************************************************************************
 * Local prototypes
 *****************************************************************************/
static block_t *EncodeVideo( encoder_t *p_enc, picture_t *p_pict );

#define SURFACE_NUM 18
#define CODEDBUF_NUM 5
struct encoder_sys_t
{
	VADisplay va_dpy;
	VASurfaceID surface_id[SURFACE_NUM];
	VABufferID coded_buf[CODEDBUF_NUM];
	VAContextID context_id;
	VAConfigID config_id;
	Display *x11_display;
};

/*****************************************************************************
 * OpenDecoder: open the dummy encoder.
 *****************************************************************************/
static int OpenEncoder( vlc_object_t *p_this )
{
	VAEntrypoint entrypoints[5];
	VAStatus va_status;
	VAConfigAttrib attrib[2];
	int major_ver, minor_ver;
	int num_entrypoints, slice_entrypoint;

	encoder_t *p_enc = (encoder_t *)p_this;
	encoder_sys_t *p_sys;

	if(p_enc->fmt_out.i_codec != VLC_CODEC_H264 && !p_enc->b_force)
		return VLC_EGENERIC;
	
	p_enc->fmt_out.i_cat = VIDEO_ES;
	p_enc->fmt_out.i_codec = VLC_CODEC_H264;
	p_enc->p_sys = p_sys = malloc(sizeof(encoder_sys_t));
	if(!p_sys)
		return VLC_ENOMEM;

	p_enc->pf_encode_video = EncodeVideo;
	p_enc->pf_encode_audio = NULL;
	p_enc->fmt_in.i_codec = VLC_CODEC_I420;

	p_sys->x11_display = XOpenDisplay(getenv("DISPLAY"));
	p_sys->va_dpy = vaGetDisplay(p_sys->x11_display);
	va_status = vaInitialize(p_sys->va_dpy, &major_ver, &minor_ver);
	//CHECK_VASTATUS(va_status, "vaInitialize");

	vaQueryConfigEntrypoints(p_sys->va_dpy, VAProfileH264Baseline, entrypoints, &num_entrypoints);

	for(slice_entrypoint = 0; slice_entrypoint < num_entrypoints; slice_entrypoint++)
		if(entrypoints[slice_entrypoint] == VAEntrypointEncSlice)
			break;
	
	if(slice_entrypoint == num_entrypoints)
	{
		//TODO: Close displays, print error
		return VLC_EGENERIC;
	}

	attrib[0].type = VAConfigAttribRTFormat;
	attrib[1].type = VAConfigAttribRateControl;
	vaGetConfigAttributes(p_sys->va_dpy, VAProfileH264Baseline, VAEntrypointEncSlice, &attrib[0], 2);

	if((attrib[0].value & VA_RT_FORMAT_YUV420) == 0)
	{
		//TODO...
		return VLC_EGENERIC;
	}

	if((attrib[1].value & VA_RC_VBR) == 0)
	{
		//TODO...
		return VLC_EGENERIC;
	}

	attrib[0].value = VA_RT_FORMAT_YUV420;
	attrib[1].value = VA_RC_VBR;

	va_status = vaCreateConfig(p_sys->va_dpy, VAProfileH264Baseline, VAEntrypointEncSlice, &attrib[0], 2, &(p_sys->config_id));
	//CHECK_VASTATUS(va_status, "vaCreateConfig");

	va_status = vaCreateSurfaces(p_sys->va_dpy, p_enc->fmt_in.video.i_width, p_enc->fmt_in.video.i_height, VA_RT_FORMAT_YUV420, SURFACE_NUM, &(p_sys->surface_id[0]));
	//CHECK_VASTATUS(va_status, "vaCreateSurfaces");

	va_status = vaCreateContext(p_sys->va_dpy, p_sys->config_id, p_enc->fmt_in.video.i_width, ((p_enc->fmt_in.video.i_height+15)/16)*16, VA_PROGRESSIVE, &(p_sys->surface_id[0]), SURFACE_NUM, &(p_sys->context_id));
	//CHECK_VASTATUS(va_status, "vaCreateContext");

	return VLC_SUCCESS;
}

/****************************************************************************
 * EncodeVideo: the whole thing
 ****************************************************************************/
static block_t *EncodeVideo( encoder_t *p_enc, picture_t *p_pict )
{
	VLC_UNUSED(p_enc); VLC_UNUSED(p_pict);
	return NULL;
}

/*****************************************************************************
 * CloseDecoder: decoder destruction
 *****************************************************************************/
static void CloseEncoder( vlc_object_t *p_this )
{
	VLC_UNUSED(p_this);
}

