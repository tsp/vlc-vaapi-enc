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
	add_shortcut( "h264-vaapi" )
vlc_module_end ()

#define CHECK_VASTATUS(s,t,e) {if((s)!=VA_STATUS_SUCCESS){msg_Err(p_enc, "vaapi: CHECK_STATUS for %s failed with code %d", t, s); return e;}}

/*****************************************************************************
 * Local prototypes
 *****************************************************************************/
static block_t *EncodeVideo( encoder_t *p_enc, picture_t *p_pict );

#define SID_NUMBER                              3
#define SID_INPUT_PICTURE                       0
#define SID_REFERENCE_PICTURE                   1
#define SID_RECON_PICTURE                       2
struct encoder_sys_t
{
	VADisplay va_dpy;
	VAContextID context_id;
	VAConfigID config_id;
	Display *x11_display;

	VASurfaceID surface_id[SID_NUMBER];
	VABufferID seq_parameter;
	VABufferID pic_parameter;
	VABufferID slice_parameter;
	VABufferID coded_buf;
	int codedbuf_size;

	VAEncPictureParameterBufferH264 pic_h264;
    VAEncSliceParameterBuffer slice_h264;
	
	uint32_t intra_counter;
	uint32_t intra_rate;
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
	VAEncSequenceParameterBufferH264 seq_h264;

	encoder_t *p_enc = (encoder_t *)p_this;
	encoder_sys_t *p_sys;

	if(p_enc->fmt_out.i_codec != VLC_CODEC_H264 && !p_enc->b_force)
		return VLC_EGENERIC;

	p_enc->fmt_out.i_cat = VIDEO_ES;
	p_enc->fmt_out.i_codec = VLC_CODEC_H264;
	p_enc->p_sys = p_sys = malloc(sizeof(encoder_sys_t));
	if(!p_sys)
		return VLC_ENOMEM;

	memset(&(p_sys->pic_h264), 0, sizeof(p_sys->pic_h264));
	memset(&(p_sys->slice_h264), 0, sizeof(p_sys->slice_h264));

	p_enc->pf_encode_video = EncodeVideo;
	p_enc->pf_encode_audio = 0;
	p_enc->fmt_in.i_codec = VLC_CODEC_I420;

	p_sys->x11_display = XOpenDisplay(0);
	if(!p_sys->x11_display)
	{
		msg_Err(p_enc, "Could not connect to X");
		return VLC_EGENERIC;
	}

	p_sys->va_dpy = vaGetDisplay(p_sys->x11_display);
	if(!p_sys->va_dpy)
	{
		msg_Err(p_enc, "Could not get a VAAPI device");
		XCloseDisplay(p_sys->x11_display);
		return VLC_EGENERIC;
	}

	va_status = vaInitialize(p_sys->va_dpy, &major_ver, &minor_ver);
	CHECK_VASTATUS(va_status, "vaInitialize", VLC_EGENERIC);

	vaQueryConfigEntrypoints(p_sys->va_dpy, VAProfileH264High, entrypoints, &num_entrypoints);

	for(slice_entrypoint = 0; slice_entrypoint < num_entrypoints; slice_entrypoint++)
		if(entrypoints[slice_entrypoint] == VAEntrypointEncSlice)
			break;

	if(slice_entrypoint == num_entrypoints)
	{
		vaTerminate(p_sys->va_dpy);
		XCloseDisplay(p_sys->x11_display);
		msg_Err(p_enc, "System does not have h264 encoding support");
		return VLC_EGENERIC;
	}

	attrib[0].type = VAConfigAttribRTFormat;
	attrib[1].type = VAConfigAttribRateControl;
	vaGetConfigAttributes(p_sys->va_dpy, VAProfileH264High, VAEntrypointEncSlice, &attrib[0], 2);

	if((attrib[0].value & VA_RT_FORMAT_YUV420) == 0)
	{
		vaTerminate(p_sys->va_dpy);
		XCloseDisplay(p_sys->x11_display);
		msg_Err(p_enc, "No YUV420 Format!");
		return VLC_EGENERIC;
	}

	if((attrib[1].value & VA_RC_VBR) == 0)
	{
		vaTerminate(p_sys->va_dpy);
		XCloseDisplay(p_sys->x11_display);
		msg_Err(p_enc, "No VBR RC!");
		return VLC_EGENERIC;
	}

	attrib[0].value = VA_RT_FORMAT_YUV420;
	attrib[1].value = VA_RC_VBR;

	va_status = vaCreateConfig(p_sys->va_dpy, VAProfileH264Main, VAEntrypointEncSlice, &attrib[0], 2, &(p_sys->config_id));
	CHECK_VASTATUS(va_status, "vaCreateConfig", VLC_EGENERIC);

	va_status = vaCreateContext(p_sys->va_dpy, p_sys->config_id, p_enc->fmt_in.video.i_width, p_enc->fmt_in.video.i_height, VA_PROGRESSIVE, 0, 0, &(p_sys->context_id));
	CHECK_VASTATUS(va_status, "vaCreateContext", VLC_EGENERIC);

	p_sys->seq_parameter = VA_INVALID_ID;
	p_sys->pic_parameter = VA_INVALID_ID;
	p_sys->slice_parameter = VA_INVALID_ID;

	memset(&seq_h264, 0, sizeof(seq_h264));
	seq_h264.level_idc = 30;
	seq_h264.picture_width_in_mbs = (p_enc->fmt_in.video.i_width+15)/16;
	seq_h264.picture_height_in_mbs = (p_enc->fmt_in.video.i_height+15)/16;

	p_sys->intra_rate = 30;
	seq_h264.intra_idr_period = p_sys->intra_rate;
	seq_h264.frame_rate = p_enc->fmt_in.video.i_frame_rate; //TODO: Make configurable
	seq_h264.bits_per_second = 1500*1000; //TODO: Make configurable, currently fixed to 1500 kbps
	seq_h264.initial_qp = 26; //TODO: Make configurable
	seq_h264.min_qp = 3; //TODO: ...

	va_status = vaCreateBuffer(p_sys->va_dpy, p_sys->context_id, VAEncSequenceParameterBufferType, sizeof(seq_h264), 1, &seq_h264, &(p_sys->seq_parameter));
	CHECK_VASTATUS(va_status,"vaCreateBuffer", VLC_EGENERIC);

	va_status = vaCreateSurfaces(p_sys->va_dpy, p_enc->fmt_in.video.i_width, p_enc->fmt_in.video.i_height, VA_RT_FORMAT_YUV420, SID_NUMBER, &(p_sys->surface_id[0]));
	CHECK_VASTATUS(va_status, "vaCreateSurfaces", VLC_EGENERIC);

	p_sys->codedbuf_size =  p_enc->fmt_in.video.i_width * p_enc->fmt_in.video.i_height * 1.5;
	va_status = vaCreateBuffer(p_sys->va_dpy, p_sys->context_id, VAEncCodedBufferType, p_sys->codedbuf_size, 1, NULL, &(p_sys->coded_buf));
	CHECK_VASTATUS(va_status,"vaCreateBuffer", VLC_EGENERIC);

	p_sys->intra_counter = 0;
	
	return VLC_SUCCESS;
}

/****************************************************************************
 * EncodeVideo: the whole thing
 ****************************************************************************/
void CopyPlane(uint8_t *dst, size_t dst_pitch, const uint8_t *src, size_t src_pitch, uint32_t width, uint32_t height)
{
	for(uint32_t y = 0; y < height; y++)
	{
		memcpy(dst, src, width);
		src += src_pitch;
		dst += dst_pitch;
	}
}

void CopyToYv12(picture_t *src, uint8_t *dst[3], size_t dst_pitch[3], uint32_t width, uint32_t height)
{
	CopyPlane(dst[0], dst_pitch[0], src->p[0].p_pixels, src->p[0].i_pitch, width, height);
	CopyPlane(dst[1], dst_pitch[1], src->p[1].p_pixels, src->p[1].i_pitch, width/2, height/2);
	CopyPlane(dst[2], dst_pitch[2], src->p[2].p_pixels, src->p[2].i_pitch, width/2, height/2);
}

void RevSplitPlanes(const uint8_t *srcu, size_t srcu_pitch, uint8_t *srcv, size_t srcv_pitch, uint8_t *dst, size_t dst_pitch, uint32_t width, uint32_t height)
{
	for(uint32_t y = 0; y < height; y++)
	{
		for(uint32_t x = 0; x < width; x++)
		{
			dst[2*x+0] = srcu[x];
			dst[2*x+1] = srcv[x];
		}
		dst += dst_pitch;
		srcu += srcu_pitch;
		srcv += srcv_pitch;
	}
}

void CopyToNv12(picture_t *src, uint8_t *dst[2], size_t dst_pitch[2], uint32_t width, uint32_t height)
{
	CopyPlane(dst[0], dst_pitch[0], src->p[0].p_pixels, src->p[0].i_pitch, width, height);
	RevSplitPlanes(src->p[2].p_pixels, src->p[2].i_pitch, src->p[1].p_pixels, src->p[1].i_pitch, dst[1], dst_pitch[1], width/2, height/2);
}

bool UploadPictureToSurface(encoder_t *p_enc, picture_t *p_pict, VASurfaceID surface_id)
{
	encoder_sys_t *p_sys = p_enc->p_sys;

	VAImage surface_image;
	VAStatus va_status = vaDeriveImage(p_sys->va_dpy, surface_id, &surface_image);
	CHECK_VASTATUS(va_status, "vaDeriveImage", false);

	void *surface_p = 0;
	va_status = vaMapBuffer(p_sys->va_dpy, surface_image.buf, &surface_p);
	CHECK_VASTATUS(va_status, "vaMapBuffer", false);

	if(surface_image.format.fourcc == VA_FOURCC('Y','V','1','2')
		|| surface_image.format.fourcc == VA_FOURCC('I','4','2','0'))
	{
		bool b_swap_uv = (surface_image.format.fourcc == VA_FOURCC('I','4','2','0'));
		uint8_t *pp_plane[3];
		size_t  pi_pitch[3];

		for(int i = 0; i < 3; i++)
		{
			int i_plane = (b_swap_uv && i != 0) ?  (3 - i) : i;
			pp_plane[i] = (uint8_t*)surface_p + surface_image.offsets[i_plane];
			pi_pitch[i] = surface_image.pitches[i_plane];
		}

		CopyToYv12(p_pict, pp_plane, pi_pitch, p_enc->fmt_in.video.i_width, p_enc->fmt_in.video.i_height);
	}
	else if(surface_image.format.fourcc == VA_FOURCC('N','V','1','2'))
	{
		uint8_t *pp_plane[2];
		size_t pi_pitch[2];

		for(int i = 0; i < 2; i++)
		{
			pp_plane[i] = (uint8_t*)surface_p + surface_image.offsets[i];
			pi_pitch[i] = surface_image.pitches[i];
		}

		CopyToNv12(p_pict, pp_plane, pi_pitch, p_enc->fmt_in.video.i_width, p_enc->fmt_in.video.i_height);
	}
	else
	{
		msg_Err(p_enc, "Unsupported Image Format!");
		vaUnmapBuffer(p_sys->va_dpy, surface_image.buf);
		vaDestroyImage(p_sys->va_dpy, surface_image.image_id);
		return false;
	}

	vaUnmapBuffer(p_sys->va_dpy, surface_image.buf);
	vaDestroyImage(p_sys->va_dpy, surface_image.image_id);

	return true;
}

block_t *GenCodedBlock(encoder_t *p_enc)
{
	encoder_sys_t *p_sys = p_enc->p_sys;
	
	VACodedBufferSegment *buf_list = 0;
	VAStatus va_status;
	
	va_status = vaMapBuffer(p_sys->va_dpy, p_sys->coded_buf, (void**)(&buf_list));
	CHECK_VASTATUS(va_status, "vaMapBuffer", 0);

	block_t *block = 0;
	block_t *chain = 0;
	
	while(buf_list != 0)
	{
		block = block_Alloc(buf_list->size);
		block_ChainAppend(&chain, block);
		
		memcpy(block->p_buffer, buf_list->buf, buf_list->size);
		
		buf_list = (VACodedBufferSegment*)buf_list->next;
	}
	
	vaUnmapBuffer(p_sys->va_dpy, p_sys->coded_buf);
	
	return chain;
}

static block_t *EncodeVideo( encoder_t *p_enc, picture_t *p_pict )
{
	encoder_sys_t *p_sys = p_enc->p_sys;
	VAStatus va_status;
	VACodedBufferSegment *coded_buffer_segment = NULL;
	unsigned char *coded_mem;
	VABufferID tempID;

	va_status = vaBeginPicture(p_sys->va_dpy, p_sys->context_id, p_sys->surface_id[SID_INPUT_PICTURE]);
	CHECK_VASTATUS(va_status, "vaBeginPicture", 0);

	va_status = vaRenderPicture(p_sys->va_dpy, p_sys->context_id, &(p_sys->seq_parameter), 1);
    CHECK_VASTATUS(va_status, "vaRenderPicture", 0);

	if(!UploadPictureToSurface(p_enc, p_pict, p_sys->surface_id[SID_INPUT_PICTURE]))
	{
		msg_Err(p_enc, "Error uploading Image");
		return 0;
	}

	p_sys->pic_h264.reference_picture = p_sys->surface_id[SID_REFERENCE_PICTURE];
    p_sys->pic_h264.reconstructed_picture = p_sys->surface_id[SID_RECON_PICTURE];
    p_sys->pic_h264.coded_buf = p_sys->coded_buf;
    p_sys->pic_h264.picture_width = p_enc->fmt_in.video.i_width;
    p_sys->pic_h264.picture_height = p_enc->fmt_in.video.i_height;
    p_sys->pic_h264.last_picture = 0;
	if(p_sys->pic_parameter != VA_INVALID_ID)
		vaDestroyBuffer(p_sys->va_dpy, p_sys->pic_parameter);

	va_status = vaCreateBuffer(p_sys->va_dpy, p_sys->context_id, VAEncPictureParameterBufferType, sizeof(p_sys->pic_h264), 1, &(p_sys->pic_h264), &(p_sys->pic_parameter));
	CHECK_VASTATUS(va_status, "vaCreateBuffer", 0);

	va_status = vaRenderPicture(p_sys->va_dpy, p_sys->context_id, &(p_sys->pic_parameter), 1);
    CHECK_VASTATUS(va_status, "vaRenderPicture", 0);

	va_status = vaMapBuffer(p_sys->va_dpy, p_sys->coded_buf, (void **)(&coded_buffer_segment));
    CHECK_VASTATUS(va_status, "vaMapBuffer", 0);
    coded_mem = coded_buffer_segment->buf;
    memset(coded_mem, 0, coded_buffer_segment->size);
    vaUnmapBuffer(p_sys->va_dpy, p_sys->coded_buf);

	p_sys->slice_h264.start_row_number = 0;
    p_sys->slice_h264.slice_height = (p_enc->fmt_in.video.i_height + 15)/16;
    p_sys->slice_h264.slice_flags.bits.is_intra = (p_sys->intra_counter == 0); //TODO: Find out about the intra/idr frame handling
    p_sys->slice_h264.slice_flags.bits.disable_deblocking_filter_idc = 0;
	if(p_sys->slice_parameter != VA_INVALID_ID)
		vaDestroyBuffer(p_sys->va_dpy, p_sys->slice_parameter);

	va_status = vaCreateBuffer(p_sys->va_dpy, p_sys->context_id, VAEncSliceParameterBufferType, sizeof(p_sys->slice_h264), 1, &(p_sys->slice_h264), &(p_sys->slice_parameter));
	CHECK_VASTATUS(va_status, "vaCreateBuffer", 0);

	va_status = vaRenderPicture(p_sys->va_dpy, p_sys->context_id, &(p_sys->slice_parameter), 1);
	CHECK_VASTATUS(va_status, "vaRenderPicture", 0);

	tempID = p_sys->surface_id[SID_RECON_PICTURE];
	p_sys->surface_id[SID_RECON_PICTURE] = p_sys->surface_id[SID_REFERENCE_PICTURE];
	p_sys->surface_id[SID_REFERENCE_PICTURE] = tempID;

	va_status = vaEndPicture(p_sys->va_dpy, p_sys->context_id);
	CHECK_VASTATUS(va_status, "vaRenderPicture", 0);

	//TODO: Somehow get the rendered image into a format VLC understands and return it.

	p_sys->intra_counter += 1;
	if(p_sys->intra_counter >= p_sys->intra_rate)
		p_sys->intra_counter = 0;
	
	return GenCodedBlock(p_enc);
}

/*****************************************************************************
 * CloseDecoder: decoder destruction
 *****************************************************************************/
static void CloseEncoder( vlc_object_t *p_this )
{
	encoder_t *p_enc = (encoder_t *)p_this;
	encoder_sys_t *p_sys = p_enc->p_sys;

	vaDestroyBuffer(p_sys->va_dpy, p_sys->coded_buf);
	vaDestroySurfaces(p_sys->va_dpy, &(p_sys->surface_id[0]), SID_NUMBER);
	vaDestroyBuffer(p_sys->va_dpy, p_sys->seq_parameter);
	vaDestroyContext(p_sys->va_dpy, p_sys->context_id);
	vaDestroyConfig(p_sys->va_dpy, p_sys->config_id);
	vaTerminate(p_sys->va_dpy);
	XCloseDisplay(p_sys->x11_display);
	free(p_sys);
}

