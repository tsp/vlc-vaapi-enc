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
#include <assert.h>
#include <math.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <va/va.h>
#include <va/va_x11.h>
#include <va/va_drm.h>
#include <va/va_enc_h264.h>

#define NAL_REF_IDC_NONE        0
#define NAL_REF_IDC_LOW         1
#define NAL_REF_IDC_MEDIUM      2
#define NAL_REF_IDC_HIGH        3

#define NAL_NON_IDR             1
#define NAL_IDR                 5
#define NAL_SPS                 7
#define NAL_PPS                 8
#define NAL_SEI			        6

#define SLICE_TYPE_P            0
#define SLICE_TYPE_B            1
#define SLICE_TYPE_I            2

#define ENTROPY_MODE_CAVLC      0
#define ENTROPY_MODE_CABAC      1

#define PROFILE_IDC_BASELINE    66
#define PROFILE_IDC_MAIN        77
#define PROFILE_IDC_HIGH        100

static int OpenEncoder( vlc_object_t * );
static void CloseEncoder( vlc_object_t * );

vlc_module_begin ()
	set_shortname( "h264-vaapi" )
	set_description( "H264 VAAPI encoder" )
	set_capability( "encoder", 0 )
	set_callbacks( OpenEncoder, CloseEncoder )
	add_shortcut( "vaapi", "va" )
vlc_module_end ()

#define CHECK_VASTATUS(s,t,e) {if((s)!=VA_STATUS_SUCCESS){msg_Err(p_enc, "vaapi: CHECK_STATUS for %s failed with code %d", t, s); return e;}}

/*****************************************************************************
 * Local prototypes
 *****************************************************************************/
static block_t *EncodeVideo( encoder_t *p_enc, picture_t *p_pict );

#define MAX_SLICES                32
#define SID_INPUT_PICTURE          0
#define SID_REFERENCE_PICTURE      1
#define SID_RECON_PICTURE          2
#define SID_NUMBER                              SID_RECON_PICTURE + 1
struct encoder_sys_t
{
	VADisplay va_dpy;
	Display *x11_display;
	int drm_fd;

	int picture_width;
	int picture_height;
	int picture_width_in_mbs;
	int picture_height_in_mbs;
	int qp_value;
	int frame_bit_rate;

	uint32_t intra_counter;
	uint32_t intra_rate;
	uint32_t first_frame;

	VASurfaceID surface_id[SID_NUMBER];
	VAProfile profile;
	int constraint_set_flag;
	VAEncSequenceParameterBufferH264 seq_param;
	VAEncPictureParameterBufferH264 pic_param;
	VAEncSliceParameterBufferH264 slice_param[MAX_SLICES];
	VAContextID context_id;
	VAConfigID config_id;
	VABufferID seq_param_buf_id;
	VABufferID pic_param_buf_id;
	VABufferID slice_param_buf_id[MAX_SLICES];  /* Slice level parameter, multil slices */
	VABufferID codedbuf_buf_id;                 /* Output buffer, compressed data */
	VABufferID packed_seq_header_param_buf_id;
	VABufferID packed_seq_buf_id;
	VABufferID packed_pic_header_param_buf_id;
	VABufferID packed_pic_buf_id;
	VABufferID packed_sei_header_param_buf_id;   /* the SEI buffer */
	VABufferID packed_sei_buf_id;
	VABufferID misc_parameter_hrd_buf_id;

	int num_slices;
	int codedbuf_i_size;
	int codedbuf_pb_size;
	int current_input_surface;
	int rate_control_method;
	int i_initial_cpb_removal_delay;
	int i_initial_cpb_removal_delay_length;
	int i_cpb_removal_delay;
	int i_cpb_removal_delay_length;
	int i_dpb_output_delay_length;
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
	VAImage test_image;
	int i;

	encoder_t *p_enc = (encoder_t *)p_this;
	encoder_sys_t *p_sys;

	p_enc->pf_encode_video = EncodeVideo;
	p_enc->pf_encode_audio = 0;

	if(p_enc->fmt_out.i_codec != VLC_CODEC_H264 && !p_enc->b_force)
		return VLC_EGENERIC;

	p_enc->fmt_out.i_cat = VIDEO_ES;
	p_enc->fmt_out.i_codec = VLC_CODEC_H264;
	p_enc->p_sys = p_sys = calloc(1, sizeof(encoder_sys_t));
	if(!p_sys)
		return VLC_ENOMEM;

	{ ///START INITIALIZE VARIABLES
		p_sys->picture_width = p_enc->fmt_in.video.i_width;
		p_sys->picture_height = p_enc->fmt_in.video.i_height;
		p_sys->picture_width_in_mbs = (p_sys->picture_width + 15) / 16;
		p_sys->picture_height_in_mbs = (p_sys->picture_height + 15) / 16;
		p_sys->qp_value = 28;
		p_sys->frame_bit_rate = 0;
		p_sys->intra_rate = 30;
		p_sys->intra_counter = 0;
		p_sys->first_frame = 1;

		p_sys->profile = VAProfileH264High;

		switch (p_sys->profile)
		{
			case VAProfileH264Baseline:
				p_sys->constraint_set_flag |= (1 << 0); /* Annex A.2.1 */
				break;
			case VAProfileH264Main:
				p_sys->constraint_set_flag |= (1 << 1); /* Annex A.2.2 */
				break;
			case VAProfileH264High:
				p_sys->constraint_set_flag |= (1 << 3); /* Annex A.2.4 */
				break;
			default:
				break;
		}

		p_sys->seq_param_buf_id = VA_INVALID_ID;
		p_sys->pic_param_buf_id = VA_INVALID_ID;
		p_sys->packed_seq_header_param_buf_id = VA_INVALID_ID;
		p_sys->packed_seq_buf_id = VA_INVALID_ID;
		p_sys->packed_pic_header_param_buf_id = VA_INVALID_ID;
		p_sys->packed_pic_buf_id = VA_INVALID_ID;
		p_sys->codedbuf_buf_id = VA_INVALID_ID;
		p_sys->misc_parameter_hrd_buf_id = VA_INVALID_ID;
		p_sys->codedbuf_i_size = p_sys->picture_width * p_sys->picture_height * 1.5;
		p_sys->codedbuf_pb_size = 0;
		p_sys->current_input_surface = SID_INPUT_PICTURE;
		p_sys->packed_sei_header_param_buf_id = VA_INVALID_ID;
		p_sys->packed_sei_buf_id = VA_INVALID_ID;
		for (i = 0; i < MAX_SLICES; i++)
		{
			p_sys->slice_param_buf_id[i] = VA_INVALID_ID;
		}

		if(p_sys->qp_value == -1)
		{
			p_sys->rate_control_method = VA_RC_CBR;
		}
		else if(p_sys->qp_value == -2)
		{
			p_sys->rate_control_method = VA_RC_VBR;
		}
		else
		{
			assert(p_sys->qp_value >= 0 && p_sys->qp_value <= 51);
			p_sys->rate_control_method = VA_RC_CQP;
		}

		{ // Initialize seq_param
			int frame_cropping_flag = 0;
			int frame_crop_bottom_offset = 0;

			p_sys->seq_param.seq_parameter_set_id = 0;
			p_sys->seq_param.level_idc = 41;
			p_sys->seq_param.intra_period = p_sys->intra_rate;
			p_sys->seq_param.ip_period = 0;   /* FIXME: ??? */
			p_sys->seq_param.max_num_ref_frames = 4;
			p_sys->seq_param.picture_width_in_mbs = p_sys->picture_width_in_mbs;
			p_sys->seq_param.picture_height_in_mbs = p_sys->picture_height_in_mbs;
			p_sys->seq_param.seq_fields.bits.frame_mbs_only_flag = 1;

			if(p_sys->frame_bit_rate > 0)
				p_sys->seq_param.bits_per_second = 1024 * p_sys->frame_bit_rate;
			else
				p_sys->seq_param.bits_per_second = 0;

			p_sys->seq_param.time_scale = 900;
			p_sys->seq_param.num_units_in_tick = 15;

			if(p_sys->picture_height_in_mbs * 16 - p_sys->picture_height)
			{
				frame_cropping_flag = 1;
				frame_crop_bottom_offset =
					(p_sys->picture_height_in_mbs * 16 - p_sys->picture_height) / (2 * (!p_sys->seq_param.seq_fields.bits.frame_mbs_only_flag + 1));
			}

			p_sys->seq_param.frame_cropping_flag = frame_cropping_flag;
			p_sys->seq_param.frame_crop_left_offset = 0;
			p_sys->seq_param.frame_crop_right_offset = 0;
			p_sys->seq_param.frame_crop_top_offset = 0;
			p_sys->seq_param.frame_crop_bottom_offset = frame_crop_bottom_offset;

			p_sys->seq_param.seq_fields.bits.pic_order_cnt_type = 0;
			p_sys->seq_param.seq_fields.bits.direct_8x8_inference_flag = 0;

			p_sys->seq_param.seq_fields.bits.log2_max_frame_num_minus4 = 0;
			p_sys->seq_param.seq_fields.bits.log2_max_pic_order_cnt_lsb_minus4 = 2;

			if(p_sys->frame_bit_rate > 0)
				p_sys->seq_param.vui_parameters_present_flag = 1;
			else
				p_sys->seq_param.vui_parameters_present_flag = 0;
		}

		{ // Initialize pic_param
			p_sys->pic_param.seq_parameter_set_id = 0;
			p_sys->pic_param.pic_parameter_set_id = 0;

			p_sys->pic_param.last_picture = 0;
			p_sys->pic_param.frame_num = 0;

			p_sys->pic_param.pic_init_qp = (p_sys->qp_value >= 0 ?  p_sys->qp_value : 26);
			p_sys->pic_param.num_ref_idx_l0_active_minus1 = 0;
			p_sys->pic_param.num_ref_idx_l1_active_minus1 = 0;

			p_sys->pic_param.pic_fields.bits.idr_pic_flag = 0;
			p_sys->pic_param.pic_fields.bits.reference_pic_flag = 0;
			p_sys->pic_param.pic_fields.bits.entropy_coding_mode_flag = ENTROPY_MODE_CABAC;
			p_sys->pic_param.pic_fields.bits.weighted_pred_flag = 0;
			p_sys->pic_param.pic_fields.bits.weighted_bipred_idc = 0;

			if(p_sys->constraint_set_flag & 0x7)
				p_sys->pic_param.pic_fields.bits.transform_8x8_mode_flag = 0;
			else
				p_sys->pic_param.pic_fields.bits.transform_8x8_mode_flag = 1;

			p_sys->pic_param.pic_fields.bits.deblocking_filter_control_present_flag = 1;
		}

		if(p_sys->rate_control_method == VA_RC_CBR)
		{ // Initialize sei
			int target_bit_rate = p_sys->seq_param.bits_per_second;
			int init_cpb_size = (target_bit_rate * 8) >> 10;
			p_sys->i_initial_cpb_removal_delay = init_cpb_size * 0.5 * 1024 / target_bit_rate * 90000;

			p_sys->i_cpb_removal_delay = 2;
			p_sys->i_initial_cpb_removal_delay_length = 24;
			p_sys->i_cpb_removal_delay_length = 24;
			p_sys->i_dpb_output_delay_length = 24;
		}
	} ///DONE INITIALIZE VARIABLES

	{ ///START OPEN DISPLAY
		p_sys->x11_display = XOpenDisplay(0);
		if(!p_sys->x11_display)
		{
			p_sys->drm_fd = open("/dev/dri/card0", O_RDWR);
			if(p_sys->drm_fd < 0)
			{
				msg_Err(p_enc, "Could not open XDisplay and DRM device!");
				free(p_sys);
				return VLC_EGENERIC;
			}
		}
		else
		{
			p_sys->drm_fd = -1;
		}

		if(p_sys->x11_display)
		{
			msg_Info(p_enc, "Using X11 VA Display");
			p_sys->va_dpy = vaGetDisplay(p_sys->x11_display);
		}
		else if(p_sys->drm_fd >= 0)
		{
			msg_Info(p_enc, "Using DRM VA Display");
			p_sys->va_dpy = vaGetDisplayDRM(p_sys->drm_fd);
		}
		else assert(0);

		if(!p_sys->va_dpy)
		{
			msg_Err(p_enc, "Could not get a VAAPI device");
			if(p_sys->x11_display)
				XCloseDisplay(p_sys->x11_display);
			if(p_sys->drm_fd >= 0)
				close(p_sys->drm_fd);
			free(p_sys);
			return VLC_EGENERIC;
		}
	} ///DONE OPEN DISPLAY

	{ /// START CREATE PIPE
		va_status = vaInitialize(p_sys->va_dpy, &major_ver, &minor_ver);
		CHECK_VASTATUS(va_status, "vaInitialize", VLC_EGENERIC);

		vaQueryConfigEntrypoints(p_sys->va_dpy, p_sys->profile, entrypoints, &num_entrypoints);

		for(slice_entrypoint = 0; slice_entrypoint < num_entrypoints; slice_entrypoint++)
			if(entrypoints[slice_entrypoint] == VAEntrypointEncSlice)
				break;

		if(slice_entrypoint == num_entrypoints)
		{
			vaTerminate(p_sys->va_dpy);
			if(p_sys->x11_display)
				XCloseDisplay(p_sys->x11_display);
			if(p_sys->drm_fd >= 0)
				close(p_sys->drm_fd);
			free(p_sys);
			msg_Err(p_enc, "System does not have h264 encoding support");
			return VLC_EGENERIC;
		}

		attrib[0].type = VAConfigAttribRTFormat;
		attrib[1].type = VAConfigAttribRateControl;
		vaGetConfigAttributes(p_sys->va_dpy, p_sys->profile, VAEntrypointEncSlice, &attrib[0], 2);

		if((attrib[0].value & VA_RT_FORMAT_YUV420) == 0 || (attrib[1].value & p_sys->rate_control_method) == 0)
		{
			vaTerminate(p_sys->va_dpy);
			if(p_sys->x11_display)
				XCloseDisplay(p_sys->x11_display);
			if(p_sys->drm_fd >= 0)
				close(p_sys->drm_fd);
			free(p_sys);
			msg_Err(p_enc, "No YUV420 Format or no desired RC!");
			return VLC_EGENERIC;
		}

		attrib[0].value = VA_RT_FORMAT_YUV420;
		attrib[1].value = p_sys->rate_control_method;

		va_status = vaCreateConfig(p_sys->va_dpy, p_sys->profile, VAEntrypointEncSlice, &attrib[0], 2, &(p_sys->config_id));
		CHECK_VASTATUS(va_status, "vaCreateConfig", VLC_EGENERIC);

		va_status = vaCreateContext(p_sys->va_dpy, p_sys->config_id, p_sys->picture_width, p_sys->picture_height, VA_PROGRESSIVE, 0, 0, &(p_sys->context_id));
		CHECK_VASTATUS(va_status, "vaCreateContext", VLC_EGENERIC);
	} /// DONE CREATING PIPE

	{ /// START ALLOC RESOURCES
		va_status = vaCreateBuffer(p_sys->va_dpy, p_sys->context_id, VAEncSequenceParameterBufferType, sizeof(p_sys->seq_param), 1, &(p_sys->seq_param), &(p_sys->seq_param_buf_id));
		CHECK_VASTATUS(va_status, "vaCreateBuffer", VLC_EGENERIC);

		va_status = vaCreateSurfaces(
			p_sys->va_dpy,
			VA_RT_FORMAT_YUV420, p_sys->picture_width, p_sys->picture_height,
			&(p_sys->surface_id[0]), SID_NUMBER,
			NULL, 0
		);
		CHECK_VASTATUS(va_status, "vaCreateSurfaces", VLC_EGENERIC);

		va_status = vaCreateBuffer(p_sys->va_dpy, p_sys->context_id, VAEncCodedBufferType, p_sys->codedbuf_i_size, 1, NULL, &(p_sys->codedbuf_buf_id));
		CHECK_VASTATUS(va_status, "vaCreateBuffer", VLC_EGENERIC);

		p_sys->pic_param.coded_buf = p_sys->codedbuf_buf_id;
	} /// DONE ALLOC RESOURCES

	{ /// START PROBE IMAGE FORMAT
		va_status = vaDeriveImage(p_sys->va_dpy, p_sys->surface_id[SID_INPUT_PICTURE], &test_image);
		CHECK_VASTATUS(va_status, "vaDeriveImage", false);

		p_enc->fmt_in.i_codec = test_image.format.fourcc;

		va_status = vaDestroyImage(p_sys->va_dpy, test_image.image_id);
		CHECK_VASTATUS(va_status, "vaDestroyImage", false);
	} /// DONE PROBE IMAGE FORMAT

	return VLC_SUCCESS;
}

/*****************************************************************************
 * CloseDecoder: decoder destruction
 *****************************************************************************/

static void CloseEncoder( vlc_object_t *p_this )
{
	encoder_t *p_enc = (encoder_t *)p_this;
	encoder_sys_t *p_sys = p_enc->p_sys;

	vaDestroyBuffer(p_sys->va_dpy, p_sys->codedbuf_buf_id);
	vaDestroySurfaces(p_sys->va_dpy, &(p_sys->surface_id[0]), SID_NUMBER);
	vaDestroyBuffer(p_sys->va_dpy, p_sys->seq_param_buf_id);
	vaDestroyContext(p_sys->va_dpy, p_sys->context_id);
	vaDestroyConfig(p_sys->va_dpy, p_sys->config_id);
	vaTerminate(p_sys->va_dpy);
	if(p_sys->x11_display)
		XCloseDisplay(p_sys->x11_display);
	if(p_sys->drm_fd >= 0)
		close(p_sys->drm_fd);
	free(p_sys);
}

/****************************************************************************
 * EncodeVideo: the whole thing
 ****************************************************************************/

block_t *GenCodedBlock(encoder_t *p_enc, int is_intra, mtime_t date)
{
	encoder_sys_t *p_sys = p_enc->p_sys;

	VACodedBufferSegment *buf_list = 0;
	VAStatus va_status;
	VASurfaceStatus surface_status = 0;

	va_status = vaSyncSurface(p_sys->va_dpy, p_sys->surface_ids[SID_INPUT_PICTURE]);
	CHECK_VASTATUS(va_status, "vaSyncSurface", 0);
	
	va_status = vaQuerySurfaceStatus(p_sys->va_dpy, p_sys->surface_id[SID_INPUT_PICTURE], &surface_status);
	CHECK_VASTATUS(va_status, "vaQuerySurfaceStatus", 0);

	va_status = vaMapBuffer(p_sys->va_dpy, p_sys->codedbuf_buf_id, (void**)(&buf_list));
	CHECK_VASTATUS(va_status, "vaMapBuffer", 0);

	block_t *block = 0;
	block_t *chain = 0;

	while(buf_list != 0)
	{
		block = block_Alloc(buf_list->size);
		block_ChainAppend(&chain, block);

		memcpy(block->p_buffer, buf_list->buf, buf_list->size);
		msg_Dbg(p_enc, "Added Block with size %d to chain", buf_list->size);

		buf_list = (VACodedBufferSegment*)buf_list->next;
	}

	vaUnmapBuffer(p_sys->va_dpy, p_sys->codedbuf_buf_id);
	msg_Dbg(p_enc, "Done Writing output");

	return chain;
}
 
static block_t *EncodeVideo(encoder_t *p_enc, picture_t *p_pict)
{
	VABufferID tempID;
	VAStatus va_status;
	VACodedBufferSegment *coded_buffer_segment = 0;
	int is_intra = 0;

	encoder_sys_t *p_sys = p_enc->p_sys;

	if(p_sys->intra_counter % p_sys->intra_rate == 0)
	{
		p_sys->intra_counter = 0;
		is_intra = 1;
	}
	p_sys->intra_counter += 1;

	va_status = vaBeginPicture(p_sys->va_dpy, p_sys->context_id, p_sys->surface_id[SID_INPUT_PICTURE]);
	CHECK_VASTATUS(va_status, "vaBeginPicture", 0);

	va_status = vaRenderPicture(p_sys->va_dpy, p_sys->context_id, &(p_sys->seq_param_buf_id), 1);
	CHECK_VASTATUS(va_status, "vaRenderPicture", 0);

	//TODO: Upload YUV

	if(p_sys->pic_param_buf_id != VA_INVALID_ID)
		vaDestroyBuffer(p_sys->va_dpy, p_sys->pic_param_buf_id);
	p_sys->pic_param.ReferenceFrames[0].picture_id = p_sys->surface_id[SID_REFERENCE_PICTURE];
	p_sys->pic_param.ReferenceFrames[1].picture_id = VA_INVALID_ID;
	p_sys->pic_param.CurrPic.picture_id = p_sys->surface_id[SID_RECON_PICTURE];
	va_status = vaCreateBuffer(p_sys->va_dpy, p_sys->context_id, VAEncPictureParameterBufferType, sizeof(p_sys->pic_param), 1, &p_sys->pic_param, &p_sys->pic_param_buf_id);
	CHECK_VASTATUS(va_status,"vaCreateBuffer", 0);
	va_status = vaRenderPicture(p_sys->va_dpy, p_sys->context_id, &p_sys->pic_param_buf_id, 1);
	CHECK_VASTATUS(va_status,"vaRenderPicture", 0);

	va_status = vaMapBuffer(p_sys->va_dpy, p_sys->codedbuf_buf_id, (void **)(&coded_buffer_segment));
	CHECK_VASTATUS(va_status, "vaMapBuffer", 0);
	memset(coded_buffer_segment->buf, 0, coded_buffer_segment->size);
	vaUnmapBuffer(p_sys->va_dpy, p_sys->codedbuf_buf_id);

	p_sys->slice_param[0].start_row_number = 0;
	p_sys->slice_param[0].slice_height = p_sys->picture_height/16;
	p_sys->slice_param[0].slice_flags.bits.is_intra = is_intra;
	p_sys->slice_param[0].slice_flags.bits.disable_deblocking_filter_idc = 0;
	if(p_sys->slice_param_buf_id[0] != VA_INVALID_ID)
		vaDestroyBuffer(p_sys->va_dpy, p_sys->slice_param_buf_id[0]);
	va_status = vaCreateBuffer(p_sys->va_dpy, p_sys->context_id, VAEncSliceParameterBufferType, sizeof(p_sys->slice_param[0]), 1, &(p_sys->slice_param[0]), &(p_sys->slice_param_buf_id[0]));
	CHECK_VASTATUS(va_status, "vaCreateBuffer", 0);
	va_status = vaRenderPicture(p_sys->va_dpy, p_sys->context_id, &(p_sys->slice_param_buf_id[0]), 1);
	CHECK_VASTATUS(va_status,"vaRenderPicture", 0);

	tempID = p_sys->surface_id[SID_RECON_PICTURE];
	p_sys->surface_id[SID_RECON_PICTURE] = p_sys->surface_id[SID_REFERENCE_PICTURE];
	p_sys->surface_id[SID_REFERENCE_PICTURE] = tempID;

	va_status = vaEndPicture(p_sys->va_dpy , p_sys->context_id);
	CHECK_VASTATUS(va_status, "vaEndPicture", 0);

	return GenCodedBlock(p_enc, is_intra, p_pict->date);
}
