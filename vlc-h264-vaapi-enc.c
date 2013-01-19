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
#define NAL_SEI                 6

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

struct picture_stack
{
    picture_t *pic;
    struct picture_stack *next;
};

#define MAX_SLICES                32
#define SID_INPUT_PICTURE          0
#define SID_REFERENCE_PICTURE_1    1
#define SID_REFERENCE_PICTURE_2    2
#define SID_RECON_PICTURE          3
#define SID_NUMBER                              SID_RECON_PICTURE + 1
struct encoder_sys_t
{
    VADisplay va_dpy;
    Display *x11_display;
    int drm_fd;

    mtime_t pts;
    mtime_t initial_date;

    int picture_width;
    int picture_height;
    int picture_width_in_mbs;
    int picture_height_in_mbs;
    int qp_value;
    int frame_bit_rate;

    uint64_t intra_counter;
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

    struct picture_stack *pic_stack_head;
    uint32_t pic_stack_count;
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

    p_sys->pic_stack_head = 0;
    p_sys->pic_stack_count = 0;

    { ///START INITIALIZE VARIABLES
        p_sys->qp_value = 28; ///TODO: Make this three configurable
        p_sys->frame_bit_rate = 3000;
        p_sys->intra_rate = 30;  
 
        if(p_sys->frame_bit_rate > 0)
            p_sys->qp_value = -1;
 
        p_sys->picture_width = p_enc->fmt_in.video.i_width;
        p_sys->picture_height = p_enc->fmt_in.video.i_height;
        p_sys->picture_width_in_mbs = (p_sys->picture_width + 15) / 16;
        p_sys->picture_height_in_mbs = (p_sys->picture_height + 15) / 16;

        p_sys->intra_counter = 0;
        p_sys->first_frame = 1;
        p_sys->pts = 0;
        p_sys->initial_date = -1;

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
        p_sys->codedbuf_i_size = p_sys->picture_width * p_sys->picture_height * 5;
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

            if(p_enc->fmt_in.video.i_frame_rate > 0 && p_enc->fmt_in.video.i_frame_rate_base > 0)
            {
                p_sys->seq_param.time_scale = p_enc->fmt_in.video.i_frame_rate;
                p_sys->seq_param.num_units_in_tick = p_enc->fmt_in.video.i_frame_rate_base;
            }
            else
            {
                p_sys->seq_param.time_scale = 0;
                p_sys->seq_param.num_units_in_tick = 0;
            }

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


            p_sys->seq_param.vui_parameters_present_flag = (p_sys->seq_param.time_scale > 0 || p_sys->frame_bit_rate > 0)?1:0;
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

        char str[5];
        str[4] = 0;
        vlc_fourcc_to_char(p_enc->fmt_in.i_codec, str);
        msg_Info(p_enc, "Negotiated to fourcc format \"%s\"", str);

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

static void pick_stack_push(encoder_t *p_enc, picture_t *pic)
{
    encoder_sys_t *p_sys = p_enc->p_sys;

    struct picture_stack *tmp = (struct picture_stack *)malloc(sizeof(struct picture_stack));
    tmp->pic = pic;

    struct picture_stack *cur = p_sys->pic_stack_head;
    struct picture_stack **cur_ptr = &p_sys->pic_stack_head;
    while(cur != 0 && pic->date > cur->pic->date)
    {
        cur_ptr = &cur->next;
        cur = cur->next;
    }
    
    tmp->next = cur;
    *cur_ptr = tmp;
    
    p_sys->pic_stack_count++;
}

static picture_t * pick_stack_pop(encoder_t *p_enc)
{
    encoder_sys_t *p_sys = p_enc->p_sys;
    
    if(p_sys->pic_stack_head == 0)
        return 0;

    picture_t *res = p_sys->pic_stack_head->pic;

    struct picture_stack *tmp = p_sys->pic_stack_head;
    p_sys->pic_stack_head = p_sys->pic_stack_head->next;
    free(tmp);

    p_sys->pic_stack_count--;

    return res;
}
 
#define BITSTREAM_ALLOCATE_STEPPING     4096

struct __bitstream {
    unsigned int *buffer;
    int bit_offset;
    int max_size_in_dword;
};

typedef struct __bitstream bitstream;

static unsigned int
va_swap32(unsigned int val)
{
    unsigned char *pval = (unsigned char *)&val;

    return ((pval[0] << 24)     |
            (pval[1] << 16)     |
            (pval[2] << 8)      |
            (pval[3] << 0));
}

static void
bitstream_start(bitstream *bs)
{
    bs->max_size_in_dword = BITSTREAM_ALLOCATE_STEPPING;
    bs->buffer = calloc(bs->max_size_in_dword * sizeof(int), 1);
    bs->bit_offset = 0;
}

static void
bitstream_end(bitstream *bs)
{
    int pos = (bs->bit_offset >> 5);
    int bit_offset = (bs->bit_offset & 0x1f);
    int bit_left = 32 - bit_offset;

    if (bit_offset) {
        bs->buffer[pos] = va_swap32((bs->buffer[pos] << bit_left));
    }
}

static void
bitstream_put_ui(bitstream *bs, unsigned int val, int size_in_bits)
{
    int pos = (bs->bit_offset >> 5);
    int bit_offset = (bs->bit_offset & 0x1f);
    int bit_left = 32 - bit_offset;

    if (!size_in_bits)
        return;

    bs->bit_offset += size_in_bits;

    if (bit_left > size_in_bits) {
        bs->buffer[pos] = (bs->buffer[pos] << size_in_bits | val);
    } else {
        size_in_bits -= bit_left;
        bs->buffer[pos] = (bs->buffer[pos] << bit_left) | (val >> size_in_bits);
        bs->buffer[pos] = va_swap32(bs->buffer[pos]);

        if (pos + 1 == bs->max_size_in_dword) {
            bs->max_size_in_dword += BITSTREAM_ALLOCATE_STEPPING;
            bs->buffer = realloc(bs->buffer, bs->max_size_in_dword * sizeof(unsigned int));
        }

        bs->buffer[pos + 1] = val;
    }
}

static void
bitstream_put_ue(bitstream *bs, unsigned int val)
{
    int size_in_bits = 0;
    int tmp_val = ++val;

    while (tmp_val) {
        tmp_val >>= 1;
        size_in_bits++;
    }

    bitstream_put_ui(bs, 0, size_in_bits - 1); // leading zero
    bitstream_put_ui(bs, val, size_in_bits);
}

static void
bitstream_put_se(bitstream *bs, int val)
{
    unsigned int new_val;

    if (val <= 0)
        new_val = -2 * val;
    else
        new_val = 2 * val - 1;

    bitstream_put_ue(bs, new_val);
}

static void
bitstream_byte_aligning(bitstream *bs, int bit)
{
    int bit_offset = (bs->bit_offset & 0x7);
    int bit_left = 8 - bit_offset;
    int new_val;

    if (!bit_offset)
        return;

    assert(bit == 0 || bit == 1);

    if (bit)
        new_val = (1 << bit_left) - 1;
    else
        new_val = 0;

    bitstream_put_ui(bs, new_val, bit_left);
}

static void rbsp_trailing_bits(bitstream *bs)
{
    bitstream_put_ui(bs, 1, 1);
    bitstream_byte_aligning(bs, 0);
}

static void nal_start_code_prefix(bitstream *bs)
{
    bitstream_put_ui(bs, 0x00000001, 32);
}

static void nal_header(bitstream *bs, int nal_ref_idc, int nal_unit_type)
{
    bitstream_put_ui(bs, 0, 1);                /* forbidden_zero_bit: 0 */
    bitstream_put_ui(bs, nal_ref_idc, 2);
    bitstream_put_ui(bs, nal_unit_type, 5);
}

static void sps_rbsp(encoder_t *p_enc, bitstream *bs)
{
    encoder_sys_t *p_sys = p_enc->p_sys;

    VAEncSequenceParameterBufferH264 *seq_param = &p_sys->seq_param;
    int profile_idc = PROFILE_IDC_BASELINE;

    if (p_sys->profile == VAProfileH264High)
        profile_idc = PROFILE_IDC_HIGH;
    else if (p_sys->profile == VAProfileH264Main)
        profile_idc = PROFILE_IDC_MAIN;

    bitstream_put_ui(bs, profile_idc, 8);               /* profile_idc */
    bitstream_put_ui(bs, !!(p_sys->constraint_set_flag & 1), 1);                         /* constraint_set0_flag */
    bitstream_put_ui(bs, !!(p_sys->constraint_set_flag & 2), 1);                         /* constraint_set1_flag */
    bitstream_put_ui(bs, !!(p_sys->constraint_set_flag & 4), 1);                         /* constraint_set2_flag */
    bitstream_put_ui(bs, !!(p_sys->constraint_set_flag & 8), 1);                         /* constraint_set3_flag */
    bitstream_put_ui(bs, 0, 4);                         /* reserved_zero_4bits */
    bitstream_put_ui(bs, seq_param->level_idc, 8);      /* level_idc */
    bitstream_put_ue(bs, seq_param->seq_parameter_set_id);      /* seq_parameter_set_id */

    if ( profile_idc == PROFILE_IDC_HIGH) {
        bitstream_put_ue(bs, 1);        /* chroma_format_idc = 1, 4:2:0 */
        bitstream_put_ue(bs, 0);        /* bit_depth_luma_minus8 */
        bitstream_put_ue(bs, 0);        /* bit_depth_chroma_minus8 */
        bitstream_put_ui(bs, 0, 1);     /* qpprime_y_zero_transform_bypass_flag */
        bitstream_put_ui(bs, 0, 1);     /* seq_scaling_matrix_present_flag */
    }

    bitstream_put_ue(bs, seq_param->seq_fields.bits.log2_max_frame_num_minus4); /* log2_max_frame_num_minus4 */
    bitstream_put_ue(bs, seq_param->seq_fields.bits.pic_order_cnt_type);        /* pic_order_cnt_type */

    if (seq_param->seq_fields.bits.pic_order_cnt_type == 0)
        bitstream_put_ue(bs, seq_param->seq_fields.bits.log2_max_pic_order_cnt_lsb_minus4);     /* log2_max_pic_order_cnt_lsb_minus4 */
    else {
        assert(0);
    }

    bitstream_put_ue(bs, seq_param->max_num_ref_frames);        /* num_ref_frames */
    bitstream_put_ui(bs, 0, 1);                                 /* gaps_in_frame_num_value_allowed_flag */

    bitstream_put_ue(bs, seq_param->picture_width_in_mbs - 1);  /* pic_width_in_mbs_minus1 */
    bitstream_put_ue(bs, seq_param->picture_height_in_mbs - 1); /* pic_height_in_map_units_minus1 */
    bitstream_put_ui(bs, seq_param->seq_fields.bits.frame_mbs_only_flag, 1);    /* frame_mbs_only_flag */

    if (!seq_param->seq_fields.bits.frame_mbs_only_flag) {
        assert(0);
    }

    bitstream_put_ui(bs, seq_param->seq_fields.bits.direct_8x8_inference_flag, 1);      /* direct_8x8_inference_flag */
    bitstream_put_ui(bs, seq_param->frame_cropping_flag, 1);            /* frame_cropping_flag */

    if (seq_param->frame_cropping_flag) {
        bitstream_put_ue(bs, seq_param->frame_crop_left_offset);        /* frame_crop_left_offset */
        bitstream_put_ue(bs, seq_param->frame_crop_right_offset);       /* frame_crop_right_offset */
        bitstream_put_ue(bs, seq_param->frame_crop_top_offset);         /* frame_crop_top_offset */
        bitstream_put_ue(bs, seq_param->frame_crop_bottom_offset);      /* frame_crop_bottom_offset */
    }

    if((seq_param->num_units_in_tick > 0 && seq_param->time_scale > 0) || p_sys->frame_bit_rate > 0)
    {
        bitstream_put_ui(bs, 1, 1); /* vui_parameters_present_flag */
        bitstream_put_ui(bs, 0, 1); /* aspect_ratio_info_present_flag */
        bitstream_put_ui(bs, 0, 1); /* overscan_info_present_flag */
        bitstream_put_ui(bs, 0, 1); /* video_signal_type_present_flag */
        bitstream_put_ui(bs, 0, 1); /* chroma_loc_info_present_flag */
        if(seq_param->num_units_in_tick > 0 && seq_param->time_scale > 0)
        {
            bitstream_put_ui(bs, 1, 1); /* timing_info_present_flag */
            {
                bitstream_put_ui(bs, seq_param->num_units_in_tick, 32);
                bitstream_put_ui(bs, seq_param->time_scale, 32);
                bitstream_put_ui(bs, 1, 1);
            }
        }
        else
        {
            bitstream_put_ui(bs, 0, 1); /* timing_info_present_flag */
        }

        if(p_sys->frame_bit_rate <= 0)
        {
            bitstream_put_ui(bs, 0, 1);/* nal_hrd_parameters_present_flag */
        }
        else
        {
            bitstream_put_ui(bs, 1, 1); /* nal_hrd_parameters_present_flag */

            // hrd_parameters
            bitstream_put_ue(bs, 0);    /* cpb_cnt_minus1 */
            bitstream_put_ui(bs, 4, 4); /* bit_rate_scale */
            bitstream_put_ui(bs, 6, 4); /* cpb_size_scale */

            bitstream_put_ue(bs, p_sys->frame_bit_rate - 1); /* bit_rate_value_minus1[0] */
            bitstream_put_ue(bs, p_sys->frame_bit_rate*8 - 1); /* cpb_size_value_minus1[0] */
            bitstream_put_ui(bs, 1, 1);  /* cbr_flag[0] */

            bitstream_put_ui(bs, 23, 5);   /* initial_cpb_removal_delay_length_minus1 */
            bitstream_put_ui(bs, 23, 5);   /* cpb_removal_delay_length_minus1 */
            bitstream_put_ui(bs, 23, 5);   /* dpb_output_delay_length_minus1 */
            bitstream_put_ui(bs, 23, 5);   /* time_offset_length  */
        }
        bitstream_put_ui(bs, 0, 1);   /* vcl_hrd_parameters_present_flag */
        bitstream_put_ui(bs, 0, 1);   /* low_delay_hrd_flag */

        bitstream_put_ui(bs, 0, 1); /* pic_struct_present_flag */
        bitstream_put_ui(bs, 0, 1); /* bitstream_restriction_flag */
    }
    else
    {
        bitstream_put_ui(bs, 0, 1); /* vui_parameters_present_flag */
    }

    rbsp_trailing_bits(bs);     /* rbsp_trailing_bits */
}

static void pps_rbsp(encoder_t *p_enc, bitstream *bs)
{
    encoder_sys_t *p_sys = p_enc->p_sys;

    VAEncPictureParameterBufferH264 *pic_param = &p_sys->pic_param;

    bitstream_put_ue(bs, pic_param->pic_parameter_set_id);      /* pic_parameter_set_id */
    bitstream_put_ue(bs, pic_param->seq_parameter_set_id);      /* seq_parameter_set_id */

    bitstream_put_ui(bs, pic_param->pic_fields.bits.entropy_coding_mode_flag, 1);  /* entropy_coding_mode_flag */

    bitstream_put_ui(bs, 0, 1);                         /* pic_order_present_flag: 0 */

    bitstream_put_ue(bs, 0);                            /* num_slice_groups_minus1 */

    bitstream_put_ue(bs, pic_param->num_ref_idx_l0_active_minus1);      /* num_ref_idx_l0_active_minus1 */
    bitstream_put_ue(bs, pic_param->num_ref_idx_l1_active_minus1);      /* num_ref_idx_l1_active_minus1 1 */

    bitstream_put_ui(bs, pic_param->pic_fields.bits.weighted_pred_flag, 1);     /* weighted_pred_flag: 0 */
    bitstream_put_ui(bs, pic_param->pic_fields.bits.weighted_bipred_idc, 2);    /* weighted_bipred_idc: 0 */

    bitstream_put_se(bs, pic_param->pic_init_qp - 26);  /* pic_init_qp_minus26 */
    bitstream_put_se(bs, 0);                            /* pic_init_qs_minus26 */
    bitstream_put_se(bs, 0);                            /* chroma_qp_index_offset */

    bitstream_put_ui(bs, pic_param->pic_fields.bits.deblocking_filter_control_present_flag, 1); /* deblocking_filter_control_present_flag */
    bitstream_put_ui(bs, 0, 1);                         /* constrained_intra_pred_flag */
    bitstream_put_ui(bs, 0, 1);                         /* redundant_pic_cnt_present_flag */

    /* more_rbsp_data */
    bitstream_put_ui(bs, pic_param->pic_fields.bits.transform_8x8_mode_flag, 1);    /*transform_8x8_mode_flag */
    bitstream_put_ui(bs, 0, 1);                         /* pic_scaling_matrix_present_flag */
    bitstream_put_se(bs, pic_param->second_chroma_qp_index_offset );    /*second_chroma_qp_index_offset */

    rbsp_trailing_bits(bs);
}

static int build_packed_pic_buffer(encoder_t *p_enc, unsigned char **header_buffer)
{
    bitstream bs;

    bitstream_start(&bs);
    nal_start_code_prefix(&bs);
    nal_header(&bs, NAL_REF_IDC_HIGH, NAL_PPS);
    pps_rbsp(p_enc, &bs);
    bitstream_end(&bs);

    *header_buffer = (unsigned char *)bs.buffer;
    return bs.bit_offset;
}

static int
build_packed_seq_buffer(encoder_t *p_enc, unsigned char **header_buffer)
{
    bitstream bs;

    bitstream_start(&bs);
    nal_start_code_prefix(&bs);
    nal_header(&bs, NAL_REF_IDC_HIGH, NAL_SPS);
    sps_rbsp(p_enc, &bs);
    bitstream_end(&bs);

    *header_buffer = (unsigned char *)bs.buffer;
    return bs.bit_offset;
}

static int
build_packed_sei_buffer_timing(encoder_t *p_enc, unsigned int init_cpb_removal_length,
                unsigned int init_cpb_removal_delay,
                unsigned int init_cpb_removal_delay_offset,
                unsigned int cpb_removal_length,
                unsigned int cpb_removal_delay,
                unsigned int dpb_output_length,
                unsigned int dpb_output_delay,
                unsigned char **sei_buffer)
{
    VLC_UNUSED(init_cpb_removal_length);
    VLC_UNUSED(p_enc);

    unsigned char *byte_buf;
    int bp_byte_size, i, pic_byte_size;

    bitstream nal_bs;
    bitstream sei_bp_bs, sei_pic_bs;

    bitstream_start(&sei_bp_bs);
    bitstream_put_ue(&sei_bp_bs, 0);       /*seq_parameter_set_id*/
    bitstream_put_ui(&sei_bp_bs, init_cpb_removal_delay, cpb_removal_length);
    bitstream_put_ui(&sei_bp_bs, init_cpb_removal_delay_offset, cpb_removal_length);
    if ( sei_bp_bs.bit_offset & 0x7) {
        bitstream_put_ui(&sei_bp_bs, 1, 1);
    }
    bitstream_end(&sei_bp_bs);
    bp_byte_size = (sei_bp_bs.bit_offset + 7) / 8;

    bitstream_start(&sei_pic_bs);
    bitstream_put_ui(&sei_pic_bs, cpb_removal_delay, cpb_removal_length);
    bitstream_put_ui(&sei_pic_bs, dpb_output_delay, dpb_output_length);
    if ( sei_pic_bs.bit_offset & 0x7) {
        bitstream_put_ui(&sei_pic_bs, 1, 1);
    }
    bitstream_end(&sei_pic_bs);
    pic_byte_size = (sei_pic_bs.bit_offset + 7) / 8;

    bitstream_start(&nal_bs);
    nal_start_code_prefix(&nal_bs);
    nal_header(&nal_bs, NAL_REF_IDC_NONE, NAL_SEI);

    /* Write the SEI buffer period data */
    bitstream_put_ui(&nal_bs, 0, 8);
    bitstream_put_ui(&nal_bs, bp_byte_size, 8);

    byte_buf = (unsigned char *)sei_bp_bs.buffer;
    for(i = 0; i < bp_byte_size; i++) {
        bitstream_put_ui(&nal_bs, byte_buf[i], 8);
    }
    free(byte_buf);
    /* write the SEI timing data */
    bitstream_put_ui(&nal_bs, 0x01, 8);
    bitstream_put_ui(&nal_bs, pic_byte_size, 8);

    byte_buf = (unsigned char *)sei_pic_bs.buffer;
    for(i = 0; i < pic_byte_size; i++) {
        bitstream_put_ui(&nal_bs, byte_buf[i], 8);
    }
    free(byte_buf);

    rbsp_trailing_bits(&nal_bs);
    bitstream_end(&nal_bs);

    *sei_buffer = (unsigned char *)nal_bs.buffer;

    return nal_bs.bit_offset;
}

static int safe_destroy_buffers(encoder_t *p_enc, VABufferID *va_buffers, unsigned int num_va_buffers)
{
    encoder_sys_t *p_sys = p_enc->p_sys;
    VAStatus va_status;
    unsigned int i;

    for (i = 0; i < num_va_buffers; i++) {
        if (va_buffers[i] != VA_INVALID_ID) {
            va_status = vaDestroyBuffer(p_sys->va_dpy, va_buffers[i]);
            CHECK_VASTATUS(va_status, "vaDestroyBuffer", 0);
            va_buffers[i] = VA_INVALID_ID;
        }
    }

    return 1;
}

block_t *GenCodedBlock(encoder_t *p_enc, int is_intra, mtime_t date)
{
    encoder_sys_t *p_sys = p_enc->p_sys;

    VACodedBufferSegment *buf_list = 0;
    VAStatus va_status;
    VASurfaceStatus surface_status = 0;

    va_status = vaSyncSurface(p_sys->va_dpy, p_sys->surface_id[SID_INPUT_PICTURE]);
    CHECK_VASTATUS(va_status, "vaSyncSurface", 0);

    va_status = vaQuerySurfaceStatus(p_sys->va_dpy, p_sys->surface_id[SID_INPUT_PICTURE], &surface_status);
    CHECK_VASTATUS(va_status, "vaQuerySurfaceStatus", 0);

    va_status = vaMapBuffer(p_sys->va_dpy, p_sys->codedbuf_buf_id, (void**)(&buf_list));
    CHECK_VASTATUS(va_status, "vaMapBuffer", 0);

    if(buf_list->status & VA_CODED_BUF_STATUS_SLICE_OVERFLOW_MASK)
    {
        msg_Err(p_enc, "Output buffer overflown");
        return 0;
    }

    block_t *block = block_Alloc(buf_list->size);

    memcpy(block->p_buffer, buf_list->buf, buf_list->size);

    if(p_sys->seq_param.time_scale > 0 && p_sys->seq_param.num_units_in_tick > 0)
    {
        block->i_length = INT64_C(1000000) * p_sys->seq_param.num_units_in_tick / p_sys->seq_param.time_scale;
        block->i_pts = p_sys->pts;
        block->i_dts = p_sys->pts;
    } else {
        block->i_length = 0;
        block->i_pts = date - p_sys->initial_date;
        block->i_dts = date - p_sys->initial_date;
    }

    block->i_flags |= (is_intra?BLOCK_FLAG_TYPE_I:BLOCK_FLAG_TYPE_P);

    p_sys->pts += block->i_length;

    vaUnmapBuffer(p_sys->va_dpy, p_sys->codedbuf_buf_id);

    return block;
}

void CopyPlane(uint8_t *dst, size_t dst_pitch, const uint8_t *src, size_t src_pitch, uint32_t width, uint32_t height)
{
    for(uint32_t y = 0; y < height; y++)
    {
        memcpy(dst, src, width);
        src += src_pitch;
        dst += dst_pitch;
    }
}

int UploadPictureToSurface(encoder_t *p_enc, picture_t *p_pict, VASurfaceID surface_id)
{
    encoder_sys_t *p_sys = p_enc->p_sys;

    VAImage surface_image;
    uint8_t *surface_p = 0;

    VAStatus va_status = vaDeriveImage(p_sys->va_dpy, surface_id, &surface_image);
    CHECK_VASTATUS(va_status, "vaDeriveImage", 0);

    va_status = vaMapBuffer(p_sys->va_dpy, surface_image.buf, (void**)&surface_p);
    CHECK_VASTATUS(va_status, "vaMapBuffer", 0);

    for(int i = 0; i < p_pict->i_planes; i++)
    {
        CopyPlane(surface_p + surface_image.offsets[i], surface_image.pitches[i], p_pict->p[i].p_pixels, p_pict->p[i].i_pitch, p_pict->p[i].i_pitch, p_pict->p[i].i_lines);
    }

    vaUnmapBuffer(p_sys->va_dpy, surface_image.buf);
    vaDestroyImage(p_sys->va_dpy, surface_image.image_id);

    return 1;
}

static block_t *EncodeVideo(encoder_t *p_enc, picture_t *p_pict)
{
    VAEncMiscParameterBuffer *misc_param;
    VAEncMiscParameterHRD *misc_hrd_param;
    VABufferID tempID;
    VAStatus va_status;
    VACodedBufferSegment *coded_buffer_segment = 0;
    VABufferID va_buffers[10];
    unsigned int num_va_buffers = 0;
    int is_intra = 0;
    int is_idr = 0;

    encoder_sys_t *p_sys = p_enc->p_sys;

    int pop_count = 1;
    if(!p_pict)
    {
        pop_count = p_sys->pic_stack_count;
        msg_Info(p_enc, "Got an empty picture, emptying list: %d remaining!", pop_count);
    }
    else
    {
        picture_Hold(p_pict);
        pick_stack_push(p_enc, p_pict);
        
        if(p_sys->pic_stack_count < 25)
            return 0;
    }

    block_t *chain = 0;
    for(int kk = 0; kk < pop_count; ++kk)
    {
        if(p_sys->intra_counter % p_sys->intra_rate == 0)
        {
            is_intra = 1;
        }

        picture_t *p_pict = pick_stack_pop(p_enc);
        if(!UploadPictureToSurface(p_enc, p_pict, p_sys->surface_id[SID_INPUT_PICTURE]))
            return 0;
        mtime_t date = p_pict->date;
        picture_Release(p_pict);
        p_pict = 0;

        if(p_sys->first_frame)
        {
            is_idr = 1;
            p_sys->initial_date = date;

            VAEncPackedHeaderParameterBuffer packed_header_param_buffer;
            unsigned int length_in_bits;
            unsigned char *packed_seq_buffer = NULL, *packed_pic_buffer = NULL;

            length_in_bits = build_packed_seq_buffer(p_enc, &packed_seq_buffer);
            packed_header_param_buffer.type = VAEncPackedHeaderSequence;
            packed_header_param_buffer.bit_length = length_in_bits;
            packed_header_param_buffer.has_emulation_bytes = 0;
            va_status = vaCreateBuffer(p_sys->va_dpy, p_sys->context_id, VAEncPackedHeaderParameterBufferType, sizeof(packed_header_param_buffer), 1, &packed_header_param_buffer, &p_sys->packed_seq_header_param_buf_id);
            CHECK_VASTATUS(va_status, "vaCreateBuffer", 0);

            va_status = vaCreateBuffer(p_sys->va_dpy, p_sys->context_id, VAEncPackedHeaderDataBufferType, (length_in_bits + 7) / 8, 1, packed_seq_buffer, &p_sys->packed_seq_buf_id);
            CHECK_VASTATUS(va_status, "vaCreateBuffer", 0);

            length_in_bits = build_packed_pic_buffer(p_enc, &packed_pic_buffer);
            packed_header_param_buffer.type = VAEncPackedHeaderPicture;
            packed_header_param_buffer.bit_length = length_in_bits;
            packed_header_param_buffer.has_emulation_bytes = 0;

            va_status = vaCreateBuffer(p_sys->va_dpy, p_sys->context_id, VAEncPackedHeaderParameterBufferType, sizeof(packed_header_param_buffer), 1, &packed_header_param_buffer, &p_sys->packed_pic_header_param_buf_id);
            CHECK_VASTATUS(va_status, "vaCreateBuffer", 0);

            va_status = vaCreateBuffer(p_sys->va_dpy, p_sys->context_id, VAEncPackedHeaderDataBufferType, (length_in_bits + 7) / 8, 1, packed_pic_buffer, &p_sys->packed_pic_buf_id);
            CHECK_VASTATUS(va_status, "vaCreateBuffer", 0);

            free(packed_seq_buffer);
            free(packed_pic_buffer);
            p_sys->first_frame = 0;
        }

        va_status = vaCreateBuffer(p_sys->va_dpy, p_sys->context_id, VAEncSequenceParameterBufferType, sizeof(p_sys->seq_param), 1, &(p_sys->seq_param), &(p_sys->seq_param_buf_id));
        CHECK_VASTATUS(va_status, "vaCreateBuffer", 0);

        va_status = vaCreateBuffer(p_sys->va_dpy, p_sys->context_id, VAEncMiscParameterBufferType, sizeof(VAEncMiscParameterBuffer) + sizeof(VAEncMiscParameterRateControl), 1, NULL,  &p_sys->misc_parameter_hrd_buf_id);
        CHECK_VASTATUS(va_status, "vaCreateBuffer", 0);

        va_status = vaMapBuffer(p_sys->va_dpy, p_sys->misc_parameter_hrd_buf_id, (void **)&misc_param);
        CHECK_VASTATUS(va_status, "vaMapBuffer", 0);
        misc_param->type = VAEncMiscParameterTypeHRD;
        misc_hrd_param = (VAEncMiscParameterHRD *)misc_param->data;

        if (p_sys->frame_bit_rate > 0)
        {
            misc_hrd_param->initial_buffer_fullness = p_sys->frame_bit_rate * 1024 * 4;
            misc_hrd_param->buffer_size = p_sys->frame_bit_rate * 1024 * 8;
        }
        else
        {
            misc_hrd_param->initial_buffer_fullness = 0;
            misc_hrd_param->buffer_size = 0;
        }

        vaUnmapBuffer(p_sys->va_dpy, p_sys->misc_parameter_hrd_buf_id);

        p_sys->slice_param[0].slice_type = (is_intra?SLICE_TYPE_I:SLICE_TYPE_P);
        p_sys->slice_param[0].num_macroblocks = p_sys->picture_height_in_mbs * p_sys->picture_width_in_mbs;
        p_sys->slice_param[0].slice_alpha_c0_offset_div2 = 2;
        p_sys->slice_param[0].slice_beta_offset_div2 = 2;
        if(p_sys->slice_param_buf_id[0] != VA_INVALID_ID)
            vaDestroyBuffer(p_sys->va_dpy, p_sys->slice_param_buf_id[0]);
        va_status = vaCreateBuffer(p_sys->va_dpy, p_sys->context_id, VAEncSliceParameterBufferType, sizeof(p_sys->slice_param[0]), 1, &(p_sys->slice_param[0]), &(p_sys->slice_param_buf_id[0]));
        CHECK_VASTATUS(va_status, "vaCreateBuffer", 0);

        if(p_sys->pic_param_buf_id != VA_INVALID_ID)
            vaDestroyBuffer(p_sys->va_dpy, p_sys->pic_param_buf_id);
        p_sys->pic_param.ReferenceFrames[0].picture_id = p_sys->surface_id[SID_REFERENCE_PICTURE_1];
        p_sys->pic_param.ReferenceFrames[1].picture_id = p_sys->surface_id[SID_REFERENCE_PICTURE_2];
        p_sys->pic_param.ReferenceFrames[2].picture_id = VA_INVALID_ID;
        p_sys->pic_param.CurrPic.picture_id = p_sys->surface_id[SID_RECON_PICTURE];
        p_sys->pic_param.CurrPic.TopFieldOrderCnt = p_sys->intra_counter * 2;
        p_sys->pic_param.pic_fields.bits.idr_pic_flag = !!is_idr;
        p_sys->pic_param.pic_fields.bits.reference_pic_flag = (p_sys->slice_param[0].slice_type != SLICE_TYPE_B);
        p_sys->pic_param.frame_num = p_sys->intra_counter;
        va_status = vaCreateBuffer(p_sys->va_dpy, p_sys->context_id, VAEncPictureParameterBufferType, sizeof(p_sys->pic_param), 1, &p_sys->pic_param, &p_sys->pic_param_buf_id);
        CHECK_VASTATUS(va_status,"vaCreateBuffer", 0);

        if(p_sys->rate_control_method == VA_RC_CBR)
        {
            VAEncPackedHeaderParameterBuffer packed_header_param_buffer;
            unsigned int length_in_bits;
            unsigned char *packed_sei_buffer = NULL;

            length_in_bits = build_packed_sei_buffer_timing(p_enc, p_sys->i_initial_cpb_removal_delay_length, p_sys->i_initial_cpb_removal_delay, 0, p_sys->i_cpb_removal_delay_length, p_sys->i_cpb_removal_delay * p_sys->intra_counter, p_sys->i_dpb_output_delay_length, 0, &packed_sei_buffer);

            packed_header_param_buffer.type = VAEncPackedHeaderH264_SEI;
            packed_header_param_buffer.bit_length = length_in_bits;
            packed_header_param_buffer.has_emulation_bytes = 0;

            va_status = vaCreateBuffer(p_sys->va_dpy, p_sys->context_id, VAEncPackedHeaderParameterBufferType, sizeof(packed_header_param_buffer), 1, &packed_header_param_buffer, &p_sys->packed_sei_header_param_buf_id);
            CHECK_VASTATUS(va_status, "vaCreateBuffer", 0);

            va_status = vaCreateBuffer(p_sys->va_dpy, p_sys->context_id, VAEncPackedHeaderDataBufferType, (length_in_bits + 7) / 8, 1, packed_sei_buffer, &p_sys->packed_sei_buf_id);
            CHECK_VASTATUS(va_status, "vaCreateBuffer", 0);
            free(packed_sei_buffer);
        }

        num_va_buffers = 0;
        va_buffers[num_va_buffers++] = p_sys->seq_param_buf_id;
        va_buffers[num_va_buffers++] = p_sys->pic_param_buf_id;

        if (p_sys->packed_seq_header_param_buf_id != VA_INVALID_ID)
            va_buffers[num_va_buffers++] = p_sys->packed_seq_header_param_buf_id;

        if (p_sys->packed_seq_buf_id != VA_INVALID_ID)
            va_buffers[num_va_buffers++] = p_sys->packed_seq_buf_id;

        if (p_sys->packed_pic_header_param_buf_id != VA_INVALID_ID)
            va_buffers[num_va_buffers++] = p_sys->packed_pic_header_param_buf_id;

        if (p_sys->packed_pic_buf_id != VA_INVALID_ID)
            va_buffers[num_va_buffers++] = p_sys->packed_pic_buf_id;

        if (p_sys->packed_sei_header_param_buf_id != VA_INVALID_ID)
            va_buffers[num_va_buffers++] = p_sys->packed_sei_header_param_buf_id;

        if (p_sys->packed_sei_buf_id != VA_INVALID_ID)
            va_buffers[num_va_buffers++] = p_sys->packed_sei_buf_id;

        if (p_sys->misc_parameter_hrd_buf_id != VA_INVALID_ID)
            va_buffers[num_va_buffers++] =  p_sys->misc_parameter_hrd_buf_id;

        va_status = vaMapBuffer(p_sys->va_dpy, p_sys->codedbuf_buf_id, (void **)(&coded_buffer_segment));
        CHECK_VASTATUS(va_status, "vaMapBuffer", 0);
        memset(coded_buffer_segment->buf, 0, coded_buffer_segment->size);
        vaUnmapBuffer(p_sys->va_dpy, p_sys->codedbuf_buf_id);

        va_status = vaBeginPicture(p_sys->va_dpy, p_sys->context_id, p_sys->surface_id[SID_INPUT_PICTURE]);
        CHECK_VASTATUS(va_status, "vaBeginPicture", 0);

        va_status = vaRenderPicture(p_sys->va_dpy, p_sys->context_id, va_buffers, num_va_buffers);
        CHECK_VASTATUS(va_status, "vaRenderPicture", 0);

        va_status = vaRenderPicture(p_sys->va_dpy, p_sys->context_id, &(p_sys->slice_param_buf_id[0]), 1);
        CHECK_VASTATUS(va_status,"vaRenderPicture", 0);

        va_status = vaEndPicture(p_sys->va_dpy , p_sys->context_id);
        CHECK_VASTATUS(va_status, "vaEndPicture", 0);

        tempID = p_sys->surface_id[SID_RECON_PICTURE];
        p_sys->surface_id[SID_RECON_PICTURE] = p_sys->surface_id[SID_REFERENCE_PICTURE_1];
        p_sys->surface_id[SID_REFERENCE_PICTURE_1] = tempID;

        block_t *res = GenCodedBlock(p_enc, is_intra, date);

        safe_destroy_buffers(p_enc, &p_sys->seq_param_buf_id, 1);
        safe_destroy_buffers(p_enc, &p_sys->packed_seq_header_param_buf_id, 1);
        safe_destroy_buffers(p_enc, &p_sys->packed_seq_buf_id, 1);
        safe_destroy_buffers(p_enc, &p_sys->packed_pic_header_param_buf_id, 1);
        safe_destroy_buffers(p_enc, &p_sys->packed_pic_buf_id, 1);
        safe_destroy_buffers(p_enc, &p_sys->packed_sei_header_param_buf_id, 1);
        safe_destroy_buffers(p_enc, &p_sys->packed_sei_buf_id, 1);
        safe_destroy_buffers(p_enc, &p_sys->misc_parameter_hrd_buf_id, 1);

        p_sys->intra_counter += 1;

        block_ChainAppend(&chain, res);
    }

    return chain;
}
