#include <vlc_common.h>
#include <vlc_plugin.h>

#include "encoder.h"


static int OpenEncoder(vlc_object_t *);
static void CloseEncoder(vlc_object_t *);
static block_t *EncodeVideo(encoder_t *p_enc, picture_t *p_pict);

vlc_module_begin()
set_shortname("vaapienc")
set_description("H264 VAAPI encoder")
set_capability("encoder", 0)
set_callbacks(OpenEncoder, CloseEncoder)
add_shortcut("vaapi", "va")
vlc_module_end()

struct encoder_sys_t
{
    VaEncoder *enc;
};

static int OpenEncoder(vlc_object_t *p_this)
{
    encoder_t *p_enc = (encoder_t*)p_this;
    encoder_sys_t *p_sys;

    p_enc->p_sys = p_sys = (encoder_sys_t*)calloc(1, sizeof(encoder_sys_t));
    if(!p_sys)
        return VLC_ENOMEM;

    p_enc->pf_encode_audio = 0;
    p_enc->pf_encode_sub = 0;
    p_enc->pf_encode_video = EncodeVideo;

    p_sys->enc = new VaEncoder(p_enc);
    if(!p_sys->enc)
    {
        free(p_sys);
        return VLC_ENOMEM;
    }

    if(p_sys->enc->initialize())
    {
        return VLC_SUCCESS;
    }
    else
    {
        CloseEncoder(p_this);
        return VLC_EGENERIC;
    }
}

static void CloseEncoder(vlc_object_t *p_this)
{
    encoder_t *p_enc = (encoder_t*)p_this;
    encoder_sys_t *p_sys = p_enc->p_sys;

    if(!p_sys)
        return;

    if(p_sys->enc)
    {
        delete p_sys->enc;
        p_sys->enc = 0;
    }

    free(p_sys);
    p_enc->p_sys = 0;
}

static block_t *EncodeVideo(encoder_t *p_enc, picture_t *p_pict)
{
    encoder_sys_t *p_sys = p_enc->p_sys;

    return p_sys->enc->encode(p_pict);
}
