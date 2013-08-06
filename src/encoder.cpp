#include "encoder.h"


VaEncoder::VaEncoder(encoder_t *p_enc)
    :p_enc(p_enc)
{

}

VaEncoder::~VaEncoder()
{

}

bool VaEncoder::initialize()
{
    if(p_enc->fmt_out.i_codec != VLC_CODEC_H264 && !p_enc->b_force)
        return false;

    p_enc->fmt_out.i_cat = VIDEO_ES;
    p_enc->fmt_out.i_codec = VLC_CODEC_H264;

    return true;
}

block_t *VaEncoder::encode(picture_t *pic)
{
    return 0;
}
