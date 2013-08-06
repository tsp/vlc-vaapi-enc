#ifndef ENCODER_CLASS_H__
#define ENCODER_CLASS_H__

#include <vlc_common.h>
#include <vlc_picture.h>
#include <vlc_codec.h>
#include <vlc_block.h>
#include <vlc_fourcc.h>

class VaEncoder
{
    public:
    VaEncoder(encoder_t *p_enc);
    ~VaEncoder();

    bool initialize();
    block_t *encode(picture_t *pic);

    private:
    encoder_t *p_enc;
};

#endif
