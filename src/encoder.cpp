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
    return true;
}

block_t *VaEncoder::encode(picture_t *pic)
{
    return 0;
}
