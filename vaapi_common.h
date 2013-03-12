/*****************************************************************************
 * vaapi_common.h: Video Acceleration API common code
 *****************************************************************************
 * Copyright (C) 2013 Timo Rothenpieler
 * $Id: 3654e15f02b4167b433aa2f11cc6aae468d7c554 $
 *
 * Authors: Timo Rothenpieler
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

#ifndef _VLC_VAAPI_COMMON_H
#define _VLC_VAAPI_COMMON_H 1

int vaapi_open_display(vlc_object_t *p_obj, VADisplay *p_display);
#define vaapi_open_display(a, b) vaapi_open_display(VLC_OBJECT(a), b)

int vaapi_close_display(VADisplay display);

typedef struct vaapi_surface_manage vaapi_surface_manage_t;
typedef struct vaapi_surface
{
    vaapi_surface_manage_t *owner;
    uint8_t index;

    VASurfaceID id;
} vaapi_surface_t;

vaapi_surface_manage_t * vaapi_surface_manage_init(VASurfaceID *surface_ids, uint32_t num_surface_ids);
vaapi_surface_t * vaapi_surface_manage_get_surface(vaapi_surface_manage_t *mng);
vaapi_surface_t * vaapi_surface_manage_find_surface(vaapi_surface_manage_t *mng, VASurfaceID id);
void vaapi_surface_manage_hold_surface(vaapi_surface_t *surf);
void vaapi_surface_manage_release_surface(vaapi_surface_t *surf);
void vaapi_surface_manage_close(vaapi_surface_manage_t *mng);

#endif
