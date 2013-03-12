/*****************************************************************************
 * vaapi_common.c: Video Acceleration API common code
 *****************************************************************************
 * Copyright (C) 2013 Timo Rothenpieler
 * $Id: d560d042c0ee3f436f2f3cdb2f220ab06cf6d222 $
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

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

#include <vlc_common.h>
#include <vlc_threads.h>
#include <vlc_xlib.h>
#include <vlc_picture.h>

#include <X11/Xlib.h>
#include <va/va.h>
#include <va/va_x11.h>

#include "vaapi_common.h"

typedef struct vaapi_common_private
{
    vlc_mutex_t  lock;
    Display     *p_display_x11;
    VADisplay    display;
    uint32_t     i_refs;
} vaapi_common_private_t;

//#define vaapi_internal_common (*((vaapi_common_private_t**)store_VAAPI()))
//Redefined for use outside of custom VLC branch
static vaapi_common_private_t *vaapi_internal_common = 0;

#undef vaapi_open_display
int vaapi_open_display(vlc_object_t *p_obj, VADisplay *p_display)
{
    if(!p_obj || !p_display)
        return 0;

    if(vaapi_internal_common == 0)
    { //This could, very unlikely, cause a race condition if two modules try to initialize at the same time
        vaapi_internal_common = (void*)calloc(1, sizeof(vaapi_common_private_t));
        vlc_mutex_init(&vaapi_internal_common->lock);
        vaapi_internal_common->i_refs = 0;
    }

    vlc_mutex_lock(&vaapi_internal_common->lock);
    if(vaapi_internal_common->i_refs == 0)
    {
        if(!vlc_xlib_init(p_obj))
        {
            vlc_mutex_unlock(&vaapi_internal_common->lock);
            return 0;
        }

        vaapi_internal_common->p_display_x11 = XOpenDisplay(NULL);
        if(!vaapi_internal_common->p_display_x11)
        {
            vlc_mutex_unlock(&vaapi_internal_common->lock);
            msg_Err(p_obj, "VAAPI: Could not connect to X11!");
            return 0;
        }

        vaapi_internal_common->display = vaGetDisplay(vaapi_internal_common->p_display_x11);
        if(!vaapi_internal_common->display)
        {
            XCloseDisplay(vaapi_internal_common->p_display_x11);
            vlc_mutex_unlock(&vaapi_internal_common->lock);
            msg_Err(p_obj, "VAAPI: Failed to get a VA Display!");
            return 0;
        }

        int major, minor;
        if(vaInitialize(vaapi_internal_common->display, &major, &minor) != VA_STATUS_SUCCESS)
        {
            vaTerminate(vaapi_internal_common->display);
            XCloseDisplay(vaapi_internal_common->p_display_x11);
            vlc_mutex_unlock(&vaapi_internal_common->lock);
            msg_Err(p_obj, "VAAPI: Failed to initialize VAAPI!");
            return 0;
        }

        msg_Info(p_obj, "VAAPI: Successfully initialized VAAPI %d.%d", major, minor);
    }

    vaapi_internal_common->i_refs += 1;
    *p_display = vaapi_internal_common->display;
    vlc_mutex_unlock(&vaapi_internal_common->lock);

    return 1;
}

int vaapi_close_display(VADisplay display)
{
    if(vaapi_internal_common == 0)
        return 0;

    vlc_mutex_lock(&vaapi_internal_common->lock);
    if(vaapi_internal_common->i_refs == 0 || vaapi_internal_common->display != display)
    {
        vlc_mutex_unlock(&vaapi_internal_common->lock);
        return 0;
    }

    vaapi_internal_common->i_refs -= 1;

    if(vaapi_internal_common->i_refs == 0)
    {
        vaTerminate(vaapi_internal_common->display);
        XCloseDisplay(vaapi_internal_common->p_display_x11);
        vlc_mutex_unlock(&vaapi_internal_common->lock);
        free(vaapi_internal_common);
        vaapi_internal_common = 0;
        return 1;
    }

    vlc_mutex_unlock(&vaapi_internal_common->lock);
    return 1;
}


typedef struct vaapi_surface_manage
{
    vlc_mutex_t lock;
    vlc_cond_t cond;

    uint32_t free_surfaces;
    uint32_t num_surfaces;
    uint8_t *surfs_used;
    VASurfaceID *surfaces;
} vaapi_surface_manage_t;

vaapi_surface_manage_t * vaapi_surface_manage_init(VASurfaceID *surface_ids, uint32_t num_surface_ids)
{
    if(surface_ids == NULL || num_surface_ids == 0)
        return NULL;

    vaapi_surface_manage_t *res = (vaapi_surface_manage_t*)calloc(1, sizeof(vaapi_surface_manage_t));

    res->num_surfaces = num_surface_ids;
    res->surfs_used = (uint8_t*)calloc(num_surface_ids, sizeof(uint8_t));
    res->surfaces = (VASurfaceID*)calloc(num_surface_ids, sizeof(VASurfaceID));

    for(uint8_t i = 0; i < num_surface_ids; ++i)
    {
        res->surfs_used[i] = 0;
        res->surfaces[i] = surface_ids[i];
    }

    res->free_surfaces = res->num_surfaces;
    vlc_mutex_init(&res->lock);
    vlc_cond_init(&res->cond);

    return res;
}

vaapi_surface_t * vaapi_surface_manage_get_surface(vaapi_surface_manage_t *mng)
{
    vaapi_surface_t *res = 0;

    vlc_mutex_lock(&mng->lock);

    while(mng->free_surfaces == 0)
        vlc_cond_wait(&mng->cond, &mng->lock);

    uint8_t use;
    for(use = 0; use < mng->num_surfaces && mng->surfs_used[use]; ++use);
    if(use >= mng->num_surfaces)
    {
        vlc_mutex_unlock(&mng->lock);
        return 0;
    }

    res = (vaapi_surface_t*)calloc(1, sizeof(vaapi_surface_t));
    res->owner = mng;
    res->id = mng->surfaces[use];
    res->index = use;

    mng->surfs_used[use] = 1;
    mng->free_surfaces -= 1;

    vlc_mutex_unlock(&mng->lock);

    return res;
}

vaapi_surface_t * vaapi_surface_manage_find_surface(vaapi_surface_manage_t *mng, VASurfaceID id)
{
    vaapi_surface_t *res = NULL;

    vlc_mutex_lock(&mng->lock);

    uint8_t use;
    for(use = 0; use < mng->num_surfaces; ++use)
        if(mng->surfaces[use] == id && mng->surfs_used[use])
            break;

    if(use >= mng->num_surfaces)
    {
        vlc_mutex_unlock(&mng->lock);
        return 0;
    }

    res = (vaapi_surface_t*)calloc(1, sizeof(vaapi_surface_t));
    res->owner = mng;
    res->id = mng->surfaces[use];
    res->index = use;

    vlc_mutex_unlock(&mng->lock);

    return res;
}

void vaapi_surface_manage_hold_surface(vaapi_surface_t *surf)
{
    if(surf == NULL)
        return;

    vlc_mutex_lock(&surf->owner->lock);
    surf->owner->surfs_used[surf->index] += 1;
    vlc_mutex_unlock(&surf->owner->lock);
}

void vaapi_surface_manage_release_surface(vaapi_surface_t *surf)
{
    if(surf == NULL)
        return;

    vlc_mutex_lock(&surf->owner->lock);

    if(surf->owner->surfs_used[surf->index] == 0)
    {
        vlc_mutex_unlock(&surf->owner->lock);
        return;
    }

    surf->owner->surfs_used[surf->index] -= 1;
    if(surf->owner->surfs_used[surf->index] == 0)
    {
        surf->owner->free_surfaces += 1;
    }

    vlc_cond_signal(&surf->owner->cond);
    vlc_mutex_unlock(&surf->owner->lock);
}

void vaapi_surface_manage_close(vaapi_surface_manage_t *mng)
{
    vlc_mutex_lock(&mng->lock);
    while(mng->free_surfaces < mng->num_surfaces)
        vlc_cond_timedwait(&mng->cond, &mng->lock, INT64_C(250000));
    mng->free_surfaces = 0;
    vlc_mutex_unlock(&mng->lock);

    vlc_cond_destroy(&mng->cond);
    vlc_mutex_destroy(&mng->lock);

    free(mng->surfs_used);
    free(mng->surfaces);

    free(mng);
}
