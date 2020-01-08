/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "TextureLogo.h"
#include "GLDebug.h"

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include <OVR_CAPI_GL.h>

#include "img-yarp-robot-64.h"
#include "img-crosshairs.h"

TextureLogo::TextureLogo(ovrSession session) :
    session(session),
    textureSwapChain(nullptr),
    width(yarp_logo.width),
    height(yarp_logo.height),
    bytes_per_pixel(yarp_logo.bytes_per_pixel),
    // see http://stackoverflow.com/questions/27294882/glteximage2d-fails-with-error-1282-using-pbo-bad-texture-resolution
    padding((4 - (width * bytes_per_pixel) % 4) % 4),
    rowSize(width * bytes_per_pixel + padding),
    bufferSize(rowSize * height),
    ptr((GLubyte*)yarp_logo.pixel_data)
{
    yDebug() << width;
    createTexture();
}

TextureLogo::~TextureLogo()
{
    deleteTexture();
}

void TextureLogo::createTexture()
{
    ovrTextureSwapChainDesc desc = {};
    desc.Type = ovrTexture_2D;
    desc.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;
    desc.ArraySize = 1;
    desc.Width = width;
    desc.Height = height;
    desc.MipLevels = 1;
    desc.SampleCount = 1;
    desc.StaticImage = ovrTrue;
    desc.MiscFlags = ovrTextureMisc_None;
    desc.BindFlags = ovrTextureBind_None;

    if (ovr_CreateTextureSwapChainGL(session, &desc, &textureSwapChain) != ovrSuccess) {
        yFatal() << "Failed to create texture swap chain";
    }

    ovr_GetTextureSwapChainBufferGL(session, textureSwapChain, -1, &chainTexId);
    checkGlErrorMacro;

    glBindTexture(GL_TEXTURE_2D, chainTexId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, ptr);
    checkGlErrorMacro;

    glBindTexture(GL_TEXTURE_2D, 0);

    ovr_CommitTextureSwapChain(session, textureSwapChain);
    checkGlErrorMacro;
}

void TextureLogo::deleteTexture()
{
    ovr_DestroyTextureSwapChain(session, textureSwapChain);
}




TextureCrosshairs::TextureCrosshairs(ovrSession session) :
    session(session),
    textureSwapChain(nullptr),
    width(crosshairs.width),
    height(crosshairs.height),
    bytes_per_pixel(crosshairs.bytes_per_pixel),
    // see http://stackoverflow.com/questions/27294882/glteximage2d-fails-with-error-1282-using-pbo-bad-texture-resolution
    padding((4 - (width * bytes_per_pixel) % 4) % 4),
    rowSize(width * bytes_per_pixel + padding),
    bufferSize(rowSize * height),
    ptr((GLubyte*)crosshairs.pixel_data)
{
    checkGlErrorMacro;
    createTexture();
}

TextureCrosshairs::~TextureCrosshairs()
{
    deleteTexture();
}

void TextureCrosshairs::createTexture()
{
    ovrTextureSwapChainDesc desc = {};
    desc.Type = ovrTexture_2D;
    desc.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;
    desc.ArraySize = 1;
    desc.Width = width;
    desc.Height = height;
    desc.MipLevels = 1;
    desc.SampleCount = 1;
    desc.StaticImage = ovrTrue;
    desc.MiscFlags = ovrTextureMisc_None;
    desc.BindFlags = ovrTextureBind_None;

    if (ovr_CreateTextureSwapChainGL(session, &desc, &textureSwapChain) != ovrSuccess) {
        yFatal() << "Failed to create texture swap chain";
    }

    ovr_GetTextureSwapChainBufferGL(session, textureSwapChain, -1, &chainTexId);
    checkGlErrorMacro;

    glBindTexture(GL_TEXTURE_2D, chainTexId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, ptr);
    checkGlErrorMacro;

    glBindTexture(GL_TEXTURE_2D, 0);

    ovr_CommitTextureSwapChain(session, textureSwapChain);
    checkGlErrorMacro;
}

void TextureCrosshairs::deleteTexture()
{
    ovr_DestroyTextureSwapChain(session, textureSwapChain);
}
