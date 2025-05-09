//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) Contributors to the OpenEXR Project.
//

#ifndef INCLUDED_READ_INPUT_IMAGE_H
#define INCLUDED_READ_INPUT_IMAGE_H

//-----------------------------------------------------------------------------
//
//      function readInputImage() --
//      reads an image file and constructs an EnvMapImage object
//
//-----------------------------------------------------------------------------

#include <ImfEnvmap.h>
#include <ImfForward.h>
#include <ImfRgba.h>
#include <EnvmapImage.h>

#include "namespaceAlias.h"

void readInputImage (
    const char         inFileName[],
    float              padTop,
    float              padBottom,
    IMF::Envmap        overrideType,
    bool               verbose,
    EnvmapImage&       image,
    IMF::Header&       header,
    IMF::RgbaChannels& channels,
    EnvmapPixelType    pixelType);

#endif
