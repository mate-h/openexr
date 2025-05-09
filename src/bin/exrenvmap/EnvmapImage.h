//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) Contributors to the OpenEXR Project.
//

#ifndef INCLUDED_ENVMAP_IMAGE_H
#define INCLUDED_ENVMAP_IMAGE_H

//-----------------------------------------------------------------------------
//
//        class EnvmapImage
//
//-----------------------------------------------------------------------------

#include "namespaceAlias.h"

#include <ImathBox.h>
#include <ImfArray.h>
#include <ImfEnvmap.h>
#include <ImfRgba.h>

// Pixel type for the environment map
enum EnvmapPixelType
{
    ENVMAP_HALF = 0,  // 16-bit half float (default)
    ENVMAP_FLOAT = 1  // 32-bit float
};

class EnvmapImage
{
public:
    EnvmapImage ();
    EnvmapImage (IMF::Envmap type, const IMATH::Box2i& dataWindow, EnvmapPixelType pixelType = ENVMAP_HALF);

    void resize (IMF::Envmap type, const IMATH::Box2i& dataWindow, EnvmapPixelType pixelType = ENVMAP_HALF);

    void clear ();

    IMF::Envmap         type () const;
    EnvmapPixelType     pixelType() const;
    const IMATH::Box2i& dataWindow () const;

    IMF::Array2D<IMF::Rgba>&       pixels ();
    const IMF::Array2D<IMF::Rgba>& pixels () const;

    IMF::Rgba
    filteredLookup (IMATH::V3f direction, float radius, int numSamples) const;

private:
    IMF::Rgba sample (const IMATH::V2f& pos) const;

    IMF::Envmap             _type;
    EnvmapPixelType         _pixelType;
    IMATH::Box2i            _dataWindow;
    IMF::Array2D<IMF::Rgba> _pixels;
};

#endif
