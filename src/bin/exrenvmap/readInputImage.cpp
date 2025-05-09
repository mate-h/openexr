//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) Contributors to the OpenEXR Project.
//

//-----------------------------------------------------------------------------
//
//      function readInputImage() --
//      reads an image file and constructs an EnvMapImage object
//
//-----------------------------------------------------------------------------

#include <makeCubeMap.h>

#include "Iex.h"
#include "IexMacros.h"
#include <EnvmapImage.h>
#include <ImfRgbaFile.h>
#include <ImfStandardAttributes.h>
#include <iostream>
#include <string.h>
#include <string>
#include <cmath>  // For std::isfinite

#include "namespaceAlias.h"
using namespace IMF;
using namespace std;
using namespace IMATH;

namespace
{

void
sanitizePixelValues(Array2D<Rgba>& pixels, const Box2i& dataWindow, bool verbose = false)
{
    int w = dataWindow.max.x - dataWindow.min.x + 1;
    int h = dataWindow.max.y - dataWindow.min.y + 1;

    // Keep track of statistics for feedback
    int numNaNs = 0;
    int numInfs = 0;
    int numExtremePos = 0;
    int numExtremeNeg = 0;
    float maxValue = 0;
    
    // Maximum representable value without causing artifacts
    const float safeMaxValue = 1.0e30f;
    
    // Threshold for "extreme" values worth counting
    const float extremeThreshold = 1.0e15f;

    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            Rgba& p = pixels[y][x];
            
            // Convert to float for checking
            float r = float(p.r);
            float g = float(p.g);
            float b = float(p.b);
            float a = float(p.a);
            
            // Check for NaN values
            bool rNaN = r != r;
            bool gNaN = g != g;
            bool bNaN = b != b;
            bool aNaN = a != a;
            
            if (rNaN || gNaN || bNaN || aNaN) numNaNs++;
            
            // Check for infinity
            bool rInf = std::isinf(r);
            bool gInf = std::isinf(g);
            bool bInf = std::isinf(b);
            bool aInf = std::isinf(a);
            
            if (rInf || gInf || bInf || aInf) numInfs++;
            
            // Track max value for reporting
            maxValue = std::max(maxValue, std::max(std::max(r, g), b));
            
            // Count extreme values
            if (r > extremeThreshold || g > extremeThreshold || b > extremeThreshold)
                numExtremePos++;
            
            if (r < -extremeThreshold || g < -extremeThreshold || b < -extremeThreshold)
                numExtremeNeg++;
            
            // Apply sanitization
            if (rNaN || !std::isfinite(r) || r > safeMaxValue) 
                p.r = (r < 0) ? 0 : safeMaxValue;
            
            if (gNaN || !std::isfinite(g) || g > safeMaxValue) 
                p.g = (g < 0) ? 0 : safeMaxValue;
            
            if (bNaN || !std::isfinite(b) || b > safeMaxValue) 
                p.b = (b < 0) ? 0 : safeMaxValue;
            
            if (aNaN || !std::isfinite(a) || a > 1.0f) 
                p.a = 1.0f;
            
            if (a < 0.0f) 
                p.a = 0.0f;
        }
    }
    
    // Report statistics if interesting
    if (verbose && (numNaNs > 0 || numInfs > 0 || numExtremePos > 0 || numExtremeNeg > 0))
    {
        std::cout << "Image sanitization statistics:" << std::endl;
        std::cout << "  - NaN values found: " << numNaNs << std::endl;
        std::cout << "  - Infinity values found: " << numInfs << std::endl;
        std::cout << "  - Extreme positive values (>" << extremeThreshold << "): " << numExtremePos << std::endl;
        std::cout << "  - Extreme negative values (<-" << extremeThreshold << "): " << numExtremeNeg << std::endl;
        std::cout << "  - Maximum value found: " << maxValue << std::endl;
    }
}

void
readSingleImage (
    const char    inFileName[],
    float         padTop,
    float         padBottom,
    Envmap        overrideType,
    bool          verbose,
    EnvmapImage&  image,
    Header&       header,
    RgbaChannels& channels,
    EnvmapPixelType pixelType)
{
    //
    // Read the input image, and if necessary,
    // pad the image at the top and bottom.
    //

    RgbaInputFile in (inFileName);

    if (verbose) cout << "reading file " << inFileName << endl;

    header   = in.header ();
    channels = in.channels ();

    Envmap type = ENVMAP_LATLONG;

    if (overrideType == ENVMAP_LATLONG || overrideType == ENVMAP_CUBE)
    {
        type = overrideType;
        addEnvmap (header, overrideType);
    }
    else if (hasEnvmap (in.header ()))
    {
        // validate type is known before using
        const Envmap& typeInFile = envmap (in.header ());

        if (typeInFile != ENVMAP_LATLONG && typeInFile != ENVMAP_CUBE)
        {
            THROW (IEX::InputExc, "unknown envmap type " << int (typeInFile));
        }
        type = typeInFile;
    }

    const Box2i& dw = in.dataWindow ();
    int          w  = dw.max.x - dw.min.x + 1;
    int          h  = dw.max.y - dw.min.y + 1;

    int pt = 0;
    int pb = 0;

    if (type == ENVMAP_LATLONG)
    {
        pt = int (padTop * h + 0.5f);
        pb = int (padBottom * h + 0.5f);
    }

    Box2i paddedDw (
        V2i (dw.min.x, dw.min.y - pt), V2i (dw.max.x, dw.max.y + pb));

    image.resize (type, paddedDw, pixelType);
    Array2D<Rgba>& pixels = image.pixels ();

    in.setFrameBuffer (&pixels[-paddedDw.min.y][-paddedDw.min.x], 1, w);
    in.readPixels (dw.min.y, dw.max.y);

    // Clean up extreme values in the image immediately after reading
    if (verbose) cout << "cleaning up extreme pixel values" << endl;
    sanitizePixelValues(pixels, paddedDw, verbose);

    for (int y = 0; y < pt; ++y)
        for (int x = 0; x < w; ++x)
            pixels[y][x] = pixels[pt][x];

    for (int y = h + pt; y < h + pt + pb; ++y)
    {
        for (int x = 0; x < w; ++x)
            pixels[y][x] = pixels[h + pt - 1][x];
    }
}

void
readSixImages (
    const char    inFileName[],
    bool          verbose,
    EnvmapImage&  image,
    Header&       header,
    RgbaChannels& channels,
    EnvmapPixelType pixelType)
{
    //
    // Generate six file names by replacing the first '%' character in
    // inFileName with +X, -X, ... -Z.  Interpreting the corresponding
    // image files as the six sides of a cube, assemble a single cube-
    // face map image.
    //

    static const char* faceNames[] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};

    size_t pos  = strchr (inFileName, '%') - inFileName;
    string name = string (inFileName).replace (pos, 1, faceNames[0]);

    Box2i dw;
    int   w, h;

    {
        RgbaInputFile in (name.c_str ());

        if (verbose)
            cout << "reading cube face size from file " << name << endl;

        dw = in.dataWindow ();
        w  = dw.max.x - dw.min.x + 1;
        h  = dw.max.y - dw.min.y + 1;

        if (w != h)
        {
            THROW (
                IEX::InputExc, "Cube face image " << name << " is not square.");
        }

        header   = in.header ();
        channels = in.channels ();
        addEnvmap (header, ENVMAP_CUBE);
    }

    const Box2i imageDw (V2i (0, 0), V2i (w - 1, 6 * h - 1));

    image.resize (ENVMAP_CUBE, imageDw, pixelType);
    Rgba* pixels = &(image.pixels ()[0][0]);

    for (int i = 0; i < 6; ++i)
    {
        string name = string (inFileName).replace (pos, 1, faceNames[i]);

        RgbaInputFile in (name.c_str ());

        if (verbose) cout << "reading file " << name << endl;

        if (in.dataWindow () != dw)
        {
            THROW (
                IEX::InputExc,
                "The data window of cube face "
                    << name
                    << " differs "
                       "from the data window of other cube faces.");
        }

        in.setFrameBuffer (ComputeBasePointer (pixels, dw), 1, w);
        in.readPixels (dw.min.y, dw.max.y);
        
        // Clean up extreme values for each face
        if (verbose) cout << "cleaning up extreme pixel values in face " << i << endl;
        Box2i faceDw(V2i(0, i*h), V2i(w-1, (i+1)*h-1));
        sanitizePixelValues(image.pixels(), faceDw, verbose);

        pixels += w * h;
    }
}

} // namespace

void
readInputImage (
    const char    inFileName[],
    float         padTop,
    float         padBottom,
    Envmap        overrideType,
    bool          verbose,
    EnvmapImage&  image,
    Header&       header,
    RgbaChannels& channels,
    EnvmapPixelType pixelType)
{
    if (strchr (inFileName, '%'))
    {
        readSixImages (inFileName, verbose, image, header, channels, pixelType);
    }
    else
    {
        readSingleImage (
            inFileName,
            padTop,
            padBottom,
            overrideType,
            verbose,
            image,
            header,
            channels,
            pixelType);
    }
}
