//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) Contributors to the OpenEXR Project.
//

//-----------------------------------------------------------------------------
//
//	function blurImage() -- performs a hemispherical blur
//
//-----------------------------------------------------------------------------

#include "blurImage.h"

#include "namespaceAlias.h"

#include "Iex.h"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <resizeImage.h>
#include <string.h>

using namespace IMF;
using namespace std;
using namespace IMATH;

namespace {
// Add a helper function to sanitize pixel values
inline float sanitizeValue(float x, float maxValue = 1.0e30f) {
    if (x != x || !std::isfinite(x)) return 0.0f; // Handle NaN and infinities
    if (x > maxValue) return maxValue;            // Clamp very high values
    if (x < -maxValue) return -maxValue;          // Clamp very low values
    return x;
}
} // namespace

inline int
toInt (float x)
{
    return int (x + 0.5f);
}

inline double
sqr (double x)
{
    return x * x;
}

void
blurImage (EnvmapImage& image1, bool verbose)
{
    //
    // Ideally we would blur the input image directly by convolving
    // it with a 180-degree wide blur kernel.  Unfortunately this
    // is prohibitively expensive when the input image is large.
    // In order to keep running times reasonable, we perform the
    // blur on a small proxy image that will later be re-sampled
    // to the desired output resolution.
    //
    // Here's how it works:
    //
    // * If the input image is in latitude-longitude format,
    //   convert it into a cube-face environment map.
    //
    // * Repeatedly resample the image, each time shrinking
    //   it to no less than half its current size, until the
    //   width of each cube face is MAX_IN_WIDTH pixels.
    //
    // * Multiply each pixel by a weight that is proportional
    //   to the solid angle subtended by the pixel as seen
    //   from the center of the environment cube.
    //
    // * Create an output image in cube-face format.
    //   The cube faces of the output image are OUT_WIDTH
    //   pixels wide.
    //
    // * For each pixel of the output image:
    //
    //       Set the output pixel's color to black
    //
    //       Determine the direction, d2, from the center of the
    //       output environment cube to the center of the output
    //	     pixel.
    //
    //       For each pixel of the input image:
    //
    //           Determine the direction, d1, from the center of
    //           the input environment cube to the center of the
    //           input pixel.
    //
    //           Multiply the input pixel's color by max (0, d1.dot(d2))
    //           and add the result to the output pixel.
    //

    const int MAX_IN_WIDTH = 40;
    const int OUT_WIDTH    = 100;

    if (verbose) cout << "blurring map image" << endl;

    EnvmapImage  image2;
    EnvmapImage* iptr1 = &image1;
    EnvmapImage* iptr2 = &image2;

    int w = image1.dataWindow ().max.x - image1.dataWindow ().min.x + 1;
    int h = w * 6;

    if (iptr1->type () == ENVMAP_LATLONG)
    {
        //
        // Convert the input image from latitude-longitude
        // to cube-face format.
        //

        if (verbose) cout << "    converting to cube-face format" << endl;

        w /= 4;
        h = w * 6;

        Box2i dw (V2i (0, 0), V2i (w - 1, h - 1));
        resizeCube (*iptr1, *iptr2, dw, 1, 7);

        swap (iptr1, iptr2);
    }

    while (w > MAX_IN_WIDTH)
    {
        //
        // Shrink the image.
        //

        if (w >= MAX_IN_WIDTH * 2)
            w /= 2;
        else
            w = MAX_IN_WIDTH;

        h = w * 6;

        if (verbose)
        {
            cout << "    resizing cube faces "
                    "to "
                 << w << " by " << w << " pixels" << endl;
        }

        Box2i dw (V2i (0, 0), V2i (w - 1, h - 1));
        resizeCube (*iptr1, *iptr2, dw, 1, 7);

        swap (iptr1, iptr2);
    }

    if (verbose) cout << "    computing pixel weights" << endl;

    {
        //
        // Multiply each pixel by a weight that is proportional
        // to the solid angle subtended by the pixel.
        //

        Box2i dw  = iptr1->dataWindow ();
        int   sof = CubeMap::sizeOfFace (dw);

        Array2D<Rgba>& pixels = iptr1->pixels ();

        double weightTotal = 0;

        for (int f = CUBEFACE_POS_X; f <= CUBEFACE_NEG_Z; ++f)
        {
            if (verbose) cout << "        face " << f << endl;

            CubeMapFace face = CubeMapFace (f);
            V3f         faceDir (0, 0, 0);
            int         ix = 0, iy = 0, iz = 0;

            switch (face)
            {
                case CUBEFACE_POS_X:
                    faceDir = V3f (1, 0, 0);
                    ix      = 0;
                    iy      = 1;
                    iz      = 2;
                    break;

                case CUBEFACE_NEG_X:
                    faceDir = V3f (-1, 0, 0);
                    ix      = 0;
                    iy      = 1;
                    iz      = 2;
                    break;

                case CUBEFACE_POS_Y:
                    faceDir = V3f (0, 1, 0);
                    ix      = 1;
                    iy      = 0;
                    iz      = 2;
                    break;

                case CUBEFACE_NEG_Y:
                    faceDir = V3f (0, -1, 0);
                    ix      = 1;
                    iy      = 0;
                    iz      = 2;
                    break;

                case CUBEFACE_POS_Z:
                    faceDir = V3f (0, 0, 1);
                    ix      = 2;
                    iy      = 0;
                    iz      = 1;
                    break;

                case CUBEFACE_NEG_Z:
                    faceDir = V3f (0, 0, -1);
                    ix      = 2;
                    iy      = 0;
                    iz      = 1;
                    break;
            }

            for (int y = 0; y < sof; ++y)
            {
                bool yEdge = (y == 0 || y == sof - 1);

                for (int x = 0; x < sof; ++x)
                {
                    bool xEdge = (x == 0 || x == sof - 1);

                    V2f posInFace (x, y);

                    V3f dir =
                        CubeMap::direction (face, dw, posInFace).normalized ();

                    V2f pos = CubeMap::pixelPosition (face, dw, posInFace);

                    //
                    // The solid angle subtended by pixel (x,y), as seen
                    // from the center of the cube, is proportional to the
                    // square of the distance of the pixel from the center
                    // of the cube and proportional to the dot product of
                    // the viewing direction and the normal of the cube
                    // face that contains the pixel.
                    //

                    double weight =
                        (dir ^ faceDir) *
                        (sqr (dir[iy] / dir[ix]) + sqr (dir[iz] / dir[ix]) + 1);

                    //
                    // Pixels at the edges and corners of the
                    // cube are duplicated; we must adjust the
                    // pixel weights accordingly.
                    //

                    if (xEdge && yEdge)
                        weight /= 3;
                    else if (xEdge || yEdge)
                        weight /= 2;

                    Rgba& pixel = pixels[toInt (pos.y)][toInt (pos.x)];

                    // Apply weight
                    float r = float(pixel.r) * weight;
                    float g = float(pixel.g) * weight;
                    float b = float(pixel.b) * weight;
                    float a = float(pixel.a) * weight;
                    
                    // Check for NaN or extreme values
                    r = sanitizeValue(r);
                    g = sanitizeValue(g);
                    b = sanitizeValue(b);
                    a = sanitizeValue(a, 1.0f);
                    
                    pixel.r = r;
                    pixel.g = g;
                    pixel.b = b;
                    pixel.a = a;

                    weightTotal += weight;
                }
            }
        }

        //
        // The weighting operation above has made the overall image darker.
        // Apply a correction to recover the image's original brightness.
        //

        int    w         = dw.max.x - dw.min.x + 1;
        int    h         = dw.max.y - dw.min.y + 1;
        size_t numPixels = w * h;
        double weight    = numPixels / weightTotal;

        Rgba* p   = &pixels[0][0];
        Rgba* end = p + numPixels;

        while (p < end)
        {
            p->r *= weight;
            p->g *= weight;
            p->b *= weight;
            p->a *= weight;

            ++p;
        }
    }

    {
        if (verbose) cout << "    generating blurred image" << endl;

        Box2i dw1  = iptr1->dataWindow ();
        int   sof1 = CubeMap::sizeOfFace (dw1);

        Box2i dw2 (V2i (0, 0), V2i (OUT_WIDTH - 1, OUT_WIDTH * 6 - 1));
        int   sof2 = CubeMap::sizeOfFace (dw2);

        iptr2->resize (ENVMAP_CUBE, dw2);
        iptr2->clear ();

        Array2D<Rgba>& pixels1 = iptr1->pixels ();
        Array2D<Rgba>& pixels2 = iptr2->pixels ();

        for (int f2 = CUBEFACE_POS_X; f2 <= CUBEFACE_NEG_Z; ++f2)
        {
            if (verbose) cout << "        face " << f2 << endl;

            CubeMapFace face2 = CubeMapFace (f2);

            for (int y2 = 0; y2 < sof2; ++y2)
            {
                for (int x2 = 0; x2 < sof2; ++x2)
                {
                    V2f posInFace2 (x2, y2);

                    V3f dir2 = CubeMap::direction (face2, dw2, posInFace2);

                    V2f pos2 = CubeMap::pixelPosition (face2, dw2, posInFace2);

                    double weightTotal = 0;
                    double rTotal      = 0;
                    double gTotal      = 0;
                    double bTotal      = 0;
                    double aTotal      = 0;

                    Rgba& pixel2 = pixels2[toInt (pos2.y)][toInt (pos2.x)];

                    for (int f1 = CUBEFACE_POS_X; f1 <= CUBEFACE_NEG_Z; ++f1)
                    {
                        CubeMapFace face1 = CubeMapFace (f1);

                        for (int y1 = 0; y1 < sof1; ++y1)
                        {
                            for (int x1 = 0; x1 < sof1; ++x1)
                            {
                                V2f posInFace1 (x1, y1);

                                V3f dir1 =
                                    CubeMap::direction (face1, dw1, posInFace1);

                                V2f pos1 = CubeMap::pixelPosition (
                                    face1, dw1, posInFace1);

                                double weight = dir1 ^ dir2;

                                if (weight <= 0) continue;

                                Rgba& pixel1 =
                                    pixels1[toInt (pos1.y)][toInt (pos1.x)];

                                // Get float values and sanitize
                                float r1 = sanitizeValue(float(pixel1.r));
                                float g1 = sanitizeValue(float(pixel1.g));
                                float b1 = sanitizeValue(float(pixel1.b));
                                float a1 = sanitizeValue(float(pixel1.a), 1.0f);

                                weightTotal += weight;
                                rTotal += r1 * weight;
                                gTotal += g1 * weight;
                                bTotal += b1 * weight;
                                aTotal += a1 * weight;
                            }
                        }
                    }

                    // Before setting the final values, check for NaN/extreme values
                    if (rTotal != rTotal || rTotal > 1.0e30 || rTotal < -1.0e30) rTotal = 0;
                    if (gTotal != gTotal || gTotal > 1.0e30 || gTotal < -1.0e30) gTotal = 0;
                    if (bTotal != bTotal || bTotal > 1.0e30 || bTotal < -1.0e30) bTotal = 0;
                    if (aTotal != aTotal || aTotal > 1.0e30 || aTotal < -1.0e30) aTotal = 0;
                    
                    if (weightTotal > 0)
                    {
                        pixel2.r = rTotal / weightTotal;
                        pixel2.g = gTotal / weightTotal;
                        pixel2.b = bTotal / weightTotal;
                        pixel2.a = aTotal / weightTotal;
                    }
                    else
                    {
                        pixel2.r = 0;
                        pixel2.g = 0;
                        pixel2.b = 0;
                        pixel2.a = 0;
                    }
                }
            }
        }

        swap (iptr1, iptr2);
    }

    //
    // Depending on how many times we've re-sampled the image,
    // the result is now either in image1 or in image2.
    // If necessary, copy the result into image1.
    //

    if (iptr1 != &image1)
    {
        if (verbose) cout << "    copying" << endl;

        Box2i dw = iptr1->dataWindow ();
        image1.resize (ENVMAP_CUBE, dw);

        int    w    = dw.max.x - dw.min.x + 1;
        int    h    = dw.max.y - dw.min.y + 1;
        size_t size = w * h * sizeof (Rgba);

        memcpy (&image1.pixels ()[0][0], &iptr1->pixels ()[0][0], size);
    }
}
