//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) Contributors to the OpenEXR Project.
//

//-----------------------------------------------------------------------------
//
//	class EnvmapImage
//
//-----------------------------------------------------------------------------

#include "EnvmapImage.h"
#include <ImathFun.h>

#include "namespaceAlias.h"
using namespace IMF;
using namespace IMATH;

EnvmapImage::EnvmapImage ()
    : _type (ENVMAP_LATLONG)
    , _pixelType (ENVMAP_HALF)
    , _dataWindow (V2i (0, 0), V2i (0, 0))
    , _pixels (1, 1)
{
    clear ();
}

EnvmapImage::EnvmapImage (Envmap type, const Box2i& dataWindow, EnvmapPixelType pixelType)
    : _type (type)
    , _pixelType (pixelType)
    , _dataWindow (dataWindow)
    , _pixels (
          dataWindow.max.y - dataWindow.min.y + 1,
          dataWindow.max.x - dataWindow.min.x + 1)
{
    clear ();
}

void
EnvmapImage::resize (Envmap type, const Box2i& dataWindow, EnvmapPixelType pixelType)
{
    _pixels.resizeEraseUnsafe (
        dataWindow.max.y - dataWindow.min.y + 1,
        dataWindow.max.x - dataWindow.min.x + 1);
    _type       = type;
    _pixelType  = pixelType;
    _dataWindow = dataWindow;

    clear ();
}

void
EnvmapImage::clear ()
{
    int w = _dataWindow.max.x - _dataWindow.min.x + 1;
    int h = _dataWindow.max.y - _dataWindow.min.y + 1;

    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            Rgba& p = _pixels[y][x];

            p.r = 0;
            p.g = 0;
            p.b = 0;
            p.a = 0;
        }
    }
}

Envmap
EnvmapImage::type () const
{
    return _type;
}

EnvmapPixelType
EnvmapImage::pixelType () const
{
    return _pixelType;
}

const Box2i&
EnvmapImage::dataWindow () const
{
    return _dataWindow;
}

Array2D<Rgba>&
EnvmapImage::pixels ()
{
    return _pixels;
}

const Array2D<Rgba>&
EnvmapImage::pixels () const
{
    return _pixels;
}

namespace
{

V2f
dirToPosLatLong (const Box2i& dataWindow, const V3f& dir)
{
    return LatLongMap::pixelPosition (dataWindow, dir);
}

V2f
dirToPosCube (const Box2i& dataWindow, const V3f& dir)
{
    CubeMapFace face;
    V2f         posInFace;
    CubeMap::faceAndPixelPosition (dir, dataWindow, face, posInFace);
    return CubeMap::pixelPosition (face, dataWindow, posInFace);
}

} // namespace

Rgba
EnvmapImage::filteredLookup (V3f d, float r, int n) const
{
    //
    // Filtered environment map lookup: Take n by n point samples
    // from the environment map, clustered around direction d, and
    // combine the samples with a tent filter.
    //

    //
    // Depending on the type of map, pick an appropriate function
    // to convert 3D directions to 2D pixel poitions.
    //

    V2f (*dirToPos) (const Box2i&, const V3f&);

    if (_type == ENVMAP_LATLONG)
        dirToPos = dirToPosLatLong;
    else
        dirToPos = dirToPosCube;

    //
    // Pick two vectors, dx and dy, of length r, that are orthogonal
    // to the lookup direction, d, and to each other.
    //

    d.normalize ();
    V3f dx, dy;

    if (abs (d.x) > 0.707f)
        dx = (d % V3f (0, 1, 0)).normalized () * r;
    else
        dx = (d % V3f (1, 0, 0)).normalized () * r;

    dy = (d % dx).normalized () * r;

    //
    // Take n by n point samples from the map, and add them up.
    // The directions for the point samples are all within the pyramid
    // defined by the vectors d-dy-dx, d-dy+dx, d+dy-dx, d+dy+dx.
    //

    float wt = 0;

    float cr = 0;
    float cg = 0;
    float cb = 0;
    float ca = 0;

    for (int y = 0; y < n; ++y)
    {
        float ry = float (2 * y + 2) / float (n + 1) - 1;
        float wy = 1 - abs (ry);
        V3f   ddy (ry * dy);

        for (int x = 0; x < n; ++x)
        {
            float rx = float (2 * x + 2) / float (n + 1) - 1;
            float wx = 1 - abs (rx);
            V3f   ddx (rx * dx);

            Rgba s = sample (dirToPos (_dataWindow, d + ddx + ddy));

            float w = wx * wy;
            wt += w;

            // Check for NaN or extreme values before accumulating
            float r = float(s.r);
            float g = float(s.g);
            float b = float(s.b);
            float a = float(s.a);
            
            // Replace problematic values with a large but finite value
            if (r != r || r > 1.0e30f || r < -1.0e30f) r = 1.0e30f;
            if (g != g || g > 1.0e30f || g < -1.0e30f) g = 1.0e30f;
            if (b != b || b > 1.0e30f || b < -1.0e30f) b = 1.0e30f;
            if (a != a || a > 1.0f || a < 0.0f) a = 1.0f;

            cr += r * w;
            cg += g * w;
            cb += b * w;
            ca += a * w;
        }
    }

    wt = 1 / wt;

    Rgba c;

    c.r = cr * wt;
    c.g = cg * wt;
    c.b = cb * wt;
    c.a = ca * wt;

    return c;
}

Rgba
EnvmapImage::sample (const V2f& pos) const
{
    //
    // Point-sample the environment map image at 2D position pos.
    // Interpolate bilinearly between the four nearest pixels.
    //

    int   x1 = IMATH::floor (pos.x);
    int   x2 = x1 + 1;
    float sx = x2 - pos.x;
    float tx = 1 - sx;

    x1 = clamp (x1, _dataWindow.min.x, _dataWindow.max.x) - _dataWindow.min.x;
    x2 = clamp (x2, _dataWindow.min.x, _dataWindow.max.x) - _dataWindow.min.x;

    int   y1 = IMATH::floor (pos.y);
    int   y2 = y1 + 1;
    float sy = y2 - pos.y;
    float ty = 1 - sy;

    y1 = clamp (y1, _dataWindow.min.y, _dataWindow.max.y) - _dataWindow.min.y;
    y2 = clamp (y2, _dataWindow.min.y, _dataWindow.max.y) - _dataWindow.min.y;

    Rgba p11 = _pixels[y1][x1];
    Rgba p12 = _pixels[y1][x2];
    Rgba p21 = _pixels[y2][x1];
    Rgba p22 = _pixels[y2][x2];

    // Check and clean input values before interpolation to avoid introducing artifacts
    const float maxValue = 1.0e30f;
    
    // Clean p11
    float r11 = float(p11.r);
    float g11 = float(p11.g);
    float b11 = float(p11.b);
    float a11 = float(p11.a);
    if (r11 != r11 || !std::isfinite(r11) || r11 > maxValue) r11 = maxValue;
    if (g11 != g11 || !std::isfinite(g11) || g11 > maxValue) g11 = maxValue;
    if (b11 != b11 || !std::isfinite(b11) || b11 > maxValue) b11 = maxValue;
    if (a11 != a11 || !std::isfinite(a11) || a11 > 1.0f) a11 = 1.0f;
    if (a11 < 0.0f) a11 = 0.0f;
    
    // Clean p12
    float r12 = float(p12.r);
    float g12 = float(p12.g);
    float b12 = float(p12.b);
    float a12 = float(p12.a);
    if (r12 != r12 || !std::isfinite(r12) || r12 > maxValue) r12 = maxValue;
    if (g12 != g12 || !std::isfinite(g12) || g12 > maxValue) g12 = maxValue;
    if (b12 != b12 || !std::isfinite(b12) || b12 > maxValue) b12 = maxValue;
    if (a12 != a12 || !std::isfinite(a12) || a12 > 1.0f) a12 = 1.0f;
    if (a12 < 0.0f) a12 = 0.0f;
    
    // Clean p21
    float r21 = float(p21.r);
    float g21 = float(p21.g);
    float b21 = float(p21.b);
    float a21 = float(p21.a);
    if (r21 != r21 || !std::isfinite(r21) || r21 > maxValue) r21 = maxValue;
    if (g21 != g21 || !std::isfinite(g21) || g21 > maxValue) g21 = maxValue;
    if (b21 != b21 || !std::isfinite(b21) || b21 > maxValue) b21 = maxValue;
    if (a21 != a21 || !std::isfinite(a21) || a21 > 1.0f) a21 = 1.0f;
    if (a21 < 0.0f) a21 = 0.0f;
    
    // Clean p22
    float r22 = float(p22.r);
    float g22 = float(p22.g);
    float b22 = float(p22.b);
    float a22 = float(p22.a);
    if (r22 != r22 || !std::isfinite(r22) || r22 > maxValue) r22 = maxValue;
    if (g22 != g22 || !std::isfinite(g22) || g22 > maxValue) g22 = maxValue;
    if (b22 != b22 || !std::isfinite(b22) || b22 > maxValue) b22 = maxValue;
    if (a22 != a22 || !std::isfinite(a22) || a22 > 1.0f) a22 = 1.0f;
    if (a22 < 0.0f) a22 = 0.0f;

    // Perform bilinear interpolation using the cleaned values
    Rgba p;
    p.r = (r11 * sx + r12 * tx) * sy + (r21 * sx + r22 * tx) * ty;
    p.g = (g11 * sx + g12 * tx) * sy + (g21 * sx + g22 * tx) * ty;
    p.b = (b11 * sx + b12 * tx) * sy + (b21 * sx + b22 * tx) * ty;
    p.a = (a11 * sx + a12 * tx) * sy + (a21 * sx + a22 * tx) * ty;

    // Final validation check on the result
    float r = float(p.r);
    float g = float(p.g);
    float b = float(p.b);
    float a = float(p.a);
    
    // Replace problematic values with a large but finite value
    if (r != r || !std::isfinite(r) || r > maxValue) p.r = maxValue;
    if (g != g || !std::isfinite(g) || g > maxValue) p.g = maxValue;
    if (b != b || !std::isfinite(b) || b > maxValue) p.b = maxValue;
    if (a != a || !std::isfinite(a) || a > 1.0f) p.a = 1.0f;
    if (a < 0.0f) p.a = 0.0f;

    return p;
}
