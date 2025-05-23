//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) Contributors to the OpenEXR Project.
//

//-----------------------------------------------------------------------------
//
//	Conversion between RGBA and YCA data.
//
//-----------------------------------------------------------------------------

#include <ImfRgbaYca.h>
#include <algorithm>
#include <assert.h>

using namespace IMATH_NAMESPACE;
using namespace std;
#include "ImfNamespace.h"

OPENEXR_IMF_INTERNAL_NAMESPACE_SOURCE_ENTER

namespace RgbaYca
{

V3f
computeYw (const Chromaticities& cr)
{
    M44f m = RGBtoXYZ (cr, 1);
    return V3f (m[0][1], m[1][1], m[2][1]) / (m[0][1] + m[1][1] + m[2][1]);
}

void
RGBAtoYCA (
    const V3f& yw,
    int        n,
    bool       aIsValid,
    const Rgba rgbaIn[/*n*/],
    Rgba       ycaOut[/*n*/])
{
    for (int i = 0; i < n; ++i)
    {
        Rgba  in  = rgbaIn[i];
        Rgba& out = ycaOut[i];

        //
        // Conversion to YCA and subsequent chroma subsampling
        // work only if R, G and B are finite and non-negative.
        //

        if (!in.r.isFinite () || in.r < 0) in.r = 0;

        if (!in.g.isFinite () || in.g < 0) in.g = 0;

        if (!in.b.isFinite () || in.b < 0) in.b = 0;

        if (in.r == in.g && in.g == in.b)
        {
            //
            // Special case -- R, G and B are equal. To avoid rounding
            // errors, we explicitly set the output luminance channel
            // to G, and the chroma channels to 0.
            //
            // The special cases here and in YCAtoRGBA() ensure that
            // converting black-and white images from RGBA to YCA and
            // back is lossless.
            //

            out.r = 0;
            out.g = in.g;
            out.b = 0;
        }
        else
        {
            out.g = in.r * yw.x + in.g * yw.y + in.b * yw.z;

            float Y = out.g;

            if (abs (in.r - Y) < HALF_MAX * Y)
                out.r = (in.r - Y) / Y;
            else
                out.r = 0;

            if (abs (in.b - Y) < HALF_MAX * Y)
                out.b = (in.b - Y) / Y;
            else
                out.b = 0;
        }

        if (aIsValid)
            out.a = in.a;
        else
            out.a = 1;
    }
}

void
decimateChromaHoriz (int n, const Rgba ycaIn[/*n+N-1*/], Rgba ycaOut[/*n*/])
{
#ifdef DEBUG
    assert (ycaIn != ycaOut);
#endif

    int begin = N2;
    int end   = begin + n;

    for (int i = begin, j = 0; i < end; ++i, ++j)
    {
        if ((j & 1) == 0)
        {
            ycaOut[j].r =
                ycaIn[i - 13].r * 0.001064f + ycaIn[i - 11].r * -0.003771f +
                ycaIn[i - 9].r * 0.009801f + ycaIn[i - 7].r * -0.021586f +
                ycaIn[i - 5].r * 0.043978f + ycaIn[i - 3].r * -0.093067f +
                ycaIn[i - 1].r * 0.313659f + ycaIn[i].r * 0.499846f +
                ycaIn[i + 1].r * 0.313659f + ycaIn[i + 3].r * -0.093067f +
                ycaIn[i + 5].r * 0.043978f + ycaIn[i + 7].r * -0.021586f +
                ycaIn[i + 9].r * 0.009801f + ycaIn[i + 11].r * -0.003771f +
                ycaIn[i + 13].r * 0.001064f;

            ycaOut[j].b =
                ycaIn[i - 13].b * 0.001064f + ycaIn[i - 11].b * -0.003771f +
                ycaIn[i - 9].b * 0.009801f + ycaIn[i - 7].b * -0.021586f +
                ycaIn[i - 5].b * 0.043978f + ycaIn[i - 3].b * -0.093067f +
                ycaIn[i - 1].b * 0.313659f + ycaIn[i].b * 0.499846f +
                ycaIn[i + 1].b * 0.313659f + ycaIn[i + 3].b * -0.093067f +
                ycaIn[i + 5].b * 0.043978f + ycaIn[i + 7].b * -0.021586f +
                ycaIn[i + 9].b * 0.009801f + ycaIn[i + 11].b * -0.003771f +
                ycaIn[i + 13].b * 0.001064f;
        }

        ycaOut[j].g = ycaIn[i].g;
        ycaOut[j].a = ycaIn[i].a;
    }
}

void
decimateChromaVert (int n, const Rgba* const ycaIn[N], Rgba ycaOut[/*n*/])
{
    for (int i = 0; i < n; ++i)
    {
        if ((i & 1) == 0)
        {
            ycaOut[i].r =
                ycaIn[0][i].r * 0.001064f + ycaIn[2][i].r * -0.003771f +
                ycaIn[4][i].r * 0.009801f + ycaIn[6][i].r * -0.021586f +
                ycaIn[8][i].r * 0.043978f + ycaIn[10][i].r * -0.093067f +
                ycaIn[12][i].r * 0.313659f + ycaIn[13][i].r * 0.499846f +
                ycaIn[14][i].r * 0.313659f + ycaIn[16][i].r * -0.093067f +
                ycaIn[18][i].r * 0.043978f + ycaIn[20][i].r * -0.021586f +
                ycaIn[22][i].r * 0.009801f + ycaIn[24][i].r * -0.003771f +
                ycaIn[26][i].r * 0.001064f;

            ycaOut[i].b =
                ycaIn[0][i].b * 0.001064f + ycaIn[2][i].b * -0.003771f +
                ycaIn[4][i].b * 0.009801f + ycaIn[6][i].b * -0.021586f +
                ycaIn[8][i].b * 0.043978f + ycaIn[10][i].b * -0.093067f +
                ycaIn[12][i].b * 0.313659f + ycaIn[13][i].b * 0.499846f +
                ycaIn[14][i].b * 0.313659f + ycaIn[16][i].b * -0.093067f +
                ycaIn[18][i].b * 0.043978f + ycaIn[20][i].b * -0.021586f +
                ycaIn[22][i].b * 0.009801f + ycaIn[24][i].b * -0.003771f +
                ycaIn[26][i].b * 0.001064f;
        }

        ycaOut[i].g = ycaIn[13][i].g;
        ycaOut[i].a = ycaIn[13][i].a;
    }
}

void
roundYCA (
    int          n,
    unsigned int roundY,
    unsigned int roundC,
    const Rgba   ycaIn[/*n*/],
    Rgba         ycaOut[/*n*/])
{
    for (int i = 0; i < n; ++i)
    {
        ycaOut[i].g = ycaIn[i].g.round (roundY);
        ycaOut[i].a = ycaIn[i].a;

        if ((i & 1) == 0)
        {
            ycaOut[i].r = ycaIn[i].r.round (roundC);
            ycaOut[i].b = ycaIn[i].b.round (roundC);
        }
    }
}

void
reconstructChromaHoriz (int n, const Rgba ycaIn[/*n+N-1*/], Rgba ycaOut[/*n*/])
{
#ifdef DEBUG
    assert (ycaIn != ycaOut);
#endif

    int begin = N2;
    int end   = begin + n;

    for (int i = begin, j = 0; i < end; ++i, ++j)
    {
        if (j & 1)
        {
            ycaOut[j].r =
                ycaIn[i - 13].r * 0.002128f + ycaIn[i - 11].r * -0.007540f +
                ycaIn[i - 9].r * 0.019597f + ycaIn[i - 7].r * -0.043159f +
                ycaIn[i - 5].r * 0.087929f + ycaIn[i - 3].r * -0.186077f +
                ycaIn[i - 1].r * 0.627123f + ycaIn[i + 1].r * 0.627123f +
                ycaIn[i + 3].r * -0.186077f + ycaIn[i + 5].r * 0.087929f +
                ycaIn[i + 7].r * -0.043159f + ycaIn[i + 9].r * 0.019597f +
                ycaIn[i + 11].r * -0.007540f + ycaIn[i + 13].r * 0.002128f;

            ycaOut[j].b =
                ycaIn[i - 13].b * 0.002128f + ycaIn[i - 11].b * -0.007540f +
                ycaIn[i - 9].b * 0.019597f + ycaIn[i - 7].b * -0.043159f +
                ycaIn[i - 5].b * 0.087929f + ycaIn[i - 3].b * -0.186077f +
                ycaIn[i - 1].b * 0.627123f + ycaIn[i + 1].b * 0.627123f +
                ycaIn[i + 3].b * -0.186077f + ycaIn[i + 5].b * 0.087929f +
                ycaIn[i + 7].b * -0.043159f + ycaIn[i + 9].b * 0.019597f +
                ycaIn[i + 11].b * -0.007540f + ycaIn[i + 13].b * 0.002128f;
        }
        else
        {
            ycaOut[j].r = ycaIn[i].r;
            ycaOut[j].b = ycaIn[i].b;
        }

        ycaOut[j].g = ycaIn[i].g;
        ycaOut[j].a = ycaIn[i].a;
    }
}

void
reconstructChromaVert (int n, const Rgba* const ycaIn[N], Rgba ycaOut[/*n*/])
{
    for (int i = 0; i < n; ++i)
    {
        ycaOut[i].r = ycaIn[0][i].r * 0.002128f + ycaIn[2][i].r * -0.007540f +
                      ycaIn[4][i].r * 0.019597f + ycaIn[6][i].r * -0.043159f +
                      ycaIn[8][i].r * 0.087929f + ycaIn[10][i].r * -0.186077f +
                      ycaIn[12][i].r * 0.627123f + ycaIn[14][i].r * 0.627123f +
                      ycaIn[16][i].r * -0.186077f + ycaIn[18][i].r * 0.087929f +
                      ycaIn[20][i].r * -0.043159f + ycaIn[22][i].r * 0.019597f +
                      ycaIn[24][i].r * -0.007540f + ycaIn[26][i].r * 0.002128f;

        ycaOut[i].b = ycaIn[0][i].b * 0.002128f + ycaIn[2][i].b * -0.007540f +
                      ycaIn[4][i].b * 0.019597f + ycaIn[6][i].b * -0.043159f +
                      ycaIn[8][i].b * 0.087929f + ycaIn[10][i].b * -0.186077f +
                      ycaIn[12][i].b * 0.627123f + ycaIn[14][i].b * 0.627123f +
                      ycaIn[16][i].b * -0.186077f + ycaIn[18][i].b * 0.087929f +
                      ycaIn[20][i].b * -0.043159f + ycaIn[22][i].b * 0.019597f +
                      ycaIn[24][i].b * -0.007540f + ycaIn[26][i].b * 0.002128f;

        ycaOut[i].g = ycaIn[13][i].g;
        ycaOut[i].a = ycaIn[13][i].a;
    }
}

void
YCAtoRGBA (
    const IMATH_NAMESPACE::V3f& yw,
    int                         n,
    const Rgba                  ycaIn[/*n*/],
    Rgba                        rgbaOut[/*n*/])
{
    for (int i = 0; i < n; ++i)
    {
        const Rgba& in  = ycaIn[i];
        Rgba&       out = rgbaOut[i];

        // Ensure input Y (luminance) is valid and non-negative
        float Y = float(in.g);
        if (!std::isfinite(Y) || Y < 0) Y = 0.0f;
        
        if (in.r == 0 && in.b == 0)
        {
            //
            // Special case -- both chroma channels are 0.  To avoid
            // rounding errors, we explicitly set the output R, G and B
            // channels equal to the input luminance.
            //
            // The special cases here and in RGBAtoYCA() ensure that
            // converting black-and white images from RGBA to YCA and
            // back is lossless.
            //

            out.r = Y;
            out.g = Y;
            out.b = Y;
            out.a = in.a;
        }
        else
        {
            // Ensure chroma values are finite
            float r_chroma = float(in.r);
            float b_chroma = float(in.b);
            
            if (!std::isfinite(r_chroma)) r_chroma = 0.0f;
            if (!std::isfinite(b_chroma)) b_chroma = 0.0f;

            // Calculate RGB values
            float r = (r_chroma + 1) * Y;
            float b = (b_chroma + 1) * Y;
            float g = (Y - r * yw.x - b * yw.z) / yw.y;
            
            // Check for negative green channel (can happen with extreme values)
            if (g < 0.0f)
            {
                // Re-normalize keeping luminance proportional
                float sum = r + b;
                if (sum > 0.0f)
                {
                    float scale = Y / sum;
                    r *= scale;
                    b *= scale;
                    g = 0.0f;
                }
                else
                {
                    r = Y;
                    g = Y;
                    b = Y;
                }
            }
            
            // Final sanity check for all channels
            if (!std::isfinite(r) || r < 0.0f) r = 0.0f;
            if (!std::isfinite(g) || g < 0.0f) g = 0.0f;
            if (!std::isfinite(b) || b < 0.0f) b = 0.0f;
            
            // Set output channels
            out.r = r;
            out.g = g;
            out.b = b;
            out.a = in.a;
        }
    }
}

namespace
{

inline float
saturation (const Rgba& in)
{
    float rgbMax = max (in.r, max (in.g, in.b));
    float rgbMin = min (in.r, min (in.g, in.b));

    if (rgbMax > 0)
        return 1 - rgbMin / rgbMax;
    else
        return 0;
}

void
desaturate (const Rgba& in, float f, const V3f& yw, Rgba& out)
{
    float rgbMax = max (in.r, max (in.g, in.b));

    out.r = max (float (rgbMax - (rgbMax - in.r) * f), 0.0f);
    out.g = max (float (rgbMax - (rgbMax - in.g) * f), 0.0f);
    out.b = max (float (rgbMax - (rgbMax - in.b) * f), 0.0f);
    out.a = in.a;

    float Yin  = in.r * yw.x + in.g * yw.y + in.b * yw.z;
    float Yout = out.r * yw.x + out.g * yw.y + out.b * yw.z;

    if (Yout > 0)
    {
        out.r *= Yin / Yout;
        out.g *= Yin / Yout;
        out.b *= Yin / Yout;
    }
}

} // namespace

void
fixSaturation (
    const IMATH_NAMESPACE::V3f& yw,
    int                         n,
    const Rgba* const           rgbaIn[3],
    Rgba                        rgbaOut[/*n*/])
{
    float neighborA2 = saturation (rgbaIn[0][0]);
    float neighborA1 = neighborA2;

    float neighborB2 = saturation (rgbaIn[2][0]);
    float neighborB1 = neighborB2;

    for (int i = 0; i < n; ++i)
    {
        float neighborA0 = neighborA1;
        neighborA1       = neighborA2;

        float neighborB0 = neighborB1;
        neighborB1       = neighborB2;

        if (i < n - 1)
        {
            neighborA2 = saturation (rgbaIn[0][i + 1]);
            neighborB2 = saturation (rgbaIn[2][i + 1]);
        }

        //
        // A0       A1       A2
        //      rgbaOut[i]
        // B0       B1       B2
        //

        float sMean = min (
            1.0f, 0.25f * (neighborA0 + neighborA2 + neighborB0 + neighborB2));

        const Rgba& in  = rgbaIn[1][i];
        Rgba&       out = rgbaOut[i];

        float s = saturation (in);

        if (s > sMean)
        {
            float sMax = min (1.0f, 1 - (1 - sMean) * 0.25f);

            if (s > sMax)
            {
                desaturate (in, sMax / s, yw, out);
                continue;
            }
        }

        out = in;
    }
}

} // namespace RgbaYca
OPENEXR_IMF_INTERNAL_NAMESPACE_SOURCE_EXIT
