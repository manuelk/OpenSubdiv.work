//
//   Copyright 2013 Pixar
//
//   Licensed under the Apache License, Version 2.0 (the "Apache License")
//   with the following modification; you may not use this file except in
//   compliance with the Apache License and the following modification to it:
//   Section 6. Trademarks. is deleted and replaced with:
//
//   6. Trademarks. This License does not grant permission to use the trade
//      names, trademarks, service marks, or product names of the Licensor
//      and its affiliates, except as required to comply with Section 4(c) of
//      the License and to reproduce the content of the NOTICE file.
//
//   You may obtain a copy of the Apache License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the Apache License with the above modification is
//   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
//   KIND, either express or implied. See the Apache License for the specific
//   language governing permissions and limitations under the Apache License.
//

#include "../far/patchBasis.h"
#include "../far/patchDescriptor.h"

#include <cassert>
#include <cstring>
#include <cmath>
#include <cstdio>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {
namespace internal {


//
//  Basis support for quadrilateral patches:
//
//  Quadrilateral patches are parameterized in terms of (s,t) as follows:
//
//      (1,0) *---------* (1,1)
//            | 3     2 |
//          t |         |
//            |         |
//            | 0     1 |
//      (0,0) *---------* (1,0)
//                 s
//


//
//  Simple bilinear quad:
//
template <typename REAL>
int
EvalBasisLinear(REAL s, REAL t,
    REAL wP[4], REAL wDs[4], REAL wDt[4],
    REAL wDss[4], REAL wDst[4], REAL wDtt[4]) {

    REAL sC = 1.0f - s;
    REAL tC = 1.0f - t;

    if (wP) {
        wP[0] = sC * tC;
        wP[1] =  s * tC;
        wP[2] =  s * t;
        wP[3] = sC * t;
    }
    if (wDs && wDt) {
        wDs[0] = -tC;
        wDs[1] =  tC;
        wDs[2] =   t;
        wDs[3] =  -t;

        wDt[0] = -sC;
        wDt[1] =  -s;
        wDt[2] =   s;
        wDt[3] =  sC;

        if (wDss && wDst && wDtt) {
            for(int i = 0; i < 4; ++i) {
                wDss[i] = 0.0f;
                wDtt[i] = 0.0f;
            }

            wDst[0] =  1.0f;
            wDst[1] = -1.0f;
            wDst[2] = -1.0f;
            wDst[3] =  1.0f;
        }
    }
    return 4;
}

//
//  Bicubic BSpline patch:
//
//     12-----13------14-----15
//      |      |      |      |
//      |      |      |      |
//      8------9------10-----11
//      |      | t    |      |
//      |      |   s  |      |
//      4------5------6------7
//      |      |      |      |
//      |      |      |      |
//      O------1------2------3
//
//  The basis if a bicubic BSpline patch is a tensor product, which we make
//  use of here by evaluating, differentiating and combining basis functions
//  in each of the two parametric directions.
//
//  Not all 16 points will be present.  The boundary mask indicates boundary
//  edges beyond which phantom points are implicitly extrapolated.  Weights
//  for missing points are set to zero while those contributing to their
//  implicit extrapolation will be adjusted.
//
namespace {
    //
    //  Cubic BSpline curve basis evaluation:
    //
    template <typename REAL>
    void
    evalBSplineCurve(REAL t, REAL wP[4], REAL wDP[4], REAL wDP2[4]) {

        REAL const one6th = (REAL)(1.0 / 6.0);

        REAL t2 = t * t;
        REAL t3 = t * t2;

        wP[0] = one6th * (1.0f - 3.0f*(t -      t2) -      t3);
        wP[1] = one6th * (4.0f           - 6.0f*t2  + 3.0f*t3);
        wP[2] = one6th * (1.0f + 3.0f*(t +      t2  -      t3));
        wP[3] = one6th * (                                 t3);

        if (wDP) {
            wDP[0] = -0.5f*t2 +      t - 0.5f;
            wDP[1] =  1.5f*t2 - 2.0f*t;
            wDP[2] = -1.5f*t2 +      t + 0.5f;
            wDP[3] =  0.5f*t2;
        }
        if (wDP2) {
            wDP2[0] = -       t + 1.0f;
            wDP2[1] =  3.0f * t - 2.0f;
            wDP2[2] = -3.0f * t + 1.0f;
            wDP2[3] =         t;
        }
    }

    //
    //  Weight adjustments to account for phantom end points:
    //
    template <typename REAL>
    void
    adjustBSplineBoundaryWeights(int boundary, REAL w[16]) {

        if ((boundary & 1) != 0) {
            for (int i = 0; i < 4; ++i) {
                w[i + 8] -= w[i + 0];
                w[i + 4] += w[i + 0] * 2.0f;
                w[i + 0]  = 0.0f;
            }
        }
        if ((boundary & 2) != 0) {
            for (int i = 0; i < 16; i += 4) {
                w[i + 1] -= w[i + 3];
                w[i + 2] += w[i + 3] * 2.0f;
                w[i + 3]  = 0.0f;
            }
        }
        if ((boundary & 4) != 0) {
            for (int i = 0; i < 4; ++i) {
                w[i +  4] -= w[i + 12];
                w[i +  8] += w[i + 12] * 2.0f;
                w[i + 12]  = 0.0f;
            }
        }
        if ((boundary & 8) != 0) {
            for (int i = 0; i < 16; i += 4) {
                w[i + 2] -= w[i + 0];
                w[i + 1] += w[i + 0] * 2.0f;
                w[i + 0]  = 0.0f;
            }
        }
    }

    template <typename REAL>
    void
    boundBasisBSpline(int boundary,
        REAL wP[16], REAL wDs[16], REAL wDt[16],
        REAL wDss[16], REAL wDst[16], REAL wDtt[16]) {

        if (wP) {
            adjustBSplineBoundaryWeights(boundary, wP);
        }
        if (wDs && wDt) {
            adjustBSplineBoundaryWeights(boundary, wDs);
            adjustBSplineBoundaryWeights(boundary, wDt);

            if (wDss && wDst && wDtt) {
                adjustBSplineBoundaryWeights(boundary, wDss);
                adjustBSplineBoundaryWeights(boundary, wDst);
                adjustBSplineBoundaryWeights(boundary, wDtt);
            }
        }
    }

    template <typename REAL> 
    REAL
    mix(REAL s1, REAL s2, REAL t) {
        return ((REAL)1.0-t) * s1 + t * s2;
    }

    template <typename REAL>
    void
    flipMatrix(REAL const * a, REAL * m) {
        m[ 0]=a[15]; m[ 1]=a[14]; m[ 2]=a[13]; m[ 3]=a[12];
        m[ 4]=a[11]; m[ 5]=a[10]; m[ 6]=a[ 9]; m[ 7]=a[ 8];
        m[ 8]=a[ 7]; m[ 9]=a[ 6]; m[10]=a[ 5]; m[11]=a[ 4];
        m[12]=a[ 3]; m[13]=a[ 2]; m[14]=a[ 1]; m[15]=a[ 0];
    }

    // v x m (column major)
    template <typename REAL>
    void
    applyMatrix(REAL * v, REAL const * m) {
        REAL r[4];
        r[0] = v[0]*m[0] + v[1]*m[4] + v[2]*m[ 8] + v[3]*m[12];
        r[1] = v[0]*m[1] + v[1]*m[5] + v[2]*m[ 9] + v[3]*m[13];
        r[2] = v[0]*m[2] + v[1]*m[6] + v[2]*m[10] + v[3]*m[14];
        r[3] = v[0]*m[3] + v[1]*m[7] + v[2]*m[11] + v[3]*m[15];
        memcpy(v, r, 4 * sizeof(REAL));
    }

    template <typename REAL>
    void
    computeMixedCreaseMatrix(REAL sharp1, REAL sharp2, REAL t, REAL tInf, REAL m[16]) {
      REAL s1 = (REAL)exp2(sharp1),
           s2 = (REAL)exp2(sharp2);

      REAL sOver3 = mix(s1, s2, t) / REAL(3),
           oneOverS1 = (REAL)1 / s1,
           oneOverS2 = (REAL)1 / s2,
           oneOver6S = mix(oneOverS1, oneOverS2, t) / (REAL)6,
           sSqr = mix(s1*s1, s2*s2, t);

      REAL A = -sSqr + sOver3 * (REAL)5.5 + oneOver6S            - (REAL)1.0,
           B =         sOver3             + oneOver6S            + (REAL)0.5,
           C =         sOver3             - oneOver6S*(REAL)2.0  + (REAL)1.0,
           E =         sOver3             + oneOver6S            - (REAL)0.5,
           F =       - sOver3 * (REAL)0.5 + oneOver6S;

        m[ 0] = (REAL)1.0; m[ 1] = A*tInf;                   m[ 2] = (REAL)-2.0*A*tInf;                  m[ 3] = A*tInf;
        m[ 4] = (REAL)0.0; m[ 5] = mix((REAL)1.0, B, tInf);  m[ 6] = (REAL)-2.0*E*tInf;                  m[ 7] = E*tInf;
        m[ 8] = (REAL)0.0; m[ 9] = F*tInf;                   m[10] = mix((REAL)1.0, C, tInf);            m[11] = F*tInf;
        m[12] = (REAL)0.0; m[13] = mix((REAL)-1.0, E, tInf); m[14] = mix((REAL)2.0, -(REAL)2.0*E, tInf); m[15] = B*tInf;
    }

    // compute the "crease matrix" for modifying basis weights at parametric
    // location 't', given a sharpness value (see Matthias Niessner derivation
    // for 'single crease' regular patches)
    template <typename REAL>
    void
    computeCreaseMatrix(REAL sharpness, REAL t, REAL m[16]) {

        REAL sharpFloor = (REAL)floor(sharpness),
             sharpCeil = sharpFloor + 1,
             sharpFrac = sharpness - sharpFloor;

        REAL creaseWidthFloor = (REAL)1.0 - exp2(-sharpFloor),
             creaseWidthCeil = (REAL)1.0 - exp2(-sharpCeil);

        // we compute the matrix for both the floor and ceiling of
        // the sharpness value, and then interpolate between them
        // as needed.
        REAL tA = (t > creaseWidthCeil) ? sharpFrac : (REAL)0.0,
             tB = (REAL)0.0;
        if (t > creaseWidthFloor)
          tB = (REAL)1.0-sharpFrac;
        if (t > creaseWidthCeil)
          tB = (REAL)1.0;

        computeMixedCreaseMatrix<REAL>(sharpFloor, sharpCeil, tA, tB, m);
    }

} // end namespace

template <typename REAL>
int
EvalBasisBSpline(REAL s, REAL t,
    REAL wP[16], REAL wDs[16], REAL wDt[16],
    REAL wDss[16], REAL wDst[16], REAL wDtt[16], int boundaryMask, REAL sharpness) {

    REAL sWeights[4], tWeights[4], dsWeights[4], dtWeights[4], dssWeights[4], dttWeights[4];

    evalBSplineCurve(s, wP ? sWeights : 0, wDs ? dsWeights : 0, wDss ? dssWeights : 0);
    evalBSplineCurve(t, wP ? tWeights : 0, wDt ? dtWeights : 0, wDtt ? dttWeights : 0);

    if (boundaryMask!=0 && sharpness > (REAL)0.0) {
        REAL m[16], mflip[16];
        if (boundaryMask & 1) {
            computeCreaseMatrix<REAL>(sharpness, (REAL)1.0-t, m);
            flipMatrix(m, mflip);
            applyMatrix(  tWeights, mflip);
            applyMatrix( dtWeights, mflip);
            applyMatrix(dttWeights, mflip);
        }
        if (boundaryMask & 2) {
            computeCreaseMatrix<REAL>(sharpness, s, m);
            applyMatrix(  sWeights, m);
            applyMatrix( dsWeights, m);
            applyMatrix(dssWeights, m);
        }
        if (boundaryMask & 4) {
            computeCreaseMatrix<REAL>(sharpness, t, m);
            applyMatrix(  tWeights, m);
            applyMatrix( dtWeights, m);
            applyMatrix(dttWeights, m);
        }
        if (boundaryMask & 8) {
            computeCreaseMatrix<REAL>(sharpness, (REAL)1.0-s, m);
            flipMatrix(m, mflip);
            applyMatrix(  sWeights, mflip);
            applyMatrix( dsWeights, mflip);
            applyMatrix(dssWeights, mflip);
        }
    }

    if (wP) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                wP[4*i+j] = sWeights[j] * tWeights[i];
            }
        }
    }
    if (wDs && wDt) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                wDs[4*i+j] = dsWeights[j] * tWeights[i];
                wDt[4*i+j] = sWeights[j] * dtWeights[i];
            }
        }

        if (wDss && wDst && wDtt) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    wDss[4*i+j] = dssWeights[j] * tWeights[i];
                    wDst[4*i+j] = dsWeights[j] * dtWeights[i];
                    wDtt[4*i+j] = sWeights[j] * dttWeights[i];
                }
            }
        }
    }
    return 16;
}

//
//  Bicubic Bezier patch:
//
//     12-----13------14-----15
//      |      |      |      |
//      |      |      |      |
//      8------9------10-----11
//      |      |      |      |
//      |      |      |      |
//      4------5------6------7
//      | t    |      |      |
//      |   s  |      |      |
//      O------1------2------3
//
//  As was the case with the BSpline patch, a bicubic Bezier patch can also
//  make use of its tensor product property by evaluating, differentiating
//  and combining basis functions in each of the two parametric directions.
//
namespace {
    //
    //  Cubic Bezier curve basis evaluation:
    //
    template <typename REAL>
    void
    evalBezierCurve(REAL t, REAL wP[4], REAL wDP[4], REAL wDP2[4]) {

        // The four uniform cubic Bezier basis functions (in terms of t and its
        // complement tC) evaluated at t:
        REAL t2 = t*t;
        REAL tC = 1.0f - t;
        REAL tC2 = tC * tC;

        wP[0] = tC2 * tC;
        wP[1] = tC2 * t * 3.0f;
        wP[2] = t2 * tC * 3.0f;
        wP[3] = t2 * t;

        // Derivatives of the above four basis functions at t:
        if (wDP) {
           wDP[0] = -3.0f * tC2;
           wDP[1] =  9.0f * t2 - 12.0f * t + 3.0f;
           wDP[2] = -9.0f * t2 +  6.0f * t;
           wDP[3] =  3.0f * t2;
        }

        // Second derivatives of the basis functions at t:
        if (wDP2) {
            wDP2[0] =   6.0f * tC;
            wDP2[1] =  18.0f * t - 12.0f;
            wDP2[2] = -18.0f * t +  6.0f;
            wDP2[3] =   6.0f * t;
        }
    }
} // end namespace

template <typename REAL>
int
EvalBasisBezier(REAL s, REAL t,
    REAL wP[16], REAL wDs[16], REAL wDt[16],
    REAL wDss[16], REAL wDst[16], REAL wDtt[16]) {

    REAL sWeights[4], tWeights[4], dsWeights[4], dtWeights[4], dssWeights[4], dttWeights[4];

    evalBezierCurve(s, wP ? sWeights : 0, wDs ? dsWeights : 0, wDss ? dssWeights : 0);
    evalBezierCurve(t, wP ? tWeights : 0, wDt ? dtWeights : 0, wDtt ? dttWeights : 0);

    if (wP) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                wP[4*i+j] = sWeights[j] * tWeights[i];
            }
        }
    }
    if (wDs && wDt) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                wDs[4*i+j] = dsWeights[j] * tWeights[i];
                wDt[4*i+j] = sWeights[j] * dtWeights[i];
            }
        }

        if (wDss && wDst && wDtt) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    wDss[4*i+j] = dssWeights[j] * tWeights[i];
                    wDst[4*i+j] = dsWeights[j] * dtWeights[i];
                    wDtt[4*i+j] = sWeights[j] * dttWeights[i];
                }
            }
        }
    }
    return 16;
}

//
//  Cubic Gregory patch:
//
//      P3         e3-      e2+         P2
//         15------17-------11--------10
//         |        |        |        |
//         |        |        |        |
//         |        | f3-    | f2+    |
//         |       19       13        |
//     e3+ 16-----18           14-----12 e2-
//         |     f3+          f2-     |
//         |                          |
//         |                          |
//         |      f0-         f1+     |
//     e0- 2------4            8------6 e1+
//         |        3        9        |
//         |        | f0+    | f1-    |
//         | t      |        |        |
//         |   s    |        |        |
//         O--------1--------7--------5
//      P0         e0+      e1-         P1
//
//  The 20-point cubic Gregory patch is an extension of the 16-point bicubic
//  Bezier patch with the 4 interior points of the Bezier patch replaced with
//  pairs of points (face points -- fi+ and fi-) that are rationally combined.
//
//  The point ordering of the Gregory patch deviates considerably from the
//  BSpline and Bezier patches by grouping the 5 points at each corner and
//  ordering the groups by corner index.
//
template <typename REAL>
int
EvalBasisGregory(REAL s, REAL t,
    REAL point[20], REAL wDs[20], REAL wDt[20],
    REAL wDss[20], REAL wDst[20], REAL wDtt[20]) {

    //  Indices of boundary and interior points and their corresponding Bezier points
    //  (this can be reduced with more direct indexing and unrolling of loops):
    //
    static int const boundaryGregory[12] = { 0, 1, 7, 5, 2, 6, 16, 12, 15, 17, 11, 10 };
    static int const boundaryBezSCol[12] = { 0, 1, 2, 3, 0, 3,  0,  3,  0,  1,  2,  3 };
    static int const boundaryBezTRow[12] = { 0, 0, 0, 0, 1, 1,  2,  2,  3,  3,  3,  3 };

    static int const interiorGregory[8] = { 3, 4,  8, 9,  13, 14,  18, 19 };
    static int const interiorBezSCol[8] = { 1, 1,  2, 2,   2,  2,   1,  1 };
    static int const interiorBezTRow[8] = { 1, 1,  1, 1,   2,  2,   2,  2 };

    //
    //  Bezier basis functions are denoted with B while the rational multipliers for the
    //  interior points will be denoted G -- so we have B(s), B(t) and G(s,t):
    //
    //  Directional Bezier basis functions B at s and t:
    REAL Bs[4], Bds[4], Bdss[4];
    REAL Bt[4], Bdt[4], Bdtt[4];

    evalBezierCurve(s, Bs, wDs ? Bds : 0, wDss ? Bdss : 0);
    evalBezierCurve(t, Bt, wDt ? Bdt : 0, wDtt ? Bdtt : 0);

    //  Rational multipliers G at s and t:
    REAL sC = 1.0f - s;
    REAL tC = 1.0f - t;

    //  Use <= here to avoid compiler warnings -- the sums should always be non-negative:
    REAL df0 = s  + t;   df0 = (df0 <= 0.0f) ? (REAL)1.0f : (1.0f / df0);
    REAL df1 = sC + t;   df1 = (df1 <= 0.0f) ? (REAL)1.0f : (1.0f / df1);
    REAL df2 = sC + tC;  df2 = (df2 <= 0.0f) ? (REAL)1.0f : (1.0f / df2);
    REAL df3 = s  + tC;  df3 = (df3 <= 0.0f) ? (REAL)1.0f : (1.0f / df3);

    REAL G[8] = { s*df0, t*df0,  t*df1, sC*df1,  sC*df2, tC*df2,  tC*df3, s*df3 };

    //  Combined weights for boundary and interior points:
    for (int i = 0; i < 12; ++i) {
        point[boundaryGregory[i]] = Bs[boundaryBezSCol[i]] * Bt[boundaryBezTRow[i]];
    }
    for (int i = 0; i < 8; ++i) {
        point[interiorGregory[i]] = Bs[interiorBezSCol[i]] * Bt[interiorBezTRow[i]] * G[i];
    }

    //
    //  For derivatives, the basis functions for the interior points are rational and ideally
    //  require appropriate differentiation, i.e. product rule for the combination of B and G
    //  and the quotient rule for the rational G itself.  As initially proposed by Loop et al
    //  though, the approximation using the 16 Bezier points arising from the G(s,t) has
    //  proved adequate (and is what the GPU shaders use) so we continue to use that here.
    //
    //  An implementation of the true derivatives is provided and conditionally compiled for
    //  those that require it, e.g.:
    //
    //    dclyde's note: skipping half of the product rule like this does seem to change the
    //    result a lot in my tests.  This is not a runtime bottleneck for cloth sims anyway
    //    so I'm just using the accurate version.
    //
    if (wDs && wDt) {
        bool find_second_partials = wDs && wDst && wDtt;

        //  Combined weights for boundary points -- simple tensor products:
        for (int i = 0; i < 12; ++i) {
            int iDst = boundaryGregory[i];
            int tRow = boundaryBezTRow[i];
            int sCol = boundaryBezSCol[i];

            wDs[iDst] = Bds[sCol] * Bt[tRow];
            wDt[iDst] = Bdt[tRow] * Bs[sCol];

            if (find_second_partials) {
                wDss[iDst] = Bdss[sCol] * Bt[tRow];
                wDst[iDst] = Bds[sCol] * Bdt[tRow];
                wDtt[iDst] = Bs[sCol] * Bdtt[tRow];
            }
        }

#ifndef OPENSUBDIV_GREGORY_EVAL_TRUE_DERIVATIVES
        //  Approximation to the true Gregory derivatives by differentiating the Bezier patch
        //  unique to the given (s,t), i.e. having F = (g^+ * f^+) + (g^- * f^-) as its four
        //  interior points:
        //
        //  Combined weights for interior points -- tensor products with G+ or G-:
        for (int i = 0; i < 8; ++i) {
            int iDst = interiorGregory[i];
            int tRow = interiorBezTRow[i];
            int sCol = interiorBezSCol[i];

            wDs[iDst] = Bds[sCol] * Bt[tRow] * G[i];
            wDt[iDst] = Bdt[tRow] * Bs[sCol] * G[i];

            if (find_second_partials) {
                wDss[iDst] = Bdss[sCol] * Bt[tRow] * G[i];
                wDst[iDst] = Bds[sCol] * Bdt[tRow] * G[i];
                wDtt[iDst] = Bs[sCol] * Bdtt[tRow] * G[i];
            }
        }
#else
        //  True Gregory derivatives using appropriate differentiation of composite functions:
        //
        //  Note that for G(s,t) = N(s,t) / D(s,t), all N' and D' are trivial constants (which
        //  simplifies things for higher order derivatives).  And while each pair of functions
        //  G (i.e. the G+ and G- corresponding to points f+ and f-) must sum to 1 to ensure
        //  Bezier equivalence (when f+ = f-), the pairs of G' must similarly sum to 0.  So we
        //  can potentially compute only one of the pair and negate the result for the other
        //  (and with 4 or 8 computations involving these constants, this is all very SIMD
        //  friendly...) but for now we treat all 8 independently for simplicity.
        //
        //REAL N[8] = {   s,     t,      t,     sC,      sC,     tC,      tC,     s };
        REAL D[8] = {   df0,   df0,    df1,    df1,     df2,    df2,     df3,   df3 };

        static REAL const Nds[8] = {  1.0f,  0.0f,  0.0f, -1.0f, -1.0f,  0.0f,  0.0f,  1.0f };
        static REAL const Ndt[8] = {  0.0f,  1.0f,  1.0f,  0.0f,  0.0f, -1.0f, -1.0f,  0.0f };

        static REAL const Dds[8] = {  1.0f,  1.0f, -1.0f, -1.0f, -1.0f, -1.0f,  1.0f,  1.0f };
        static REAL const Ddt[8] = {  1.0f,  1.0f,  1.0f,  1.0f, -1.0f, -1.0f, -1.0f, -1.0f };

        //  Combined weights for interior points -- combinations of B, B', G and G':
        for (int i = 0; i < 8; ++i) {
            int iDst = interiorGregory[i];
            int tRow = interiorBezTRow[i];
            int sCol = interiorBezSCol[i];

            //  Quotient rule for G' (re-expressed in terms of G to simplify (and D = 1/D)):
            REAL Gds = (Nds[i] - Dds[i] * G[i]) * D[i];
            REAL Gdt = (Ndt[i] - Ddt[i] * G[i]) * D[i];

            //  Product rule combining B and B' with G and G':
            wDs[iDst] = (Bds[sCol] * G[i] + Bs[sCol] * Gds) * Bt[tRow];
            wDt[iDst] = (Bdt[tRow] * G[i] + Bt[tRow] * Gdt) * Bs[sCol];

            if (find_second_partials) {
                REAL Dsqr_inv = D[i]*D[i];

                REAL Gdss = 2.0f * Dds[i] * Dsqr_inv * (G[i] * Dds[i] - Nds[i]);
                REAL Gdst = Dsqr_inv * (2.0f * G[i] * Dds[i] * Ddt[i] - Nds[i] * Ddt[i] - Ndt[i] * Dds[i]);
                REAL Gdtt = 2.0f * Ddt[i] * Dsqr_inv * (G[i] * Ddt[i] - Ndt[i]);

                wDss[iDst] = (Bdss[sCol] * G[i] + 2.0f * Bds[sCol] * Gds + Bs[sCol] * Gdss) * Bt[tRow];
                wDst[iDst] =  Bt[tRow] * (Bs[sCol] * Gdst + Bds[sCol] * Gdt) +
                             Bdt[tRow] * (Bds[sCol] * G[i] + Bs[sCol] * Gds);
                wDtt[iDst] = (Bdtt[tRow] * G[i] + 2.0f * Bdt[tRow] * Gdt + Bt[tRow] * Gdtt) * Bs[sCol];
            }
        }
#endif
    }
    return 20;
}


//
//  Basis support for triangular patches:
//
//  Triangular patches may be evaluated in barycentric (trivariate) or
//  bivariate form, depending on the complexity of their basis functions.
//  The parametric orientation for a triangle is as follows:
//
//            (1,0)
//              *
//             . .
//          t . 2 .
//           .     .
//          . 0   1 .
//   (0,0) *---------* (1,0)
//              s
//
//  With the origin (0,0) -- barycentric (0,0,w = 1) -- oriented at the
//  corner V0, the corners V0, V1, and V2 correspond to barycentric
//  coordinates W, U and V.  This is consistent with GPU tessellation
//  shaders, but not with many publications where the corners correspond
//  more intuitively to U, V and W.
//


//
//  Simple linear triangle:
//
template <typename REAL>
int
EvalBasisLinearTri(REAL s, REAL t,
    REAL wP[3], REAL wDs[3], REAL wDt[3],
    REAL wDss[3], REAL wDst[3], REAL wDtt[3]) {

    if (wP) {
        wP[0] = 1.0f - s - t;
        wP[1] = s;
        wP[2] = t;
    }
    if (wDs && wDt) {
        wDs[0] = -1.0f;
        wDs[1] =  1.0f;
        wDs[2] =  0.0f;

        wDt[0] = -1.0f;
        wDt[1] =  0.0f;
        wDt[2] =  1.0f;

        if (wDss && wDst && wDtt) {
            wDss[0] = wDss[1] = wDss[2] = 0.0f;
            wDst[0] = wDst[1] = wDst[2] = 0.0f;
            wDtt[0] = wDtt[1] = wDtt[2] = 0.0f;
        }
    }
    return 3;
}


//
//  Quartic Box spline triangle:
//
//  Points for the quartic triangular Box spline (representing regular
//  patches for Loop subdivision) are as follows:
//
//         10-----11
//         . .   . .
//        .   . .   .
//       7-----8-----9
//      . .   . .   . .
//     .   . .   . .   .
//    3-----4-----5-----6
//     .   . .   . .   .
//      . .   . .   . /
//       0-----1-----2
//
//  Stam provided the basis functions for these patches in terms of barycentric
//  coordinates (u,v,w) (see Stam's "Evaluation of Loop Subdivision Surfaces").
//  Unfortunately, unlike the basis functions for a quartic Bezier triangle,
//  they are not very compact -- 3 functions involving 9 quartic terms and 3
//  others involving 15 quartic terms.  (In contrast, the maximum number of
//  terms in bivariate form is 15.)
//
//  Since we also need to differentiate with respect to u and v, we eliminate w
//  and use the coefficient matrix C multiplied by the set of monomials M
//  evaluated at (u,v), i.e. the full set of basis functions is:
//
//      B(u,v) = C * M(u,v)
//
//  where
//
//      M(u,v) = { 1, u,v, uu,uv,vv, uuu,uuv,uvv,vvv, uuuu,uuuv,uuvv,uvvv,vvvv }
//
//  and the 12 x 15 matrix C is as follows, scaled by a common factor of 1/12:
//
//      { 1, -2,-4,    0,  6,  6,   2,  0, -6, -4,  -1, -2, 0,  2,  1 },
//      { 1,  2,-2,    0, -6,  0,  -4,  0,  6,  2,   2,  4, 0, -2, -1 },
//      { 0,  0, 0,    0,  0,  0,   2,  0,  0,  0,  -1, -2, 0,  0,  0 },
//      { 1, -4,-2,    6,  6,  0,  -4, -6,  0,  2,   1,  2, 0, -2, -1 },
//      { 6,  0, 0,  -12,-12,-12,   8, 12, 12,  8,  -1, -2, 0, -2, -1 },
//      { 1,  4, 2,    6,  6,  0,  -4, -6,-12, -4,  -1, -2, 0,  4,  2 },
//      { 0,  0, 0,    0,  0,  0,   0,  0,  0,  0,   1,  2, 0,  0,  0 },
//      { 1, -2, 2,    0, -6,  0,   2,  6,  0, -4,  -1, -2, 0,  4,  2 },
//      { 1,  2, 4,    0,  6,  6,  -4,-12, -6, -4,   2,  4, 0, -2, -1 },
//      { 0,  0, 0,    0,  0,  0,   2,  6,  6,  2,  -1, -2, 0, -2, -1 },
//      { 0,  0, 0,    0,  0,  0,   0,  0,  0,  2,   0,  0, 0, -2, -1 },
//      { 0,  0, 0,    0,  0,  0,   0,  0,  0,  0,   0,  0, 0,  2,  1 } 
//
//  Differentiating the monomials and refactoring yields a unique set of
//  coefficients for each of the derivatives, which we multiply by M(u,v).
//
namespace {
    template <typename REAL>
    inline void
    evalBivariateMonomialsQuartic(REAL s, REAL t, REAL M[]) {

        M[0] = 1.0;

        M[1] = s;
        M[2] = t;

        M[3] = s * s;
        M[4] = s * t;
        M[5] = t * t;

        M[6] = M[3] * s;
        M[7] = M[4] * s;
        M[8] = M[4] * t;
        M[9] = M[5] * t;

        M[10] = M[6] * s;
        M[11] = M[7] * s;
        M[12] = M[3] * M[5];
        M[13] = M[8] * t;
        M[14] = M[9] * t;
    }

    template <typename REAL>
    void
    evalBoxSplineTriDerivWeights(REAL const stMonomials[], int ds, int dt, REAL w[]) {

        REAL const * M = stMonomials;

        REAL S = 1.0;

        int totalOrder = ds + dt;
        if (totalOrder == 0) {
            S *= (REAL) (1.0 / 12.0);

            w[0]  = S * (1 - 2*M[1] - 4*M[2]          + 6*M[4] + 6*M[5] + 2*M[6]          - 6*M[8] - 4*M[9] -   M[10] - 2*M[11] + 2*M[13] +   M[14]);
            w[1]  = S * (1 + 2*M[1] - 2*M[2]          - 6*M[4]          - 4*M[6]          + 6*M[8] + 2*M[9] + 2*M[10] + 4*M[11] - 2*M[13] -   M[14]);
            w[2]  = S * (                                                 2*M[6]                            -   M[10] - 2*M[11]                    );
            w[3]  = S * (1 - 4*M[1] - 2*M[2] + 6*M[3] + 6*M[4]          - 4*M[6] - 6*M[7]          + 2*M[9] +   M[10] + 2*M[11] - 2*M[13] -   M[14]);
            w[4]  = S * (6                   -12*M[3] -12*M[4] -12*M[5] + 8*M[6] +12*M[7] +12*M[8] + 8*M[9] -   M[10] - 2*M[11] - 2*M[13] -   M[14]);
            w[5]  = S * (1 + 4*M[1] + 2*M[2] + 6*M[3] + 6*M[4]          - 4*M[6] - 6*M[7] -12*M[8] - 4*M[9] -   M[10] - 2*M[11] + 4*M[13] + 2*M[14]);
            w[6]  = S * (                                                                                       M[10] + 2*M[11]                    );
            w[7]  = S * (1 - 2*M[1] + 2*M[2]          - 6*M[4]          + 2*M[6] + 6*M[7]          - 4*M[9] -   M[10] - 2*M[11] + 4*M[13] + 2*M[14]);
            w[8]  = S * (1 + 2*M[1] + 4*M[2]          + 6*M[4] + 6*M[5] - 4*M[6] -12*M[7] - 6*M[8] - 4*M[9] + 2*M[10] + 4*M[11] - 2*M[13] -   M[14]);
            w[9]  = S * (                                                 2*M[6] + 6*M[7] + 6*M[8] + 2*M[9] -   M[10] - 2*M[11] - 2*M[13] -   M[14]);
            w[10] = S * (                                                                            2*M[9]                     - 2*M[13] -   M[14]);
            w[11] = S * (                                                                                                         2*M[13] +   M[14]);
        } else if (totalOrder == 1) {
            S *= (REAL) (1.0 / 6.0);

            if (ds) {
                w[0]  = S * (-1          + 3*M[2] + 3*M[3]          - 3*M[5] - 2*M[6] - 3*M[7] +   M[9]);
                w[1]  = S * ( 1          - 3*M[2] - 6*M[3]          + 3*M[5] + 4*M[6] + 6*M[7] -   M[9]);
                w[2]  = S * (                       3*M[3]                   - 2*M[6] - 3*M[7]         );
                w[3]  = S * (-2 + 6*M[1] + 3*M[2] - 6*M[3] - 6*M[4]          + 2*M[6] + 3*M[7] -   M[9]);
                w[4]  = S * (   -12*M[1] - 6*M[2] +12*M[3] +12*M[4] + 6*M[5] - 2*M[6] - 3*M[7] -   M[9]);
                w[5]  = S * ( 2 + 6*M[1] + 3*M[2] - 6*M[3] - 6*M[4] - 6*M[5] - 2*M[6] - 3*M[7] + 2*M[9]);
                w[6]  = S * (                                                  2*M[6] + 3*M[7]         );
                w[7]  = S * (-1          - 3*M[2] + 3*M[3] + 6*M[4]          - 2*M[6] - 3*M[7] + 2*M[9]);
                w[8]  = S * ( 1          + 3*M[2] - 6*M[3] -12*M[4] - 3*M[5] + 4*M[6] + 6*M[7] -   M[9]);
                w[9]  = S * (                       3*M[3] + 6*M[4] + 3*M[5] - 2*M[6] - 3*M[7] -   M[9]);
                w[10] = S * (                                                                  -   M[9]);
                w[11] = S * (                                                                      M[9]);
            } else {
                w[0]  = S * (-2 + 3*M[1] + 6*M[2]          - 6*M[4] - 6*M[5]  -   M[6] + 3*M[8] + 2*M[9]);
                w[1]  = S * (-1 - 3*M[1]                   + 6*M[4] + 3*M[5]  + 2*M[6] - 3*M[8] - 2*M[9]);
                w[2]  = S * (                                                 -   M[6]                  );
                w[3]  = S * (-1 + 3*M[1]          - 3*M[3]          + 3*M[5]  +   M[6] - 3*M[8] - 2*M[9]);
                w[4]  = S * (   - 6*M[1] -12*M[2] + 6*M[3] +12*M[4] +12*M[5]  -   M[6] - 3*M[8] - 2*M[9]);
                w[5]  = S * ( 1 + 3*M[1]          - 3*M[3] -12*M[4] - 6*M[5]  -   M[6] + 6*M[8] + 4*M[9]);
                w[6]  = S * (                                                 +   M[6]                  );
                w[7]  = S * ( 1 - 3*M[1]          + 3*M[3]          - 6*M[5]  -   M[6] + 6*M[8] + 4*M[9]);
                w[8]  = S * ( 2 + 3*M[1] + 6*M[2] - 6*M[3] - 6*M[4] - 6*M[5]  + 2*M[6] - 3*M[8] - 2*M[9]);
                w[9]  = S * (                     + 3*M[3] + 6*M[4] + 3*M[5]  -   M[6] - 3*M[8] - 2*M[9]);
                w[10] = S * (                                         3*M[5]           - 3*M[8] - 2*M[9]);
                w[11] = S * (                                                            3*M[8] + 2*M[9]);
            }
        } else if (totalOrder == 2) {
            if (ds == 2) {
                w[0]  = S * (   +   M[1]          -   M[3] -   M[4]);
                w[1]  = S * (   - 2*M[1]          + 2*M[3] + 2*M[4]);
                w[2]  = S * (       M[1]          -   M[3] -   M[4]);
                w[3]  = S * ( 1 - 2*M[1] -   M[2] +   M[3] +   M[4]);
                w[4]  = S * (-2 + 4*M[1] + 2*M[2] -   M[3] -   M[4]);
                w[5]  = S * ( 1 - 2*M[1] -   M[2] -   M[3] -   M[4]);
                w[6]  = S * (                         M[3] +   M[4]);
                w[7]  = S * (   +   M[1] +   M[2] -   M[3] -   M[4]);
                w[8]  = S * (   - 2*M[1] - 2*M[2] + 2*M[3] + 2*M[4]);
                w[9]  = S * (       M[1] +   M[2] -   M[3] -   M[4]);
                w[10] =     0;
                w[11] =     0;
            } else if (dt == 2) {
                w[0]  = S * ( 1 -   M[1] - 2*M[2] +   M[4] +   M[5]);
                w[1]  = S * (   +   M[1] +   M[2] -   M[4] -   M[5]);
                w[2]  =     0;
                w[3]  = S * (            +   M[2] -   M[4] -   M[5]);
                w[4]  = S * (-2 + 2*M[1] + 4*M[2] -   M[4] -   M[5]);
                w[5]  = S * (   - 2*M[1] - 2*M[2] + 2*M[4] + 2*M[5]);
                w[6]  =     0;
                w[7]  = S * (            - 2*M[2] + 2*M[4] + 2*M[5]);
                w[8]  = S * ( 1 -   M[1] - 2*M[2] -   M[4] -   M[5]);
                w[9]  = S * (   +   M[1] +   M[2] -   M[4] -   M[5]);
                w[10] = S * (                M[2] -   M[4] -   M[5]);
                w[11] = S * (                         M[4] +   M[5]);
            } else {
                S *= (REAL) (1.0 / 2.0);

                w[0]  = S * ( 1          - 2*M[2] -   M[3] +   M[5]);
                w[1]  = S * (-1          + 2*M[2] + 2*M[3] -   M[5]);
                w[2]  = S * (                     -   M[3]         );
                w[3]  = S * ( 1 - 2*M[1]          +   M[3] -   M[5]);
                w[4]  = S * (-2 + 4*M[1] + 4*M[2] -   M[3] -   M[5]);
                w[5]  = S * ( 1 - 2*M[1] - 4*M[2] -   M[3] + 2*M[5]);
                w[6]  = S * (                     +   M[3]         );
                w[7]  = S * (-1 + 2*M[1]          -   M[3] + 2*M[5]);
                w[8]  = S * ( 1 - 4*M[1] - 2*M[2] + 2*M[3] -   M[5]);
                w[9]  = S * (   + 2*M[1] + 2*M[2] -   M[3] -   M[5]);
                w[10] = S * (                              -   M[5]);
                w[11] = S * (                                  M[5]);
            }
        } else {
            assert(totalOrder <= 2);
        }
    }

    template <typename REAL>
    void
    adjustBoxSplineTriBoundaryWeights(int boundaryMask, REAL weights[]) {

        if (boundaryMask == 0) return;

        //
        //  Determine boundary edges and vertices from the lower 3 and upper
        //  2 bits of the 5-bit mask:
        //
        bool edgeIsBoundary[3 + 2];  // +2 filled in to avoid +1 and +2 mod 3
        bool vertexIsBoundary[3];

        bool lowerBits[3];
        lowerBits[0] = (boundaryMask & 0x1) != 0;
        lowerBits[1] = (boundaryMask & 0x2) != 0;
        lowerBits[2] = (boundaryMask & 0x4) != 0;

        int upperBits = (boundaryMask >> 3) & 0x3;
        if (upperBits == 0) {
            //  Boundary edges only:
            for (int i = 0; i < 3; ++i) {
                edgeIsBoundary[i] = lowerBits[i];
                vertexIsBoundary[i] = false;
            }
        } else if (upperBits == 1) {
            //  Boundary vertices only:
            for (int i = 0; i < 3; ++i) {
                vertexIsBoundary[i] = lowerBits[i];
                edgeIsBoundary[i] = false;
            }
        } else if (upperBits == 2) {
            //  Boundary edge and opposite boundary vertex:
            edgeIsBoundary[0] = vertexIsBoundary[2] = lowerBits[0];
            edgeIsBoundary[1] = vertexIsBoundary[0] = lowerBits[1];
            edgeIsBoundary[2] = vertexIsBoundary[1] = lowerBits[2];
        }
        //  Wrap the 2 additional values to avoid modulo 3 in the edge tests:
        edgeIsBoundary[3] = edgeIsBoundary[0];
        edgeIsBoundary[4] = edgeIsBoundary[1];

        //
        //  Adjust weights for the 4 boundary points (eB) and 3 interior points
        //  (eI) to account for the 3 phantom points (eP) adjacent to each
        //  boundary edge:
        //
        int const eP[3][3] = { {  0, 1, 2 }, {  6, 9, 11  }, {  10, 7, 3 } };
        int const eB[3][4] = { {3, 4, 5, 6}, {2, 5, 8, 10 }, {11, 8, 4, 0} };
        int const eI[3][3] = { {  7, 8, 9 }, {  1, 4, 7   }, {   9, 5, 1 } };

        for (int i = 0; i < 3; ++i) {
            if (edgeIsBoundary[i]) {
                int const * iPhantom  = eP[i];
                int const * iBoundary = eB[i];
                int const * iInterior = eI[i];

                //  Adjust weights for points contributing to phantom point
                //  P0 -- extrapolated according to the presence of adj edge:
                REAL w0 = weights[iPhantom[0]];
                if (edgeIsBoundary[i + 2]) {
                    //  P0 = B1 + (B1 - I1)
                    weights[iBoundary[1]] += w0;
                    weights[iBoundary[1]] += w0;
                    weights[iInterior[1]] -= w0;
                } else {
                    //  P0 = B1 + (B0 - I0)
                    weights[iBoundary[1]] += w0;
                    weights[iBoundary[0]] += w0;
                    weights[iInterior[0]] -= w0;
                }

                //  Adjust weights for points contributing to phantom point
                //  P1 = B1 + (B2 - I1)
                REAL w1 = weights[iPhantom[1]];
                weights[iBoundary[1]] += w1;
                weights[iBoundary[2]] += w1;
                weights[iInterior[1]] -= w1;

                //  Adjust weights for points contributing to phantom point
                //  P2 -- extrapolated according to the presence of adj edge:
                REAL w2 = weights[iPhantom[2]];
                if (edgeIsBoundary[i + 1]) {
                    //  P2 = B2 + (B2 - I1)
                    weights[iBoundary[2]] += w2;
                    weights[iBoundary[2]] += w2;
                    weights[iInterior[1]] -= w2;
                } else {
                    //  P2 = B2 + (B3 - I2)
                    weights[iBoundary[2]] += w2;
                    weights[iBoundary[3]] += w2;
                    weights[iInterior[2]] -= w2;
                }

                //  Clear weights for the phantom points:
                weights[iPhantom[0]] = 0.0f;
                weights[iPhantom[1]] = 0.0f;
                weights[iPhantom[2]] = 0.0f;
            }
        }

        //
        //  Adjust weights for the 3 boundary points (vB) and the 2 interior
        //  points (vI) to account for the 2 phantom points (vP) adjacent to
        //  each boundary vertex:
        //
        int const vP[3][2] = { {   3, 0  }, {   2, 6  }, {  11, 10  } };
        int const vB[3][3] = { { 7, 4, 1 }, { 1, 5, 9 }, { 9,  8, 7 } };
        int const vI[3][2] = { {   8, 5  }, {   4, 8  }, {   5,  4  } };

        for (int i = 0; i < 3; ++i) {
            if (vertexIsBoundary[i]) {
                int const * iPhantom  = vP[i];
                int const * iBoundary = vB[i];
                int const * iInterior = vI[i];

                //  Adjust weights for points contributing to phantom point
                //  P0 = B1 + (B0 - I0)
                REAL w0 = weights[iPhantom[0]];
                weights[iBoundary[1]] += w0;
                weights[iBoundary[0]] += w0;
                weights[iInterior[0]] -= w0;

                //  Adjust weights for points contributing to phantom point
                //  P1 = B1 + (B2 - I1)
                REAL w1 = weights[iPhantom[1]];
                weights[iBoundary[1]] += w1;
                weights[iBoundary[2]] += w1;
                weights[iInterior[1]] -= w1;

                //  Clear weights for the phantom points:
                weights[iPhantom[0]] = 0.0f;
                weights[iPhantom[1]] = 0.0f;
            }
        }
    }

    template <typename REAL>
    void
    boundBasisBoxSplineTri(int boundary,
        REAL wP[12], REAL wDs[12], REAL wDt[12],
        REAL wDss[12], REAL wDst[12], REAL wDtt[12]) {

        if (wP) {
            adjustBoxSplineTriBoundaryWeights(boundary, wP);
        }
        if (wDs && wDt) {
            adjustBoxSplineTriBoundaryWeights(boundary, wDs);
            adjustBoxSplineTriBoundaryWeights(boundary, wDt);

            if (wDss && wDst && wDtt) {
                adjustBoxSplineTriBoundaryWeights(boundary, wDss);
                adjustBoxSplineTriBoundaryWeights(boundary, wDst);
                adjustBoxSplineTriBoundaryWeights(boundary, wDtt);
            }
        }
    }
}  // namespace

template <typename REAL>
int EvalBasisBoxSplineTri(REAL s, REAL t,
    REAL wP[12], REAL wDs[12], REAL wDt[12],
    REAL wDss[12], REAL wDst[12], REAL wDtt[12]) {

    REAL stMonomials[15];
    evalBivariateMonomialsQuartic(s, t, stMonomials);

    if (wP) {
        evalBoxSplineTriDerivWeights<REAL>(stMonomials, 0, 0, wP);
    }
    if (wDs && wDt) {
        evalBoxSplineTriDerivWeights(stMonomials, 1, 0, wDs);
        evalBoxSplineTriDerivWeights(stMonomials, 0, 1, wDt);

        if (wDss && wDst && wDtt) {
            evalBoxSplineTriDerivWeights(stMonomials, 2, 0, wDss);
            evalBoxSplineTriDerivWeights(stMonomials, 1, 1, wDst);
            evalBoxSplineTriDerivWeights(stMonomials, 0, 2, wDtt);
        }
    }
    return 12;
}


//
//  Hybrid (cubic-quartic) Bezier triangle:
//
//  The regular patch for Loop subdivision is a quartic triangular Box spline
//  with cubic boundaries.  So we need a quartic Bezier patch to represent it
//  faithfully, but we use a cubic-quartic hybrid to keep the representation
//  of boundaries as cubic -- useful for a number of purposes, in addition to
//  reducing the number points required from 15 to 12.
//
//  Ultimately this patch is quartic and its basis functions are of maximum
//  quartic degree.  The formulae for the 15 true quartic basis functions are:
//
//                       4!        i   j   k
//      B   (u,v,w) =  ------- * (u * v * w )
//       ijk           i!j!k!
//
//  for each i + j + k = 4, and the quartic points and corresponding p<i,j,k>
//  are oriented as follows:
//
//                    Q14                                   p040
//                Q12     Q13                           p031    p130
//            Q9      Q10     Q11                   p022    p121    p220
//        Q5      Q6      Q7      Q8            p013    p112    p211    p310
//    Q0      Q1      Q2      Q3      Q4    p004    p103    p202    p301    p400
//
//  The points for the corresponding hybrid patch are oriented and numbered:
//
//                    H11
//               H8        H10
//                    H9
//        H4      H5      H6      H7
//    H0       H1            H2       H3
//
//  Their corresponding basis functions h(u,v,w) are derived by combining the
//  quartic basis functions according to degree elevation of their boundary
//  curves.  This leads to the 12 basis functions:
//
//      h[0]  = w^3
//      h[3]  = u^3
//      h[11] = v^3
//
//      h[1]  =  3 * u   * w^2 * (u + w)
//      h[2]  =  3 * u^2 * w   * (u + w)
//
//      h[7]  =  3 * u^2 * v   * (u + v)
//      h[10] =  3 * u   * v^2 * (u + v)
//
//      h[8]  =  3 * v^2 * w    * (w + v)
//      h[10] =  3 * v   * w^2  * (v + w)
//
//      h[5]  = 12 * u   * v   * w^2;
//      h[6]  = 12 * u^2 * v   * w;
//      h[9]  = 12 * u   * v^2 * w;
//
//  These remain compact with at most two trivariate terms, and so relatively
//  easy to differentiate in this form while keeping the number of terms low.
//
namespace {
    template <typename REAL>
    void
    evalBezierTriDerivWeights(REAL s, REAL t, int ds, int dt, REAL wB[]) {

        REAL u  = s;
        REAL v  = t;
        REAL w  = 1 - u - v;

        REAL u2 = u * u;
        REAL v2 = v * v;
        REAL w2 = w * w;

        REAL uv = u * v;
        REAL vw = v * w;
        REAL uw = u * w;

        int totalOrder = ds + dt;
        if (totalOrder == 0) {
            wB[0]  = w*w2;
            wB[3]  = u*u2;
            wB[11] = v*v2;

            wB[1]  =  3 * uw * (uw + w2);
            wB[2]  =  3 * uw * (uw + u2);

            wB[7]  =  3 * uv * (uv + u2);
            wB[10] =  3 * uv * (uv + v2);

            wB[8]  =  3 * vw * (vw + v2);
            wB[4]  =  3 * vw * (vw + w2);

            wB[5]  = 12 * w2 * uv;
            wB[6]  = 12 * u2 * vw;
            wB[9]  = 12 * v2 * uw;
        } else if (totalOrder == 1) {
            if (ds) {
                wB[0]  = -3 * w2;
                wB[3]  =  3 * u2;
                wB[11] =  0;

                wB[1]  =  3 * w * (w2 - uw - 2*u2);
                wB[2]  = -3 * u * (u2 - uw - 2*w2);

                wB[7]  =  9 * u2*v + 6 * u*v2;
                wB[10] =  3 * v*v2 + 6 * u*v2;

                wB[8]  = -3 * v*v2 - 6 * v2*w;
                wB[4]  = -9 * v*w2 - 6 * v2*w;

                wB[5]  = 12 * vw * (w - 2*u);
                wB[6]  = 12 * uv * (2*w - u);
                wB[9]  = 12 * v2 * (w - u);
            } else {
                wB[0]  = -3 * w2;
                wB[3]  =  0;
                wB[11] =  3 * v2;

                wB[1]  = -9 * u*w2 - 6 * u2*w;
                wB[2]  = -3 * u*u2 - 6 * u2*w;

                wB[7]  =  3 * u*u2 + 6 * u2*v;
                wB[10] =  9 * u*v2 + 6 * u2*v;

                wB[8]  = -3 * v * (v2 - vw - 2*w2);
                wB[4]  =  3 * w * (w2 - vw - 2*v2);

                wB[5]  = 12 * uw * (w - 2*v);
                wB[6]  = 12 * u2 * (w - v);
                wB[9]  = 12 * uv * (2*w - v);
            }
        } else if (totalOrder == 2) {
            if (ds == 2) {
                wB[0]  =  6 * w;
                wB[3]  =  6 * u;
                wB[11] =  0;

                wB[1]  =  6 * (u2 - uw - 2*w2);
                wB[2]  =  6 * (w2 - uw - 2*u2);

                wB[7]  =  6 * v2 + 18 * uv;
                wB[10] =  6 * v2;

                wB[8]  =  6 * v2;
                wB[4]  =  6 * v2 + 18 * vw;

                wB[5]  =  24 * (uv - 2*vw);
                wB[6]  =  24 * (vw - 2*uv);
                wB[9]  = -24 *  v2;
            } else if (dt == 2) {
                wB[0]  =  6 * w;
                wB[3]  =  0;
                wB[11] =  6 * v;

                wB[1]  =  6 * u2 + 18 * uw;
                wB[2]  =  6 * u2;

                wB[7]  =  6 * u2;
                wB[10] =  6 * u2 + 18 * uv;

                wB[8]  =  6 * (w2 - vw - 2*v2);
                wB[4]  =  6 * (v2 - vw - 2*w2);

                wB[5]  =  24 * (uv - 2*uw);
                wB[6]  = -24 *  u2;
                wB[9]  =  24 * (uw - 2*uv);
            } else {
                wB[0]  =  6 * w;
                wB[3]  =  0;
                wB[11] =  0;

                wB[1]  =  6 * (u2 +   uw - 1.5f*w2);
                wB[2]  = -3 * (u2 + 4*uw);

                wB[7]  =  9 * u2 + 12 * uv;
                wB[10] =  9 * v2 + 12 * uv;

                wB[8]  = -3 * (v2 + 4*vw);
                wB[4]  =  6 * (v2 +   vw - 1.5f*w2);

                wB[5]  =  24 * (uv - vw - uw + 0.5f*w2);
                wB[6]  = -24 * (uv - uw      + 0.5f*u2);
                wB[9]  = -24 * (uv - vw      + 0.5f*v2);
            }
        } else {
            assert(totalOrder <= 2);
        }
    }
} // end namespace

template <typename REAL>
int
EvalBasisBezierTri(REAL s, REAL t,
    REAL wP[12], REAL wDs[12], REAL wDt[12],
    REAL wDss[12], REAL wDst[12], REAL wDtt[12]) {

    if (wP) {
        evalBezierTriDerivWeights<REAL>(s, t, 0, 0, wP);
    }
    if (wDs && wDt) {
        evalBezierTriDerivWeights(s, t, 1, 0, wDs);
        evalBezierTriDerivWeights(s, t, 0, 1, wDt);

        if (wDss && wDst && wDtt) {
            evalBezierTriDerivWeights(s, t, 2, 0, wDss);
            evalBezierTriDerivWeights(s, t, 1, 1, wDst);
            evalBezierTriDerivWeights(s, t, 0, 2, wDtt);
        }
    }
    return 12;
}


//
//  Hybrid (cubic-quartic) Gregory triangle:
//
//  As with the Bezier triangle, and consistent with Loop, Schaefer at al (in
//  ("Approximating Subdivision Surfaces with Gregory Patches for Hardware
//  Tessellation") we use a cubic-quartic hybrid Gregory patch.  Like the
//  quad Gregory patch, this patch uses Bezier basis functions (from the
//  cubic-quartic hybrid above) and rational multipliers to blend pairs of
//  interior points (face points).
//
namespace {
    //
    //  Expanding a set of 12 Bezier basis functions for the 6 (3 pairs) of 
    //  rational weights for the 15 Gregory basis functions:
    //
    template <typename REAL>
    void
    convertBezierWeightsToGregory(REAL const wB[12], REAL const rG[6], REAL wG[15]) {

        wG[0]  = wB[0];
        wG[1]  = wB[1];
        wG[2]  = wB[4];
        wG[3]  = wB[5] * rG[0];
        wG[4]  = wB[5] * rG[1];

        wG[5]  = wB[3];
        wG[6]  = wB[7];
        wG[7]  = wB[2];
        wG[8]  = wB[6] * rG[2];
        wG[9]  = wB[6] * rG[3];

        wG[10] = wB[11];
        wG[11] = wB[8];
        wG[12] = wB[10];
        wG[13] = wB[9] * rG[4];
        wG[14] = wB[9] * rG[5];
    }
} // end namespace

template <typename REAL>
int
EvalBasisGregoryTri(REAL s, REAL t,
    REAL wP[15], REAL wDs[15], REAL wDt[15],
    REAL wDss[15], REAL wDst[15], REAL wDtt[15]) {

    //
    //  Bezier basis functions are denoted with B while the rational multipliers for the
    //  interior points will be denoted G -- so we have B(s,t) and G(s,t) (though we
    //  switch to barycentric (u,v,w) briefly to compute G)
    //
    REAL BP[12], BDs[12], BDt[12], BDss[12], BDst[12], BDtt[12];

    REAL G[6] = { 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f };
    REAL u = s;
    REAL v = t;
    REAL w = 1 - u - v;

    if ((u + v) > 0) {
        G[0]  = u / (u + v);
        G[1]  = v / (u + v);
    }
    if ((v + w) > 0) {
        G[2] = v / (v + w);
        G[3] = w / (v + w);
    }
    if ((w + u) > 0) {
        G[4] = w / (w + u);
        G[5] = u / (w + u);
    }

    //
    //  Compute Bezier basis functions and convert, adjusting interior points:
    //
    if (wP) {
        evalBezierTriDerivWeights<REAL>(s, t, 0, 0, BP);
        convertBezierWeightsToGregory(BP, G, wP);
    }
    if (wDs && wDt) {
        //  TBD -- ifdef OPENSUBDIV_GREGORY_EVAL_TRUE_DERIVATIVES

        evalBezierTriDerivWeights(s, t, 1, 0, BDs);
        evalBezierTriDerivWeights(s, t, 0, 1, BDt);

        convertBezierWeightsToGregory(BDs, G, wDs);
        convertBezierWeightsToGregory(BDt, G, wDt);

        if (wDss && wDst && wDtt) {
            evalBezierTriDerivWeights(s, t, 2, 0, BDss);
            evalBezierTriDerivWeights(s, t, 1, 1, BDst);
            evalBezierTriDerivWeights(s, t, 0, 2, BDtt);

            convertBezierWeightsToGregory(BDss, G, wDss);
            convertBezierWeightsToGregory(BDst, G, wDst);
            convertBezierWeightsToGregory(BDtt, G, wDtt);
        }
    }
    return 15;
}

//
//  Higher level basis evaluation functions that deal with parameterization and
//  boundary issues (reflected in PatchParam) for all patch types:
//
template <typename REAL>
int
EvaluatePatchBasisNormalized(int patchType, PatchParam const & param, REAL s, REAL t,
    REAL wP[], REAL wDs[], REAL wDt[],
    REAL wDss[], REAL wDst[], REAL wDtt[], REAL sharpness) {

    int boundaryMask = param.GetBoundary();

    bool needBounding = (sharpness==0) && boundaryMask;

    int nPoints = 0;
    if (patchType == PatchDescriptor::REGULAR) {
        nPoints = EvalBasisBSpline(s, t, wP, wDs, wDt, wDss, wDst, wDtt, boundaryMask, sharpness);
        if (needBounding) {
            boundBasisBSpline(boundaryMask, wP, wDs, wDt, wDss, wDst, wDtt);
        }
    } else if (patchType == PatchDescriptor::LOOP) {
        nPoints = EvalBasisBoxSplineTri(s, t, wP, wDs, wDt, wDss, wDst, wDtt);
        if (needBounding) {
            boundBasisBoxSplineTri(boundaryMask, wP, wDs, wDt, wDss, wDst, wDtt);
        }
    } else if (patchType == PatchDescriptor::GREGORY_BASIS) {
        nPoints = EvalBasisGregory(s, t, wP, wDs, wDt, wDss, wDst, wDtt);
    } else if (patchType == PatchDescriptor::GREGORY_TRIANGLE) {
        nPoints = EvalBasisGregoryTri(s, t, wP, wDs, wDt, wDss, wDst, wDtt);
    } else if (patchType == PatchDescriptor::QUADS) {
        nPoints = EvalBasisLinear(s, t, wP, wDs, wDt, wDss, wDst, wDtt);
    } else if (patchType == PatchDescriptor::TRIANGLES) {
        nPoints = EvalBasisLinearTri(s, t, wP, wDs, wDt, wDss, wDst, wDtt);
    } else {
        assert(0);
    }
    return nPoints;
}

template <typename REAL>
int
EvaluatePatchBasis(int patchType, PatchParam const & param, REAL s, REAL t,
    REAL wP[], REAL wDs[], REAL wDt[],
    REAL wDss[], REAL wDst[], REAL wDtt[], REAL sharpness) {

    REAL derivSign = 1.0f;

    if ((patchType == PatchDescriptor::LOOP) ||
        (patchType == PatchDescriptor::GREGORY_TRIANGLE) ||
        (patchType == PatchDescriptor::TRIANGLES)) {
        param.NormalizeTriangle(s, t);
        if (param.IsTriangleRotated()) {
            derivSign = -1.0f;
        }
    } else {
        param.Normalize(s, t);
    }

    int nPoints = EvaluatePatchBasisNormalized(
        patchType, param, s, t, wP, wDs, wDt, wDss, wDst, wDtt, sharpness);

    if (wDs && wDt) {
        REAL d1Scale = derivSign * (REAL)(1 << param.GetDepth());

        for (int i = 0; i < nPoints; ++i) {
            wDs[i] *= d1Scale;
            wDt[i] *= d1Scale;
        }

        if (wDss && wDst && wDtt) {
            REAL d2Scale = derivSign * d1Scale * d1Scale;

            for (int i = 0; i < nPoints; ++i) {
                wDss[i] *= d2Scale;
                wDst[i] *= d2Scale;
                wDtt[i] *= d2Scale;
            }
        }
    }
    return nPoints;
}

//
//  Explicit float and double instantiations:
//
template int EvaluatePatchBasisNormalized<float>(int patchType, PatchParam const & param,
    float s, float t, float wP[], float wDs[], float wDt[], float wDss[], float wDst[], float wDtt[], float sharpness);
template int EvaluatePatchBasis<float>(int patchType, PatchParam const & param,
    float s, float t, float wP[], float wDs[], float wDt[], float wDss[], float wDst[], float wDtt[], float sharpness);

template int EvaluatePatchBasisNormalized<double>(int patchType, PatchParam const & param,
    double s, double t, double wP[], double wDs[], double wDt[], double wDss[], double wDst[], double wDtt[], double sharpness);
template int EvaluatePatchBasis<double>(int patchType, PatchParam const & param,
    double s, double t, double wP[], double wDs[], double wDt[], double wDss[], double wDst[], double wDtt[], double sharpness);

//
//   Most basis evaluation functions are implicitly instantiated above -- Bezier
//   require explicit instantiation as they are not invoked via a patch type:
//
template int EvalBasisBezier<float>(float s, float t,
    float wP[16], float wDs[16], float wDt[16], float wDss[16], float wDst[16], float wDtt[16]);
template int EvalBasisBezierTri<float>(float s, float t,
    float wP[12], float wDs[12], float wDt[12], float wDss[12], float wDst[12], float wDtt[12]);

template int EvalBasisBezier<double>(double s, double t,
    double wP[16], double wDs[16], double wDt[16], double wDss[16], double wDst[16], double wDtt[16]);
template int EvalBasisBezierTri<double>(double s, double t,
    double wP[12], double wDs[12], double wDt[12], double wDss[12], double wDst[12], double wDtt[12]);

} // end namespace internal
} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
