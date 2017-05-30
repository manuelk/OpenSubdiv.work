#version 430
#extension GL_ARB_gpu_shader_int64 : enable
#extension GL_ARB_shader_clock : enable
#extension GL_NV_shader_thread_group : enable

#define BITHACKS                       1
#define RECURSION                      1
#define SHADE_PATCHES                  0
#define SHADE_DOMAIN                   0
#define SHADE_PHONG                    0
#define SHADE_DIVERGENCE               0
#define FRAGMENT_NORMALS               0
#define RECORD_STATS                   0
#define TRACE                          0
#define ARBITRARY_TWEAK                0
#define DIRECT_EVAL_SEMI_SHARP         1
#define CMAP_USE_ATTRS                 1
#define CMAP_SKIP_SUPPORT_COMPUTATION  0
#define TESS_EVAL_MODE                 equal_spacing
#define TESS_ADAPTIVE_DFAS             1

#define TRACE_CS 	0
#define TRACE_VS 	1
#define TRACE_TCS 	2
#define TRACE_TES 	3
#define TRACE_GS 	4
#define TRACE_FS 	5

layout( std430, binding = 7 ) buffer TraceData {
    int tracePos;
    int pad[3];
    uvec4 traceData[1];
};

uint warpStartClock;

void BeginTrace(int id, int stage)
{
#if TRACE 
	warpStartClock = clock2x32ARB().y;
#endif	
}

void EndTrace(int id, int stage)
{
#if TRACE 
	uint activeMask = activeThreadsNV();
	if ( (activeMask & gl_ThreadLtMaskNV) == 0 )
	{
		uint header = (bitCount( activeMask ) - 1)
		            | (stage << 5)
		            | (gl_SMIDNV << 8)
		            | (gl_WarpIDNV << 16);

		uint warpEndClock = clock2x32ARB().y;

		int warpPos = atomicAdd( tracePos, 1 );
		traceData[warpPos] = uvec4( header, id, warpStartClock, warpEndClock ); 
	}
#endif	
}

uniform mat4 ModelView;
uniform mat4 ModelViewProj;
uniform mat3 Normal;

#if TESS_ADAPTIVE_COD
uniform float invErrorPixels;
#endif
#if TESS_ADAPTIVE_TIM
uniform float invTargetLength;
#endif
#if TESS_UNIFORM
uniform float tessFactor;
#endif

#if RECORD_STATS
layout( std430, binding = 7 ) buffer StatsData {
    int tessevalThreads;
    int tableThreads;
    uint tessControlOutputVerts;
};
#endif


#define REGULAR_PATCH           0
#define RECURSIVE_PATCH         1
#define TERMINAL_PATCH          2
#define END_PATCH               3

void EvaluateBSplineBasis( float u, out vec4 b, out vec4 d )
{
  float s = 1.0 - u;
  float t = u;

  b.x = (s*s*s                               ) * 1.0/6.0;
  b.y = (4*s*s*s + t*t*t + 12*s*t*s + 6*t*s*t) * 1.0/6.0;
  b.z = (4*t*t*t + s*s*s + 12*t*s*t + 6*s*t*s) * 1.0/6.0;
  b.w = (t*t*t                               ) * 1.0/6.0;

  d.x = -s*s;
  d.y = -t*t - 4*s*t;
  d.z =  s*s + 4*s*t;
  d.w =  t*t;
}

vec3 EvaluateBSplineEdge( vec3 cp[12], float u )
{
  vec4 uBasis, uDeriv;
  EvaluateBSplineBasis( u, uBasis, uDeriv );

  return 1.0/6.0 * (cp[0]  * uBasis.x +
                    cp[1]  * uBasis.y +
                    cp[2]  * uBasis.z +
                    cp[3]  * uBasis.w) +
         4.0/6.0 * (cp[4]  * uBasis.x +
                    cp[5]  * uBasis.y +
                    cp[6]  * uBasis.z +
                    cp[7]  * uBasis.w) +
         1.0/6.0 * (cp[8]  * uBasis.x +
                    cp[9]  * uBasis.y +
                    cp[10] * uBasis.z +
                    cp[11] * uBasis.w);
}

void EvaluateBSpline( vec3 cp[16], float u, float v, out vec3 position, out vec3 normal )
{
  vec4 uBasis, vBasis, uDeriv, vDeriv;
  EvaluateBSplineBasis(u, uBasis, uDeriv);
  EvaluateBSplineBasis(v, vBasis, vDeriv);

  position = vec3(0);
  vec3 tangent = vec3(0);
  vec3 bitangent = vec3(0);
  
  for (int i = 0; i < 4; i++) 
  {
    vec3 positionBasis = (cp[i*4 + 0] * uBasis.x +
                          cp[i*4 + 1] * uBasis.y +
                          cp[i*4 + 2] * uBasis.z +
                          cp[i*4 + 3] * uBasis.w);

    vec3 positionDeriv = (cp[i*4 + 0] * uDeriv.x +
                          cp[i*4 + 1] * uDeriv.y +
                          cp[i*4 + 2] * uDeriv.z +
                          cp[i*4 + 3] * uDeriv.w);

    position  += vBasis[i] * positionBasis;
    tangent   += vBasis[i] * positionDeriv;
    bitangent += vDeriv[i] * positionBasis;
  }

  normal = normalize( cross( bitangent, tangent ) );
}

void EvaluateBSplineInner(vec3 cp[16], vec4 uBasis, vec4 vBasis, vec4 uDeriv, vec4 vDeriv, out vec3 position, out vec3 tan0, out vec3 tan1)
{
  position = vec3(0);
  tan0 = vec3(0);
  tan1 = vec3(0);

  for (int i = 0; i < 4; i++)
  {
    vec3 positionBasis = (cp[i * 4 + 0] * uBasis.x +
      cp[i * 4 + 1] * uBasis.y +
      cp[i * 4 + 2] * uBasis.z +
      cp[i * 4 + 3] * uBasis.w);

    vec3 positionDeriv = (cp[i * 4 + 0] * uDeriv.x +
      cp[i * 4 + 1] * uDeriv.y +
      cp[i * 4 + 2] * uDeriv.z +
      cp[i * 4 + 3] * uDeriv.w);

    position += vBasis[i] * positionBasis;
    tan0 += vBasis[i] * positionDeriv;
    tan1 += vDeriv[i] * positionBasis;
  }
}

// compute the "crease matrix" for modifying basis weights,
// given a desired (integral) sharpness value.
mat4 ComputeCreaseMatrixForSharpness(float creaseSharpness)
{
  float s = exp2(creaseSharpness);

  float sOver3 = s / 3.0;
  float oneOver6S = 1.0 / (6.0 * s);

  float A = -s*s + sOver3 * 5.5 + oneOver6S     - 1.0;
  float B =        sOver3       + oneOver6S     + 0.5;
  float C =        sOver3       - oneOver6S*2.0 + 1.0;
  float E =        sOver3       + oneOver6S     - 0.5;
  float F =      - sOver3 * 0.5 + oneOver6S;

  mat4 M_s = mat4(
    1.0, A, -2.0*A, A,
    0.0, B, -2.0*E, E,
    0.0, F, C, F,
    0.0, E, -2.0*E, B);

  return M_s;
}

// equivalent of GLSL `mix`, for the `mat4` type
mat4 mixMat(mat4 x, mat4 y, float a)
{
  return x * (1 - a) + y * a;
}

// compute the correct crease matrix to apply to our basis weights,
// based on the coordinate orthogonal to the crease edge, and
// the sharpness of the edge (which might be fractional)
//
// TODO: drop last param
mat4 ComputeCreaseMatrix( float creaseCoord, float creaseSharpness)
{
  mat4 M_inf = mat4(
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, -1.0, 2.0, 0.0);

  float creaseSharpnessFloor = floor(creaseSharpness);
  float creaseSharpnessCeil = creaseSharpnessFloor + 1;
  float creaseSharpnessFrac = creaseSharpness - creaseSharpnessFloor;

  float creaseWidthFloor = 1 - exp2(-creaseSharpnessFloor);
  float creaseWidthCeil = 1 - exp2(-creaseSharpnessCeil);

    // we compute the matrix for both the floor and ceiling of
    // the sharpness value, and then interpolate between them
    // as needed.

  mat4 M_s_floor = ComputeCreaseMatrixForSharpness(creaseSharpnessFloor);
  mat4 M_s_ceil = ComputeCreaseMatrixForSharpness(creaseSharpnessCeil);

  mat4 M = M_inf;
  if (creaseCoord > creaseWidthCeil)
  {
    M = mixMat(M_s_floor, M_s_ceil, creaseSharpnessFrac);
  }
  else if (creaseCoord > creaseWidthFloor)
  {
    M = mixMat(M_s_floor, M_inf, creaseSharpnessFrac);
  }

  return M;
}

// compute the "crease matrix" for modifying basis weights,
// given two desired (integral) sharpness values.
mat4 ComputeMixedCreaseMatrix(float sharp1, float sharp2, float t, float tInf)
{
  float s1 = exp2(sharp1);
  float s2 = exp2(sharp2);

  float sOver3 = mix(s1, s2, t) / 3.0;
  float oneOverS1 = 1.0 / s1;
  float oneOverS2 = 1.0 / s2;
  float oneOver6S = mix(oneOverS1, oneOverS2, t) / 6.0;
  float sSqr = mix(s1*s1, s2*s2, t);
  
  float A = -sSqr + sOver3 * 5.5 + oneOver6S     - 1.0;
  float B =         sOver3       + oneOver6S     + 0.5;
  float C =         sOver3       - oneOver6S*2.0 + 1.0;
  float E =         sOver3       + oneOver6S     - 0.5;
  float F =       - sOver3 * 0.5 + oneOver6S;

  mat4 M_s = mat4(
    1.0, A*tInf,           -2.0*A*tInf,          A*tInf,
    0.0, mix(1.0,B,tInf),  -2.0*E*tInf,          E*tInf,
    0.0, F*tInf,           mix(1.0,C,tInf),      F*tInf,
    0.0, mix(-1.0,E,tInf), mix(2.0,-2.0*E,tInf), B*tInf);

  return M_s;
}

// compute the correct crease matrix to apply to our basis weights,
// based on the coordinate orthogonal to the crease edge, and
// the sharpness of the edge (which might be fractional)
void EvaluateCreaseBSplineBasis(float v, float sharp, inout vec4 vBasis, inout vec4 vDeriv)
{
  float sharpFloor = floor(sharp);
  float sharpCeil = sharpFloor + 1;
  float sharpFrac = sharp - sharpFloor;

  float creaseWidthFloor = 1 - exp2(-sharpFloor);
  float creaseWidthCeil = 1 - exp2(-sharpCeil);

  // we compute the matrix for both the floor and ceiling of
  // the sharpness value, and then interpolate between them
  // as needed.

  float tA = v > creaseWidthCeil ? sharpFrac : 0.0;
  float tB = 0;

  if (v > creaseWidthFloor)
    tB = 1.0-sharpFrac;
  if (v > creaseWidthCeil)
    tB = 1.0;

  mat4 M = ComputeMixedCreaseMatrix(sharpFloor, sharpCeil, tA, tB);

  EvaluateBSplineBasis(v, vBasis, vDeriv);
  vBasis = M * vBasis;
  vDeriv = M * vDeriv;
}

vec2 EdgeIndexToCoord(uint index)
{
	// {0, 1, 2, 3} -> {{0,0}, {1,0}, {1,1}, {0,1}}
	float u = ((index ^ (index>>1)) & 1) != 0 ? 1.0 : 0.0;
	float v = (index & 2) != 0 ? 1.0 : 0.0;
	return vec2(u, v);
}

vec2 GetScreenVert( vec3 p )
{
	vec4 clipPos = ModelViewProj * vec4( p, 1 );

	// Clamp w to near plane... avoids divide by 0 and mirror when points are behind camera.
	vec2 screenPos = clipPos.xy / max( 0.0001, clipPos.w );
	
	// Clamp to slightly extended screen borders to avoid excessive tessellation for offscreen edges,
	//  especially if they approach w<=0.
	screenPos = max( vec2( -2.0 ), min( vec2( 2.0 ), screenPos ) );

	return screenPos;
}

#if TESS_ADAPTIVE_DFAS
float EvaluateDFASEdgeFactor( vec3 origin, vec3 end )
{
	vec3 center = (origin + end) * 0.5;
	center = ( ModelView * vec4( center, 1 ) ).xyz;

	return clamp( 32.0 * (1.0-abs(center.z)/10.0), 1.0, 64.0 );
}
#endif

#if TESS_ADAPTIVE_COD
float EvaluateCODEdgeFactor( vec3 origin, vec3 end, vec3 mid )
{
	vec3 midEst = (origin + end) * 0.5;

	vec2 midEstScreen = GetScreenVert( midEst );
	vec2 midScreen = GetScreenVert( mid );

	float d = length( midScreen - midEstScreen );

	return max( 1.0, round( sqrt( d * invErrorPixels ) ) );
}
#endif

#if TESS_ADAPTIVE_TIM
float GetScale( vec3 p )
{
	return (ModelViewProj * vec4(p,1)).w;
}

float EvaluateTimEdgeFactor( vec3 originSupport, vec3 endSupport, vec3 mid )
{
	vec3 midEst = (originSupport + endSupport) * 0.5;

	float len0 = length( originSupport - endSupport );
	float len1 = length( mid - midEst );

	float scale = 1.0f / GetScale( midEst );
	len0 *= scale;
	len1 *= scale;

	// combine both arithmetic and geometric means
	float len = 0.5*(len0 + len1) + sqrt( len0 * len1 );

	// throw in an extra `sqrt`, since it biases us toward
	// higher tessellation rates at distance, which seems
	// to improve our results.
	len = sqrt(len);

	return len * invTargetLength;
}
#endif

float RoundFactor( float factor )
{
#if TESS_EVAL_MODE_FRACTIONAL_EVEN
	// round the factor up to a multiple of four
	factor = factor * 0.25;
	factor = ceil( factor );
	factor = factor * 4;
#else
	// round the factor up to an even integer
	factor = factor * 0.5;
	factor = ceil( factor );
	factor = factor * 2;
#endif
	return factor;
}

float EvaluateInnerFactor( uint axis )
{
	return 0.5 * (gl_TessLevelOuter[axis ^ 1] + gl_TessLevelOuter[axis ^ 3]);
}



#define SUBD_PATCH_FLAG_EDGE0_NGON_JUNCTION		0x01
#define SUBD_PATCH_FLAG_EDGE1_NGON_JUNCTION		0x02
#define SUBD_PATCH_FLAG_EDGE2_NGON_JUNCTION		0x04
#define SUBD_PATCH_FLAG_EDGE3_NGON_JUNCTION		0x08
#define SUBD_PATCH_FLAG_NGON					0x10

struct SubDPatch
{
	uint 	cmapOffset;
	uint 	weightsOffset;
	uint 	flags;
	uint 	controlCount;
	uint 	rootNode;

	uint	cornerLimits[4];
	uint 	midLimits[4];

	uint 	firstControl;
	uint 	firstSupport;
	uint 	supportCountForLevel[8];
	
	uint 	cmap;
};

layout( binding = 0 ) buffer PatchData {
    SubDPatch patches[1];
};

layout( binding = 1 ) buffer PatchControlVertsData {
    uint patchControlVerts[1];
};

layout( binding = 2 ) buffer CMapData {
	uint cmap[1];
};

layout( binding = 3 ) buffer WeightsData {
    float weights[1];
};

layout( std430, binding = 4 ) buffer SupportsData {
    vec4 supports[];
};

layout( std430, binding = 5 ) buffer VertsData {
    vec4 verts[];
};

#if CMAP_USE_ATTRS
vec3 GetControlVertAttr(
	uint firstControl,
	uint index );
#endif

vec3 GetControlVert(
	uint firstControl,
	uint index )
{
	uint vertIndex = patchControlVerts[firstControl + index];
	return verts[vertIndex].xyz;	
}

vec3 EvaluateSupport(
	uint patchIndex,
	uint weightsOffset,
	uint influenceCount,
	uint firstControl,
	uint supportIndex )
{
	vec3 result = vec3( 0 );

	uint influenceOffset = weightsOffset + influenceCount*supportIndex;

	uint influenceIter = 0;

#if CMAP_USE_ATTRS
	uint attrCount = min(influenceCount, 32);
	// get first 32 influences from attributes
	for ( ; influenceIter < attrCount; influenceIter++ )
	{
		float weight = weights[influenceOffset + influenceIter];
		vec3 p = GetControlVertAttr(firstControl, influenceIter);

		result += p * weight;
	}
	// fall back to memory for the remaining attributes
#endif
	for (; influenceIter < influenceCount; influenceIter++ )
	{
		float weight = weights[influenceOffset + influenceIter];
		vec3 p = GetControlVert(firstControl, influenceIter);

		result += p * weight;
	}

	return result;
}

void EvaluateSupportsInner( uint patchIndex, uint startAt, uint supportCount )
{
	uint weightsOffset = patches[patchIndex].weightsOffset;
	uint firstControl = patches[patchIndex].firstControl;
	uint firstSupport = patches[patchIndex].firstSupport;
	uint influenceCount = patches[patchIndex].controlCount;

	for ( uint supportIter = 0; ; supportIter += 32 )
	{
		uint supportIndex = (supportIter + startAt) + gl_InvocationID;
		if ( supportIndex >= supportCount )
			break;

		vec3 result = EvaluateSupport(patchIndex, weightsOffset, influenceCount, firstControl, supportIndex);

		uint supportOffset = firstSupport + supportIndex;

		supports[supportOffset] = vec4( result, 0 );
	}

#if RECORD_STATS
	if ( gl_InvocationID == 0)
	{
		atomicAdd( tessControlOutputVerts, supportCount - startAt );	
	}
#endif
}

void EvaluateSupports( uint patchIndex, int level, uint startAt )
{
	uint supportCount = patches[patchIndex].supportCountForLevel[level];
	EvaluateSupportsInner( patchIndex, startAt, supportCount );
}

vec3 GetSupport( uint firstSupport, uint index )
{
	uint supportOffset = firstSupport + index;
	return supports[supportOffset].xyz;
}

int controlCount;
int firstControl;
int firstSupport;

vec3 getSupportVert(int supportIndex)
{
	uint supportOffset = firstSupport + supportIndex;
	return supports[supportOffset].xyz;
}

vec3 getVert(uint index)
{
	int supportIndex = int(index);
	return getSupportVert(supportIndex);
}

vec3 getLimitVert(uint index) { return getVert(index); }

struct EvaluateResult
{
	vec3 position;
	vec3 normal;
	vec2 leafCoord;
	bool debugKill;
};

uint cmapOffset = 0;

EvaluateResult EvaluateRecursivePatchImpl(
	uint inKind,
	uint inNode,
	vec2 coord )
{
	EvaluateResult eval;

	eval.debugKill = false;

	float u = coord.x;
	float v = coord.y;

	uint kind = inKind;
	uint node = inNode;
	uint offset;
	uint corner = 0;

#if RECURSION
	while ( kind == RECURSIVE_PATCH )
#endif
	{
		offset = bitfieldExtract( node, 0, 30 );

		if ( u >= 0.5 ) { corner ^= 1; u = 1 - u; }
		if ( v >= 0.5 ) { corner ^= 2; v = 1 - v; }
		u *= 2.0;
		v *= 2.0;

		offset = offset + corner;

		node = cmap[cmapOffset + offset];
		kind = bitfieldExtract( node, 30, 2 );
	}

	// kind is REGULAR_PATCH or TERMINAL_PATCH.
	
	offset = bitfieldExtract( node, 0, 22 );
	uint evCorner = bitfieldExtract( node, 28, 2 );
	uint stride = 4;
	bool snapToEV = false;

	corner ^= evCorner;
	if ( (corner & 1) != 0 ) u = 1.0 - u;
	if ( (corner & 2) != 0 ) v = 1.0 - v;

	if ( kind == TERMINAL_PATCH )
	{
		bool regularLimit = bitfieldExtract( node, 23, 1 ) != 0;
		uint subdivisions = bitfieldExtract( node, 24, 4 );

		float range = max( u, v );

#if BITHACKS
		uint e = bitfieldExtract( floatBitsToUint( range ), 23, 8 );
		uint l = max( 0, 126 - int( e ) );
#else
		uint l = (range == 0.0) ? 64 : uint( trunc( -log2( range ) ) );
#endif

		if ( l >= subdivisions && !regularLimit )
		{
			snapToEV = true;
		}
		else
		{
			l = min( subdivisions, l );

#if BITHACKS
			uint eDelta =  l << 23;
			u = uintBitsToFloat( floatBitsToUint( u ) + eDelta );
			v = uintBitsToFloat( floatBitsToUint( v ) + eDelta );
#else
			float scale = exp2( l );
			u *= scale;
			v *= scale;
#endif
	
			offset += 3 + 25 * l;
			stride = 5;

			if ( u >= 0.5 ) { offset += 1; u -= 0.5; }
			if ( v >= 0.5 ) { offset += 5; v -= 0.5; }
			u *= 2.0;
			v *= 2.0;
		}
	}

	vec3 tan0, tan1;
	
	if ( snapToEV )
	{
		uint positionOffset = cmap[cmapOffset + offset + 0];
		eval.position = getLimitVert( positionOffset );

		uint tan0Offset = cmap[cmapOffset + offset + 1];
		tan1 = getLimitVert( tan0Offset );

		uint tan1Offset = cmap[cmapOffset + offset + 2];
		tan0 = getLimitVert( tan1Offset );
		
		evCorner = 0;
	}
	else
	{
		vec4 vBasis, vDeriv;
#if DIRECT_EVAL_SEMI_SHARP
		bool hasCrease = bitfieldExtract( node, 22, 1 ) != 0;
		if ( hasCrease )
		{
			bool creaseRot = bitfieldExtract( node, 24, 1 ) != 0;
			if ( creaseRot )
			{
				float tmp = u;
				u = 1 - v;
				v = tmp;
			}

			uint creaseOffset = offset + stride*stride;
			float s = uintBitsToFloat(cmap[cmapOffset + creaseOffset + 1]);

	        // At this point the patch has been rotated and flipped
	        // as necessary to guarantee that any crease runs along
	        // the V==1.0 edge.
	        EvaluateCreaseBSplineBasis(v, s, vBasis, vDeriv);
	    }
	    else
#endif
	    {
			EvaluateBSplineBasis( v, vBasis, vDeriv );
	    }
	    
	    vec4 uBasis, uDeriv;
		EvaluateBSplineBasis( u, uBasis, uDeriv );

		vec3 controlPoints[16];
		uint rowOffset = cmapOffset + offset;
		controlPoints[0] = getVert( cmap[rowOffset + 0] );
		controlPoints[1] = getVert( cmap[rowOffset + 1] );
		controlPoints[2] = getVert( cmap[rowOffset + 2] );
		controlPoints[3] = getVert( cmap[rowOffset + 3] );
		rowOffset += stride;
		controlPoints[4] = getVert( cmap[rowOffset + 0] );
		controlPoints[5] = getVert( cmap[rowOffset + 1] );
		controlPoints[6] = getVert( cmap[rowOffset + 2] );
		controlPoints[7] = getVert( cmap[rowOffset + 3] );
		rowOffset += stride;
		controlPoints[8] = getVert( cmap[rowOffset + 0] );
		controlPoints[9] = getVert( cmap[rowOffset + 1] );
		controlPoints[10] = getVert( cmap[rowOffset + 2] );
		controlPoints[11] = getVert( cmap[rowOffset + 3] );
		rowOffset += stride;
		controlPoints[12] = getVert( cmap[rowOffset + 0] );
		controlPoints[13] = getVert( cmap[rowOffset + 1] );
		controlPoints[14] = getVert( cmap[rowOffset + 2] );
		controlPoints[15] = getVert( cmap[rowOffset + 3] );

		EvaluateBSplineInner( controlPoints, uBasis, vBasis, uDeriv, vDeriv, eval.position, tan0, tan1 );
	}

	eval.normal = normalize( cross( tan1, tan0 ) );

	if ( bitCount( evCorner ) == 1 )
		eval.normal = -eval.normal;

	eval.leafCoord = vec2( u, v );

	return eval;
}

EvaluateResult EvaluateRecursivePatch( uint patchIndex, vec2 coord )
{
	return EvaluateRecursivePatchImpl(
		RECURSIVE_PATCH,
		patchIndex * 4,
		coord );
}

EvaluateResult EvaluateRecursivePatchTop( uint node, vec2 coord )
{
	uint kind = bitfieldExtract( node, 30, 2 );
	return EvaluateRecursivePatchImpl(
		kind,
		node,
		coord );
}

EvaluateResult EvaluatePatch( uint patchIndex, vec2 coord )
{
	controlCount = int(patches[patchIndex].controlCount);
	firstControl = int(patches[patchIndex].firstControl);
	firstSupport = int(patches[patchIndex].firstSupport);

	cmapOffset = patches[patchIndex].cmapOffset;
	uint node = patches[patchIndex].rootNode;
	return EvaluateRecursivePatchTop(
		node,
		coord );	
}


layout(quads) in;

layout(TESS_EVAL_MODE) in;

out vec3 tePosition;
out vec3 teNormal;

#if SHADE_PATCHES || FRAGMENT_NORMALS
out flat uint tePrimitiveID;
out vec2 tePatchCoord;
#endif
#if SHADE_PATCHES || SHADE_DOMAIN
out vec2 teTessCoord;
#endif

void main()
{
	BeginTrace( gl_PrimitiveID, TRACE_TES );

	EvaluateResult eval = EvaluatePatch( gl_PrimitiveID, gl_TessCoord.xy );

#if SHADE_PATCHES || FRAGMENT_NORMALS
	tePrimitiveID = gl_PrimitiveID;
	tePatchCoord = gl_TessCoord.xy;
#endif
#if SHADE_PATCHES || SHADE_DOMAIN
    teTessCoord = eval.leafCoord;
#endif

	teNormal = Normal * eval.normal;
	tePosition = eval.position;

	if ( eval.debugKill )
		tePosition = vec3( 1.0 / 0.0 );

    gl_Position = ModelViewProj * vec4( tePosition, 1 );

#if RECORD_STATS
	atomicAdd( tessevalThreads, 1 );
#endif
	EndTrace( gl_PrimitiveID, TRACE_TES );
}

