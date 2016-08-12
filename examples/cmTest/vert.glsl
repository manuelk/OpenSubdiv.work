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


#if CMAP_USE_ATTRS
in vec4 Position;
out vec3 vPosition;
#endif

void main()
{
	BeginTrace( 0, TRACE_VS );
#if CMAP_USE_ATTRS
	vPosition = Position.xyz;
#endif
	EndTrace( 0, TRACE_VS );
}

