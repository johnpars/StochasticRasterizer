Shader "StochasticRasterizer/UnlitStochastic"
{
    Properties
    {
        _MainTex ("_MainTex (RGBA)", 2D) = "white" {}
        _Color("Main Color", Color) = (1,1,1,1)
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }

        Pass
        {
            Tags { "LightMode" = "StochasticRasterizer_AlphaPass" }
            
            BlendOp Add
            Blend Zero OneMinusSrcColor

            ZWrite Off

            HLSLPROGRAM

            float _DebugT;

            #pragma vertex Vertex
            #pragma fragment Fragment
            #pragma enable_d3d11_debug_symbols
            #include "../_General/ShaderLibrary/Input/Transformation.hlsl"

            struct appdata
            {
                float4 vertex : POSITION;  
            };

            struct v2f 
            {
                float4 positionCS : SV_POSITION;
            };

            v2f Vertex(appdata i)
            {
                v2f o;
                o.positionCS = TransformObjectToHClip(i.vertex.xyz);
                return o;
            }

            float Fragment(v2f i) : SV_Target
            {
                return _DebugT;
            }

            ENDHLSL
        }

        Pass
        {
            Tags { "LightMode" = "StochasticRasterizer_Pass" }

            HLSLPROGRAM

            // NOTE: Required for SV_Coverage
            #pragma target 5.0

            #pragma require  geometry
            #pragma vertex   Vertex
            #pragma geometry Geometry
            #pragma fragment Fragment
            //#pragma enable_d3d11_debug_symbols
            #include "../_General/ShaderLibrary/Input/Transformation.hlsl"
            #include "ConvexHull.hlsl"

            struct appdata
            {
                float4 vertex   : POSITION;
                float2 uv       : TEXCOORD0;
            };

            struct v2g
            {
                float4 vtx_CS   : SV_POSITION;
                float3 vtx_VS   : TEXCOORD0;
                float4 pvtx_CS  : TEXCOORD1;
                float3 pvtx_VS  : TEXCOORD2;
                float2 uv       : TEXCOORD3;
            };

            struct g2f
            {
                centroid noperspective float4 convex_hull      : SV_POSITION;
                float3 view_dir         : TEXCOORD0;

#if 0
                nointerpolation float3 vertex_v0        : TEXCOORD1;
                nointerpolation float3 vertex_v1        : TEXCOORD2;
                nointerpolation float3 vertex_v2        : TEXCOORD3;
                nointerpolation float3 prev_vertex_v0   : TEXCOORD4;
                nointerpolation float3 prev_vertex_v1   : TEXCOORD5;
                nointerpolation float3 prev_vertex_v2   : TEXCOORD6;
                nointerpolation float2 uv0              : TEXCOORD7;
                nointerpolation float2 uv1              : TEXCOORD8;
                nointerpolation float2 uv2              : TEXCOORD9;                
#else
                float3 vertex_v0        : TEXCOORD1;
                float3 vertex_v1        : TEXCOORD2;
                float3 vertex_v2        : TEXCOORD3;
                float3 prev_vertex_v0   : TEXCOORD4;
                float3 prev_vertex_v1   : TEXCOORD5;
                float3 prev_vertex_v2   : TEXCOORD6;
                float2 uv0              : TEXCOORD7;
                float2 uv1              : TEXCOORD8;
                float2 uv2              : TEXCOORD9;                
#endif
            };

            // Material Property Block Parameters
            float _DebugT;
            float _DebugShowHull;
            int   _DebugShowPrim;

            CBUFFER_START(UnityPerMaterial)
            TEXTURE2D(_MainTex);
            SAMPLER(sampler_MainTex);

            float4 _MainTex_ST;
            float4 _Color;
            CBUFFER_END

            TEXTURE2D(_AlphaMaskTexture);
            SAMPLER(sampler_AlphaMaskTexture);

            float _SubframeIndex;
            TEXTURE2D(_StochasticSampleTexture);
            SAMPLER(sampler_StochasticSampleTexture);

            float _Jitter;

            uint _MSAASampleCount;
            float cam_lensRadius;
            float cam_focalDist;
            float cam_cocScale;
            float cam_pixelHeight;
            int   cam_primDebug;
            float3 topLeft;
            float3 bottomRight;
            float2 screenRes;


            int intersect3D_RayTriangle( float3 R, float3 V0, float3 V1, float3 V2, inout float3 I )
            {
                float3    u, v, n;                          // triangle floattors
                float3    dir, w0, w;                       // ray vectors
                float     r, a, b;                          // params to calc ray-plane intersect

                // get triangle edge vectors and plane normal
                u = V1 - V0;
                v = V2 - V0;
                n = normalize(cross(u,v));                  // cross product
                if (n.x == 0 && n.y == 0 && n.z == 0)       // triangle is degenerate
                    return -1;                              // do not deal with this case

                dir = R;                                    // ray direction vector
                w0 = -V0;
                a = -dot(n,w0);
                b = dot(n,dir);
                if (abs(b) < 0.000001f) {                   // ray is  parallel to triangle plane
                    if (a == 0)                             // ray lies in triangle plane
                        return 2;
                    else return 0;                          // ray disjoint from plane
                }

                // get intersect point of ray with triangle plane
                r = a / b;
                if (r < 0.0)                                // ray goes away from triangle
                    return 0;                               // => no intersect
                // for a segment, also test if (r > 1.0) => no intersect

                I = r * dir;                                // intersect point of ray and plane

                float    uu, uv, vv, wu, wv, D;
                uu = dot(u,u);
                uv = dot(u,v);
                vv = dot(v,v);
                w = I - V0;
                wu = dot(w,u);
                wv = dot(w,v);
                D = uv * uv - uu * vv;

                // get and test parametric coords
                float s, t;
                s = (uv * wv - vv * wu) / D;
                if (s < 0.0 || s > 1.0)                     // I is outside T
                    return 0;
                t = (uv * wu - uu * wv) / D;
                if (t < 0.0 || (s + t) > 1.0)               // I is outside T
                    return 0;

                I = float3(s,t,r);
                return 1;                                   // I is in T
            }

            float coneOfConfusion( float dist, float lr, float fd )
            {
                float d = abs( dist - fd );
                float ratio = d / dist;
                return ratio * lr * 2.0 / fd;
            }

            bool isBackface( float3 p0, float3 p1, float3 p2 )
            {
                return dot( p0, cross( p1 - p0, p2 - p0 ) ) < 0;
            }

            bool intersect( float3 pos, float3 dir, float3 p0, float3 p1, float3 p2, out float2 uv, out float dist )
            {
                uv = 0.0.xx;
                dist = 0.0;

                float3 b = p2 - p0;
                float3 c = p1 - p0;
                float3 nrm = cross( c, b );
                float d = -dot( pos - p0, nrm ) / dot( dir, nrm );

                if( d <= 0.0 || dot( dir, nrm ) < 0.0 )
                    return false;

                int2 idcs = { 0, 1 };

                float3 absn = abs(nrm);
                if( absn.x > absn.y )
                    idcs = absn.x > absn.z ? int2( 1, 2 ) : int2( 0, 1 );
                else
                    idcs = absn.y > absn.z ? int2( 2, 0 ) : int2( 0, 1 );

                int u = idcs[0];
                int v = idcs[1];

                float3 H = pos + dir * d - p0;

                float area  =  b[u] * c[v] - b[v] * c[u];
                uv.x = (b[u] * H[v] - b[v] * H[u]) / area;
                uv.y = (c[v] * H[u] - c[u] * H[v]) / area;
                dist = d;

                const float EPSILON = 0.0;
                return uv.x >= EPSILON && uv.y >= EPSILON && (1.0 - uv.x - uv.y) >= EPSILON;
            }

            float4x4 _ShutterOpenM;
            float4x4 _ShutterOpenViewMatrix;
            float4x4 _ShutterOpenProjectionMatrix;
            
            v2g Vertex (appdata v)
            {
                v2g o;
                float3 vertex = v.vertex.xyz;
                float3 prev_vertex = v.vertex.xyz;

                float4 vtx_WS    = mul( GetObjectToWorldMatrix(), float4(vertex, 1.0) );
                float4 pvtx_WS   = mul( unity_MatrixPreviousM, float4(vertex, 1.0) );
                float3 mv_WS     = (vtx_WS.xyz - pvtx_WS.xyz) * 1.f;
                vtx_WS.xyz      += mv_WS;
                pvtx_WS.xyz     -= mv_WS;
                float4 vtx_VS    = mul( GetWorldToViewMatrix(), vtx_WS );
                float4 vtx_CS    = mul( UNITY_MATRIX_P, vtx_VS );
                float4 pvtx_VS   = mul( _ShutterOpenViewMatrix, pvtx_WS );
                float4 pvtx_CS   = mul( _ShutterOpenProjectionMatrix, pvtx_VS );
                       o.vtx_VS  = vtx_VS.xyz;
                       o.vtx_CS  = vtx_CS;
                       o.pvtx_VS = pvtx_VS.xyz;
                       o.pvtx_CS = pvtx_CS;
                       o.uv      = TRANSFORM_TEX(v.uv, _MainTex);
                return o;
            }

            float2 rotvec( float2 vec, bool backface )
            {
                return backface ? float2( -vec.y, vec.x ) : -float2( -vec.y, vec.x );
            }

            [maxvertexcount(21)]
            void Geometry (triangle v2g input[3], inout TriangleStream<g2f> tristream, uint id : SV_PrimitiveID)
            {
                if(cam_primDebug && _DebugShowPrim != -1 && id != (uint) _DebugShowPrim )
                    return;

                // cull backfaces
                bool2 backface = { isBackface( input[0].vtx_VS, input[1].vtx_VS, input[2].vtx_VS ),
                                   isBackface( input[0].pvtx_VS, input[1].pvtx_VS, input[2].pvtx_VS ) };

                if( all( backface ) )
                    return;

                float3 v0_CS  = input[0].vtx_CS.xyz  / input[0].vtx_CS.w;
                float3 v1_CS  = input[1].vtx_CS.xyz  / input[1].vtx_CS.w;
                float3 v2_CS  = input[2].vtx_CS.xyz  / input[2].vtx_CS.w;
                float3 pv0_CS = input[0].pvtx_CS.xyz / input[0].pvtx_CS.w;
                float3 pv1_CS = input[1].pvtx_CS.xyz / input[1].pvtx_CS.w;
                float3 pv2_CS = input[2].pvtx_CS.xyz / input[2].pvtx_CS.w;

                // cull outside unit cube
                bool3 outside = { { (v0_CS.x < -1.0 && v1_CS.x < -1.0 && v2_CS.x < -1.0 && pv0_CS.x < -1.0 && pv1_CS.x < -1.0 && pv2_CS.x < -1.0) ||
                                    (v0_CS.x >  1.0 && v1_CS.x >  1.0 && v2_CS.x >  1.0 && pv0_CS.x >  1.0 && pv1_CS.x >  1.0 && pv2_CS.x >  1.0) },
                                  { (v0_CS.y < -1.0 && v1_CS.y < -1.0 && v2_CS.y < -1.0 && pv0_CS.y < -1.0 && pv1_CS.y < -1.0 && pv2_CS.y < -1.0) ||
                                    (v0_CS.y >  1.0 && v1_CS.y >  1.0 && v2_CS.y >  1.0 && pv0_CS.y >  1.0 && pv1_CS.y >  1.0 && pv2_CS.y >  1.0) },
                                  { (v0_CS.z < -1.0 && v1_CS.z < -1.0 && v2_CS.z < -1.0 && pv0_CS.z < -1.0 && pv1_CS.z < -1.0 && pv2_CS.z < -1.0) ||
                                    (v0_CS.z >  1.0 && v1_CS.z >  1.0 && v2_CS.z >  1.0 && pv0_CS.z >  1.0 && pv1_CS.z >  1.0 && pv2_CS.z >  1.0) } };

                if( any( outside ) )
                    return;

                // prepare vertex constant data
                g2f vtx;
                vtx.vertex_v0 = input[0].vtx_VS.xyz;
                vtx.vertex_v1 = input[1].vtx_VS.xyz;
                vtx.vertex_v2 = input[2].vtx_VS.xyz;

                vtx.prev_vertex_v0 = input[0].pvtx_VS.xyz;
                vtx.prev_vertex_v1 = input[1].pvtx_VS.xyz;
                vtx.prev_vertex_v2 = input[2].pvtx_VS.xyz;

                vtx.uv0 = input[0].uv;
                vtx.uv1 = input[1].uv;
                vtx.uv2 = input[2].uv;

#if 1
                // fix triangles from disappearing when behind plane
                float3 z[2] = { {v0_CS.z, v1_CS.z, v2_CS.z}, {pv0_CS.z, pv1_CS.z, pv2_CS.z } };
                if( any( z[0] < 0.0 || z[1] < 0.0 ) )
                {
                    // emit a triangle strip of 4 vertices
                    vtx.convex_hull = float4(-1.0, -1.0, 0.99, 1.0);
                    vtx.view_dir = mul( unity_CameraInvProjection, vtx.convex_hull).xyz;
                    tristream.Append(vtx);

                    vtx.convex_hull = float4( 1.0, -1.0, 0.99, 1.0);
                    vtx.view_dir = mul( unity_CameraInvProjection, vtx.convex_hull).xyz;
                    tristream.Append(vtx);

                    vtx.convex_hull = float4(-1.0,  1.0, 0.99, 1.0);
                    vtx.view_dir = mul( unity_CameraInvProjection, vtx.convex_hull).xyz;
                    tristream.Append(vtx);

                    vtx.convex_hull = float4( 1.0, 1.0, 0.99, 1.0);
                    vtx.view_dir = mul( unity_CameraInvProjection, vtx.convex_hull).xyz;
                    tristream.Append(vtx);

                    tristream.RestartStrip();
                    return;
                }
#endif
                // calculate cone of confusion radius
                float lr = cam_lensRadius;
                float fd = cam_focalDist;
                float cocs[6] = {   coneOfConfusion( input[0].vtx_CS.w , lr, fd ), coneOfConfusion( input[1].vtx_CS.w , lr, fd ), coneOfConfusion( input[2].vtx_CS.w , lr, fd ), 
                                    coneOfConfusion( input[0].pvtx_CS.w, lr, fd ), coneOfConfusion( input[1].pvtx_CS.w, lr, fd ), coneOfConfusion( input[2].pvtx_CS.w, lr, fd  ) };

                float3 triangle_points[12];

                float2 e, erot;
                // first triangle
                float max_coc = max( max( max( cocs[0], cocs[1] ), cocs[2] ) * cam_cocScale, cam_pixelHeight );
                e = normalize( v1_CS.xy - v0_CS.xy );
                erot = rotvec( e, backface.x );
                triangle_points[0] = float3( v0_CS.xy + erot * max_coc - e * max_coc, v0_CS.z );
                triangle_points[1] = float3( v1_CS.xy + erot * max_coc + e * max_coc, v1_CS.z );
                e = normalize( v2_CS.xy - v1_CS.xy );
                erot = rotvec( e, backface.x );
                triangle_points[2] = float3( v1_CS.xy + erot * max_coc - e * max_coc, v1_CS.z );
                triangle_points[3] = float3( v2_CS.xy + erot * max_coc + e * max_coc, v2_CS.z );
                e = normalize( v0_CS.xy - v2_CS.xy );
                erot = rotvec( e, backface.x );
                triangle_points[4] = float3( v2_CS.xy + erot * max_coc - e * max_coc, v2_CS.z );
                triangle_points[5] = float3( v0_CS.xy + erot * max_coc + e * max_coc, v0_CS.z );
                // second triangle
                max_coc = max( max( max( cocs[3], cocs[4] ), cocs[5] ) * cam_cocScale, cam_pixelHeight );
                e = normalize( pv1_CS.xy - pv0_CS.xy );
                erot = rotvec( e, backface.y );
                triangle_points[6] = float3( pv0_CS.xy + erot * max_coc - e * max_coc, pv0_CS.z );
                triangle_points[7] = float3( pv1_CS.xy + erot * max_coc + e * max_coc, pv1_CS.z );
                e = normalize( pv2_CS.xy - pv1_CS.xy );
                erot = rotvec( e, backface.y );
                triangle_points[8] = float3( pv1_CS.xy + erot * max_coc - e * max_coc, pv1_CS.z );
                triangle_points[9] = float3( pv2_CS.xy + erot * max_coc + e * max_coc, pv2_CS.z );
                e = normalize( pv0_CS.xy - pv2_CS.xy );
                erot = rotvec( e, backface.y );
                triangle_points[10] = float3( pv2_CS.xy + erot * max_coc - e * max_coc, pv2_CS.z );
                triangle_points[11] = float3( pv0_CS.xy + erot * max_coc + e * max_coc, pv0_CS.z );


                // enlarge CS triangle based on coc
                float2 ctr  =  v0_CS.xy + 0.333 * ( v1_CS.xy -  v0_CS.xy) + 0.333 * ( v2_CS.xy -  v0_CS.xy);
                float2 pctr = pv0_CS.xy + 0.333 * (pv1_CS.xy - pv0_CS.xy) + 0.333 * (pv2_CS.xy - pv0_CS.xy);
                float  coc_max = max( max( cocs[0], cocs[1] ), max( max( cocs[2], cocs[3] ), max( cocs[4], cocs[5] ) ) );
                 v0_CS.xy += normalize(  v0_CS.xy -  ctr ) * (coc_max * cam_cocScale + 2 * cam_pixelHeight);
                 v1_CS.xy += normalize(  v1_CS.xy -  ctr ) * (coc_max * cam_cocScale + 2 * cam_pixelHeight);
                 v2_CS.xy += normalize(  v2_CS.xy -  ctr ) * (coc_max * cam_cocScale + 2 * cam_pixelHeight);
                pv0_CS.xy += normalize( pv0_CS.xy - pctr ) * (coc_max * cam_cocScale + 2 * cam_pixelHeight);
                pv1_CS.xy += normalize( pv1_CS.xy - pctr ) * (coc_max * cam_cocScale + 2 * cam_pixelHeight);
                pv2_CS.xy += normalize( pv2_CS.xy - pctr ) * (coc_max * cam_cocScale + 2 * cam_pixelHeight);

                float3 min_xyz      = min(v0_CS - float3( cocs[0].xx, 0.0 ), v1_CS - float3( cocs[1].xx, 0.0 ));
                       min_xyz      = min(v2_CS - float3( cocs[2].xx, 0.0 ), min_xyz);
                float3 max_xyz      = max(v0_CS + float3( cocs[0].xx, 0.0 ), v1_CS + float3( cocs[1].xx, 0.0 ));
                       max_xyz      = max(v2_CS + float3( cocs[2].xx, 0.0 ), max_xyz);
                float3 prev_min_xyz = min(pv0_CS - float3( cocs[3].xx, 0.0 ), pv1_CS - float3( cocs[4].xx, 0.0 ));
                       prev_min_xyz = min(pv2_CS - float3( cocs[5].xx, 0.0 ), prev_min_xyz);
                float3 prev_max_xyz = max(pv0_CS + float3( cocs[3].xx, 0.0 ), pv1_CS + float3( cocs[4].xx, 0.0 ));
                       prev_max_xyz = max(pv2_CS + float3( cocs[5].xx, 0.0 ), prev_max_xyz);

                min_xyz = min(min_xyz, prev_min_xyz);
                max_xyz = max(max_xyz, prev_max_xyz);
                max_xyz.z = min( 0.999, max_xyz.z );
#if 1
#if 0
                float2 zero = 0.0.xx;
                uint emitted;
                float2 convex_hull[9];
                for( int ifoo = 0; ifoo < 9; ifoo++ ) { convex_hull[ifoo] = zero; }
                findConvexHull( v0_CS, v1_CS, v2_CS, pv0_CS, pv1_CS, pv2_CS, convex_hull, emitted );
                for( uint i = 0; i < emitted; i++ )
                {
                    vtx.convex_hull = float4( convex_hull[i].xy, max_xyz.z, 1.0 );
                    vtx.view_dir = mul( unity_CameraInvProjection, vtx.convex_hull ).xyz;
                    tristream.Append(vtx);
                }
#else
                float2 zero = 0.0.xx;
                uint emitted;
                float2 convex_hull[outmax];
                for( int ifoo = 0; ifoo < outmax; ifoo++ ) { convex_hull[ifoo] = zero; }
                findConvexHullExt( triangle_points, convex_hull, emitted );
                for( uint i = 0; i < emitted; i++ )
                {
                    vtx.convex_hull = float4( convex_hull[i].xy, max_xyz.z, 1.0 );
                    vtx.view_dir = mul( unity_CameraInvProjection, vtx.convex_hull ).xyz;
                    tristream.Append(vtx);
                }
#endif

#else
                // emit a triangle strip of 4 vertices
                vtx.convex_hull = float4(min_xyz.x, min_xyz.y, max_xyz.z, 1.0); 
                vtx.view_dir = mul( unity_CameraInvProjection, vtx.convex_hull).xyz;
                tristream.Append(vtx);

                vtx.convex_hull = float4(max_xyz.x, min_xyz.y, max_xyz.z, 1.0); 
                vtx.view_dir = mul( unity_CameraInvProjection, vtx.convex_hull).xyz;
                tristream.Append(vtx);

                vtx.convex_hull = float4(min_xyz.x, max_xyz.y, max_xyz.z, 1.0); 
                vtx.view_dir = mul( unity_CameraInvProjection, vtx.convex_hull).xyz;
                tristream.Append(vtx);

                vtx.convex_hull = float4(max_xyz.x, max_xyz.y, max_xyz.z, 1.0); 
                vtx.view_dir = mul( unity_CameraInvProjection, vtx.convex_hull).xyz;
                tristream.Append(vtx);
#endif
                tristream.RestartStrip();
            }


            float radicalInverse(int val, int scramble)
            {
                int res = val;
                res = ((res >> 16) & 0x0000ffff) | ((res << 16) & 0xffff0000);
                res = ((res >>  8) & 0x00ff00ff) | ((res <<  8) & 0xff00ff00);
                res = ((res >>  4) & 0x0f0f0f0f) | ((res <<  4) & 0xf0f0f0f0);
                res = ((res >>  2) & 0x33333333) | ((res <<  2) & 0xcccccccc);
                res = ((res >>  1) & 0x55555555) | ((res <<  1) & 0xaaaaaaaa);
                res ^= scramble;
                return (float) res * 2.3283064365386963e-10;
            }

            float RI_S( uint i, uint r )
            {
                for( uint v = 1 << 31; i; i >>= 1, v ^= v>>1 )
                {
                    if( i & 1 )
                        r ^= v;
                }
                return (float) r * 2.3283064365386963e-10;
            }

            float RI_LP(uint i, uint r)
            {
                for(uint v = 1<<31; i; i >>= 1, v |= v>>1)
                {
                    if(i & 1)
                        r ^= v;
                }
                return (float) r * 2.3283064365386963e-10;
            }

            float hash12(float2 p)
            {
                float3 p3  = frac(float3(p.xyx) * .1031);
                p3 += dot(p3, p3.yzx + 19.19);
                return frac((p3.x + p3.y) * p3.z);
            }

            float GetNoise(float2 p, int offset)
            {
                offset += _Jitter;
                int ITERATIONS = 2;
                float a = 0.0, b = a;
                for (int t = 0; t < ITERATIONS; t++)
                {
                    float v = float(t+1) * .152;
                    float2 pos = (p * v + offset + 50.0);
                    a += hash12(pos);
                }

                return a / float(ITERATIONS);
            }

            float3 GetNoise3( float2 p, int offset )
            {
                float3 res = { GetNoise( p, offset + 0 ), GetNoise( p, offset + 1 ), GetNoise( p, offset + 2 ) };
                return res;
            }

            static float2 samplePos8[] = { { 1, -3 }, { -1, 3 }, { 5, 1 }, { -3, -5 }, { -5, 5 }, { -7, -1 }, { 3, 7 }, { 7, -7 } }; 

            float4 Fragment (g2f i, inout uint coverageMask : SV_Coverage, out float depth : SV_Depth, uint id : SV_PrimitiveID) : SV_Target
            {
                float3 cvgCol[] = { float3( 0,0,0 ), float3( 1,0,0 ), float3( 0,1,0 ), float3( 0,0,1 ), float3( 1,1,0 ), float3( 1,0,1 ), float3( 0,1,1 ), float3( 1,1,1 ) };

#if 0
                // If this is evaluated, it tends to break stochastic fragment shading
                if( cam_primDebug && _DebugShowHull > 0 )
                {
                    depth = i.convex_hull.z;
                    return float4( cvgCol[id & 7], 1.0 );
                }
#endif
                depth = 0.0;
                float4 position = i.convex_hull;

                // lens stuff
                float fd = cam_focalDist;
                float lr = cam_lensRadius;
                float3 view_dir    = i.view_dir * float3(1, -1, 1);
                float3 view_target = view_dir / abs(view_dir.z) * fd;

                coverageMask = 0;
                float  hits = 0;
                float  avg_depth = 0.0;
                float  last_depth = 0.0;
                float2 uv;
                uint   bit = 1;
                for (uint k = 0; k < _MSAASampleCount; k++, bit <<= 1)
                {
                    float2 pixel_offset = samplePos8[k] / 8.0 * cam_pixelHeight.xx;
                    view_dir = view_dir / abs( view_dir.z ) + float3( pixel_offset, 0.0 );

                    float3 noise = GetNoise3(position.xy, k);
                    float noise_alpha = GetNoise(position.xy, k + (id % 16000) );
                    float3 v0 = lerp(i.vertex_v0, i.prev_vertex_v0, noise.x);
                    float3 v1 = lerp(i.vertex_v1, i.prev_vertex_v1, noise.x);
                    float3 v2 = lerp(i.vertex_v2, i.prev_vertex_v2, noise.x);

#if 1
                    float2 lens_noise = noise.yz;
                           lens_noise = lens_noise * 2.0 - 1.0;
                    float3 view_pos = { lr * lens_noise, 0.0 };
#else
                    float2 lens_noise = noise.yz;
                    float  r = saturate( lens_noise.x ) * lr;
                    float  phi = saturate( lens_noise.y ) * 2.0 * 3.1416;
                    float3 view_pos = { sin( phi ) * r, cos( phi ) * r, 0.0 };
#endif
                           view_dir = normalize( view_target - view_pos );

                    float dist;
                    if (intersect( view_pos, view_dir, v0, v1, v2, uv, dist ) && (noise_alpha >= _DebugT)) 
                    {
                        coverageMask |= bit;
                        float4 p_CS = mul( UNITY_MATRIX_P, float4( view_dir * dist, 1.0 ) );
                        last_depth  = p_CS.z / p_CS.w;
                        avg_depth  += last_depth;
                        hits++;
                    }
                    else
                    {
                        //coverageMask &= ~bit;
                    }
                }

                if (coverageMask == 0)
                    discard;

                //if( coverageMask == (1 << _MSAASampleCount) - 1 )
                {
                    depth = last_depth;//avg_depth / hits;
                }

                // Reconstruct vertex, normal, texcoord.
                float2 tcs = i.uv0 + uv.x * (i.uv1 - i.uv0) + uv.y * (i.uv2 - i.uv0);

                uint cvg = coverageMask;
                uint bits = 0;
                while( cvg )
                {
                    bits += (cvg & 1) ? 1 : 0;
                    cvg >>= 1;
                }


                // Shade
                float3 c = SAMPLE_TEXTURE2D_LOD(_MainTex, sampler_MainTex, tcs, 0).rgb * _Color.rgb;
                //coverageMask = 0xff;

                // TODO
                //float c = SAMPLE_TEXTURE2D_GRAD(_MainTex, sampler_MainTex, tcs, 0, 0).rgb;

                return float4( c, 1 );
            }

            ENDHLSL
        }
    }
}

