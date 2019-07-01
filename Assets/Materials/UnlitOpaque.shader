Shader "StochasticRasterizer/UnlitOpaque"
{
    Properties
    {
        [NoScaleOffset] _MainTex ("_MainTex (RGBA)", 2D) = "white" {}
        _Color("Main Color", Color) = (1,1,1,1)
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }

        Pass
        {
            Tags { "LightMode" = "StochasticRasterizer_Pass" }

            ZWrite On
            ZTest LEqual
            Cull Off

            HLSLPROGRAM

            #pragma vertex   Vertex
            #pragma fragment Fragment
            #include "../_General/ShaderLibrary/Input/Transformation.hlsl"

            struct appdata
            {
                float4 vertex   : POSITION;
                float2 uv       : TEXCOORD0;
            };

            struct v2f
            {
                float4 vtx_CS   : SV_POSITION;
                float2 uv       : TEXCOORD3;
                float3 vertexOS : TEXCOORD4;
            };

            CBUFFER_START(UnityPerMaterial)
            TEXTURE2D(_MainTex);
            SAMPLER(sampler_MainTex);

            float4 _MainTex_ST;
            float4 _Color;
            CBUFFER_END

            v2f Vertex (appdata v)
            {
                v2f o;
                o.vtx_CS  = TransformObjectToHClip(v.vertex.xyz);
                o.vertexOS = float3(v.vertex.x, -v.vertex.y, v.vertex.z);
                o.uv      = TRANSFORM_TEX(v.uv, _MainTex);

                return o;
            }

            float4 Fragment (v2f i) : SV_Target
            {             
                // Shade
                float3 c = SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, i.uv).rgb;

                return float4( c, 1 ) * _Color;
            }

            ENDHLSL
        }
    }
}
