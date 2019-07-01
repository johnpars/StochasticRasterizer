Shader "Hidden/StochasticRasterizer/FinalPass"
{
    HLSLINCLUDE

        #pragma target 4.5
        #pragma only_renderers d3d11 ps4 xboxone vulkan metal switch
        #include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Common.hlsl"

        TEXTURE2D(_ColorBuffer);
        SAMPLER(sampler_ColorBuffer);

        TEXTURE2D(_HistoryBuffer);
        SAMPLER(sampler_HistoryBuffer);

        //Debug
        //--------------------------------------------
        TEXTURE2D(_StochasticSampleTexture);
        SAMPLER(sampler_StochasticSampleTexture);

        TEXTURE2D(_FrameToRecord);
        SAMPLER(sampler_FrameToRecord);
        //--------------------------------------------

        // NOTE: This is controlled by runtime depending on continuous / finite mode
        float _AccumulationWeight;

        struct Attributes
        {
            uint vertexID : SV_VertexID;
        };

        struct Varyings
        {
            float4 positionCS : SV_POSITION;
            float2 texcoord   : TEXCOORD0;
        };

        Varyings Vert(Attributes input)
        {
            Varyings output;
            output.positionCS = GetFullScreenTriangleVertexPosition(input.vertexID);
            output.texcoord   = GetFullScreenTriangleTexCoord(input.vertexID);
            return output;
        }

        float4 Frag(Varyings input) : SV_Target0
        {
            float3 c = SAMPLE_TEXTURE2D(_ColorBuffer,   sampler_ColorBuffer,   input.texcoord.xy).rgb;
            float3 h = SAMPLE_TEXTURE2D(_HistoryBuffer, sampler_HistoryBuffer, input.texcoord.xy).rgb;
            return float4(lerp(c, h, _AccumulationWeight), 1);
        }


        float4 FragForceGamma(Varyings input) : SV_Target0
        {
            float3 s = SAMPLE_TEXTURE2D(_FrameToRecord, sampler_FrameToRecord, input.texcoord.xy).rgb;
            return float4(pow(s, 0.4545), 1);
        }


    ENDHLSL

    SubShader
    {
        Pass
        {
            ZWrite Off ZTest Always Blend Off Cull Off

            HLSLPROGRAM
                #pragma vertex   Vert
                #pragma fragment Frag
            ENDHLSL
        }

        // HACK FOR RECORDING
        Pass
        {
            ZWrite Off ZTest Always Blend Off Cull Off

            HLSLPROGRAM
                #pragma vertex Vert
                #pragma fragment FragForceGamma
            ENDHLSL
        }

    }

    Fallback Off
}