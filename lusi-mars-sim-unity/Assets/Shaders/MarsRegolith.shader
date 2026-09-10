Shader "LUSI/MarsRegolith"
{
    Properties
    {
        _BaseColor ("Regolith Color", Color) = (0.37, 0.15, 0.095, 1)
        _Roughness ("Roughness", Range(0, 1)) = 0.92
        _NoiseScale ("Surface Variation", Range(0, 2)) = 0.35
    }

    SubShader
    {
        Tags { "RenderPipeline" = "UniversalPipeline" "RenderType" = "Opaque" "Queue" = "Geometry" }

        Pass
        {
            Name "MarsRegolithForward"
            Tags { "LightMode" = "UniversalForward" }

            HLSLPROGRAM
            #pragma target 3.5
            #pragma vertex Vert
            #pragma fragment Frag
            #pragma multi_compile_fog
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS _MAIN_LIGHT_SHADOWS_CASCADE _MAIN_LIGHT_SHADOWS_SCREEN

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"

            CBUFFER_START(UnityPerMaterial)
                half4 _BaseColor;
                half _Roughness;
                half _NoiseScale;
            CBUFFER_END

            struct Attributes
            {
                float3 positionOS : POSITION;
                float3 normalOS : NORMAL;
                float2 uv : TEXCOORD0;
            };

            struct Varyings
            {
                float4 positionHCS : SV_POSITION;
                float3 positionWS : TEXCOORD0;
                half3 normalWS : TEXCOORD1;
                float2 uv : TEXCOORD2;
                UNITY_VERTEX_OUTPUT_STEREO
            };

            Varyings Vert(Attributes input)
            {
                Varyings output = (Varyings)0;
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(output);
                VertexPositionInputs positions = GetVertexPositionInputs(input.positionOS);
                VertexNormalInputs normals = GetVertexNormalInputs(input.normalOS);
                output.positionHCS = positions.positionCS;
                output.positionWS = positions.positionWS;
                output.normalWS = normals.normalWS;
                output.uv = input.uv;
                return output;
            }

            half4 Frag(Varyings input) : SV_Target
            {
                half3 normalWS = NormalizeNormalPerPixel(input.normalWS);
                Light mainLight = GetMainLight(TransformWorldToShadowCoord(input.positionWS));
                half diffuse = saturate(dot(normalWS, mainLight.direction));
                half variation = 0.94 + 0.06 * sin((input.positionWS.x + input.positionWS.z) * _NoiseScale);
                half3 color = _BaseColor.rgb * variation * (0.18 + diffuse * mainLight.color * mainLight.distanceAttenuation * mainLight.shadowAttenuation);
                color += SampleSH(normalWS) * 0.22;
                color = MixFog(color, ComputeFogFactor(input.positionHCS.z));
                return half4(color, 1.0);
            }
            ENDHLSL
        }
    }
}
