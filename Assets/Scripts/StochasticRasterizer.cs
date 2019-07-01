using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

using UnityEngine.Rendering.PostProcessing;

using RTHandle = RTHandleSystem.RTHandle;

[ExecuteInEditMode]
public class StochasticRasterizer : RenderPipelineAsset
{
    #if UNITY_EDITOR
    [UnityEditor.MenuItem("Assets/Create/Render Pipeline/StochasticRasterizer", priority = 1)]
    static void CreateStochasticRasterizer()
    {
        var instance = ScriptableObject.CreateInstance<StochasticRasterizer>();
        UnityEditor.AssetDatabase.CreateAsset(instance, "Assets/Scripts/StochasticRasterizer.asset");
    }
#endif

    public enum AccumulationMode
    {
        Disabled   = 0,
        Finite     = 1,
        Continuous = 2
    }

    public AccumulationMode accumulationMode;

    [Range(1, 200)] public int accumulationIterations;

    public RenderTexture recordingTarget;

    protected override RenderPipeline CreatePipeline()
    {
        return new StochasticRasterizerInstance();
    }
}

public static class ShaderIDs
{
    public static readonly int _MSAASampleCount = Shader.PropertyToID("_MSAASampleCount");
    public static readonly int _AlphaMaskTexture = Shader.PropertyToID("_AlphaMaskTexture");
    public static readonly int _StochasticSampleTexture = Shader.PropertyToID("_StochasticSampleTexture");
    public static readonly int _PreviousViewMatrix = Shader.PropertyToID("_ShutterOpenViewMatrix");
    public static readonly int _PreviousProjectionMatrix = Shader.PropertyToID("_ShutterOpenProjectionMatrix");
}

public class StochasticRasterizerInstance : RenderPipeline
{
    private static readonly ShaderTagId m_PassName = new ShaderTagId("StochasticRasterizer_Pass"); //The shader pass tag just for StochasticRasterizer
    private static readonly ShaderTagId m_AlphaPassName = new ShaderTagId("StochasticRasterizer_AlphaPass");

    // MSAA
    private const int k_MSAASamples = 1;
    
    // RT 
    RTHandle m_ColorBuffer;
    RTHandle m_DepthStencilBuffer;
    RTHandle m_OpacityBuffer;

    int m_HistorySourceIndex;
    int m_HistoryDestIndex;
    RTHandle[] m_HistoryBuffers;

    StochasticRasterizer.AccumulationMode m_AccumulationMode;
    int m_AccumulationIterations;

    //Stochastic Sampling
    private const int k_SampleDim  = 128;
    private const int k_Iterations = 256;
    private const int k_PatternShift = 4;

    static readonly System.Random m_Random = new System.Random();
    Texture2D[] m_StochasticSampleTexArray;

    //Object Motion Blur: Shutter Open Matrix Hack
    private Matrix4x4 m_ObjectShutterOpenMatrix = Matrix4x4.identity;

    // Matrices
    private int m_LastFrameActive;
    private Matrix4x4 m_ShutterOpenViewMatrix_scene;
    private Matrix4x4 m_ShutterOpenProjectionMatrix_scene;
    private Matrix4x4 m_ShutterOpenViewMatrix_game;
    private Matrix4x4 m_ShutterOpenProjectionMatrix_game;

    // Engine Materials
    private Material m_FinalPass;

    // Post Processing
    private PostProcessRenderContext m_PostProcessRenderContext;

    // TEMP: Recording
    RTHandle m_RecordingBuffer;
    private bool m_IsRecording = false;

    public StochasticRasterizerInstance()
    {
        m_PostProcessRenderContext = new PostProcessRenderContext();

        int w = Screen.width; int h = Screen.height;

        // Initial state of the RTHandle system.
        // Tells the system that we will require MSAA or not so that we can avoid wasteful render texture allocation.
        // TODO: Might want to initialize to at least the window resolution to avoid un-necessary re-alloc in the player
        RTHandles.Initialize(1, 1, true, (MSAASamples)k_MSAASamples);
        RTHandles.SetReferenceSize(w, h, (MSAASamples)k_MSAASamples);

        InitializeBuffers();

        m_FinalPass = CoreUtils.CreateEngineMaterial(Shader.Find("Hidden/StochasticRasterizer/FinalPass"));
    }

    private void InitializeBuffers()
    {
        m_ColorBuffer = RTHandles.Alloc(Vector2.one, 
                                        colorFormat: SystemInfo.GetGraphicsFormat(UnityEngine.Experimental.Rendering.DefaultFormat.HDR),
                                        enableMSAA: true,
                                        name: "ColorBuffer");

        m_DepthStencilBuffer = RTHandles.Alloc(Vector2.one,
                                         depthBufferBits: DepthBits.Depth32,
                                         enableMSAA: true,
                                         name: "DepthStencilBuffer");

        m_OpacityBuffer = RTHandles.Alloc(Vector2.one,
                                          colorFormat: UnityEngine.Experimental.Rendering.GraphicsFormat.R16_SFloat,
                                          enableMSAA: false,
                                          name: "OpacityBuffer"); // NOTE: No MSAA on opacity for now.

        // Alloc History Buffers
        m_HistoryBuffers = new RTHandle[2];
        for(int i = 0; i < m_HistoryBuffers.Length; ++i)
        {
            m_HistoryBuffers[i] = RTHandles.Alloc(Vector2.one,
                                                  colorFormat: SystemInfo.GetGraphicsFormat(UnityEngine.Experimental.Rendering.DefaultFormat.HDR),
                                                  enableMSAA: false,
                                                  name: "HistoryBuffer" + i);
        }
        
        var stochasticRasterizer = GraphicsSettings.renderPipelineAsset as StochasticRasterizer;
        if (stochasticRasterizer.recordingTarget != null)
        {
            m_RecordingBuffer = RTHandles.Alloc(Vector2.one,
                                    colorFormat: UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB,
                                    depthBufferBits: DepthBits.None,
                                    enableMSAA: false,
        
                                    name: "RecordingBuffer"); // NOTE: No MSAA on recording for now.
        
            m_RecordingBuffer.SetRenderTexture(stochasticRasterizer.recordingTarget, RTCategory.Regular);
            m_IsRecording = true;
        }
        else
        {
            m_IsRecording = false;
        }
        
        //Build stochastic sampling array
        InitializeStochasticBuffer();
    }

    private void InitializeStochasticBuffer()
    {
        const int N = k_Iterations / k_PatternShift;

        m_StochasticSampleTexArray = new Texture2D[N];

        float[] times = new float[N * N];
        for(int i = 0; i < m_StochasticSampleTexArray.Length; ++i)
        {
            m_StochasticSampleTexArray[i] = new Texture2D(N, N, TextureFormat.RGBAHalf, false)
            {
                hideFlags  = HideFlags.HideAndDontSave,
                filterMode = FilterMode.Point,
                name       = "StochasticTexture" + i
            };

            // T: 0 --> 1
            for (int t = 0; t < times.Length; ++t)
            {
                times[t] = t / (float)times.Length;
            }
           
            // T: Shuffle
            int n = times.Length;  
            while (n > 1) {  
                n--;  
                int k = m_Random.Next(n + 1);  
                float t = times[k];  
                times[k] = times[n];  
                times[n] = t;  
            } 

            for (int v = 0; v < N; ++v)
            {
                for(int u = 0; u < N; ++u)
                {
                    // Lens: Normalize on unit circle
                    Vector2 lens;
                    do
                    {
                        lens.x = Random.Range(-1f, 1f);
                        lens.y = Random.Range(-1f, 1f);
                    } while (lens.SqrMagnitude() > 1f);
                    
                    m_StochasticSampleTexArray[i].SetPixel(u, v, new Vector4(times[u + v * N], lens.x, lens.y, 1));
                }
            }
            m_StochasticSampleTexArray[i].Apply();
        }
    }

    protected override void Render(ScriptableRenderContext context, Camera[] cameras)
    {
        BeginFrameRendering(cameras);

        foreach (Camera camera in cameras)
        {
            BeginCameraRendering(camera);
            
            //Culling
            ScriptableCullingParameters cullingParams;
            if (!camera.TryGetCullingParameters(out cullingParams))
                continue;
            CullingResults cull = context.Cull(ref cullingParams);

            //Camera setup some builtin variables e.g. camera projection matrices etc
            context.SetupCameraProperties(camera);
            camera.depthTextureMode |= DepthTextureMode.MotionVectors | DepthTextureMode.Depth;
            
            //Setup sort, filter, draw settings
            var sortingSettings = new SortingSettings(camera);

            var drawSettings = new DrawingSettings(m_PassName, sortingSettings);
            drawSettings.perObjectData |= PerObjectData.MotionVectors;

            var filterSettings = new FilteringSettings(RenderQueueRange.all);
            filterSettings.excludeMotionVectorObjects = false;

            InitializeFrameSettings();

#if true
            int iterations = Application.isPlaying && m_AccumulationMode == StochasticRasterizer.AccumulationMode.Finite ? m_AccumulationIterations : 1;
            for(int i = 0; i < iterations; ++i)
            { 
                //Clear
                ClearBuffers(context);
                 
                //Sky
                DrawSkybox(camera, context);

                //Shader Inputs
                PushShadingConstants(camera, context, i);

                //Opacity Accumulation
                //RenderOpacity(sortingSettings, drawSettings, filterSettings, cull, context);

                //Stochastic
                RenderStochasticForward(sortingSettings, drawSettings, filterSettings, cull, context);

                //Final Pass    
                RenderFinalPass(context, i, camera);
            }
            PresentAccumulation(context);
#endif
            
            // Gather Shutter-Open information.
            var gpuView = camera.worldToCameraMatrix;
            var gpuProj = GL.GetGPUProjectionMatrix(camera.projectionMatrix, true);

            bool isSceneCam = camera.name == "SceneCamera";
            if( isSceneCam )
            {
                m_ShutterOpenViewMatrix_scene = gpuView;
                m_ShutterOpenProjectionMatrix_scene = gpuProj;
            }
            else
            {
                m_ShutterOpenViewMatrix_game = gpuView;
                m_ShutterOpenProjectionMatrix_game = gpuProj;
            }
            
            //Get the world matrix of an object's transform on shutter close to demonstrate object motion blur
            var hack = camera.GetComponent<StochasticMaterialDebugger>();
            if(hack != null)
            {
                m_ObjectShutterOpenMatrix = hack.FirstObjectMatrix;
            }

            context.Submit();
        }
    }

    private void InitializeFrameSettings()
    {
        var stochasticRasterizer = GraphicsSettings.renderPipelineAsset as StochasticRasterizer;
        m_AccumulationMode = stochasticRasterizer.accumulationMode;
        m_AccumulationIterations = stochasticRasterizer.accumulationIterations;
    }

    private void ClearBuffers(ScriptableRenderContext context)
    {
        CommandBuffer cmd = CommandBufferPool.Get("Clear");
        
        cmd.SetRenderTarget(m_OpacityBuffer);
        cmd.ClearRenderTarget(true, true, Color.white);

        cmd.SetRenderTarget(m_ColorBuffer);
        cmd.ClearRenderTarget(true, true, Color.clear);

        cmd.SetRenderTarget(m_DepthStencilBuffer);
        cmd.ClearRenderTarget(true, false, Color.black);

        context.ExecuteCommandBuffer(cmd);
        CommandBufferPool.Release(cmd);
    }

    private void PresentAccumulation(ScriptableRenderContext context)
    {
        CommandBuffer cmd = CommandBufferPool.Get("Present Accumulation");

        if(!Application.isPlaying || m_AccumulationMode == StochasticRasterizer.AccumulationMode.Disabled)
        {
            cmd.Blit(m_ColorBuffer, BuiltinRenderTextureType.CameraTarget);
        }
        else
        { 
            cmd.Blit(m_HistoryBuffers[m_HistoryDestIndex], BuiltinRenderTextureType.CameraTarget);
        
            // Clear history in case of finite accumulation
            if(m_AccumulationMode == StochasticRasterizer.AccumulationMode.Finite)
            { 
                //Recording
                if(m_IsRecording)
                {
                    cmd.SetGlobalTexture("_FrameToRecord", m_HistoryBuffers[m_HistoryDestIndex]);
                    CoreUtils.DrawFullScreen(cmd, m_FinalPass, m_RecordingBuffer, m_ColorBuffer, shaderPassId: 1);
                }

                for(int i = 0; i < 2; ++i)
                {
                    cmd.SetRenderTarget(m_HistoryBuffers[i]);
                    cmd.ClearRenderTarget(false, true, Color.black);
                }
            }
        }

        context.ExecuteCommandBuffer(cmd);
        CommandBufferPool.Release(cmd);
    }

    private void DrawSkybox(Camera camera, ScriptableRenderContext context)
    {
        CommandBuffer cmd = new CommandBuffer() { name = "" };
        cmd.SetRenderTarget(m_ColorBuffer, m_DepthStencilBuffer);
        context.ExecuteCommandBuffer(cmd);
        cmd.Release();
            
        //Skybox
        if (camera.clearFlags == CameraClearFlags.Skybox)  {  context.DrawSkybox(camera);  }
    }

    private void PushShadingConstants(Camera camera, ScriptableRenderContext context, int iteration)
    {
        CommandBuffer cmd = CommandBufferPool.Get("Push");

        cmd.SetGlobalInt(ShaderIDs._MSAASampleCount, k_MSAASamples);
        camera.GetComponent<CamParams>()?.SetShaderParams(cmd, camera);

        cmd.SetGlobalMatrix("_ShutterOpenM", m_ObjectShutterOpenMatrix);
        bool isSceneCam = camera.name == "SceneCamera";
        if( isSceneCam )
        {
            cmd.SetGlobalMatrix(ShaderIDs._PreviousViewMatrix, m_ShutterOpenViewMatrix_scene != null ? m_ShutterOpenViewMatrix_scene : Matrix4x4.identity);
            cmd.SetGlobalMatrix(ShaderIDs._PreviousProjectionMatrix, m_ShutterOpenProjectionMatrix_scene != null ? m_ShutterOpenProjectionMatrix_scene : Matrix4x4.identity);
        }
        else
        {
            cmd.SetGlobalMatrix(ShaderIDs._PreviousViewMatrix, m_ShutterOpenViewMatrix_game != null ? m_ShutterOpenViewMatrix_game : Matrix4x4.identity);
            cmd.SetGlobalMatrix(ShaderIDs._PreviousProjectionMatrix, m_ShutterOpenProjectionMatrix_game != null ? m_ShutterOpenProjectionMatrix_game : Matrix4x4.identity);
        }

        if(m_AccumulationMode == StochasticRasterizer.AccumulationMode.Continuous)
        { 
            cmd.SetGlobalTexture(ShaderIDs._StochasticSampleTexture, m_StochasticSampleTexArray[(int)(m_Random.NextDouble() * (m_StochasticSampleTexArray.Length - 1))]);

            cmd.SetGlobalFloat("_Jitter", Time.time * 100);
        }
        else if (m_AccumulationMode == StochasticRasterizer.AccumulationMode.Finite)
        { 
            cmd.SetGlobalTexture(ShaderIDs._StochasticSampleTexture, m_StochasticSampleTexArray[iteration / k_PatternShift]);
            cmd.SetGlobalFloat("_SubframeIndex", iteration % k_PatternShift);

            cmd.SetGlobalFloat("_Jitter", iteration * 100);
        }

        context.ExecuteCommandBuffer(cmd);
        cmd.Clear();
    }

    private void RenderOpacity(SortingSettings sortingSettings, DrawingSettings drawSettings, FilteringSettings filterSettings,
                               CullingResults cull, ScriptableRenderContext context)
    {
        CommandBuffer cmd = CommandBufferPool.Get("Opacity");

        //Set Alpha RT
        cmd.SetRenderTarget(m_OpacityBuffer, m_DepthStencilBuffer);
        cmd.SetGlobalInt(ShaderIDs._MSAASampleCount, k_MSAASamples);

        context.ExecuteCommandBuffer(cmd);
        CommandBufferPool.Release(cmd);

        //Opaque objects
        sortingSettings.criteria = SortingCriteria.CommonOpaque;
        drawSettings.sortingSettings = sortingSettings;
        drawSettings.SetShaderPassName(0, m_AlphaPassName);
        filterSettings.renderQueueRange = RenderQueueRange.opaque;
        context.DrawRenderers(cull, ref drawSettings, ref filterSettings);
    }

    private void RenderStochasticForward(SortingSettings sortingSettings, DrawingSettings drawSettings, FilteringSettings filterSettings,
                                         CullingResults cull, ScriptableRenderContext context)
    {
        CommandBuffer cmd = CommandBufferPool.Get("Stochastic");

        //Set Color RT
        cmd.SetRenderTarget(m_ColorBuffer, m_DepthStencilBuffer);
        cmd.SetGlobalInt(ShaderIDs._MSAASampleCount, k_MSAASamples);

        context.ExecuteCommandBuffer(cmd);
        CommandBufferPool.Release(cmd);

        //Opaque objects
        sortingSettings.criteria = SortingCriteria.CommonOpaque;
        drawSettings.sortingSettings = sortingSettings;
        drawSettings.SetShaderPassName(0, m_PassName);
        filterSettings.renderQueueRange = RenderQueueRange.opaque;          
        context.DrawRenderers(cull, ref drawSettings, ref filterSettings);
    }

    private void RenderFinalPass(ScriptableRenderContext context, int iteration, Camera camera)
    {
        CommandBuffer cmd = CommandBufferPool.Get("FinalPass");

        if(Application.isPlaying)
        { 
            // Ping-pong indices
            if(m_AccumulationMode == StochasticRasterizer.AccumulationMode.Continuous)
            {
                m_HistorySourceIndex = (Time.frameCount + 0) % 2;
                m_HistoryDestIndex   = (Time.frameCount + 1) % 2;
                
                cmd.SetGlobalFloat("_AccumulationWeight", 0.99f);
            }
            else
            {
                m_HistorySourceIndex = (iteration + 0) % 2;
                m_HistoryDestIndex   = (iteration + 1) % 2;

                // TODO: Note
                //if(iteration == 0)
                //    cmd.SetGlobalFloat("_AccumulationWeight", 0f);
                //else
                    cmd.SetGlobalFloat("_AccumulationWeight", 1f - (1f / (float)m_AccumulationIterations));
            }

            // Post Process
            var postProcessLayer = camera.GetComponent<PostProcessLayer>();
            if(postProcessLayer != null)
            {
                RenderPostProcess(postProcessLayer, cmd, camera);
            }

            cmd.SetGlobalTexture("_ColorBuffer",   m_ColorBuffer); 
            cmd.SetGlobalTexture("_HistoryBuffer", m_HistoryBuffers[m_HistorySourceIndex]);
            CoreUtils.DrawFullScreen(cmd, m_FinalPass, m_HistoryBuffers[m_HistoryDestIndex], m_ColorBuffer);
        }
        else
        {
            //TODO: Final Pass here still needs opacity buffer for final resolve.
            // Post Process
            var postProcessLayer = camera.GetComponent<PostProcessLayer>();
            if(postProcessLayer != null)
            {
                RenderPostProcess(postProcessLayer, cmd, camera);
            }
            else
            {
                cmd.Blit(m_ColorBuffer, BuiltinRenderTextureType.CameraTarget);
            }
        }

        
        context.ExecuteCommandBuffer(cmd);
        CommandBufferPool.Release(cmd);
    }

    private void RenderPostProcess(PostProcessLayer layer, CommandBuffer cmd, Camera camera)
    {
        var context = m_PostProcessRenderContext;
        context.Reset();
        context.source = m_ColorBuffer;
        context.destination = m_ColorBuffer;
        context.command = cmd;
        context.camera = camera;
        context.sourceFormat = RenderTextureFormat.ARGBHalf;
        context.flip = false;
#if !UNITY_2019_1_OR_NEWER // Y-flip correction available in 2019.1
        context.flip = context.flip && (!hdcamera.camera.stereoEnabled);
#endif

        layer.Render(context);
    }

    protected override void Dispose(bool disposing)
    {
        RTHandles.Release(m_ColorBuffer);
        RTHandles.Release(m_DepthStencilBuffer);
        RTHandles.Release(m_OpacityBuffer);
        for (int i = 0; i < m_HistoryBuffers.Length; ++i)
            RTHandles.Release(m_HistoryBuffers[i]);

        RTHandles.Release(m_RecordingBuffer);

        CoreUtils.Destroy(m_StochasticSampleTexArray);
        m_StochasticSampleTexArray = null;

        CoreUtils.Destroy(m_FinalPass);
    }
}