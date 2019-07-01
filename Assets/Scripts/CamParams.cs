using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

[ExecuteInEditMode]
public class CamParams : MonoBehaviour
{
    public float lensRadius;
    public float focalDist;
    public float cocScale;
    public bool  primDebug;

    private static class ShaderIDs
    {
        public static readonly int _lensRadius = Shader.PropertyToID("cam_lensRadius");
        public static readonly int _focalDist = Shader.PropertyToID("cam_focalDist");
        public static readonly int _cocScale = Shader.PropertyToID("cam_cocScale");
        public static readonly int _pixelHeight = Shader.PropertyToID("cam_pixelHeight");
        public static readonly int _primDebug = Shader.PropertyToID("cam_primDebug");
    }

    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        lensRadius = Mathf.Max(0.0f, lensRadius);
        focalDist = Mathf.Max(0.1f, focalDist);
        cocScale = Mathf.Max(0.0f, cocScale);
    }

    public void SetShaderParams(CommandBuffer cmdbuf, Camera c)
    {
        float pixelHeight = 1.0f / (float)c.pixelHeight * 0.5f;// Mathf.Tan( c.fieldOfView * Mathf.Deg2Rad ) * 2.0f / (float) c.pixelHeight;
        cmdbuf.SetGlobalFloat(ShaderIDs._lensRadius, lensRadius * 0.1f);
        cmdbuf.SetGlobalFloat(ShaderIDs._focalDist , focalDist);
        cmdbuf.SetGlobalFloat(ShaderIDs._cocScale, cocScale);
        cmdbuf.SetGlobalFloat(ShaderIDs._pixelHeight, pixelHeight);
        cmdbuf.SetGlobalInt(ShaderIDs._primDebug, primDebug ? 1 : 0);
    }
}
