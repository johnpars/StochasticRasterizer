using UnityEngine;
using System.Collections.Generic;

[ExecuteInEditMode]
public sealed class StochasticMaterialDebugger : MonoBehaviour
{
    [SerializeField, Range(0, 1)] float _t = 0.5f;
    [SerializeField] bool _showHull = false;
    [SerializeField] int _showPrim = -1;

    [SerializeField] Renderer[] _renderers = null;

    public Matrix4x4 FirstObjectMatrix
    {
        get
        {
            if(_renderers != null && _renderers.Length > 0 && _renderers[0] != null)
            { 
                return _renderers[0].transform.localToWorldMatrix;
            }
            else
            {
                return Matrix4x4.identity;
            }
        }
    }

    static class ShaderIDs
    {
        public static readonly int _DebugT = Shader.PropertyToID("_DebugT");
        public static readonly int _DebugShowHull = Shader.PropertyToID("_DebugShowHull");
        public static readonly int _DebugShowPrim = Shader.PropertyToID("_DebugShowPrim");
    }

    MaterialPropertyBlock _sheet;

    void LateUpdate()
    {
        if (_renderers == null || _renderers.Length == 0) return;
        if (_sheet == null) _sheet = new MaterialPropertyBlock();
        
        foreach (var renderer in _renderers)
        {
            if (renderer == null) continue;
            renderer.GetPropertyBlock(_sheet);
            _sheet.SetFloat(ShaderIDs._DebugT, _t);
            _sheet.SetInt(ShaderIDs._DebugShowHull, _showHull ? 1 : 0);
            _sheet.SetInt(ShaderIDs._DebugShowPrim, _showPrim < -1 ? -1 : _showPrim);
            renderer.SetPropertyBlock(_sheet);
        }
    }
    
}