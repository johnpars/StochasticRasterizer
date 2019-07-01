using UnityEngine;
using UnityEditor;
using UnityEditorInternal;

[CustomEditor(typeof(StochasticMaterialDebugger))]
sealed class StochasticMaterialDebuggerEditor : Editor
{
    SerializedProperty _t;
    SerializedProperty _showHull;
    SerializedProperty _showPrim;
    ReorderableList _renderers;

    static class Styles
    {
    }

    void OnEnable()
    {
        _t = serializedObject.FindProperty("_t");
        _showHull = serializedObject.FindProperty("_showHull");
        _showPrim = serializedObject.FindProperty("_showPrim");

        _renderers = new ReorderableList(
            serializedObject,
            serializedObject.FindProperty("_renderers"),
            true, // draggable
            true, // displayHeader
            true, // displayAddButton
            true  // displayRemoveButton
        );

        _renderers.drawHeaderCallback = (Rect rect) => {
            EditorGUI.LabelField(rect, "Target Renderers");
        };

        _renderers.drawElementCallback = (Rect frame, int index, bool isActive, bool isFocused) => {
            var rect = frame;
            rect.y += 2;
            rect.height = EditorGUIUtility.singleLineHeight;
            var element = _renderers.serializedProperty.GetArrayElementAtIndex(index);
            EditorGUI.PropertyField(rect, element, GUIContent.none);
        };
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        EditorGUILayout.PropertyField(_t);
        EditorGUILayout.PropertyField(_showHull);
        EditorGUILayout.PropertyField(_showPrim);

        _renderers.DoLayoutList();

        EditorGUILayout.Space();

        serializedObject.ApplyModifiedProperties();
    }
}