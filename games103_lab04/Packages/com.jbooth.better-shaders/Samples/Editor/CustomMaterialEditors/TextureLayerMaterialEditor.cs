using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

// This is a editor for what could be an entire material, or just part of one.
// It inherits from SubShaderMaterialEditor, and must be set as the "SubEditor" in
// the shaders options. The resulting shader, either this one or one which subshaders
// it in, must set the CustomEditor to JBooth.BetterShaders.BetterShaderMaterialGUI".
//
// Note that since this could be part of a stack of shaders, the names might need to be mutated
// when finding properties or setting keywords. To make this trivial, the base class has a
// FindProp function which wll do this automatically, and a GetKeywordName function for any
// keywords. Using these will make sure your shader works well with multi-stack.
//
// Finally, the base class also contains useful functions for making nice editors without
// having to write a lot of boilerplate..

namespace JBooth.BetterShaders
{
   public class TextureLayerMaterialEditor : SubShaderMaterialEditor
   {
      enum UVSpace
      {
         UV = 0,
         World = 1
      }

      enum BlendMode
      {
         Alpha = 1,
         Mult2X = 2,
         HeightBlend = 3
      }

      enum InvertHeight
      {
         Top = 0,
         Bottom = 1
      }

      enum Source
      {
         Constant = 0,
         Texture = 1,
         Vertex = 2
      }

      enum Channel
      {
         R = 0, G, B, A
      }

      public override void OnGUI(MaterialEditor materialEditor,
         ShaderGUI shaderGUI,
         MaterialProperty[] props,
         Material mat)
      {
         if (stackIndex >= 0)
         {
            DrawHeader("Texture Layer (" + stackIndex + ")");
         }
         else
         {
            DrawHeader("Texture Layer");
         }
         
         
         var spaceProp = FindProp("_LayerTextureSpace", props);
         EnumPopup<UVSpace>(new GUIContent("UV Space"), spaceProp);

         var albedoProp = FindProp("_LayerAlbedoMap", props);
         var tintProp = FindProp("_LayerAlbedoTint", props);
         materialEditor.TexturePropertySingleLine(new GUIContent("Albedo/Height"), albedoProp, tintProp);
         materialEditor.TextureScaleOffsetProperty(albedoProp);

         var normalProp = FindProp("_LayerNormalMap", props);
         var normalStrProp = FindProp("_LayerNormalStrength", props);
         materialEditor.TexturePropertySingleLine(new GUIContent("Normal"), normalProp, normalStrProp);

         var maskMap = FindProp("_LayerMaskMap", props);
         EditorGUI.BeginChangeCheck();
         materialEditor.TexturePropertySingleLine(new GUIContent("Mask Map"), maskMap);
         if (EditorGUI.EndChangeCheck())
         {
            if (maskMap.textureValue != null)
            {
               mat.EnableKeyword(GetKeywordName("_LAYERMASKMAP"));
            }
            else
            {
               mat.DisableKeyword(GetKeywordName("_LAYERMASKMAP"));
            }
            EditorUtility.SetDirty(mat);
         }


         var blendProp = FindProp("_LayerBlendMode", props);
         EnumPopup<BlendMode>(new GUIContent("Blend Mode"), blendProp);

         materialEditor.DefaultShaderProperty(FindProp("_LayerStrength", props), "Strength");
         materialEditor.DefaultShaderProperty(FindProp("_LayerBlendContrast", props), "Blend Contrast");
         materialEditor.DefaultShaderProperty(FindProp("_LayerAngleMin", props), "Angle Minimum");
         materialEditor.DefaultShaderProperty(FindProp("_LayerHeight", props), "Height Filter");

         var invertProp = FindProp("_LayerInvertHeight", props);
         EnumPopup<InvertHeight>(new GUIContent("Texture on"), invertProp);


         var source = Source.Constant;
         if (mat.IsKeywordEnabled(GetKeywordName("_LAYERSOURCE_TEXTURE")))
         {
            source = Source.Texture;
         }
         else if (mat.IsKeywordEnabled(GetKeywordName("_LAYERSOURCE_VERTEX")))
         {
            source = Source.Vertex;
         }
         var newSource = (Source)EditorGUILayout.EnumPopup("Weight Source", source);
         if (newSource != source)
         {
            source = newSource;
            mat.DisableKeyword(GetKeywordName("_LAYERSOURCE_VERTEX"));
            mat.DisableKeyword(GetKeywordName("_LAYERSOURCE_TEXTURE"));
            if (source == Source.Texture)
            {
               mat.EnableKeyword("_LAYERSOURCE_TEXTURE");
            }
            else if (source == Source.Vertex)
            {
               mat.EnableKeyword("_LAYERSOURCE_VERTEX");
            }
         }

         if (source == Source.Texture)
         {
            EditorGUI.indentLevel++;
            materialEditor.TexturePropertySingleLine(new GUIContent("Weight Mask"), FindProp("_LayerWeightMap", props));
            EditorGUI.indentLevel--;
         }
         if (source != Source.Constant)
         {
            EditorGUI.indentLevel++;
            var channelProp = FindProp("_LayerChannel", props);
            EnumPopup<Channel>(new GUIContent("Weight Channel"), channelProp);

            EditorGUI.indentLevel--;
         }
         
      }
   }
}
