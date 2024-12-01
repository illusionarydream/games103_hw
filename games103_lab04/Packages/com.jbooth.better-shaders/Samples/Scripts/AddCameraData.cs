using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class AddCameraData : MonoBehaviour
{
   private void OnEnable()
   {
      Camera c = GetComponent<Camera>();
      if (c != null)
      {
         c.depthTextureMode = DepthTextureMode.DepthNormals;
         //Shader.SetGlobalMatrix("_CamToWorld", c.cameraToWorldMatrix);
      }

   }
}
