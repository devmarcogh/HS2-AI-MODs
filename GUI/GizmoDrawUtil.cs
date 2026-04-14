using System.Collections.Generic;
using UnityEngine;

namespace StudioModsMSG
{
    /// <summary>
    /// Runtime gizmo drawing utility — works at play-time (not just in the editor).
    /// Uses LineRenderers and primitive sphere objects pooled under a single root.
    ///
    /// Usage per-frame pattern:
    ///   util.Begin();
    ///   util.DrawLine(a, b, Color.yellow);
    ///   util.DrawSphere(pos, 0.03f, Color.cyan);
    ///   util.DrawWireCircle(center, 0.12f, Vector3.up, Color.green);
    ///   util.DrawAxes(origin, rotation);
    ///   util.End();  // hides pooled objects not refreshed this frame
    ///
    /// Or fire-and-forget (objects stay until Clear() or the next Begin/End cycle):
    ///   util.DrawLine(...);  util.DrawSphere(...);  util.End();
    ///
    /// Call Clear() when the owning MonoBehaviour is destroyed.
    /// </summary>
    class GizmoDrawUtil
    {
        private Transform                gizmoRoot;
        private readonly List<LineRenderer> linePool   = new List<LineRenderer>();
        private readonly List<Transform>    spherePool = new List<Transform>();
        private int  lineIndex;
        private int  sphereIndex;
        private bool insideFrame;

        private static Shader cachedShader;

        // ------------------------------------------------------------------
        // Lifecycle
        // ------------------------------------------------------------------

        /// <summary>
        /// Optional — attaches the gizmo root under the given parent transform.
        /// If not called the root lives at world origin.
        /// </summary>
        public void SetParent(Transform parent)
        {
            EnsureRoot();
            if (parent != null)
                gizmoRoot.SetParent(parent, false);
        }

        /// <summary>Begin a new draw frame. Resets internal counters.</summary>
        public void Begin()
        {
            EnsureRoot();
            lineIndex   = 0;
            sphereIndex = 0;
            insideFrame = true;
        }

        /// <summary>End the current draw frame. Hides pooled objects not written this frame.</summary>
        public void End()
        {
            insideFrame = false;
            for (int i = lineIndex;   i < linePool.Count;   i++) if (linePool[i]   != null) linePool[i].gameObject.SetActive(false);
            for (int i = sphereIndex; i < spherePool.Count; i++) if (spherePool[i] != null) spherePool[i].gameObject.SetActive(false);
        }

        /// <summary>Destroys all created objects and releases the root.</summary>
        public void Clear()
        {
            for (int i = 0; i < linePool.Count; i++)
                if (linePool[i] != null) Object.Destroy(linePool[i].gameObject);
            linePool.Clear();

            for (int i = 0; i < spherePool.Count; i++)
                if (spherePool[i] != null) Object.Destroy(spherePool[i].gameObject);
            spherePool.Clear();

            if (gizmoRoot != null) { Object.Destroy(gizmoRoot.gameObject); gizmoRoot = null; }
        }

        // ------------------------------------------------------------------
        // Draw primitives
        // ------------------------------------------------------------------

        /// <summary>Draw a world-space line segment.</summary>
        public void DrawLine(Vector3 start, Vector3 end, Color color, float width = 0.008f)
        {
            if (!insideFrame) EnsureRoot();
            LineRenderer lr = GetOrCreateLine(lineIndex++);
            lr.gameObject.SetActive(true);
            lr.startColor      = color;
            lr.endColor        = color;
            lr.widthMultiplier = width;
            lr.SetPosition(0, start);
            lr.SetPosition(1, end);
        }

        /// <summary>Draw a world-space sphere marker.</summary>
        public void DrawSphere(Vector3 center, float radius, Color color)
        {
            if (!insideFrame) EnsureRoot();
            Transform sph = GetOrCreateSphere(sphereIndex++);
            sph.gameObject.SetActive(true);
            sph.position   = center;
            sph.localScale = Vector3.one * (radius * 2f);
            SetSphereColor(sph, color);
        }

        /// <summary>Draw an N-segment wire circle in world space.</summary>
        public void DrawWireCircle(Vector3 center, float radius, Vector3 normal, Color color, int segments = 32, float lineWidth = 0.008f)
        {
            if (segments < 3) segments = 3;
            Quaternion rot  = Quaternion.FromToRotation(Vector3.up, normal.sqrMagnitude > 0.001f ? normal.normalized : Vector3.up);
            float      step = 2f * Mathf.PI / segments;
            Vector3    prev = center + rot * new Vector3(radius, 0f, 0f);

            for (int i = 1; i <= segments; i++)
            {
                float   a    = i * step;
                Vector3 curr = center + rot * new Vector3(Mathf.Cos(a) * radius, 0f, Mathf.Sin(a) * radius);
                DrawLine(prev, curr, color, lineWidth);
                prev = curr;
            }
        }

        /// <summary>Draw RGB axis lines (red=right, green=up, blue=forward).</summary>
        public void DrawAxes(Vector3 origin, Quaternion rotation, float length = 0.05f, float width = 0.006f)
        {
            DrawLine(origin, origin + rotation * Vector3.right   * length, new Color(1f, 0.10f, 0.10f), width);
            DrawLine(origin, origin + rotation * Vector3.up      * length, new Color(0.10f, 1f, 0.10f), width);
            DrawLine(origin, origin + rotation * Vector3.forward * length, new Color(0.10f, 0.40f, 1f),  width);
        }

        /// <summary>Draw a bone: sphere at tip + line to parent position.</summary>
        public void DrawBone(Transform bone, float sphereRadius, Color color, bool drawAxes = false, float axisLength = 0.05f)
        {
            if (bone == null) return;
            DrawSphere(bone.position, sphereRadius, color);
            if (bone.parent != null)
                DrawLine(bone.parent.position, bone.position, color);
            if (drawAxes)
                DrawAxes(bone.position, bone.rotation, axisLength);
        }

        // ------------------------------------------------------------------
        // Private helpers
        // ------------------------------------------------------------------

        private void EnsureRoot()
        {
            if (gizmoRoot != null) return;
            gizmoRoot = new GameObject("GizmoDrawUtil_Root").transform;
        }

        private LineRenderer GetOrCreateLine(int idx)
        {
            while (linePool.Count <= idx)
            {
                int    n  = linePool.Count;
                var    go = new GameObject("GizmoLine_" + n);
                go.transform.SetParent(gizmoRoot, false);
                var lr = go.AddComponent<LineRenderer>();
                lr.useWorldSpace     = true;
                lr.positionCount     = 2;
                lr.loop              = false;
                lr.numCapVertices    = 2;
                lr.numCornerVertices = 2;
                lr.sortingOrder      = 32767;
                lr.material          = new Material(GetShader());
                linePool.Add(lr);
            }
            return linePool[idx];
        }

        private Transform GetOrCreateSphere(int idx)
        {
            while (spherePool.Count <= idx)
            {
                int n  = spherePool.Count;
                var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                go.name = "GizmoSphere_" + n;
                go.transform.SetParent(gizmoRoot, false);
                go.hideFlags = HideFlags.HideAndDontSave;

                // Remove collider — we don't want physics interference
                var col = go.GetComponent<SphereCollider>();
                if (col != null) Object.Destroy(col);

                var rend = go.GetComponent<Renderer>();
                if (rend != null) rend.sharedMaterial = new Material(GetShader());

                spherePool.Add(go.transform);
            }
            return spherePool[idx];
        }

        private static void SetSphereColor(Transform sph, Color color)
        {
            var rend = sph.GetComponent<Renderer>();
            if (rend != null && rend.sharedMaterial != null)
                rend.sharedMaterial.color = color;
        }

        private static Shader GetShader()
        {
            if (cachedShader == null)
                cachedShader = Shader.Find("Sprites/Default") ?? Shader.Find("Unlit/Color");
            return cachedShader;
        }
    }
}
