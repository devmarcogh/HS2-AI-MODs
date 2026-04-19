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
        private const float DEFAULT_LINE_WIDTH = 0.008f;
        private const float MIN_LINE_WIDTH = 0.0045f;
        private const float MAX_LINE_WIDTH = 0.018f;

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
        public void DrawLine(Vector3 start, Vector3 end, Color color, float width = DEFAULT_LINE_WIDTH)
        {
            if (!insideFrame) EnsureRoot();
            LineRenderer lr = GetOrCreateLine(lineIndex++);
            lr.gameObject.SetActive(true);
            lr.startColor      = color;
            lr.endColor        = color;
            lr.widthMultiplier = ClampLineWidth(width);
            lr.SetPosition(0, start);
            lr.SetPosition(1, end);
        }

        /// <summary>
        /// Draws a wire sphere marker using 3 rings (X/Y/Z axes) with axis colors.
        /// </summary>
        public void DrawSphere(Vector3 center, float radius, Color color)
        {
            DrawWireSphere(center, radius, color, DEFAULT_LINE_WIDTH);
        }

        /// <summary>
        /// Draws a 3-ring wire sphere (XY, XZ, YZ) to improve 3D readability.
        /// </summary>
        public void DrawWireSphere(Vector3 center, float radius, Color color, float lineWidth = DEFAULT_LINE_WIDTH, int segments = 36)
        {
            if (radius <= 0f) return;
            float w = ClampLineWidth(lineWidth);

            float alpha = color.a;
            Color cx = new Color(Mathf.Max(0.2f, color.r), color.g * 0.35f, color.b * 0.35f, alpha);
            Color cy = new Color(color.r * 0.35f, Mathf.Max(0.2f, color.g), color.b * 0.35f, alpha);
            Color cz = new Color(color.r * 0.35f, color.g * 0.35f, Mathf.Max(0.2f, color.b), alpha);

            DrawWireCircle(center, radius, Vector3.right,   cx, segments, w); // YZ plane
            DrawWireCircle(center, radius, Vector3.up,      cy, segments, w); // XZ plane
            DrawWireCircle(center, radius, Vector3.forward, cz, segments, w); // XY plane
        }

        /// <summary>Draw a world-space solid sphere marker.</summary>
        public void DrawSolidSphere(Vector3 center, float radius, Color color)
        {
            if (!insideFrame) EnsureRoot();
            Transform sph = GetOrCreateSphere(sphereIndex++);
            sph.gameObject.SetActive(true);
            sph.position   = center;
            sph.localScale = Vector3.one * (radius * 2f);
            SetSphereColor(sph, color);
        }

        /// <summary>Draw an N-segment wire circle in world space.</summary>
        public void DrawWireCircle(Vector3 center, float radius, Vector3 normal, Color color, int segments = 32, float lineWidth = DEFAULT_LINE_WIDTH)
        {
            if (segments < 3) segments = 3;
            if (radius <= 0f) return;
            lineWidth = ClampLineWidth(lineWidth);
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

        /// <summary>
        /// Draw a wire capsule from start to end with adaptive ring density.
        /// Larger radius automatically adds more cap rings to keep a rounded look.
        /// </summary>
        public void DrawWireCapsule(Vector3 start, Vector3 end, float radius, Color color, float lineWidth = DEFAULT_LINE_WIDTH)
        {
            if (radius <= 0f) return;

            float w = ClampLineWidth(lineWidth);
            Vector3 axis = end - start;
            float axisLen = axis.magnitude;
            if (axisLen < 0.0001f)
            {
                DrawWireSphere(start, radius, color, w);
                return;
            }

            axis /= axisLen;
            GetPerpendicularBasis(axis, out Vector3 right, out Vector3 forward);

            int segments = Mathf.Clamp(24 + Mathf.RoundToInt(radius * 80f), 24, 64);
            int capRings = Mathf.Clamp(3 + Mathf.RoundToInt(radius * 40f), 3, 12);

            // Cylinder side guide lines
            DrawLine(start + right * radius,   end + right * radius,   color, w);
            DrawLine(start - right * radius,   end - right * radius,   color, w);
            DrawLine(start + forward * radius, end + forward * radius, color, w);
            DrawLine(start - forward * radius, end - forward * radius, color, w);

            // End rings
            DrawWireCircle(start, radius, axis, color, segments, w);
            DrawWireCircle(end,   radius, axis, color, segments, w);

            // Hemispheres with adaptive extra rings
            for (int i = 1; i <= capRings; i++)
            {
                float t = i / (float)(capRings + 1);
                float ang = t * (Mathf.PI * 0.5f);

                float ringR = Mathf.Cos(ang) * radius;
                float offset = Mathf.Sin(ang) * radius;

                // Start hemisphere extends opposite to axis
                DrawWireCircle(start - axis * offset, ringR, axis, color, segments, w);

                // End hemisphere extends with axis
                DrawWireCircle(end + axis * offset, ringR, axis, color, segments, w);
            }
        }

        /// <summary>Draw a wire cube in world space.</summary>
        public void DrawWireCube(Vector3 center, Vector3 size, Quaternion rotation, Color color, float lineWidth = DEFAULT_LINE_WIDTH)
        {
            float w = ClampLineWidth(lineWidth);
            Vector3 h = size * 0.5f;

            Vector3[] local =
            {
                new Vector3(-h.x, -h.y, -h.z),
                new Vector3( h.x, -h.y, -h.z),
                new Vector3( h.x, -h.y,  h.z),
                new Vector3(-h.x, -h.y,  h.z),
                new Vector3(-h.x,  h.y, -h.z),
                new Vector3( h.x,  h.y, -h.z),
                new Vector3( h.x,  h.y,  h.z),
                new Vector3(-h.x,  h.y,  h.z),
            };

            Vector3[] p = new Vector3[8];
            for (int i = 0; i < 8; i++) p[i] = center + rotation * local[i];

            // Bottom
            DrawLine(p[0], p[1], color, w);
            DrawLine(p[1], p[2], color, w);
            DrawLine(p[2], p[3], color, w);
            DrawLine(p[3], p[0], color, w);
            // Top
            DrawLine(p[4], p[5], color, w);
            DrawLine(p[5], p[6], color, w);
            DrawLine(p[6], p[7], color, w);
            DrawLine(p[7], p[4], color, w);
            // Verticals
            DrawLine(p[0], p[4], color, w);
            DrawLine(p[1], p[5], color, w);
            DrawLine(p[2], p[6], color, w);
            DrawLine(p[3], p[7], color, w);
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

        private static float ClampLineWidth(float width)
        {
            return Mathf.Clamp(width, MIN_LINE_WIDTH, MAX_LINE_WIDTH);
        }

        private static void GetPerpendicularBasis(Vector3 axis, out Vector3 right, out Vector3 forward)
        {
            Vector3 refAxis = Mathf.Abs(Vector3.Dot(axis, Vector3.up)) > 0.95f ? Vector3.right : Vector3.up;
            right = Vector3.Cross(axis, refAxis).normalized;
            forward = Vector3.Cross(right, axis).normalized;
        }

        private static Shader GetShader()
        {
            if (cachedShader == null)
                cachedShader = Shader.Find("Sprites/Default") ?? Shader.Find("Unlit/Color");
            return cachedShader;
        }
    }
}
