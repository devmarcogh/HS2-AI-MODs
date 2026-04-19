using Studio;
using System.Collections.Generic;
using UnityEngine;

namespace StudioModsMSG
{
    /// <summary>
    /// How this collider interacts with cloth.
    /// </summary>
    enum ColliderMagneticMode
    {
        /// <summary>Normal repulsion: cloth is pushed away when it penetrates the volume.</summary>
        Repel   = 0,
        /// <summary>Positive magnet: cloth is attracted toward the surface within MagneticRange.
        /// Once grabbed, it stays near the contact point until it moves beyond the range.</summary>
        Attract = 1,
        /// <summary>Disabled: collider has no effect on cloth whatsoever.</summary>
        Off     = 2,
    }

    /// <summary>
    /// Capsule/sphere collider proxy that lives on a Studio workspace folder object.
    /// Studio's gizmos handle positioning — the cloth runtime reads the transform each frame.
    /// </summary>
    class ClothColliderProxy : MonoBehaviour
    {
        // ── Collider shape ────────────────────────────────────────────────
        public float  Radius    = 0.08f;
        public float  Height    = 0.30f;   // 0 = sphere
        public int    Direction = 1;       // 0=X, 1=Y, 2=Z

        // ── Magnetic interaction ──────────────────────────────────────────
        /// <summary>Collision/magnetic mode for this collider.</summary>
        public ColliderMagneticMode MagneticMode     = ColliderMagneticMode.Repel;
        /// <summary>For Attract: 0-1 snap fraction per substep toward the surface (1=instant, 0=none).</summary>
        public float                MagneticStrength = 0.85f;
        /// <summary>For Attract: world-space radius beyond which attraction has no effect.</summary>
        public float                MagneticRange    = 0.12f;

        // ── Collider name (shown in workspace tree) ───────────────────────
        public string ColliderName = "Cloth Collider";

        // ── Studio references ─────────────────────────────────────────────
        public OCIFolder Folder  { get; set; }

        // ── Wireframe rendering ───────────────────────────────────────────
        private LineRenderer[] _lines;
        private const int CircleSegments   = 32;   // smoother circles
        private const int LongitudeLines   = 8;    // longitudinal ribs
        private const int TotalLines       = CircleSegments * 3 + LongitudeLines; // 3 rings + ribs
        private static Shader _shader;

        // ── Global registry ───────────────────────────────────────────────
        private static readonly List<ClothColliderProxy> _all = new List<ClothColliderProxy>();
        public static IReadOnlyList<ClothColliderProxy> All => _all;

        private void OnEnable()
        {
            if (!_all.Contains(this)) _all.Add(this);
            BuildWireframe();
        }

        private void OnDisable()
        {
            _all.Remove(this);
        }

        private void OnDestroy()
        {
            _all.Remove(this);
        }

        private void LateUpdate()
        {
            UpdateWireframe();
        }

        // ── World-space capsule query (used by cloth runtime) ─────────────

        /// <summary>
        /// Returns the world-space center, direction axis, scaled radius, and half-extent.
        /// </summary>
        public void GetWorldCapsule(out Vector3 center, out Vector3 worldDir,
                                    out float worldRadius, out float halfExtent)
        {
            Transform t = transform;
            center = t.position;

            Vector3 localDir;
            float   scaleAxis;
            float   scaleRadial;
            switch (Direction)
            {
                case 0: // X
                    localDir    = Vector3.right;
                    scaleAxis   = Mathf.Abs(t.lossyScale.x);
                    scaleRadial = Mathf.Max(Mathf.Abs(t.lossyScale.y), Mathf.Abs(t.lossyScale.z));
                    break;
                case 2: // Z
                    localDir    = Vector3.forward;
                    scaleAxis   = Mathf.Abs(t.lossyScale.z);
                    scaleRadial = Mathf.Max(Mathf.Abs(t.lossyScale.x), Mathf.Abs(t.lossyScale.y));
                    break;
                default: // Y
                    localDir    = Vector3.up;
                    scaleAxis   = Mathf.Abs(t.lossyScale.y);
                    scaleRadial = Mathf.Max(Mathf.Abs(t.lossyScale.x), Mathf.Abs(t.lossyScale.z));
                    break;
            }

            worldDir    = t.TransformDirection(localDir);
            worldRadius = Radius * scaleRadial;
            halfExtent  = Mathf.Max(0f, Height * 0.5f * scaleAxis - worldRadius);
        }

        // ── Wireframe rendering ───────────────────────────────────────────

        private static Shader GetShader()
        {
            if (_shader == null)
                _shader = Shader.Find("Hidden/Internal-Colored")
                       ?? Shader.Find("Sprites/Default")
                       ?? Shader.Find("UI/Default");
            return _shader;
        }

        // Returns the wireframe color for the current magnetic mode.
        private Color WireColor()
        {
            switch (MagneticMode)
            {
                case ColliderMagneticMode.Attract: return new Color(0.20f, 1.00f, 0.30f, 0.90f); // green
                case ColliderMagneticMode.Off:     return new Color(0.55f, 0.55f, 0.55f, 0.55f); // grey
                default:                           return new Color(0.20f, 0.85f, 1.00f, 0.85f); // cyan (repel)
            }
        }

        private void BuildWireframe()
        {
            if (_lines != null) return;
            _lines = new LineRenderer[TotalLines];
            Color c = WireColor();
            for (int i = 0; i < TotalLines; i++)
            {
                var go = new GameObject("ColliderWire_" + i);
                go.transform.SetParent(transform, false);
                var lr = go.AddComponent<LineRenderer>();
                lr.useWorldSpace     = true;
                lr.loop              = false;
                lr.numCapVertices    = 0;
                lr.numCornerVertices = 0;
                lr.sortingOrder      = 32767;
                lr.material          = new Material(GetShader());
                lr.widthMultiplier   = 0.0035f;
                lr.startColor        = c;
                lr.endColor          = c;
                _lines[i] = lr;
            }
        }

        private void UpdateWireframe()
        {
            if (_lines == null) return;

            GetWorldCapsule(out Vector3 center, out Vector3 dir,
                            out float r, out float halfH);

            Vector3 top = center + dir * halfH;
            Vector3 bot = center - dir * halfH;

            Quaternion rot = Quaternion.FromToRotation(Vector3.up,
                dir.sqrMagnitude > 0.001f ? dir.normalized : Vector3.up);

            Color c = WireColor();
            int idx = 0;

            // 3 rings: top cap, equator, bottom cap
            DrawCircle(top, rot, r, c, ref idx);
            DrawCircle(center, rot, r, c, ref idx);
            DrawCircle(bot, rot, r, c, ref idx);

            // Longitudinal ribs evenly spaced around the circumference
            for (int ri = 0; ri < LongitudeLines; ri++)
            {
                float angle = ri * (2f * Mathf.PI / LongitudeLines);
                Vector3 offset = rot * new Vector3(Mathf.Cos(angle) * r, 0f, Mathf.Sin(angle) * r);
                SetLine(idx++, top + offset, bot + offset, c);
            }

            // Hide unused slots
            for (; idx < _lines.Length; idx++)
                _lines[idx].gameObject.SetActive(false);
        }

        private void DrawCircle(Vector3 center, Quaternion rot, float radius, Color c, ref int idx)
        {
            float step = 2f * Mathf.PI / CircleSegments;
            Vector3 prev = center + rot * new Vector3(radius, 0f, 0f);

            for (int i = 1; i <= CircleSegments; i++)
            {
                float a = i * step;
                Vector3 curr = center + rot * new Vector3(Mathf.Cos(a) * radius, 0f, Mathf.Sin(a) * radius);
                SetLine(idx++, prev, curr, c);
                prev = curr;
            }
        }

        private void SetLine(int idx, Vector3 a, Vector3 b, Color c)
        {
            if (idx >= _lines.Length) return;
            var lr = _lines[idx];
            lr.gameObject.SetActive(true);
            lr.positionCount = 2;
            lr.SetPosition(0, a);
            lr.SetPosition(1, b);
            lr.startColor = c;
            lr.endColor   = c;
        }

        // ── Factory ───────────────────────────────────────────────────────

        /// <summary>
        /// Creates a new cloth collider as a Studio workspace folder item.
        /// Returns the proxy component.
        /// </summary>
        public static ClothColliderProxy CreateInWorkspace(Vector3 worldPosition,
            float radius = 0.08f, float height = 0.30f, int direction = 1)
        {
            // Create a Studio folder — appears in workspace tree with gizmo support
            OCIFolder folder = AddObjectFolder.Add();
            folder.name = "Cloth Collider";
            folder.folderInfo.name = "Cloth Collider";

            // Position the folder
            folder.guideObject.transformTarget.position = worldPosition;

            // Attach our component
            ClothColliderProxy proxy = folder.guideObject.transformTarget.gameObject
                .AddComponent<ClothColliderProxy>();
            proxy.Folder    = folder;
            proxy.Radius    = radius;
            proxy.Height    = height;
            proxy.Direction = direction;

            return proxy;
        }

        /// <summary>
        /// Removes this collider and its Studio workspace folder.
        /// </summary>
        public void RemoveFromWorkspace()
        {
            if (Folder != null)
            {
                Studio.Studio.DeleteNode(Folder.treeNodeObject);
                Folder = null;
            }
            else
            {
                Destroy(gameObject);
            }
        }
    }
}
