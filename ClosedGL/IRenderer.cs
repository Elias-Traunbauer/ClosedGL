using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRageMath;

namespace ClosedGL
{
    public interface IRenderer
    {
        Vector2I RenderResolution { get; set; }
        float FarClipPlane { get; set; }
        float NearClipPlane { get; set; }
        float AspectRatio { get; }
        float FieldOfView { get; set; }
        Vector3 Position { get; set; }
        Quaternion Rotation { get; set; }

        int GetFrame(out Bitmap? bitmap);
        void Initialize(Texture[] textures);
        public bool Render(List<GameObject> gameObjects);
    }
}
