using VRageMath;

namespace ClosedGL
{
    public class Mesh
    {
        public Vector3[] Vertices { get; set; }
        public int[] Triangles { get; set; }
        public Vector2[] UVs { get; set; }
        public Mesh()
        {
            Vertices = Array.Empty<Vector3>();
            Triangles = Array.Empty<int>();
            UVs = Array.Empty<Vector2>();
        }
    }
}