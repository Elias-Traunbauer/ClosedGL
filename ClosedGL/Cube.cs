using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ClosedGL
{
    public class Cube : GameObject
    {
        public override Mesh? Mesh { get; set; } = new Mesh()
        {
            Vertices = new VRageMath.Vector3[] { 
                new VRageMath.Vector3(-0.5f, -0.5f, -0.5f),
                new VRageMath.Vector3(-0.5f, 0.5f, -0.5f),
                new VRageMath.Vector3(0.5f, 0.5f, -0.5f),
                new VRageMath.Vector3(0.5f, -0.5f, -0.5f),
                new VRageMath.Vector3(-0.5f, -0.5f, 0.5f),
                new VRageMath.Vector3(-0.5f, 0.5f, 0.5f),
                new VRageMath.Vector3(0.5f, 0.5f, 0.5f),
                new VRageMath.Vector3(0.5f, -0.5f, 0.5f),
            },
            Triangles = new int[] {
                0, 1, 2,
                0, 2, 3,
                4, 6, 5,
                4, 7, 6,
                4, 5, 1,
                4, 1, 0,
                3, 2, 6,
                3, 6, 7,
                1, 5, 6,
                1, 6, 2,
                4, 0, 3,
                4, 3, 7
            },
            UVs = new VRageMath.Vector2[]
            {
                new VRageMath.Vector2(0, 0),
                new VRageMath.Vector2(0, 1),
                new VRageMath.Vector2(1, 1),
                new VRageMath.Vector2(1, 0),
                new VRageMath.Vector2(0, 0),
                new VRageMath.Vector2(0, 1),
                new VRageMath.Vector2(1, 1),
                new VRageMath.Vector2(1, 0)
            }
        };

        public Cube()
        {
            Texture = new Texture("Textures\\triangle.png");
        }
    }
}
