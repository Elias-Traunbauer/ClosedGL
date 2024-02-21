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
            Vertices = [
                new VRageMath.Vector3(-0.5f, -0.5f, -0.5f),
                new VRageMath.Vector3( 0.5f, -0.5f, -0.5f),
                new VRageMath.Vector3(-0.5f, -0.5f,  0.5f),
                new VRageMath.Vector3( 0.5f ,-0.5f,  0.5f),

                new VRageMath.Vector3(-0.5f, 0.5f, -0.5f),
                new VRageMath.Vector3( 0.5f, 0.5f, -0.5f),
                new VRageMath.Vector3(-0.5f, 0.5f,  0.5f),
                new VRageMath.Vector3( 0.5f, 0.5f,  0.5f),
            ],
            Triangles = [
                0, 1, 2,
                2, 1, 3,

                4, 6, 5,
                6, 7, 5,

                0, 4, 1,
                1, 4, 5,

                2, 3, 6,
                3, 7, 6,

                0, 2, 4,
                2, 6, 4,

                1, 5, 3,
                3, 5, 7,
            ],
            UVs =
            [
                new VRageMath.Vector2(0.25f, 0.33f),
                new VRageMath.Vector2(0.5f, 0.33f),
                new VRageMath.Vector2(0.5f, 0.66f),
                new VRageMath.Vector2(0.25f, 0.66f),

                new VRageMath.Vector2(0.25f, 0f),
                new VRageMath.Vector2(0.5f, 0f),
                new VRageMath.Vector2(0.5f, 1f),
                new VRageMath.Vector2(0.25f, 1f),
            ]

        };

        public Cube()
        {
            Texture = new Texture("Textures\\StandardCubeMap-2.jpg");
        }
    }
}
