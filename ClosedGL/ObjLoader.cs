
using System.Globalization;
using VRageMath;

namespace ClosedGL
{
    internal class ObjLoader
    {
        public ObjLoader()
        {
        }

        internal GameObject Load(string path)
        {
            NumberFormatInfo formatProvider = new NumberFormatInfo
            {
                NumberDecimalSeparator = "."
            };

            float floatParse(string s) => float.Parse(s, formatProvider);

            string[] lines = System.IO.File.ReadAllLines(path);

            var vertices = new List<Vector3>();
            var triangles = new List<int>();
            var uvs = new List<Vector2>();

            foreach (var line in lines)
            {
                if (line.StartsWith("v "))
                {
                    var parts = line[2..].Split(' ');
                    var x = floatParse(parts[0]);
                    var y = floatParse(parts[1]);
                    var z = floatParse(parts[2]);
                    vertices.Add(new Vector3(x, y, z));
                }
                else if (line.StartsWith("f "))
                {
                    var parts = line[2..].Split(' ');

                    if (parts.Length == 3)
                    {
                        // regular triangle
                        triangles.Add(int.Parse(parts[0].Split('/')[0]) - 1);
                        triangles.Add(int.Parse(parts[1].Split('/')[0]) - 1);
                        triangles.Add(int.Parse(parts[2].Split('/')[0]) - 1);
                    }
                    else if (parts.Length == 4)
                    {
                        // quad
                        triangles.Add(int.Parse(parts[0].Split('/')[0]) - 1);
                        triangles.Add(int.Parse(parts[1].Split('/')[0]) - 1);
                        triangles.Add(int.Parse(parts[2].Split('/')[0]) - 1);

                        triangles.Add(int.Parse(parts[0].Split('/')[0]) - 1);
                        triangles.Add(int.Parse(parts[2].Split('/')[0]) - 1);
                        triangles.Add(int.Parse(parts[3].Split('/')[0]) - 1);
                    }
                }
                else if (line.StartsWith("vt "))
                {
                    var parts = line.Split(' ');
                    var u = floatParse(parts[1]);
                    var v = floatParse(parts[2]);
                    uvs.Add(new Vector2(u, v));
                }
            }

            var mesh = new Mesh()
            {
                Vertices = vertices.ToArray(),
                Triangles = triangles.ToArray(),
                UVs = uvs.ToArray()
            };

            return new GameObject()
            {
                Mesh = mesh,
                Texture = new Texture("Textures\\StandardCubeMap-2.jpg")
            };
        }
    }
}