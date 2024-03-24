using ClosedGL;
using ClosedGL.Memory;
using ComponentBenchmark.Benchmark;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRageMath;

namespace ComponentBenchmark
{
    public class ArrayHandlingComplexBenchmark
    {
        public IEnumerable<Mesh> Meshes { get; set; } = new List<Mesh>();

        [BeforeAllBenchmark]
        public void BeforeEach()
        {
            List<GameObject> list = [];
            GameObject volvo = GameObject. LoadFromObjFile("Models\\volvo 740 turbo.obj");
            GameObject volvo2 = GameObject.LoadFromObjFile("Models\\volvo 740 turbo.obj");
            for (int i = 0; i < 0; i++)
            {
                list.Add(new Cube());
            }
            list.Add(volvo);
            list.Add(volvo2);
            Meshes = list.Select(x => x.Mesh!).Where(x => x != null)!;
        }

        public void BeforeEachFast()
        {
            List<GameObject> list = [];
            GameObject volvo = GameObject.LoadFromObjFile("Models\\volvo 740 turbo.obj");
            GameObject volvo2 = GameObject.LoadFromObjFile("Models\\volvo 740 turbo.obj");
            for (int i = 0; i < 0; i++)
            {
                list.Add(new Cube());
            }
            list.Add(volvo);
            list.Add(volvo2);
            Meshes = list.Select(x => x.Mesh!)!;
        }

        [Benchmark]
        public unsafe void UnsafeArrayHandlerMeshBenchmark()
        {
            Vec3[] result = UnsafeArrayHandler.FlattenAndCastArrays<Vector3, Vec3, Mesh>(Meshes, x => x.Vertices);
        }

        [Benchmark]
        public unsafe void UnsafeArrayHandlerMeshBETABenchmark()
        {
            Vec3[] result = UnsafeArrayHandler.FlattenAndCastArraysBeta<Vector3, Vec3, Mesh>(Meshes, x => x.Vertices);
        }

        [Benchmark]
        public void LINQMeshBenchmark()
        {
            Vec3[] result = Meshes.SelectMany(x => x!.Vertices).Select(x => new Vec3(x.X, x.Y, x.Z)).ToArray();
        }

        [Benchmark]
        public void LINQMeshCastBenchmark()
        {
            Vec3[] result = Meshes.SelectMany(x => x!.Vertices).Select(x => (Vec3)x).ToArray();
        }
    }
}
