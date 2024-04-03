using ClosedGL;
using ComponentBenchmark.Benchmark;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ComponentBenchmark
{
    public class RenderBenchmark
    {
        public List<GameObject> gameObjects = new List<GameObject>();
        public IRenderer renderer = null!;

        [BeforeAllBenchmark]
        public void BeforeAll()
        {
            gameObjects.Add(GameObject.CreateCube());
            renderer = new CameraGPUFragmentedTiledExplicitGrouping();
            renderer.Position = new VRageMath.Vector3(0, 0, 50);
            var texture = gameObjects.First().Texture!;
            if (texture.Mirror != null)
            {
                texture = texture.Mirror;
            }

            renderer.Initialize([texture]);
        }

        [Benchmark]
        public void Render()
        {
            renderer.Render(gameObjects);
        }

        [Benchmark]
        public void GetFrame()
        {
            renderer.GetFrame(out Bitmap? m);
        }
    }
}
