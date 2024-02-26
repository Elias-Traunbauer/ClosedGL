using ClosedGL;
using System.Diagnostics;
using System.Windows.Forms;
using VRageMath;

namespace RenderTest
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        protected override void OnPaint(PaintEventArgs e)
        {

        }

        Queue<int> lastFps = new Queue<int>();

        PictureBox pb;
        bool closing = false;

        Vector2 PictureScale = new Vector2(10, -10);

        int frameCount = 0;
        int lastFrameTimestamp = 0;
        int unrenderedFrames = 0;
        Semaphore renderSemaphore = new Semaphore(1, 1);
        Semaphore criticalState = new Semaphore(1, 1);

        private void Form1_Load(object sender, EventArgs e)
        {
            List<GameObject> floor = new List<GameObject>();

            // fill floor with cubes
            //for (int x = -100; x < 100; x += 10)
            //{
            //    for (int z = -100; z < 100; z += 10)
            //    {
            //        GameObject cube = new Cube();
            //        cube.Position = new Vector3(x, -13, z);
            //        cube.Scale = new Vector3(9.5f);
            //        floor.Add(cube);
            //    }
            //}

            // fill wall on the right side
            for (int x = -100; x < 100; x += 10)
            {
                for (int z = -100; z < 100; z += 10)
                {
                    GameObject cube = new Cube();
                    cube.Position = new Vector3(20, x, z);
                    cube.Scale = new Vector3(9.5f);
                    floor.Add(cube);
                }
            }

            GameObject go = new Cube();
            GameObject cub = new Cube();
            GameObject cub1 = new Cube();
            GameObject cubi = new Cube();
            GameObject cub213123 = new Cube();
            cub213123.Position = new Vector3D(20, 0, 0);
            cub213123.Scale = new Vector3(6f);

            cub.Position = new Vector3(30, 19, -2);
            cub.Scale = new Vector3(5f);

            cubi.Position = new Vector3(100, 19, -2);
            cubi.Scale = new Vector3(5f);

            cub1.Position = new Vector3(30, 0, 0);

            go.Scale = new Vector3(7);
            Camera camera = new()
            {
                FieldOfView = 70f,
                Position = new Vector3(0, 0, 2)
            };

            var t = new Thread(() =>
            {
                pb = new MyPb();
                
                Invoke(() =>
                {
                    Controls.Add(pb);
                });

                Stopwatch sw = new Stopwatch();
                float x = 0;
                sw.Start();
                criticalState.WaitOne();
                while (!closing)
                {
                    sw.Stop();
                    var deltaTime = (sw.ElapsedTicks / 10000d) / 1000d;

                    x += 2f * (float)deltaTime;

                    //camera.FieldOfView = (float)Math.Sin(x) * 35 + 60;

                    Vector3 cubPos = Vector3.Up * 30 * Quaternion.CreateFromAxisAngle(Vector3.Forward, x * 0.7f + (float)Math.Sin(x));
                    cub.Position = cubPos;

                    Vector3 cudddbPos = Vector3.Forward * 30 * Quaternion.CreateFromAxisAngle(Vector3.Right, x * 0.4f + (float)Math.Sin(x));
                    cub1.Position = cudddbPos + Vector3.Left * 20;
                    cub1.Scale = new Vector3(7);
                    cub1.Rotation = Quaternion.CreateFromYawPitchRoll(0, (float)Math.Sin(x), 0);

                    cubi.Position = new Vector3(frameCount % 220f - 110, 19, -2);

                    //Vector3 camPos = Vector3.Backward * 30 * Quaternion.CreateFromAxisAngle(Vector3.Up, x * 0.2f);
                    //camera.Position = camPos;

                    //// let cam look at Vector3.Zero
                    //camera.Rotation = Quaternion.CreateFromYawPitchRoll(x * -0.2f, 0, 0);

                    go.Rotation = Quaternion.CreateFromYawPitchRoll((float)Math.Sin(x) * 3, x, x);

                    go.Scale = new Vector3(20 + 15 * (float)Math.Sin(x), 5, 5);

                    //cub.Rotation = Quaternion.CreateFromYawPitchRoll(x, x, x);

                    renderSemaphore.WaitOne();
                    Render(new List<GameObject>() { go, cub, cub1, cubi, cub213123 }.Concat(floor).ToList(), camera, ("FieldOfView", camera.FieldOfView), ("x", x));

                    sw.Restart();

                    Thread.Sleep(TimeSpan.FromTicks(10000));
                }
                criticalState.Release();
            });

            t.IsBackground = true;
            t.Name = "RenderThread";
            t.Start();
        }

        private void Render(List<GameObject> gameObjects, Camera camera, params (string, object)[] values)
        {
            frameCount++;
            // first collect all vertices and triangles to be rendered

            List<Vector3> vertices = new List<Vector3>();
            List<int> triangles = new List<int>();
            int vertexOffset = 0;

            // go through game objects by furthest to closest
            foreach (var gameObject in gameObjects.OrderByDescending(x => Vector3.Distance(x.Position, camera.Position)))
            {
                if (gameObject.Mesh == null)
                {
                    continue;
                }

                foreach (var vertex in gameObject.Mesh.Vertices)
                {
                    vertices.Add(gameObject.Position + (vertex * gameObject.Scale) * gameObject.Rotation);
                }

                foreach (var triangle in gameObject.Mesh.Triangles)
                {
                    triangles.Add(triangle + vertexOffset);
                }

                vertexOffset += gameObject.Mesh.Vertices.Length;
            }

            // then project all vertices

            List<Vector2> projectedVertices = new List<Vector2>();
            List<int> actualTriangles = new List<int>();
            int triangleOffset = 0;

            for (int i = 0; i < triangles.Count; i += 3)
            {
                var v1 = vertices[triangles[i]];
                var v2 = vertices[triangles[i + 1]];
                var v3 = vertices[triangles[i + 2]];

                var p1 = camera.ProjectPoint(v1);
                var p2 = camera.ProjectPoint(v2);
                var p3 = camera.ProjectPoint(v3);

                if (p1 != null && p2 != null && p3 != null)
                {
                    // only render if points are clockwise

                    var p1p2 = p2 - p1;
                    var p1p3 = p3 - p1;

                    var cross = Vector3.Cross(new Vector3(p1p2.Value.X, p1p2.Value.Y, 0), new Vector3(p1p3.Value.X, p1p3.Value.Y, 0));

                    if (cross.Z < 0)
                    {
                        continue;
                    }

                    projectedVertices.Add(p1.Value);
                    projectedVertices.Add(p2.Value);
                    projectedVertices.Add(p3.Value);

                    actualTriangles.Add(triangleOffset);
                    actualTriangles.Add(triangleOffset + 1);
                    actualTriangles.Add(triangleOffset + 2);

                    triangleOffset += 3;
                }
            }

            // then draw all triangles#
            Bitmap bitmap;
            try
            {
                bitmap = new Bitmap(pb.Width, pb.Height);
            }
            catch (Exception)
            {
                renderSemaphore.Release();
                return;
            }
            Vector2 offset = new Vector2(pb.Width / 2, pb.Height / 2);

            using (Graphics g = Graphics.FromImage(bitmap))
            {
                unrenderedFrames++;

                g.Clear(System.Drawing.Color.Black);
                // frame cnt
                g.DrawString("Frame #" + frameCount.ToString(), new Font("Arial", 12), Brushes.White, 0, 0);

                // fps
                var now = DateTime.Now.Millisecond;
                int fps = (int)(1000f / (now - lastFrameTimestamp));
                lastFps.Enqueue(fps);
                if (lastFps.Count > 10)
                {
                    lastFps.Dequeue();
                }
                lastFrameTimestamp = now;
                g.DrawString("Fps: " + lastFps.Average().ToString(), new Font("Arial", 12), Brushes.White, 0, 20);

                // vertices
                g.DrawString("Vertices: " + projectedVertices.Count.ToString(), new Font("Arial", 12), Brushes.White, 0, 60);

                // triangles
                g.DrawString("Triangles: " + (actualTriangles.Count / 3).ToString(), new Font("Arial", 12), Brushes.White, 0, 80);

                // unrendered frames
                g.DrawString("Unrendered frames: " + unrenderedFrames.ToString(), new Font("Arial", 12), Brushes.White, 0, 40);

                // values
                int y = 100;
                foreach (var value in values)
                {
                    g.DrawString(value.Item1 + ": " + value.Item2.ToString(), new Font("Arial", 12), Brushes.White, 0, y);
                    y += 20;
                }

                for (int i = 0; i < actualTriangles.Count; i += 3)
                {
                    var v1 = projectedVertices[actualTriangles[i]] * PictureScale + offset;
                    var v2 = projectedVertices[actualTriangles[i + 1]] * PictureScale + offset;
                    var v3 = projectedVertices[actualTriangles[i + 2]] * PictureScale + offset;

                    try
                    {
                        g.FillPolygon(Brushes.Gray, new PointF[] { new PointF(v1.X, v1.Y), new PointF(v2.X, v2.Y), new PointF(v3.X, v3.Y) });

                        g.DrawLine(Pens.White, v1.X, v1.Y, v2.X, v2.Y);
                        g.DrawLine(Pens.White, v2.X, v2.Y, v3.X, v3.Y);
                        g.DrawLine(Pens.White, v3.X, v3.Y, v1.X, v1.Y);
                    }
                    catch (Exception)
                    {

                    }
                }
            }

            try
            {
                lock (pb)
                {
                    pb.Image?.Dispose();
                    pb.Image = bitmap;
                    unrenderedFrames--;
                }
            }
            catch (Exception)
            {

            }
            finally
            {
                renderSemaphore.Release();
            }

            // cleanup

            vertices.Clear();
            triangles.Clear();
            projectedVertices.Clear();
            actualTriangles.Clear();
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            closing = true;
            criticalState.WaitOne();
        }
    }
}