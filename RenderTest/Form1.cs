using ClosedGL;
using System.Collections.Concurrent;
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
            // register mouse wheel event

            MouseWheel += Form1_MouseWheel;

            // register WASD

            KeyDown += Form1_KeyDown;
            KeyUp += Form1_KeyUp;
        }

        public float FPS { get; set; }
        
        // hashset for all keys that are currently pressed
        private HashSet<Keys> keys = new HashSet<Keys>();

        private void Form1_KeyDown(object? sender, KeyEventArgs e)
        {
            keys.Add(e.KeyCode);
        }

        private void Form1_KeyUp(object? sender, KeyEventArgs e)
        {
            keys.Remove(e.KeyCode);
        }

        float FOV = 70f;
        Vector3 camPos = new Vector3(0, 0, 50);

        private void Form1_MouseWheel(object? sender, MouseEventArgs e)
        {
            if (e.Delta < 0)
            {
                FOV += 1;
            }
            else
            {
                FOV -= 1;
            }

            if (FOV < 1)
            {
                FOV = 1;
            }

            if (FOV > 179)
            {
                FOV = 179;
            }
        }

        protected override void OnPaint(PaintEventArgs e)
        {

        }

        ConcurrentQueue<int> lastFps = new ConcurrentQueue<int>();
        ConcurrentQueue<int> lastFrametimes = new ConcurrentQueue<int>();

        PictureBox pb;
        bool closing = false;

        Vector2 PictureScale = new Vector2(10, -10);

        int frameCount = 0;
        int lastFrameTimestamp = 0;
        int unrenderedFrames = 0;
        Semaphore renderSemaphore = new(1, 1);
        Semaphore criticalState = new(1, 1);

        private void Form1_Load(object sender, EventArgs e)
        {
            GameObject house = GameObject.LoadFromObjFile("Models\\House.obj");
            house.Position = new Vector3(0, 0, -50);
            GameObject go = new Cube();
            GameObject cub = new Cube();
            GameObject cub1 = new Cube();
            GameObject cubi = new Cube();
            cub.Position = new Vector3(30, 19, -2);
            cub.Scale = new Vector3(5f);

            cubi.Position = new Vector3(100, 19, -2);
            cubi.Scale = new Vector3(5f);

            cub1.Position = new Vector3(30, 0, 0);

            List<GameObject> gameObjects = new List<GameObject>() { };

            for (int x = 0; x < 20; x++)
            {
                for (int z = 0; z < 20; z++)
                {
                    GameObject g = new Cube();
                    g.Position = new Vector3((x - 10) * 3, (z - 10) * 3, -15);
                    g.Scale = new Vector3(3f);
                    gameObjects.Add(g);
                }
            }

            for (int x = 0; x < 20; x++)
            {
                for (int z = 0; z < 20; z++)
                {
                    GameObject g = new Cube();
                    g.Position = new Vector3(30, (z - 10) * 3, (x - 10) * 3);
                    g.Scale = new Vector3(3f);
                    gameObjects.Add(g);
                }
            }

            Vector3D cameraVelicity = Vector3.Zero;

            go.Scale = new Vector3(7);
            IRenderer camera = new Camera()
            {
                FieldOfView = 70f,
                Position = new Vector3(0, 0, 10),
                RenderResolution = new Vector2I(Width, Height),
            };

            var t = new Thread(() =>
            {
                pb = new MyPb();

                Invoke(() =>
                {
                    Controls.Add(pb);
                });

                Stopwatch sw = new Stopwatch();
                Stopwatch renderStopwatch = new Stopwatch();
                float x = 0;
                sw.Start();
                criticalState.WaitOne();
                while (!closing)
                {
                    sw.Stop();
                    double deltaTime = (sw.ElapsedMilliseconds == 0 ? 1 : sw.ElapsedMilliseconds);
                    sw.Restart();
                    double fps = 1000d / deltaTime;
                    FPS = (float)fps;
                    lastFps.Enqueue((int)fps);
                    if (lastFps.Count > lastFps.Average() * 2)
                    {
                        lastFps.TryDequeue(out _);
                    }
                    x += .3f * (float)Math.Max(deltaTime / 1000d, 0.01d);
                    deltaTime /= 1000d;
                    float speed = 2;
                    if (keys.Contains(Keys.W))
                    {
                        cameraVelicity += Vector3.Forward * speed;
                    }
                    if (keys.Contains(Keys.S))
                    {
                        cameraVelicity += Vector3.Backward * speed;
                    }
                    if (keys.Contains(Keys.A))
                    {
                        cameraVelicity += Vector3.Left * speed;
                    }
                    if (keys.Contains(Keys.D))
                    {
                        cameraVelicity += Vector3.Right * speed;
                    }
                    if (keys.Contains(Keys.Shift))
                    {
                        cameraVelicity += Vector3.Up * speed;
                    }
                    if (keys.Contains(Keys.LControlKey))
                    {
                        cameraVelicity += Vector3.Down * speed;
                    }

                    cameraVelicity *= 0.95d;

                    camPos += cameraVelicity * deltaTime;

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

                    //go.Scale = new Vector3(20 + 15 * (float)Math.Sin(x), 5, 5);

                    //cub.Rotation = Quaternion.CreateFromYawPitchRoll(x, x, x);
                    camera.RenderResolution = new Vector2I(Width, Height);
                    camera.FieldOfView = FOV;
                    camera.Position = camPos;
                    renderStopwatch.Restart();
                    var res = camera.Render([go, cub, cub1, .. gameObjects/*, cubi,..   /*house*/]);
                    renderStopwatch.Stop();
                    lastFrametimes.Enqueue((int)deltaTime);
                    if (lastFrametimes.Count > 100)
                    {
                        lastFrametimes.TryDequeue(out _);
                    }
                    if (res)
                    {
                        renderCalls++;
                    }
                    //Render(new List<GameObject>() { go, cub, cub1, cubi }.Concat(gameObjects).ToList(), camera, ("FieldOfView", camera.FieldOfView), ("x", x), ("deltaTime", deltaTime), ("camPos", camera.Position), ("res", camera.RenderResolution));
                }
                criticalState.Release();
            });

            t.IsBackground = true;
            t.Name = "RenderThread";
            t.Start();

            var swapper = new Thread(() =>
            {
                try
                {
                    int lastSwapTime = 1;
                    int frameCnt = 0;
                    Stopwatch stopwatch = new Stopwatch();
                    Stopwatch crauy = new Stopwatch();
                    SetStyle(ControlStyles.DoubleBuffer | ControlStyles.UserPaint | ControlStyles.AllPaintingInWmPaint, true);
                    while (!closing)
                    {
                        stopwatch.Restart();
                        int unsuccessfulSwapAttempts = 0;
                        crauy.Restart();
                        int queueLength = camera.GetFrame(out Bitmap? frame);
                        crauy.Stop();

                        if (pb != null && frame != null)
                        {
                            frameCnt++;
                            Graphics g = Graphics.FromImage(frame);

                            Metadata(g,
                                ("Frame #", frameCnt.ToString()),
                                ("Last swap time", lastSwapTime),
                                ("Swaps per second", 1000 / (lastSwapTime == 0 ? 1 : lastSwapTime)),
                                ("FieldOfView", camera.FieldOfView),
                                ("camPos", camera.Position),
                                ("res", camera.RenderResolution),
                                ("FPS (instant)", FPS),
                                ("FPS (average)", lastFps.Average()),
                                ("res", camera.RenderResolution),
                                ("getFrame", crauy.ElapsedMilliseconds),
                                ("frameTime (average)", lastFrametimes.Average()),
                                ("unsuccessFulSwapAttempts", unsuccessfulSwapAttempts),
                                ("swapChainLength", queueLength));
                            Invoke(() =>
                            {
                                pb.Image?.Dispose();
                                pb.Image = frame;
                            });
                        }
                        else
                        {
                            unsuccessfulSwapAttempts++;
                        }
                        stopwatch.Stop();
                        lastSwapTime = (int)stopwatch.ElapsedMilliseconds;
                        //Thread.Sleep(Math.Max(0, (1000 / 60) - lastSwapTime));
                    }
                }
                catch (Exception)
                {

                }
            });

            swapper.IsBackground = true;
            swapper.Name = "SwapperThread";
            swapper.Start();
        }

        private void Metadata(Graphics g, params (string, object)[] values)
        {
            int y = 0;
            foreach (var value in values)
            {
                g.DrawString(value.Item1 + ": " + value.Item2.ToString(), new Font("Arial", 12), Brushes.White, 0, y);
                y += 20;
            }
        }

        private List<Vector3> vertices = new List<Vector3>();
        private List<int> triangles = new List<int>();
        private int renderCalls;

        private void Render(List<GameObject> gameObjects, Camera camera, params (string, object)[] values)
        {
            renderSemaphore.WaitOne();
            frameCount++;
            // first collect all vertices and triangles to be rendered

            vertices.Clear();
            triangles.Clear();
            int vertexOffset = 0;

            // Ensure the camera's projection matrix is updated with the latest FOV and aspect ratio
            MatrixD projectionMatrix = camera.UpdateProjectionMatrix();

            // Go through game objects from furthest to closest
            foreach (var gameObject in gameObjects.OrderBy(x => Vector3.Distance(x.Position, camera.Position)))
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

            // Then project all vertices
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
                    // Only render if points are clockwise
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

            // Then draw all triangles
            Bitmap bitmap;
            try
            {
                // Use camera's render resolution for the bitmap size
                bitmap = new Bitmap((int)camera.RenderResolution.X, (int)camera.RenderResolution.Y);
            }
            catch (Exception)
            {
                renderSemaphore.Release();
                return;
            }

            Vector2 offset = new Vector2(bitmap.Width / 2, bitmap.Height / 2);

            using (Graphics g = Graphics.FromImage(bitmap))
            {
                unrenderedFrames++;

                g.Clear(System.Drawing.Color.Black);
                // Frame count
                g.DrawString("Frame #" + frameCount.ToString(), new Font("Arial", 12), Brushes.White, 0, 0);

                // FPS
                var now = DateTime.Now.Millisecond;
                int fps = (int)(1000f / (now - lastFrameTimestamp));
                lastFps.Enqueue(fps);
                if (lastFps.Count > 10)
                {
                    lastFps.TryDequeue(out _);
                }
                lastFrameTimestamp = now;

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

                g.DrawString("Fps: " + lastFps.Average().ToString(), new Font("Arial", 12), Brushes.White, 0, 20);

                // Vertices
                g.DrawString("Vertices: " + projectedVertices.Count.ToString(), new Font("Arial", 12), Brushes.White, 0, 60);

                // Triangles
                g.DrawString("Triangles: " + (actualTriangles.Count / 3).ToString(), new Font("Arial", 12), Brushes.White, 0, 80);

                // Unrendered frames
                g.DrawString("Unrendered frames: " + unrenderedFrames.ToString(), new Font("Arial", 12), Brushes.White, 0, 40);

                // Values
                int y = 100;
                foreach (var value in values)
                {
                    g.DrawString(value.Item1 + ": " + value.Item2.ToString(), new Font("Arial", 12), Brushes.White, 0, y);
                    y += 20;
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

            // Cleanup
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