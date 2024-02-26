using System.Collections.Concurrent;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using VRageMath;

namespace TriangleTest
{
    public partial class Form1 : Form
    {
        Bitmap Texture;
        Vector2[] ProjectedVertices = new Vector2[3];
        Vector2[] UVs = new Vector2[3];
        ConcurrentQueue<Bitmap> SwapChain = new ConcurrentQueue<Bitmap>();
        bool w, a, s, d;
        float zoom = 1;
        public Form1()
        {
            InitializeComponent();
            KeyDown += (s, e) =>
            {
                if (e.KeyCode == Keys.W)
                    w = true;
                if (e.KeyCode == Keys.A)
                    a = true;
                if (e.KeyCode == Keys.S)
                    this.s = true;
                if (e.KeyCode == Keys.D)
                    d = true;
            };
            KeyUp += (s, e) =>
            {
                if (e.KeyCode == Keys.W)
                    w = false;
                if (e.KeyCode == Keys.A)
                    a = false;
                if (e.KeyCode == Keys.S)
                    this.s = false;
                if (e.KeyCode == Keys.D)
                    d = false;
            };

            MouseWheel += (s, e) =>
            {
                zoom += e.Delta / 1200f;
            };
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            Texture = (Bitmap)Image.FromFile("triangle.png");

            ProjectedVertices = new Vector2[3]
            {
                new Vector2(100, 100),
                new Vector2(200, 100),
                new Vector2(150, 200)
            };

            UVs = new Vector2[3]
            {
                new Vector2(0, 0),
                new Vector2(1, 0),
                new Vector2(/*0.5f*/0, 1)
            };

            Thread worker = new Thread(Worker);
            worker.IsBackground = true;
            worker.Name = "Worker";
            worker.Priority = ThreadPriority.BelowNormal;

            worker.Start();

            Thread swapper = new Thread(Swapper);
            swapper.IsBackground = true;
            swapper.Name = "Swapper";
            swapper.Priority = ThreadPriority.BelowNormal;

            swapper.Start();
        }

        private void Worker()
        {
            unsafe
            {
                for (; ; )
                {
                    Bitmap bitmap = new Bitmap(Width, Height);

                    using (Graphics graphics = Graphics.FromImage(bitmap))
                    {
                        graphics.Clear(System.Drawing.Color.Black);

                        var verticesAsArray = ProjectedVertices.Select(x => new PointF(x.X, x.Y)).ToArray();

                        // Create BitmapData for faster pixel manipulation
                        BitmapData bmpData = bitmap.LockBits(new System.Drawing.Rectangle(0, 0, bitmap.Width, bitmap.Height), ImageLockMode.WriteOnly, bitmap.PixelFormat);

                        // Pointer to the first pixel
                        IntPtr ptr = bmpData.Scan0;

                        // Bytes per pixel
                        int bytesPerPixel = Bitmap.GetPixelFormatSize(bitmap.PixelFormat) / 8;

                        // Stride: the number of bytes allocated for a single row
                        int stride = bmpData.Stride;

                        var a = ProjectedVertices[0];
                        var b = ProjectedVertices[1];
                        var c = ProjectedVertices[2];

                        Vector2 min = Vector2.Min(Vector2.Min(a, b), c);
                        Vector2 max = Vector2.Max(Vector2.Max(a, b), c);

                        for (int x = (int)min.X; x <= (int)max.X; x++)
                        {
                            for (int y = (int)min.Y; y <= (int)max.Y; y++)
                            {
                                Vector2 point = new Vector2(x, y);
                                if (IsPointInTriangle(point, a, b, c))
                                {
                                    Vector3 barycentricCoords = CalculateBarycentricCoordinates(point, a, b, c);
                                    Vector2 interpolatedUV = InterpolateUV(barycentricCoords, UVs[0], UVs[1], UVs[2]);
                                    
                                    // sample the texture
                                    int textureX = (int)(interpolatedUV.X * Texture.Width);
                                    int textureY = (int)(interpolatedUV.Y * Texture.Height);

                                    // clamp the texture coordinates
                                    textureX = Math.Max(0, Math.Min(Texture.Width - 1, textureX));
                                    textureY = Math.Max(0, Math.Min(Texture.Height - 1, textureY));

                                    System.Drawing.Color textureColor = Texture.GetPixel(textureX, textureY);

                                    byte* currentLine = (byte*)ptr + (y * stride);

                                    // Set the pixel color in the bitmap
                                    currentLine[x * bytesPerPixel] = textureColor.B;
                                    currentLine[x * bytesPerPixel + 1] = textureColor.G;
                                    currentLine[x * bytesPerPixel + 2] = textureColor.R;
                                }
                            }
                        }
                        //byte* currentLine = (byte*)ptr + (y * stride);

                        //// Set the pixel color in the bitmap
                        //currentLine[x * bytesPerPixel] = textureColor.B;
                        //currentLine[x * bytesPerPixel + 1] = textureColor.G;
                        //currentLine[x * bytesPerPixel + 2] = textureColor.R;

                        // Unlock the bitmap data
                        bitmap.UnlockBits(bmpData);

                        // Draw the triangle
                        graphics.DrawPolygon(new Pen(System.Drawing.Color.LightPink), verticesAsArray);
                    }


                    // store the drawn vertices in the swap chain
                    SwapChain.Enqueue(bitmap);
                    Thread.Sleep(10);
                }
            }
        }

        private void Swapper()
        {
            try
            {
                for (; ; )
                {
                    if (Disposing || IsDisposed || !IsHandleCreated)
                        return;
                    Invoke(new Action(() =>
                    {
                        if (SwapChain.TryDequeue(out Bitmap bitmap))
                        {
                            Graphics g = Graphics.FromHwnd(Handle);
                            // apply zoom
                            g.ScaleTransform(zoom, zoom);
                            g.DrawImage(bitmap, 0, 0, Width, Height);
                            g.Dispose();
                            bitmap.Dispose();
                        }
                    }));
                    Thread.Sleep(5);
                }
            }
            catch (Exception)
            {

            }
        }

        public static Vector3 CalculateBarycentricCoordinates(Vector2 point, Vector2 a, Vector2 b, Vector2 c)
        {
            float detT = (b.Y - c.Y) * (a.X - c.X) + (c.X - b.X) * (a.Y - c.Y);
            float wA = ((b.Y - c.Y) * (point.X - c.X) + (c.X - b.X) * (point.Y - c.Y)) / detT;
            float wB = ((c.Y - a.Y) * (point.X - c.X) + (a.X - c.X) * (point.Y - c.Y)) / detT;
            float wC = 1 - wA - wB;

            return new Vector3(wA, wB, wC);
        }

        public static Vector2 InterpolateUV(Vector3 barycentricCoords, Vector2 uvA, Vector2 uvB, Vector2 uvC)
        {
            float interpolatedU = barycentricCoords.X * uvA.X + barycentricCoords.Y * uvB.X + barycentricCoords.Z * uvC.X;
            float interpolatedV = barycentricCoords.X * uvA.Y + barycentricCoords.Y * uvB.Y + barycentricCoords.Z * uvC.Y;

            return new Vector2(interpolatedU, interpolatedV);
        }

        public static bool IsPointInTriangle(Vector2 p, Vector2 a, Vector2 b, Vector2 c)
        {
            float detT = (b.Y - c.Y) * (a.X - c.X) + (c.X - b.X) * (a.Y - c.Y);
            float wA = ((b.Y - c.Y) * (p.X - c.X) + (c.X - b.X) * (p.Y - c.Y)) / detT;
            float wB = ((c.Y - a.Y) * (p.X - c.X) + (a.X - c.X) * (p.Y - c.Y)) / detT;
            float wC = 1 - wA - wB;

            return wA >= 0 && wB >= 0 && wC >= 0;
        }
    }
}
