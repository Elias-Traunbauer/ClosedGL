using System.Collections.Concurrent;
using System.Drawing;
using System.Runtime.InteropServices;
using VRageMath;

namespace ClosedGL
{
    /// <summary>
    /// Class to project points from a position onto an lcd
    /// NOTE: Only works if the ViewPoint is infront of the lcd -> Transparent LCDS from the back dont work
    /// </summary>
    public class CameraParallel : GameObject, IRenderer
    {
        private static readonly Texture defaultTexture = new(1, 1)
        {
            // gray pixel
            Data = new byte[]
                    {
                128, 128, 128, 255
                    }
        };
        public static Texture DEFAULT_TEXTURE = defaultTexture;

        public float FieldOfView
        {
            get;
            set;
        } = 70f;
        public float AspectRatio => RenderResolution.X / RenderResolution.Y;
        public float NearClipPlane { get; set; } = 0.1f;      // Default near clip plane distance
        public float FarClipPlane { get; set; } = 1000f;      // Default far clip plane distance
        public Vector2I RenderResolution { get; set; } = new Vector2I(1920, 1080);  // Default resolution (width, height)
        private readonly ConcurrentQueue<byte[]> swapChain = new();

        public int GetFrame(out Bitmap? bitmap)
        {
            bitmap = null;
            if (swapChain.TryDequeue(out byte[]? frame))
            {
                if (frame == null)
                {
                    return 0;
                }
                try
                {
                    // Load into bitmap
                    //int stride = (int)(frame.Length / 4 / RenderResolution.Y);
                    bitmap = new Bitmap((int)RenderResolution.X, (int)RenderResolution.Y);
                    var lockInfo = bitmap.LockBits(new System.Drawing.Rectangle(0, 0, bitmap.Width, bitmap.Height), System.Drawing.Imaging.ImageLockMode.WriteOnly, System.Drawing.Imaging.PixelFormat.Format32bppArgb);

                    try
                    {
                        unsafe
                        {
                            // Calculate the number of bytes to copy
                            int byteCount = Math.Min(bitmap.Width * bitmap.Height * 4, frame.Length);

                            // Copy frame to locked bits using Marshal.Copy
                            Marshal.Copy(frame, 0, lockInfo.Scan0, byteCount);
                        }
                    }
                    finally
                    {
                        bitmap.UnlockBits(lockInfo);
                        frame = null;
                    }

                    return swapChain.Count;
                }
                catch (Exception ex)
                {
                    // Handle exceptions (e.g., log, return default value, etc.)
                    Console.WriteLine($"Error creating bitmap: {ex.Message}");
                }
            }

            return 0;
        }

        private readonly Vector3D Normal = Vector3D.Forward;

        public CameraParallel()
        {

        }

        public Vector2? ProjectPoint(Vector3D worldPoint)
        {
            // Calculate the view direction based on FOV and rotation
            var viewPoint = Vector3D.Backward * Rotation;

            // Convert world position into a world direction
            Vector3D worldDirection = worldPoint - viewPoint;

            // Convert world direction into a local direction
            Vector3D localPointToProject = Vector3D.TransformNormal(worldDirection, MatrixD.Transpose(this.WorldMatrix));

            // Ray direction in local space
            Vector3D localRayDirection = localPointToProject - viewPoint;

            // Normalize the ray direction for correct projection
            localRayDirection.Normalize();

            // Project the point onto the plane
            Vector2? projectedLocalPoint = PlaneIntersection(Vector3D.Backward, localRayDirection);

            if (projectedLocalPoint != null)
            {
                var projectedLocalPointNonNullable = (Vector2)projectedLocalPoint;

                // Convert it to pixels
                Vector2 projectedPixelPoint = new Vector2(
                    projectedLocalPointNonNullable.X * RenderResolution.X / 2f + RenderResolution.X / 2f,
                    projectedLocalPointNonNullable.Y * RenderResolution.Y / 2f + RenderResolution.Y / 2f
                );

                return projectedPixelPoint;
            }

            return null;
        }

        public Vector3? ProjectPointLocal(Vector3D localPoint)
        {
            var viewPoint = Vector3D.Backward * Rotation;

            // Ray direction in local space
            Vector3D localRayDirection = localPoint - viewPoint;

            // Normalize the ray direction for correct projection
            localRayDirection.Normalize();

            // Project the point onto the plane
            Vector2? projectedLocalPoint = PlaneIntersection(Vector3D.Backward, localRayDirection);

            if (projectedLocalPoint != null)
            {
                var projectedLocalPointNonNullable = (Vector2)projectedLocalPoint;

                // Convert it to pixels
                Vector3 projectedPixelPoint = new Vector3(
                    projectedLocalPointNonNullable.X * RenderResolution.X / 2f + RenderResolution.X / 2f,
                    projectedLocalPointNonNullable.Y * RenderResolution.Y / 2f + RenderResolution.Y / 2f,
                    // distance to the vertex
                    localPoint.Length()
                );

                return projectedPixelPoint;
            }

            return null;
        }

        /// <summary>
        /// Calculates the intersection point from the given line and a plane with origin (0,0,0) and the normal (static)
        /// </summary>
        /// <param name="origin">Line origin</param>
        /// <param name="dir">Line direction</param>
        /// <returns>The projected point</returns>
        private Vector2? PlaneIntersection(Vector3D origin, Vector3D dir)
        {
            // Check if the line is pointing away from the plane
            if (dir.Z >= 0)
            {
                return null;
            }

            // Ensure the denominator is not zero to avoid division by zero
            var denominator = Vector3D.Dot(dir, Normal);
            if (denominator == 0)
            {
                return null;
            }

            // Calculate the parameter t for the line equation
            var t = -(Vector3D.Dot(origin, Normal) + 0) / denominator;

            // Calculate the intersection point in 3D
            Vector3D intersectionPoint = origin + t * dir;

            // Return the projected point in 2D (X and Y)
            return new Vector2((float)intersectionPoint.X, (float)intersectionPoint.Y);
        }

        static Vector3D LocalDirToWorldDir(Vector3D dir, MatrixD matrix)
        {
            return Vector3D.TransformNormal(dir, matrix);
        }

        static Vector3D LocalPosToWorldPos(Vector3D pos, MatrixD matrix)
        {
            return Vector3D.Transform(pos, matrix);
        }

        /// <summary>
        /// Method to update the projection matrix based on camera properties
        /// </summary>
        /// <returns>The updated projection matrix</returns>
        public MatrixD UpdateProjectionMatrix()
        {
            // Calculate the projection matrix based on aspect ratio, field of view, near, and far clip planes
            return MatrixD.CreatePerspectiveFieldOfView(MathHelper.ToRadians(FieldOfView), AspectRatio, NearClipPlane, FarClipPlane);
        }

        float[] depthBuffer = new float[0];

        public bool Render(List<GameObject> gameObjects)
        {
            //if (swapChain.Count > 2)
            //{
            //    swapChain.TryDequeue(out _);
            //}

            PrepareRendering(gameObjects,
                out List<Vector3> vertices,
                out List<int> triangles,
                out List<Vector2> uvs,
                out List<Texture> texs,
                out List<int> trianglesPerTexture);

            int depthBufferRequiredLength = (int)RenderResolution.X * (int)RenderResolution.Y;
            if (depthBuffer.Length != depthBufferRequiredLength)
            {
                // Initialize depth buffer with farthest depth value
                float farthestDepth = float.MaxValue;
                depthBuffer = Enumerable.Repeat(farthestDepth, depthBufferRequiredLength).ToArray();

            }
            else
            {
                // Clear the depth buffer at the beginning of each frame
                float farthestDepth = float.MaxValue;
                Array.Fill(depthBuffer, farthestDepth);

            }

            int renderedTriangles = 0;

            unsafe
            {
                int bytesPerPixel = 4;
                byte[] image = new byte[(int)RenderResolution.X * (int)RenderResolution.Y * bytesPerPixel];

                int stride = (int)RenderResolution.X * bytesPerPixel;

                int currentTextureIndex = 0;
                int currentTriangleCounter = 0;

                Parallel.For(0, triangles.Count / 3, i =>
                {
                    i *= 3;

                    var v1 = vertices[triangles[i]];
                    var v2 = vertices[triangles[i + 1]];
                    var v3 = vertices[triangles[i + 2]];

                    var p1 = ProjectPointLocal(v1);
                    var p2 = ProjectPointLocal(v2);
                    var p3 = ProjectPointLocal(v3);

                    // get texture for the correct gameObject
                    Texture texture = texs[currentTextureIndex];
                    currentTriangleCounter++;

                    if (currentTriangleCounter >= trianglesPerTexture[currentTextureIndex])
                    {
                        currentTextureIndex++;
                        currentTriangleCounter = 0;
                    }

                    if (p1 != null && p2 != null && p3 != null)
                    {
                        var p1NonNullable = (Vector2)p1;
                        var p2NonNullable = (Vector2)p2;
                        var p3NonNullable = (Vector2)p3;

                        var p1Distance = p1.Value.Z;
                        var p2Distance = p2.Value.Z;
                        var p3Distance = p3.Value.Z;

                        // check if the 2d points are clockwise
                        // if not, skip the triangle

                        // Only render if points are clockwise
                        var p1p2 = p2NonNullable - p1NonNullable;
                        var p1p3 = p3NonNullable - p1NonNullable;

                        var cross = Vector3.Cross(new Vector3(p1p2.X, p1p2.Y, 0), new Vector3(p1p3.X, p1p3.Y, 0));

                        if (cross.Z < 0)
                        {
                            return;
                        }

                        if (IsTriangleOutOfBounds(p1NonNullable, p2NonNullable, p3NonNullable) && false)
                        {
                            return;  // Skip the entire triangle if it's out of bounds
                        }

                        Vector2 min = Vector2.Min(Vector2.Min(p1NonNullable, p2NonNullable), p3NonNullable);
                        Vector2 max = Vector2.Max(Vector2.Max(p1NonNullable, p2NonNullable), p3NonNullable);

                        // clamp the min and max to the render resolution
                        min.X = Math.Max(0, Math.Min(RenderResolution.X - 1, min.X));
                        min.Y = Math.Max(0, Math.Min(RenderResolution.Y - 1, min.Y));
                        max.X = Math.Max(0, Math.Min(RenderResolution.X - 1, max.X));
                        max.Y = Math.Max(0, Math.Min(RenderResolution.Y - 1, max.Y));

                        for (int x = (int)min.X; x <= (int)max.X; x++)
                        {
                            for (int y = (int)min.Y; y <= (int)max.Y; y++)
                            {
                                Vector2 point = new(x, y);
                                if (IsPointInTriangle(point, p1NonNullable, p2NonNullable, p3NonNullable)
                                    /*&& IsPointInRenderView(point)*/)
                                {
                                    Vector3 barycentricCoords = CalculateBarycentricCoordinates(point, p1NonNullable, p2NonNullable, p3NonNullable);
                                    Vector2 interpolatedUV = InterpolateUV(barycentricCoords, uvs[triangles[i]], uvs[triangles[i + 1]], uvs[triangles[i + 2]]);

                                    // Depth test
                                    float depth = barycentricCoords.X * p1Distance + barycentricCoords.Y * p2Distance + barycentricCoords.Z * p3Distance;

                                    int depthIndex = x + (int)RenderResolution.X * y;
                                    if (depth < depthBuffer[depthIndex])
                                    {
                                        // Pixel is closer, update depth buffer and render pixel
                                        depthBuffer[depthIndex] = depth;
                                        // sample the texture
                                        int textureX = (int)(interpolatedUV.X * texture.Width);
                                        int textureY = (int)(interpolatedUV.Y * texture.Height);

                                        // clamp the texture coordinates
                                        textureX = Math.Max(0, Math.Min(texture.Width - 1, textureX));
                                        textureY = Math.Max(0, Math.Min(texture.Height - 1, textureY));

                                        byte[] pixel = texture.GetPixelAsBytes(textureX, textureY);

                                        int currentLine = (y * stride);

                                        // Set the pixel color in the bitmap
                                        image[currentLine + x * bytesPerPixel + 0] = pixel[0];
                                        image[currentLine + x * bytesPerPixel + 1] = pixel[1];
                                        image[currentLine + x * bytesPerPixel + 2] = pixel[2];
                                        image[currentLine + x * bytesPerPixel + 3] = pixel[3];

                                        renderedTriangles++;
                                    }
                                }
                            }
                        }

                        // draw light purple debug lines between the points 199,67,117

                        //DrawLine(p1NonNullable, p2NonNullable, /*purle BGRA*/[117, 67, 199, 255], image, stride);
                        //DrawLine(p2NonNullable, p3NonNullable, /*purle BGRA*/[117, 67, 199, 255], image, stride);
                        //DrawLine(p3NonNullable, p1NonNullable, /*purle BGRA*/[117, 67, 199, 255], image, stride);

                        //// draw the square that got clamped
                        //DrawLine(min, new(min.X, max.Y), /*red BGRA*/[0, 0, 255, 255], image, stride);
                        //DrawLine(new(min.X, max.Y), max, /*red BGRA*/[0, 0, 255, 255], image, stride);
                        //DrawLine(max, new(max.X, min.Y), /*red BGRA*/[0, 0, 255, 255], image, stride);
                        //DrawLine(new(max.X, min.Y), min, /*red BGRA*/[0, 0, 255, 255], image, stride);
                    }
                });


                // queue the frame
                swapChain.Enqueue(image);
            }
            return true;
        }

        private bool IsPointInRenderView(Vector2 point)
        {
            return point.X >= 0 && point.X < RenderResolution.X && point.Y >= 0 && point.Y < RenderResolution.Y;
        }

        bool IsTriangleOutOfBounds(Vector2 p1, Vector2 p2, Vector2 p3)
        {
            return p1.X > RenderResolution.X - 1 || p1.X < 0 || p1.Y > RenderResolution.Y - 1 || p1.Y < 0 ||
                   p2.X > RenderResolution.X - 1 || p2.X < 0 || p2.Y > RenderResolution.Y - 1 || p2.Y < 0 ||
                   p3.X > RenderResolution.X - 1 || p3.X < 0 || p3.Y > RenderResolution.Y - 1 || p3.Y < 0;
        }

        private unsafe void DrawLine(Vector2 p1NonNullable, Vector2 p2NonNullable, byte[] value, byte[] image, int stride)
        {
            int x0 = (int)p1NonNullable.X;
            int y0 = (int)p1NonNullable.Y;
            int x1 = (int)p2NonNullable.X;
            int y1 = (int)p2NonNullable.Y;

            int dx = Math.Abs(x1 - x0);
            int dy = Math.Abs(y1 - y0);
            int sx = x0 < x1 ? 1 : -1;
            int sy = y0 < y1 ? 1 : -1;
            int err = dx - dy;

            while (true)
            {
                if (x0 >= 0 && x0 < RenderResolution.X && y0 >= 0 && y0 < RenderResolution.Y)
                {
                    int currentLine = (y0 * stride);
                    image[currentLine + x0 * 4 + 0] = value[0];
                    image[currentLine + x0 * 4 + 1] = value[1];
                    image[currentLine + x0 * 4 + 2] = value[2];
                    image[currentLine + x0 * 4 + 3] = value[3];
                }

                if (x0 == x1 && y0 == y1)
                {
                    break;
                }

                int e2 = 2 * err;
                if (e2 > -dy)
                {
                    err -= dy;
                    x0 += sx;
                }
                if (e2 < dx)
                {
                    err += dx;
                    y0 += sy;
                }
            }
        }

        private void PrepareRendering(List<GameObject> gameObjects, out List<Vector3> vertices, out List<int> triangles, out List<Vector2> uvs, out List<Texture> texs, out List<int> trianglesPerTexture)
        {
            // only take gameobjects in front of the camera
            // check with dot product
            HashSet<GameObject> gameObjectsToRender = new();
            foreach (var gameObject in gameObjects.Where(x => x.Mesh != null))
            {
                var localPosition = TransformToLocal(gameObject.Position);
                if (localPosition.Z < 0)
                {
                    gameObjectsToRender.Add(gameObject);
                }
            }

            vertices = new();
            triangles = new();
            uvs = new();
            texs = new();
            trianglesPerTexture = new();
            int vertexOffset = 0;
            foreach (var gameObject in gameObjectsToRender/*.OrderByDescending(x => Vector3.Distance(Position, x.Position))*/)
            {
                foreach (var vertex in gameObject.Mesh!.Vertices)
                {
                    Vector3 worldVertexPosition = (vertex * gameObject.Rotation * gameObject.Scale) + gameObject.Position;
                    //var localVertexPosition = TransformToLocal(worldVertexPosition);
                    var localVertexPositionBeta = TransformToLocal(worldVertexPosition);
                    vertices.Add(localVertexPositionBeta);
                }
                foreach (var uv in gameObject.Mesh.UVs)
                {
                    uvs.Add(uv);
                }
                foreach (var triangle in gameObject.Mesh.Triangles)
                {
                    triangles.Add(triangle + vertexOffset);
                }
                vertexOffset += gameObject.Mesh.Vertices.Length;
                texs.Add(gameObject.Texture ?? DEFAULT_TEXTURE);
                trianglesPerTexture.Add(gameObject.Mesh.Triangles.Length);
            }
        }

        private void OrderTrianglesByAverageDistance(List<Vector3> vertices, List<int> triangles, List<Vector2> uvs, List<Texture> texs)
        {
            // Ensure all lists have the same number of elements
            //if (vertices.Count != triangles.Count / 3 || vertices.Count != uvs.Count || vertices.Count != texs.Count)
            //{
            //    // Handle mismatched input sizes (throw an exception, log an error, etc.)
            //    throw new ArgumentException("Input lists must have the same number of elements.");
            //}

            // Calculate average distance for each triangle and store it in a dictionary
            Dictionary<int, float> triangleDistances = new Dictionary<int, float>();

            for (int i = 0; i < triangles.Count; i += 3)
            {
                // Calculate average distance for the current triangle
                float averageDistance = (vertices[triangles[i]].Length() + vertices[triangles[i + 1]].Length() + vertices[triangles[i + 2]].Length()) / 3f;

                // Store the average distance in the dictionary with the triangle index as the key
                triangleDistances[i / 3] = averageDistance;
            }

            // Order triangles based on average distance in ascending order
            List<int> orderedIndices = triangleDistances.OrderBy(pair => pair.Value).Select(pair => pair.Key * 3).ToList();

            // Reorder vertices, triangles, uvs, and texs based on the calculated order
            vertices = orderedIndices.SelectMany(index => vertices.GetRange(index, 3)).ToList();
            triangles = orderedIndices.ToList();
            uvs = orderedIndices.SelectMany(index => uvs.GetRange(index / 3, 3)).ToList();
            texs = orderedIndices.Select(index => texs[index / 3]).ToList();
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

        public void Initialize(Texture[] textures)
        {
            throw new NotImplementedException();
        }
    }
}