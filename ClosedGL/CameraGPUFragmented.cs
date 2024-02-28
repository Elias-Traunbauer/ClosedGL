using ILGPU;
using ILGPU.Runtime;
using ILGPU.Algorithms;
using System.Collections.Concurrent;
using System.Drawing;
using System.Runtime.InteropServices;
using VRageMath;
using System;

namespace ClosedGL
{
    /// <summary>
    /// Class to project points from a position onto an lcd
    /// NOTE: Only works if the ViewPoint is infront of the lcd -> Transparent LCDS from the back dont work
    /// </summary>
    public class CameraGPUFragmented : GameObject, IRenderer
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

        private readonly Context context;
        private readonly Device device;
        private readonly Accelerator accelerator;
        private Action<Index1D /*index*/,
            ArrayView<Vec2> /*uvs*/,
            ArrayView<byte> /*textures*/,
            ArrayView<int> /*textureLengths*/,
            ArrayView<int> /*textureWidths*/,
            ArrayView<int> /*textureHeights*/,
            ArrayView<int> /*trianglesPerTexture*/,
            ArrayView<Vec3> /*projectedVertices*/,
            ArrayView<int> /*uvIndices*/,
            ArrayView<byte> /*projectedVertexSet*/,
            ArrayView<float> /*depthBuffer*/,
            ArrayView<byte> /*frame*/,
            int /*fragmentWidth*/,
            int /*projectedVerticesIndex*/,
            ArrayView<Vec3> /*preBakedVectors*/ > fragmentKernel;

        private Action<Index1D /*index*/,
                       ArrayView<Vec3> /*vertices*/,
                       ArrayView<int> /*triangles*/,
                       ArrayView<Vec3> /*projectedVertices*/,
                       ArrayView<int>, /*uvIndices*/
                       ArrayView<byte> /*projectedVertexSet*/,
                       ArrayView<Vec3> /*preBakedVectors*/> projectionKernel;

        public CameraGPUFragmented()
        {
            context = Context.CreateDefault();
            device = context.Devices.First(x => x.AcceleratorType == AcceleratorType.Cuda);
            accelerator = ((Device)device).CreateAccelerator(context);
        }

        ~CameraGPUFragmented()
        {
            accelerator.Dispose();
            context.Dispose();
        }

        public void Initialize(Texture[] textures)
        {
            //Index1D index,
            //ArrayView< Vec3 > vertices,
            //ArrayView<int> triangles,
            //ArrayView< Vec2 > uvs,
            //ArrayView<byte> textures,
            //ArrayView< int > textureLengths,
            //ArrayView<int> textureWidths,
            //ArrayView< int > textureHeights,
            //ArrayView<int> trianglesPerTexture,
            //ArrayView< float > depthBuffer,
            //ArrayView<byte> frame,
            //ArrayView< Vec3 > preBakedVectors) // view point (vpx, vpy, vpz)

            fragmentKernel = accelerator.LoadAutoGroupedStreamKernel<
                Index1D /*index*/,
            ArrayView<Vec2> /*uvs*/,
            ArrayView< byte > /*textures*/,
            ArrayView<int> /*textureLengths*/,
            ArrayView< int > /*textureWidths*/,
            ArrayView<int> /*textureHeights*/,
            ArrayView< int > /*trianglesPerTexture*/,
            ArrayView<Vec3> /*projectedVertices*/,
            ArrayView< int > /*uvIndices*/,
            ArrayView<byte> /*projectedVertexSet*/,
            ArrayView< float > /*depthBuffer*/,
            ArrayView<byte> /*frame*/,
            int /*fragmentWidth*/,
            int /*projectedVerticesIndex*/,
            ArrayView< Vec3 > /*preBakedVectors*/ > (FragmentKernel);

            projectionKernel = accelerator.LoadAutoGroupedStreamKernel<
                               Index1D /*index*/,
                               ArrayView<Vec3> /*vertices*/,
                               ArrayView<int> /*triangles*/,
                               ArrayView<Vec3> /*projectedVertices*/,
                               ArrayView<int>, /*uvIndices*/
                               ArrayView<byte> /*projectedVertexSet*/,
                               ArrayView<Vec3> /*preBakedVectors*/>(ProjectionKernel);

            // allocate memory for textures
            int totalTextureLength = textures.Sum(x => x.Data.Length);

            textureMemory = accelerator.Allocate1D<byte>(totalTextureLength);
            frameMemory = accelerator.Allocate1D<byte>((int)RenderResolution.X * (int)RenderResolution.Y * 4);
            depthBufferMemory = accelerator.Allocate1D<float>((int)RenderResolution.X * (int)RenderResolution.Y);
            preBakedVectorsMemory = accelerator.Allocate1D<Vec3>(2);
            textureLengthsMemory = accelerator.Allocate1D<int>(textures.Length);
            textureWidthsMemory = accelerator.Allocate1D<int>(textures.Length);
            textureHeightsMemory = accelerator.Allocate1D<int>(textures.Length);
            trianglesPerTextureMemory = accelerator.Allocate1D<int>(textures.Length);
            projectedVerticesMemory = accelerator.Allocate1D<Vec3>((int)RenderResolution.X * (int)RenderResolution.Y);
            uvIndicesMemory = accelerator.Allocate1D<int>((int)RenderResolution.X * (int)RenderResolution.Y);
            projectedVertexSetMemory = accelerator.Allocate1D<byte>((int)RenderResolution.X * (int)RenderResolution.Y);

            textureMemory.CopyFromCPU(textures.SelectMany(x => x.Data).ToArray());
            textureLengthsMemory.CopyFromCPU(textures.Select(x => x.Data.Length).ToArray());
            textureWidthsMemory.CopyFromCPU(textures.Select(x => x.Width).ToArray());
            textureHeightsMemory.CopyFromCPU(textures.Select(x => x.Height).ToArray());
            trianglesPerTextureMemory.CopyFromCPU(textures.Select(x => x.Data.Length / 4).ToArray());

            UpdatePreBakedVectors();
        }

        void UpdatePreBakedVectors()
        {
            Vector3 viewPointCPU = Vector3.Backward * Rotation;
            Vec3 viewPoint = new(viewPointCPU.X, viewPointCPU.Y, viewPointCPU.Z);
            Vec3 renderResolution = new(RenderResolution.X, RenderResolution.Y, 0);

            preBakedVectorsMemory.CopyFromCPU([viewPoint, renderResolution]);
        }

        MemoryBuffer1D<byte, Stride1D.Dense> textureMemory;
        MemoryBuffer1D<byte, Stride1D.Dense> frameMemory;
        MemoryBuffer1D<float, Stride1D.Dense> depthBufferMemory;
        MemoryBuffer1D<Vec3, Stride1D.Dense> preBakedVectorsMemory;
        MemoryBuffer1D<int, Stride1D.Dense> textureLengthsMemory;
        MemoryBuffer1D<int, Stride1D.Dense> textureWidthsMemory;
        MemoryBuffer1D<int, Stride1D.Dense> textureHeightsMemory;
        MemoryBuffer1D<int, Stride1D.Dense> trianglesPerTextureMemory;
        MemoryBuffer1D<Vec3, Stride1D.Dense> projectedVerticesMemory;
        MemoryBuffer1D<int, Stride1D.Dense> uvIndicesMemory;
        MemoryBuffer1D<byte, Stride1D.Dense> projectedVertexSetMemory;

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
            if (swapChain.Count > 2)
            {
                swapChain.TryDequeue(out _);
            }

            PrepareRendering(gameObjects,
                out List<Vector3> vertices,
                out List<int> triangles,
                out List<Vector2> uvs,
                out List<Texture> texs,
                out List<int> trianglesPerTexture);

            UpdatePreBakedVectors();

            frameMemory.MemSetToZero();
            depthBufferMemory.MemSetToZero();
            projectedVerticesMemory.MemSetToZero();
            projectedVertexSetMemory.MemSetToZero();
            projectedVertexSetMemory.MemSetToZero();
            var verticesMemory = accelerator.Allocate1D<Vec3>(vertices.Count);
            verticesMemory.CopyFromCPU(vertices.Select(x => new Vec3(x.X, x.Y, x.Z)).ToArray());
            var trianglesMemory = accelerator.Allocate1D<int>(triangles.Count);
            trianglesMemory.CopyFromCPU(triangles.ToArray());
            var uvsMemory = accelerator.Allocate1D<Vec2>(uvs.Count);
            uvsMemory.CopyFromCPU(uvs.Select(x => new Vec2(x.X, x.Y)).ToArray());

            projectionKernel(
                triangles.Count / 3,
                               verticesMemory.View,
                                              trianglesMemory.View,
                                                             projectedVerticesMemory.View,
                                                                            uvIndicesMemory.View,
                                                                                           projectedVertexSetMemory.View,
                                                                                                          preBakedVectorsMemory.View
                                                                                                                     );

            byte[] projectedVertexSet = new byte[projectedVertexSetMemory.Length];
            projectedVertexSetMemory.CopyToCPU(projectedVertexSet);
            Vec3[] projectedVertices = new Vec3[projectedVerticesMemory.Length];
            projectedVerticesMemory.CopyToCPU(projectedVertices);

            for (int i = 0; i < projectedVertexSet.Length; i++)
            {
                if (projectedVertexSet[i] == 0)
                {
                    continue;
                }
                Vec3 v1 = projectedVertices[i];
                Vec3 v2 = projectedVertices[i + 1];
                Vec3 v3 = projectedVertices[i + 2];

                // calculate min and max 
                Vec2 vec1 = new(v1.x, v1.y);
                Vec2 vec2 = new(v2.x, v2.y);
                Vec2 vec3 = new(v3.x, v3.y);

                Vec2 min = Vec2.Min(Vec2.Min(vec1, vec2), vec3);
                Vec2 max = Vec2.Max(Vec2.Max(vec1, vec2), vec3);

                // clamp the min and max to the render resolution
                min.x = XMath.Max(0, XMath.Min(RenderResolution.X - 1, min.x));
                min.y = XMath.Max(0, XMath.Min(RenderResolution.Y - 1, min.y));
                max.x = XMath.Max(0, XMath.Min(RenderResolution.X - 1, max.x));
                max.y = XMath.Max(0, XMath.Min(RenderResolution.Y - 1, max.y));

                // get the fragment width
                int fragmentWidth = (int)(max.x - min.x);
                int fragmentHeight = (int)(max.y - min.y);

                // fragment index
                int fragmentIndex = i;

                fragmentKernel(
                        fragmentWidth * fragmentHeight, // amount of times this kernel should be called
                        uvsMemory.View,
                        textureMemory.View,
                        textureLengthsMemory.View,
                        textureWidthsMemory.View,
                        textureHeightsMemory.View,
                        trianglesPerTextureMemory.View,
                        projectedVerticesMemory.View,
                        uvIndicesMemory.View,
                        projectedVertexSetMemory.View,
                        depthBufferMemory.View,
                        frameMemory.View,
                        fragmentWidth,
                        i,
                        preBakedVectorsMemory.View
                   );
            }

            unsafe
            {
                int bytesPerPixel = 4;
                byte[] image = new byte[(int)RenderResolution.X * (int)RenderResolution.Y * bytesPerPixel];

                frameMemory.CopyToCPU(image);

                swapChain.Enqueue(image);
            }

            verticesMemory.Dispose();
            trianglesMemory.Dispose();
            uvsMemory.Dispose();
            return true;
        }
        const int bytesPerPixel = 4;
        public static void FragmentKernel(
            Index1D index,
            ArrayView<Vec2> uvs,
            ArrayView<byte> textures,
            ArrayView<int> textureLengths,
            ArrayView<int> textureWidths,
            ArrayView<int> textureHeights,
            ArrayView<int> trianglesPerTexture,
            ArrayView<Vec3> projectedVertices,
            ArrayView<int> uvIndices,
            ArrayView<byte> projectedVertexSet,
            ArrayView<float> depthBuffer,
            ArrayView<byte> frame,
            int fragmentWidth,
            int projectedVerticesIndex,
            ArrayView<Vec3> preBakedVectors) // view point (vpx, vpy, vpz)
        {
            Vec2 res = new(preBakedVectors[1].x, preBakedVectors[1].y);

            int stride = (int)res.x * bytesPerPixel;

            int x = index % fragmentWidth;
            int y = index / fragmentWidth;

            Vec3 p1 = projectedVertices[projectedVerticesIndex];
            Vec3 p2 = projectedVertices[projectedVerticesIndex + 1];
            Vec3 p3 = projectedVertices[projectedVerticesIndex + 2];

            bool enoughUVs = uvs.Length > uvIndices[projectedVerticesIndex + 2];

            Vec2 uv1 = enoughUVs ? uvs[uvIndices[projectedVerticesIndex]] : new Vec2(0, 0);
            Vec2 uv2 = enoughUVs ? uvs[uvIndices[projectedVerticesIndex + 1]] : new Vec2(1, 0);
            Vec2 uv3 = enoughUVs ? uvs[uvIndices[projectedVerticesIndex + 2]] : new Vec2(0, 1);

            int currentTextureIndex = 0;
            int currentTriangleCounter = 0;

            int textureIndex = 0;

            //for (int i = 0; i < index / 3; i++)
            //{
            //    currentTriangleCounter += 3;

            //    if (currentTriangleCounter >= trianglesPerTexture[currentTextureIndex])
            //    {
            //        currentTriangleCounter = 0;
            //        currentTextureIndex++;
            //        textureIndex += textureLengths[currentTextureIndex];
            //    }
            //}
            // we only use the first texture for now

            int textureWidth = textureWidths[currentTextureIndex] * bytesPerPixel;
            int textureHeight = textureHeights[currentTextureIndex];

            Vec2 vec1 = new(p1.x, p1.y);
            Vec2 vec2 = new(p2.x, p2.y);
            Vec2 vec3 = new(p3.x, p3.y);

            //Vec2 min = Vec2.Min(Vec2.Min(vec1, vec2), vec3);
            //Vec2 max = Vec2.Max(Vec2.Max(vec1, vec2), vec3);

            //// clamp the min and max to the render resolution
            //min.x = XMath.Max(0, XMath.Min(res.x - 1, min.x));
            //min.y = XMath.Max(0, XMath.Min(res.y - 1, min.y));
            //max.x = XMath.Max(0, XMath.Min(res.x - 1, max.x));
            //max.y = XMath.Max(0, XMath.Min(res.y - 1, max.y));

            Vec2 point = new(x, y);
            if (IsPointInTriangleKernel(point, vec1, vec2, vec3)
                /*&& IsPointInRenderView(point)*/)
            {
                Vec3 barycentricCoords = CalculateBarycentricCoordinatesKernel(point, vec1, vec2, vec3);
                Vec2 interpolatedUV = InterpolateUVKernel(barycentricCoords, uv1, uv2, uv3);

                // Depth test
                float depth = -barycentricCoords.x * p1.z + -barycentricCoords.y * p2.z + -barycentricCoords.z * p3.z;

                int depthIndex = x + (int)res.x * y;
                if (depth < depthBuffer[depthIndex] && false)
                {
                    return;
                }
                // Pixel is closer, update depth buffer and render pixel
                depthBuffer[depthIndex] = depth;

                // sample the texture
                int textureX = (int)(interpolatedUV.x * textureWidth);
                int textureY = (int)(interpolatedUV.y * textureHeight);

                // clamp the texture coordinates
                textureX = Math.Max(0, Math.Min(textureWidth - 1, textureX));
                textureY = Math.Max(0, Math.Min(textureHeight - 1, textureY));

                int texturePixelIndex = textureIndex + (textureX * bytesPerPixel) + (textureY * textureWidth);

                int yCoord = (int)res.y - y - 1;

                //byte* currentLine = ptr + (yCoord * stride);

                // Set the pixel color in the bitmap
                //currentLine[x * bytesPerPixel + 0] = pixel[0];
                //currentLine[x * bytesPerPixel + 1] = pixel[1];
                //currentLine[x * bytesPerPixel + 2] = pixel[2];
                //currentLine[x * bytesPerPixel + 3] = pixel[3];
                frame[x * bytesPerPixel + yCoord * stride + 0] = textures[texturePixelIndex];
                frame[x * bytesPerPixel + yCoord * stride + 1] = textures[texturePixelIndex + 1];
                frame[x * bytesPerPixel + yCoord * stride + 2] = textures[texturePixelIndex + 2];
                frame[x * bytesPerPixel + yCoord * stride + 3] = textures[texturePixelIndex + 3];
            }

            // draw light purple debug lines between the points 199,67,117

            //DrawLine(p1NonNullable, p2NonNullable, /*purle BGRA*/[117, 67, 199, 255], image, stride, ptr);
            //DrawLine(p2NonNullable, p3NonNullable, /*purle BGRA*/[117, 67, 199, 255], image, stride, ptr);
            //DrawLine(p3NonNullable, p1NonNullable, /*purle BGRA*/[117, 67, 199, 255], image, stride, ptr);

            //// draw the square that got clamped
            //DrawLine(min, new(min.X, max.Y), /*red BGRA*/[0, 0, 255, 255], image, stride, ptr);
            //DrawLine(new(min.X, max.Y), max, /*red BGRA*/[0, 0, 255, 255], image, stride, ptr);
            //DrawLine(max, new(max.X, min.Y), /*red BGRA*/[0, 0, 255, 255], image, stride, ptr);
            //DrawLine(new(max.X, min.Y), min, /*red BGRA*/[0, 0, 255, 255], image, stride, ptr);
        }

        public static void ProjectionKernel(
            Index1D index,
            ArrayView<Vec3> vertices,
            ArrayView<int> triangles,
            ArrayView<Vec3> projectedVertices,
            ArrayView<int> uvIndices,
            ArrayView<byte> projectedVertexSet,
            ArrayView<Vec3> preBakedVectors) // view point (vpx, vpy, vpz)
        {
            Vec2 res = new Vec2(preBakedVectors[1].x, preBakedVectors[1].y);

            int stride = (int)res.x * bytesPerPixel;

            int triangleIndex = index * 3;
            Vec3 v1 = vertices[triangles[triangleIndex]];
            Vec3 v2 = vertices[triangles[triangleIndex + 1]];
            Vec3 v3 = vertices[triangles[triangleIndex + 2]];

            var (successp1, p1) = ProjectPointLocalKernel(v1, preBakedVectors[0], res);
            var (successp2, p2) = ProjectPointLocalKernel(v2, preBakedVectors[0], res);
            var (successp3, p3) = ProjectPointLocalKernel(v3, preBakedVectors[0], res);

            //for (int i = 0; i < index / 3; i++)
            //{
            //    currentTriangleCounter += 3;

            //    if (currentTriangleCounter >= trianglesPerTexture[currentTextureIndex])
            //    {
            //        currentTriangleCounter = 0;
            //        currentTextureIndex++;
            //        textureIndex += textureLengths[currentTextureIndex];
            //    }
            //}
            // we only use the first texture for now

            if (successp1 && successp2 && successp3)
            {
                var p1NonNullable = p1;
                var p2NonNullable = p2;
                var p3NonNullable = p3;

                //DrawPoint(p1NonNullable, 1, stride, ptr);
                //DrawPoint(p2NonNullable, 1, stride, ptr);
                //DrawPoint(p3NonNullable, 1, stride, ptr);

                var p1Distance = p1NonNullable.z;
                var p2Distance = p2NonNullable.z;
                var p3Distance = p3NonNullable.z;

                // check if the 2d points are clockwise
                // if not, skip the triangle

                // Only render if points are clockwise
                var p1p2 = p2NonNullable - p1NonNullable;
                var p1p3 = p3NonNullable - p1NonNullable;

                var cross = Vec3.Cross(new Vec3(p1p2.x, p1p2.y, 0), new Vec3(p1p3.x, p1p3.y, 0));

                if (cross.z < 0)
                {
                    return;
                }

                //if (IsTriangleOutOfBounds(p1NonNullable, p2NonNullable, p3NonNullable))
                //{
                //    continue;  // Skip the entire triangle if it's out of bounds
                //}

                Vec2 vec1 = new Vec2(p1NonNullable.x, p1NonNullable.y);
                Vec2 vec2 = new Vec2(p2NonNullable.x, p2NonNullable.y);
                Vec2 vec3 = new Vec2(p3NonNullable.x, p3NonNullable.y);

                // calulate center of the triangle
                Vec2 center = (vec1 + vec2 + vec3) / 3f;

                int projectedVerticesIndex = (int)center.x + (int)res.x * (int)center.y;

                projectedVertices[projectedVerticesIndex] = p1;
                projectedVertices[projectedVerticesIndex + 1] = p2;
                projectedVertices[projectedVerticesIndex + 2] = p3;

                uvIndices[projectedVerticesIndex] = triangles[triangleIndex];
                uvIndices[projectedVerticesIndex + 1] = triangles[triangleIndex + 1];
                uvIndices[projectedVerticesIndex + 2] = triangles[triangleIndex + 2];

                projectedVertexSet[projectedVerticesIndex] = 1;

                // store it into the projected vertices

                // draw light purple debug lines between the points 199,67,117

                //DrawLine(p1NonNullable, p2NonNullable, /*purle BGRA*/[117, 67, 199, 255], image, stride, ptr);
                //DrawLine(p2NonNullable, p3NonNullable, /*purle BGRA*/[117, 67, 199, 255], image, stride, ptr);
                //DrawLine(p3NonNullable, p1NonNullable, /*purle BGRA*/[117, 67, 199, 255], image, stride, ptr);

                //// draw the square that got clamped
                //DrawLine(min, new(min.X, max.Y), /*red BGRA*/[0, 0, 255, 255], image, stride, ptr);
                //DrawLine(new(min.X, max.Y), max, /*red BGRA*/[0, 0, 255, 255], image, stride, ptr);
                //DrawLine(max, new(max.X, min.Y), /*red BGRA*/[0, 0, 255, 255], image, stride, ptr);
                //DrawLine(new(max.X, min.Y), min, /*red BGRA*/[0, 0, 255, 255], image, stride, ptr);
            }
        }

        public static (bool, Vec3) ProjectPointLocalKernel(Vec3 v, Vec3 vp, Vec2 res)
        {
            Vec3 Backward = new Vec3(0, 0, 1);
            Vec3 localRayDirection = v - vp;

            localRayDirection.Normalize();

            // Project the point onto the plane
            var (success, projectedLocalPoint) = PlaneIntersectionKernel(Backward, localRayDirection);

            if (!success)
            {
                return (false, new());
            }

            var projectedLocalPointNonNullable = projectedLocalPoint;

            // Convert it to pixels
            Vec3 projectedPixelPoint = new Vec3(
                projectedLocalPointNonNullable.x * res.x / 2f + res.x / 2f,
                projectedLocalPointNonNullable.y * res.y / 2f + res.y / 2f,
                // distance to the vertex
                v.length()
            );

            return (true, projectedPixelPoint);
        }

        private static (bool, Vec2) PlaneIntersectionKernel(/*Vector3D origin, Vector3D dir*/
            Vec3 origin, Vec3 dir
            )
        {
            Vec3 Normal = new Vec3(0, 0, -1);
            // Check if the line is pointing away from the plane
            if (dir.z >= 0)
            {
                return (false, new());
            }

            // Ensure the denominator is not zero to avoid division by zero
            var denominator = Vec3.Dot(dir, Normal);
            if (denominator == 0)
            {
                return (false, new());
            }

            // Calculate the parameter t for the line equation
            float t = -(Vec3.Dot(origin, Normal) + 0) / denominator;

            // Calculate the intersection point in 3D
            Vec3 intersectionPoint = origin + dir * t;

            // Return the projected point in 2D (X and Y)
            return (true, new Vec2(intersectionPoint.x, intersectionPoint.y));
        }

        private unsafe void DrawPoint(Vector2 p1NonNullable, float radius, int stride, byte* ptr)
        {
            int x0 = (int)p1NonNullable.X;
            int y0 = (int)p1NonNullable.Y;

            for (int x = x0 - (int)radius; x <= x0 + (int)radius; x++)
            {
                for (int y = y0 - (int)radius; y <= y0 + (int)radius; y++)
                {
                    if (x >= 0 && x < RenderResolution.X && y >= 0 && y < RenderResolution.Y)
                    {
                        byte* currentLine = ptr + (y * stride);
                        currentLine[x * 4 + 0] = 255;
                        currentLine[x * 4 + 1] = 255;
                        currentLine[x * 4 + 2] = 255;
                        currentLine[x * 4 + 3] = 255;
                    }
                }
            }
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

        private unsafe void DrawLine(Vector2 p1NonNullable, Vector2 p2NonNullable, byte[] value, byte[] image, int stride, byte* ptr)
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
                    byte* currentLine = ptr + (y0 * stride);
                    currentLine[x0 * 4 + 0] = value[0];
                    currentLine[x0 * 4 + 1] = value[1];
                    currentLine[x0 * 4 + 2] = value[2];
                    currentLine[x0 * 4 + 3] = value[3];
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

        public static Vector3 CalculateBarycentricCoordinates(Vector2 point, Vector2 a, Vector2 b, Vector2 c)
        {
            float detT = (b.Y - c.Y) * (a.X - c.X) + (c.X - b.X) * (a.Y - c.Y);
            float wA = ((b.Y - c.Y) * (point.X - c.X) + (c.X - b.X) * (point.Y - c.Y)) / detT;
            float wB = ((c.Y - a.Y) * (point.X - c.X) + (a.X - c.X) * (point.Y - c.Y)) / detT;
            float wC = 1 - wA - wB;

            return new Vector3(wA, wB, wC);
        }

        public static Vec3 CalculateBarycentricCoordinatesKernel(Vec2 point, Vec2 a, Vec2 b, Vec2 c)
        {
            float detT = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
            float wA = ((b.y - c.y) * (point.x - c.x) + (c.x - b.x) * (point.y - c.y)) / detT;
            float wB = ((c.y - a.y) * (point.x - c.x) + (a.x - c.x) * (point.y - c.y)) / detT;
            float wC = 1 - wA - wB;

            return new Vec3(wA, wB, wC);
        }

        public static Vector2 InterpolateUV(Vector3 barycentricCoords, Vector2 uvA, Vector2 uvB, Vector2 uvC)
        {
            float interpolatedU = barycentricCoords.X * uvA.X + barycentricCoords.Y * uvB.X + barycentricCoords.Z * uvC.X;
            float interpolatedV = barycentricCoords.X * uvA.Y + barycentricCoords.Y * uvB.Y + barycentricCoords.Z * uvC.Y;

            return new Vector2(interpolatedU, interpolatedV);
        }

        public static Vec2 InterpolateUVKernel(Vec3 barycentricCoords, Vec2 uvA, Vec2 uvB, Vec2 uvC)
        {
            float interpolatedU = barycentricCoords.x * uvA.x + barycentricCoords.y * uvB.x + barycentricCoords.z * uvC.x;
            float interpolatedV = barycentricCoords.x * uvA.y + barycentricCoords.y * uvB.y + barycentricCoords.z * uvC.y;

            return new Vec2(interpolatedU, interpolatedV);
        }

        public static bool IsPointInTriangle(Vector2 p, Vector2 a, Vector2 b, Vector2 c)
        {
            float detT = (b.Y - c.Y) * (a.X - c.X) + (c.X - b.X) * (a.Y - c.Y);
            float wA = ((b.Y - c.Y) * (p.X - c.X) + (c.X - b.X) * (p.Y - c.Y)) / detT;
            float wB = ((c.Y - a.Y) * (p.X - c.X) + (a.X - c.X) * (p.Y - c.Y)) / detT;
            float wC = 1 - wA - wB;

            return wA >= 0 && wB >= 0 && wC >= 0;
        }

        public static bool IsPointInTriangleKernel(Vec2 p, Vec2 a, Vec2 b, Vec2 c)
        {
            float detT = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
            float wA = ((b.y - c.y) * (p.x - c.x) + (c.x - b.x) * (p.y - c.y)) / detT;
            float wB = ((c.y - a.y) * (p.x - c.x) + (a.x - c.x) * (p.y - c.y)) / detT;
            float wC = 1 - wA - wB;

            return wA >= 0 && wB >= 0 && wC >= 0;
        }

    }
}