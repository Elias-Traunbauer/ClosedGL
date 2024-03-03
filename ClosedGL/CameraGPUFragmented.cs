using ILGPU;
using ILGPU.Runtime;
using ILGPU.Algorithms;
using System.Collections.Concurrent;
using System.Drawing;
using System.Runtime.InteropServices;
using VRageMath;
using System;
using ILGPU.IR.Values;
using ILGPU.Backends;

namespace ClosedGL
{
    /// <summary>
    /// Class to project points from a position onto an lcd
    /// NOTE: Only works if the ViewPoint is infront of the lcd -> Transparent LCDS from the back dont work
    /// </summary>
    public class CameraGPUFragmented : GameObject, IRenderer
    {
        const int TILE_SIZE = 8;
        const int TILE_SIZE_SQUARED = TILE_SIZE * TILE_SIZE;
        const int TRIANGLE_BUFFER_PER_TILE = 10000;

        #region boring
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
        #endregion

        #region fields
        private readonly Vector3D Normal = Vector3D.Forward;

        private readonly Context context;
        private readonly Device device;
        private readonly Accelerator accelerator;

        // projection kernel
        private Action<
            Index1D           /*index*/,
            VariableView<long> /*projectedVertexIndex*/,
            ArrayView<Vec3>   /*vertices*/,
            ArrayView<int>    /*triangles*/,
            ArrayView<Vec3>   /*projectedVertices*/,
            ArrayView<int>    /*uvIndices*/,
            ArrayView<Vec3>   /*preBakedVectors*/> projectionKernel;

        // triangle bin kernel
        private Action<
            Index1D /*index*/,
            ArrayView<Vec3> /*projectedVertices*/,
            VariableView<long> /*projectedVertexIndex*/,
            ArrayView<int> /*tileTriangleIndices*/,
            ArrayView<int> /*tileTriangleCounts*/,
            ArrayView<Vec3> /*preBakedVectors*/> triangleBinningKernel;

        // tile kernel
        private Action<
            Index1D /*index*/,
            ArrayView<byte> /*frame*/,
            ArrayView<float> /*depthBuffer*/,
            ArrayView<Vec2> /*uvs*/,

            ArrayView<byte> /*textures*/,
            ArrayView<int> /*textureLengths*/,
            ArrayView<int> /*textureWidths*/,
            ArrayView<int> /*textureHeights*/,
            ArrayView<int> /*trianglesPerTexture*/,

            ArrayView<Vec3> /*projectedVertices*/,
            ArrayView<int> /*uvIndices*/,
            ArrayView<int> /*tileTriangleIndices*/,
            ArrayView<int> /*tileTriangleCounts*/,
            ArrayView<Vec3> /*preBakedVectors*/> tileKernel;


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
        MemoryBuffer1D<int, Stride1D.Dense> tileTriangleIndices;
        MemoryBuffer1D<int, Stride1D.Dense> tileTriangleCounts;
        #endregion

        #region constructor/deconstructor
        public CameraGPUFragmented()
        {
            bool debug = false;
            context = Context.CreateDefault();
            device = context.Devices.First(x => debug ? x.AcceleratorType == AcceleratorType.CPU : x.AcceleratorType == AcceleratorType.Cuda);
            accelerator = device.CreateAccelerator(context);
        }

        ~CameraGPUFragmented()
        {
            accelerator.Dispose();
            context.Dispose();

            textureMemory.Dispose();
            frameMemory.Dispose();
            depthBufferMemory.Dispose();
            preBakedVectorsMemory.Dispose();
            textureLengthsMemory.Dispose();
            textureWidthsMemory.Dispose();
            textureHeightsMemory.Dispose();
            trianglesPerTextureMemory.Dispose();
            projectedVerticesMemory.Dispose();
            uvIndicesMemory.Dispose();
            tileTriangleIndices.Dispose();
            tileTriangleCounts.Dispose();
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

            projectionKernel = accelerator.LoadAutoGroupedStreamKernel<Index1D /*index*/,
            VariableView<long> /*projectedVertexIndex*/,
            ArrayView<Vec3> /*vertices*/,
            ArrayView<int> /*triangles*/,
            ArrayView<Vec3> /*projectedVertices*/,
            ArrayView<int> /*uvIndices*/,
            ArrayView<Vec3> /*preBakedVectors*/>(ProjectionKernel);

            triangleBinningKernel = accelerator.LoadAutoGroupedStreamKernel<
                Index1D /*index*/,
                ArrayView<Vec3> /*projectedVertices*/,
                VariableView<long> /*projectedVertexIndex*/,
                ArrayView<int> /*tileTriangleIndices*/,
                ArrayView<int> /*tileTriangleCounts*/,
                ArrayView<Vec3> /*preBakedVectors*/>(TriangleBinningKernel);

            tileKernel = accelerator.LoadAutoGroupedStreamKernel<
                Index1D         /*index*/,
                ArrayView<byte> /*frame*/,
                ArrayView<float> /*depthBuffer*/,
                ArrayView<Vec2> /*uvs*/,

                ArrayView<byte> /*textures*/,
                ArrayView<int>  /*textureLengths*/,
                ArrayView<int>  /*textureWidths*/,
                ArrayView<int>  /*textureHeights*/,
                ArrayView<int>  /*trianglesPerTexture*/,

                ArrayView<Vec3> /*projectedVertices*/,
                ArrayView<int>  /*uvIndices*/,
                ArrayView<int>  /*tileTriangleIndices*/,
                ArrayView<int>  /*tileTriangleCounts*/,
                ArrayView<Vec3> /*preBakedVectors*/>(TileKernel);

            // allocate memory for textures
            int totalTextureLength = textures.Sum(x => x.Data.Length);

            int tileCount = (int)RenderResolution.X / TILE_SIZE * (int)RenderResolution.Y / TILE_SIZE;

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
            tileTriangleIndices = accelerator.Allocate1D<int>(tileCount * TRIANGLE_BUFFER_PER_TILE);
            tileTriangleCounts = accelerator.Allocate1D<int>(tileCount);

            textureMemory.CopyFromCPU(textures.SelectMany(x => x.Data).ToArray());
            textureLengthsMemory.CopyFromCPU(textures.Select(x => x.Data.Length).ToArray());
            textureWidthsMemory.CopyFromCPU(textures.Select(x => x.Width).ToArray());
            textureHeightsMemory.CopyFromCPU(textures.Select(x => x.Height).ToArray());
            trianglesPerTextureMemory.CopyFromCPU(textures.Select(x => x.Data.Length / 4).ToArray());

            UpdatePreBakedVectors();
        }
        #endregion

        void UpdatePreBakedVectors()
        {
            Vector3 viewPointCPU = Vector3.Backward * Rotation;
            Vec3 viewPoint = new(viewPointCPU.X, viewPointCPU.Y, viewPointCPU.Z);
            Vec3 renderResolution = new(RenderResolution.X, RenderResolution.Y, 0);

            preBakedVectorsMemory.CopyFromCPU([viewPoint, renderResolution]);
        }

        #region helper/algorithms
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
        #endregion

        public Dictionary<string, object> Render(List<GameObject> gameObjects)
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

            if (vertices.Count == 0)
            {
                swapChain.Enqueue([]);
                return new();
            }

            UpdatePreBakedVectors();

            frameMemory.MemSetToZero();
            depthBufferMemory.CopyFromCPU(Enumerable.Repeat<float>(float.MaxValue, (int)depthBufferMemory.Length).ToArray());
            projectedVerticesMemory.MemSetToZero();
            tileTriangleCounts.MemSetToZero();
            tileTriangleIndices.MemSetToZero();

            var verticesMemory = accelerator.Allocate1D<Vec3>(vertices.Count);
            verticesMemory.CopyFromCPU(vertices.Select(x => new Vec3(x.X, x.Y, x.Z)).ToArray());

            var trianglesMemory = accelerator.Allocate1D<int>(triangles.Count);
            trianglesMemory.CopyFromCPU([.. triangles]);

            var uvsMemory = accelerator.Allocate1D<Vec2>(uvs.Count);
            uvsMemory.CopyFromCPU(uvs.Select(x => new Vec2(x.X, x.Y)).ToArray());

            var counter = accelerator.Allocate1D<long>(1);
            counter.MemSetToZero();
            var counterView = counter.View.VariableView(0);

            projectionKernel(                        
                vertices.Count / 3,                /*index*/
                counterView,                       /*projectedVertexIndex*/  
                verticesMemory.View,               /*vertices*/  
                trianglesMemory.View,              /*triangles*/  
                projectedVerticesMemory.View,      /*projectedVertices*/  
                uvIndicesMemory.View,              /*uvIndices*/  
                preBakedVectorsMemory.View         /*preBakedVectors*/
            );

            int tileCount = (int)RenderResolution.X / TILE_SIZE * (int)RenderResolution.Y / TILE_SIZE;

            triangleBinningKernel(
                tileCount,                          /*index*/
                projectedVerticesMemory.View,      /*projectedVertices*/
                counterView,                       /*projectedVertexIndex*/
                tileTriangleIndices.View,          /*tileTriangleIndices*/
                tileTriangleCounts.View,           /*tileTriangleCounts*/
                preBakedVectorsMemory.View         /*preBakedVectors*/
            );

            tileKernel(
                tileCount,                       /*index*/
                frameMemory.View,                  /*frame*/
                depthBufferMemory.View,            /*depthBuffer*/
                uvsMemory.View,                    /*uvs*/
                textureMemory.View,                /*textures*/
                textureLengthsMemory.View,         /*textureLengths*/
                textureWidthsMemory.View,          /*textureWidths*/
                textureHeightsMemory.View,         /*textureHeights*/
                trianglesPerTextureMemory.View,    /*trianglesPerTexture*/
                projectedVerticesMemory.View,      /*projectedVertices*/
                uvIndicesMemory.View,              /*uvIndices*/
                tileTriangleIndices.View,          /*tileTriangleIndices*/
                tileTriangleCounts.View,           /*tileTriangleCounts*/
                preBakedVectorsMemory.View         /*preBakedVectors*/
                         );        

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
            counter.Dispose();
            return [];
        }
        const int bytesPerPixel = 4;

        public static void ProjectionKernel(
            Index1D index,
            VariableView<long> projectedVertexIndex,
            ArrayView<Vec3> vertices,
            ArrayView<int> triangles,
            ArrayView<Vec3> projectedVertices,
            ArrayView<int> uvIndices,
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

                var p1Distance = p1NonNullable.z;
                var p2Distance = p2NonNullable.z;
                var p3Distance = p3NonNullable.z;

                // Only render if points are clockwise
                var p1p2 = p2NonNullable - p1NonNullable;
                var p1p3 = p3NonNullable - p1NonNullable;

                var cross = Vec3.Cross(new Vec3(p1p2.x, p1p2.y, 0), new Vec3(p1p3.x, p1p3.y, 0));

                if (cross.z < 0)
                {
                    return;
                }

                Vec2 vec1 = new Vec2(p1NonNullable.x, p1NonNullable.y);
                Vec2 vec2 = new Vec2(p2NonNullable.x, p2NonNullable.y);
                Vec2 vec3 = new Vec2(p3NonNullable.x, p3NonNullable.y);

                long projectedVerticesIndex = Atomic.Add(ref projectedVertexIndex.Value, 3);

                projectedVertices[projectedVerticesIndex] = p1;
                projectedVertices[projectedVerticesIndex + 1] = p2;
                projectedVertices[projectedVerticesIndex + 2] = p3;

                uvIndices[projectedVerticesIndex] = triangles[triangleIndex];
                uvIndices[projectedVerticesIndex + 1] = triangles[triangleIndex + 1];
                uvIndices[projectedVerticesIndex + 2] = triangles[triangleIndex + 2];
            }
        }

        public static void TriangleBinningKernel(
            Index1D index,
            ArrayView<Vec3> projectedVertices,
            VariableView<long> projectedVertexIndex,
            ArrayView<int> tileTriangleIndices,
            ArrayView<int> tileTriangleCounts,
            ArrayView<Vec3> preBakedVectors
            )
        {
            Vec2 res = new Vec2(preBakedVectors[1].x, preBakedVectors[1].y);

            int tileIndex = index;

            int tilesPerRow = (int)res.x / TILE_SIZE;

            int tileX = tileIndex % tilesPerRow;
            int tileY = tileIndex / tilesPerRow;

            int tileStartX = tileX * TILE_SIZE;
            int tileStartY = tileY * TILE_SIZE;

            int tileEndX = tileStartX + TILE_SIZE;
            int tileEndY = tileStartY + TILE_SIZE;

            // now check all triangles if they are in the tile

            long projectedVerticesCount = projectedVertexIndex.Value;

            for (int i = 0; i < projectedVerticesCount; i+=3)
            {
                int triangleIndex = i;

                Vec3 v1 = projectedVertices[triangleIndex];
                Vec3 v2 = projectedVertices[triangleIndex + 1];
                Vec3 v3 = projectedVertices[triangleIndex + 2];

                Vec2 vec1 = new(v1.x, v1.y);
                Vec2 vec2 = new(v2.x, v2.y);
                Vec2 vec3 = new(v3.x, v3.y);

                Vec2 min = Vec2.Min(Vec2.Min(vec1, vec2), vec3);
                Vec2 max = Vec2.Max(Vec2.Max(vec1, vec2), vec3);

                //if (IsTriangleOutOfBounds(min, max))
                //{
                //    continue;
                //}

                if (DoesOverlap(min, max, tileStartX, tileStartY, tileEndX, tileEndY))
                {
                    // register the triangle in the tile
                    int tileTriangleIndex = tileTriangleCounts[tileIndex];
                    tileTriangleIndices[tileIndex * TRIANGLE_BUFFER_PER_TILE + tileTriangleIndex] = i;
                    tileTriangleCounts[tileIndex]++;
                }

                if (tileTriangleCounts[tileIndex] >= TRIANGLE_BUFFER_PER_TILE)
                {
                    // buffer overflow
                    return;
                }
            }
        }

        static bool DoesOverlap(Vec2 min, Vec2 max, int tileStartX, int tileStartY, int tileEndX, int tileEndY)
        {
            // Check if AABB is to the right of the tile
            if (min.x > tileEndX) return false;

            // Check if AABB is to the left of the tile
            if (max.x < tileStartX) return false;

            // Check if AABB is above the tile
            if (min.y > tileEndY) return false;

            // Check if AABB is below the tile
            if (max.y < tileStartY) return false;

            // If none of the conditions are met, then there's an overlap
            return true;
        }

        public static void TileKernel(
            Index1D index,
            ArrayView<byte> frame,
            ArrayView<float> depthBuffer,
            ArrayView<Vec2> uvs,

            ArrayView<byte> textures,
            ArrayView<int> textureLengths,
            ArrayView<int> textureWidths,
            ArrayView<int> textureHeights,
            ArrayView<int> trianglesPerTexture,

            ArrayView<Vec3> projectedVertices,
            ArrayView<int> uvIndices,
            ArrayView<int> tileTriangleIndices,
            ArrayView<int> tileTriangleCounts,
            ArrayView<Vec3> preBakedVectors) // view point (vpx, vpy, vpz), resolution (resx, resy)
        {
            Vec2 res = new Vec2(preBakedVectors[1].x, preBakedVectors[1].y);

            int tileIndex = index;

            int tilesPerRow = (int)res.x / TILE_SIZE;

            int tileX = tileIndex % tilesPerRow;
            int tileY = tileIndex / tilesPerRow;

            int tileStartX = tileX * TILE_SIZE;
            int tileStartY = tileY * TILE_SIZE;

            int tileEndX = tileStartX + TILE_SIZE;
            int tileEndY = tileStartY + TILE_SIZE;

            int stride = (int)res.x * bytesPerPixel;

            int projectedTrianglesInThisTileCount = tileTriangleCounts[tileIndex];

            if (projectedTrianglesInThisTileCount == 0)
            {
                return;
            }

            int projectedVerticesStartIndex = tileIndex * TRIANGLE_BUFFER_PER_TILE;

            for (int y = tileStartY; y < tileEndY; y++)
            {
                for (int x = tileStartX; x < tileEndX; x++)
                {
                    // check every triangle for this pixel
                    int yCoord = (int)res.y - y - 1;
                    int pixelIndex = yCoord * (int)res.x + x;

                    if (pixelIndex >= res.x * res.y
                        || pixelIndex < 0)
                    {
                        continue;
                    
                    }

                    Vec2 pixel = new Vec2(x, y);

                    for (int i = 0; i < projectedTrianglesInThisTileCount; i++)
                    {
                        int triangleIndex = tileTriangleIndices[projectedVerticesStartIndex + i];

                        Vec3 v1 = projectedVertices[triangleIndex];
                        Vec3 v2 = projectedVertices[triangleIndex + 1];
                        Vec3 v3 = projectedVertices[triangleIndex + 2];

                        Vec2 vec1 = new(v1.x, v1.y);
                        Vec2 vec2 = new(v2.x, v2.y);
                        Vec2 vec3 = new(v3.x, v3.y);

                        Vec2 min = Vec2.Min(Vec2.Min(vec1, vec2), vec3);
                        Vec2 max = Vec2.Max(Vec2.Max(vec1, vec2), vec3);

                        if(IsPointInTriangleKernel(pixel, vec1, vec2, vec3))
                        {
                            // calculate the barycentric coordinates
                            Vec3 barycentric = CalculateBarycentricCoordinatesKernel(pixel, vec1, vec2, vec3);

                            bool enoughUVs = uvIndices[triangleIndex] < uvs.Length && uvIndices[triangleIndex + 1] < uvs.Length && uvIndices[triangleIndex + 2] < uvs.Length;

                            Vec2 uv1 = enoughUVs ? uvs[uvIndices[triangleIndex]] : new Vec2(0, 0);
                            Vec2 uv2 = enoughUVs ? uvs[uvIndices[triangleIndex + 1]] : new Vec2(1, 0);
                            Vec2 uv3 = enoughUVs ? uvs[uvIndices[triangleIndex + 2]] : new Vec2(0, 1);

                            // Depth test
                            float depth = barycentric.x * v1.z + barycentric.y * v1.z + barycentric.z * v1.z;
                            
                            int depthIndex = x + (int)res.x * y;
                            if (depth > depthBuffer[depthIndex])
                            {
                                continue;
                            }
                            // Pixel is closer, update depth buffer and render pixel
                            depthBuffer[depthIndex] = depth;

                            Vec2 uv = InterpolateUVKernel(barycentric, uv1, uv2, uv3);

                            int textureIndex = 0;
                            int currentTextureIndex = 0;

                            int currentTriangleCounter = 0;

                            //for (int t = 0; t < triangleIndex / 3; t++)
                            //{
                            //    currentTriangleCounter += 3;

                            //    if (currentTriangleCounter >= trianglesPerTexture[textureIndex])
                            //    {
                            //        currentTriangleCounter = 0;
                            //        currentTextureIndex += textureLengths[textureIndex];
                            //        textureIndex++;
                            //    }
                            //}

                            int textureLength = textureLengths[textureIndex];
                            int textureWidth = textureWidths[textureIndex];
                            int textureHeight = textureHeights[textureIndex];

                            int textureX = (int)(uv.x * textureWidth);
                            int textureY = (int)(uv.y * textureHeight);

                            // clamp the texture coordinates
                            textureX = Math.Max(0, Math.Min(textureX, textureWidth - 2));
                            textureY = Math.Max(0, Math.Min(textureY, textureHeight - 2));

                            int texturePixelIndex = textureY * textureWidth + textureX;

                            int textureArrayIndex = currentTextureIndex + texturePixelIndex * bytesPerPixel;

                            int frameIndex = pixelIndex * bytesPerPixel;

                            if (textureArrayIndex + 4 < textures.Length && frameIndex + 4 < textures.Length)
                            {
                                frame[frameIndex] = textures[textureArrayIndex];
                                frame[frameIndex + 1] = textures[textureArrayIndex + 1];
                                frame[frameIndex + 2] = textures[textureArrayIndex + 2];
                                frame[frameIndex + 3] = textures[textureArrayIndex + 3];
                            }
                        }
                    }
                }
            }
        }

        #region kernelmethods
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
            foreach (var gameObject in gameObjects.Where(x => x.Mesh != null).OrderBy(x => Vector3.Distance(Position, x.Position)))
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
        #endregion
    }
}