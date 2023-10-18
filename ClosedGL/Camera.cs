using System.Numerics;

namespace ClosedGL
{
    /// <summary>
    /// Class to project points from a position onto an lcd
    /// NOTE: Only works if the ViewPoint is infront of the lcd -> Transparent LCDS from the back dont work
    /// </summary>
    public class Camera : GameObject
    {
        private readonly Vector3D Normal = Vector3D.Backward;

        public Camera()
        {
        }

        /// <summary>
        /// Projects the given point onto LCD screen coordinates given in pixels
        /// </summary>
        /// <param name="worldPoint">The point to project</param>
        /// <returns>Screen coordinate in pixels or null if projection is not on lcd</returns>
        public Vector2? ProjectPoint(Vector3D worldPoint)
        {
            Vector3D referenceWorldPosition = TextPanel.WorldMatrix.Translation;
            // Convert worldPosition into a world direction
            Vector3D worldDirection = worldPoint - referenceWorldPosition;
            // Convert worldDirection into a local direction
            Vector3D localPointToProject = Vector3D.TransformNormal(worldDirection, MatrixD.Transpose(TextPanel.WorldMatrix));
            // ray direction in local space
            Vector3D localRayDirection = localPointToProject - ViewPoint;
            //// we dont normalize to keep it at max performance
            //localRayDirection.Normalize();

            // project the plane onto the plane
            Vector2? projectedLocalPoint = PlaneIntersection(ViewPoint, localRayDirection);
            if (projectedLocalPoint != null)
            {
                var projectedLocalPointNonNullable = (Vector2)projectedLocalPoint;
                // convert it to pixels
                Vector2 projectedLocalPointPixels = projectedLocalPointNonNullable * PixelMultiplier * new Vector2(1, -1);
                projectedLocalPointPixels += TextPanel.TextureSize / 2f;
                if (projectedLocalPointPixels.X >= 0 && projectedLocalPointPixels.Y >= 0 && projectedLocalPointPixels.X < TextPanel.SurfaceSize.X && projectedLocalPointPixels.Y < TextPanel.SurfaceSize.Y)
                {
                    return projectedLocalPointPixels;
                }
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
            if (dir.Z >= 0)
            {
                return null;
            }
            var t = -(Vector3D.Dot(origin, Normal) + D) / Vector3D.Dot(dir, Normal);
            Vector3D res = origin + t * dir;
            return new Vector2((float)res.X, (float)res.Y);
        }

        Vector3D LocalDirToWorldDir(Vector3D dir, MatrixD matrix)
        {
            return Vector3D.TransformNormal(dir, matrix);
        }

        Vector3D LocalPosToWorldPos(Vector3D pos, MatrixD matrix)
        {
            return Vector3D.Transform(pos, matrix);
        }

    }
}