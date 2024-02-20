using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRageMath;

namespace ClosedGL
{
    public class GameObject
    {
        public Vector3 Position { get; set; } = Vector3.Zero;
        public Quaternion Rotation { get; set; } = Quaternion.Identity;
        public Vector3 Scale { get; set; } = Vector3.One;
        public Texture? Texture { get; set; } = null;

        public virtual Mesh? Mesh { get; set; }

        public Matrix WorldMatrix
        {
            get
            {
                var rotationMatrix = Matrix.CreateFromQuaternion(Rotation);
                var worldMatrix = Matrix.CreateWorld(Position, rotationMatrix.Forward, rotationMatrix.Up);
                return worldMatrix;
            }
        }

        public Vector3 TransformToWorld(Vector3 localVector)
        {
            // Use the world matrix to transform the local vector to world space
            return Vector3.Transform(localVector, WorldMatrix);
        }

        public Vector3 TransformToLocal(Vector3 worldVector)
        {
            // Use the inverse of the world matrix to transform the world vector to local space
            Matrix worldMatrixInverse = Matrix.Invert(WorldMatrix);
            return Vector3.Transform(worldVector, worldMatrixInverse);
        }

        public Vector3 TransformToLocalBeta(Vector3 worldVector)
        {
            var worldDirection = worldVector - Position;

            // Calculate length and handle zero length case
            double directionLength = worldDirection.Length();
            if (directionLength > double.Epsilon)
            {
                // Normalize only if the length is not close to zero
                Vector3D worldDirectionNormalized = worldDirection / directionLength;

                // Transform world direction to local direction
                Vector3D localDir = Vector3D.TransformNormal(worldDirectionNormalized, MatrixD.Transpose(this.WorldMatrix));

                return localDir * directionLength;
            }
            else
            {
                // Handle zero-length vector case
                return Vector3.Zero;
            }
        }

        public GameObject() { }
    }
}
