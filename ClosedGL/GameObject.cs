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
        public Vector3 Position { get; set; }
        public Quaternion Rotation { get; set; }
        public Vector3 Scale { get; set; } = Vector3.One;

        public virtual Mesh? Mesh { get; set; }

        public Matrix WorldMatrix
        {
            get
            {
                var rotationMatrix = Matrix.CreateFromQuaternion(Rotation);
                var worldUp = Vector3D.Rotate(Vector3D.Up, rotationMatrix);
                var worldForward = Vector3D.Rotate(Vector3D.Forward, rotationMatrix);
                var worldMatrix = Matrix.CreateWorld(Position, worldForward, worldUp);
                return worldMatrix;
            }
        }
        public GameObject() { }
    }
}
