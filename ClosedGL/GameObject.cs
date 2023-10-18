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
        public Vector3D Position { get; set; }
        public Vector3D Rotation { get; set; }
        public Vector3D Scale { get; set; }

        public MatrixD WorldMatrix
        {
            get
            {
                var worldUp = Vector3D.Up * Rotation;
                var worldForward = Vector3D.Forward * Rotation;
                var worldMatrix = MatrixD.CreateWorld(Position, worldForward, worldUp);
                return worldMatrix;
            }
        }
        public GameObject() { }
    }
}
