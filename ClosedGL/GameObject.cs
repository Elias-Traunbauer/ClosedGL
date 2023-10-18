using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ClosedGL
{
    public class GameObject
    {
        public Vector3D Position { get; set; }
        public Vector3D Rotation { get; set; }
        public Vector3D Scale { get; set; }
        public GameObject() { }
    }
}
