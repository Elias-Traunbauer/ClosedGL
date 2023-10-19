using ClosedGL;
using VRageMath;

namespace RenderTest
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            GameObject go = new GameObject();
            Camera camera = new Camera();

            camera.Position = new Vector3(0, 1, -1);

            Vector3 projectionPoint = new Vector3(0, 1, 1);

            var result = camera.ProjectPoint(projectionPoint);

            MessageBox.Show(result?.ToString());
        }
    }
}