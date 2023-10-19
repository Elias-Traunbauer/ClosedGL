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

            camera.FieldOfView = 3f;
            camera.Position = new Vector3(0, 1, 1);

            var result = camera.ProjectPoint(go.Position);

            MessageBox.Show(result?.ToString());
        }
    }
}