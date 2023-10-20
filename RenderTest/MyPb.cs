using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RenderTest
{
    internal class MyPb : PictureBox
    {
        public MyPb()
        {
            Dock = DockStyle.Fill;
        }

        protected override void OnPaint(System.Windows.Forms.PaintEventArgs e)
        {
            lock (this)
            {
                base.OnPaint(e);
            }
        }
    }
}
