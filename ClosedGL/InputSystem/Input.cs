using Gma.System.MouseKeyHook;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace ClosedGL.InputSystem
{
    public class Input
    {
        private static Input? instance;
        public static Input Instance
        {
            get
            {
                instance ??= new Input();
                return instance;
            }
        }

        HashSet<Keys> keysDown = new HashSet<Keys>();
        HashSet<MouseButtons> mouseButtonsDown = new HashSet<MouseButtons>();
        float mouseWheel = 0;
        float mouseX = 0;
        float mouseY = 0;
        float deltaX = 0;
        float deltaY = 0;

        private IKeyboardMouseEvents m_GlobalHook;

        public Input()
        {
            //keyboardHook = new KeyboardHook();
            //mouseHook = new MouseHook();

            m_GlobalHook = Hook.GlobalEvents();

            m_GlobalHook.KeyDown += KeyDown;
            m_GlobalHook.KeyUp += KeyUp;
            m_GlobalHook.MouseDown += MouseDown;
            m_GlobalHook.MouseUp += MouseUp;
            m_GlobalHook.MouseWheel += MouseWheel;
            m_GlobalHook.MouseMove += MouseMove;

            //keyboardHook.Install();
            //mouseHook.Install();
        }

        private void KeyUp(object? sender, KeyEventArgs e)
        {
            keysDown.Remove(e.KeyCode);
        }

        private void KeyDown(object? sender, KeyEventArgs e)
        {
            keysDown.Add(e.KeyCode);
        }

        private void MouseDown(object? sender, MouseEventArgs e)
        {
            mouseButtonsDown.Add(e.Button);
        }

        private void MouseUp(object? sender, MouseEventArgs e)
        {
            mouseButtonsDown.Remove(e.Button);
        }

        private void MouseMove(object? sender, MouseEventArgs e)
        {
            
        }

        private void MouseWheel(object? sender, MouseEventArgs e)
        {
            mouseWheel += e.Delta;
        }

        //private void MouseHook_MouseWheel(MouseHook.MSLLHOOKSTRUCT mouseStruct)
        //{
        //    mouseWheel += mouseStruct.mouseData;
        //}

        //private void MouseHook_MouseMove(MouseHook.MSLLHOOKSTRUCT mouseStruct)
        //{
        //    mouseX = mouseStruct.pt.x;
        //    mouseY = mouseStruct.pt.y;
        //}

        //private void MouseHook_RightButtonUp(MouseHook.MSLLHOOKSTRUCT mouseStruct)
        //{
        //    mouseButtonsDown.Remove(mouseStruct);
        //}

        //private void MouseHook_RightButtonDown(MouseHook.MSLLHOOKSTRUCT mouseStruct)
        //{
        //    mouseButtonsDown.Add(mouseStruct);
        //}

        //private void MouseHook_LeftButtonUp(MouseHook.MSLLHOOKSTRUCT mouseStruct)
        //{
        //    mouseButtonsDown.Remove(mouseStruct);
        //}

        //private void MouseHook_LeftButtonDown(MouseHook.MSLLHOOKSTRUCT mouseStruct)
        //{
        //    mouseButtonsDown.Add(mouseStruct);
        //}

        //private void KeyboardHook_KeyUp(KeyboardHook.VKeys key)
        //{
        //    keysDown.Remove(key);
        //}

        //private void KeyboardHook_KeyDown(KeyboardHook.VKeys key)
        //{
        //    keysDown.Add(key);
        //}

        public static bool IsKeyDown(Keys key)
        {
            return Instance.keysDown.Contains(key);
        }

        public static bool IsMouseButtonDown(int button)
        {
            return Instance.mouseButtonsDown.Contains((MouseButtons)button);
        }

        public static void Update()
        {
            var mouse = System.Windows.Forms.Control.MousePosition;

            var centerX = System.Windows.Forms.Screen.PrimaryScreen.Bounds.Width / 2;
            var centerY = System.Windows.Forms.Screen.PrimaryScreen.Bounds.Height / 2;

            Instance.deltaX = mouse.X - centerX;
            Instance.deltaY = mouse.Y - centerY;

            System.Windows.Forms.Cursor.Position = new System.Drawing.Point(centerX, centerY);
        }

        public static float GetMouseWheel()
        {
            var wheel = Instance.mouseWheel;
            Instance.mouseWheel = 0;
            return wheel;
        }

        public static float GetMouseX()
        {
            var x = Instance.mouseX;
            Instance.mouseX = 0;
            return x;
        }

        public static float GetMouseY()
        {
            var y = Instance.mouseY;
            Instance.mouseY = 0;
            return y;
        }

        public static float GetMouseDeltaX()
        {
            var x = Instance.deltaX;

            return x;
        }

        public static float GetMouseDeltaY()
        {
            var y = Instance.deltaY;
            return y;
        }
    }
}
