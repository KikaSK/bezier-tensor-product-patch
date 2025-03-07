﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Forms;
using System.Windows.Forms.Integration;
using System.Windows.Media.Media3D;

// pouzitie potrebnych kniznic OpenTK
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;


namespace MKP2___Template
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        /////////////////////////////////////////////////////////
        //                                                     //
        //                   GLOBALNE PREMENNE                 //
        //                                                     //
        /////////////////////////////////////////////////////////

        GLControl glControl;

        // Our patches are stored in the list "Patches"
        List<Patch> Patches = new List<Patch>();
        
        // camera settings
        double Dist = new double(), Phi = new double(), Theta = new double(), oPhi = new double(), oTheta = new double(), prevPhi = new double(), prevTheta = new double(), prevDist = new double();

        // number of samples of the patch
        int nSamples;
        // nubmer of control vertices in the direction m and n, respectively
        int nDegM, nDegN;  //
    
        bool? DisplayIsoCurves; // do we want to display isocurves?
        double hlS, hlT; // parameters S and T for the isocurves

        // mouse settings
        double RightX, RightY;
        bool IsLeftDown, IsRightDown;
        int ActivePoint, ActivePatch;

        // keyboard settings
        bool IsZ = true, IsY = false, IsX = false;

        bool DrawCoordinateAxes;


        //-----------------------------------------------------------------------------------------------------------------------

        public MainWindow()
        {
            InitializeComponent();

            IsLeftDown = false;
            IsRightDown = false;

            // set value to true if you want to draw coordinate axes, false if not
            // x - red, y - green, z - blue
            DrawCoordinateAxes = false;

            // initialize the parameters
            InitializeParams(type.TENSOR, Convert.ToInt32(Mbox.Text), Convert.ToInt32(Nbox.Text), Convert.ToInt32(Ubox.Text), DisplayIso.IsChecked, SliderS.Value, SliderT.Value);
        }

        // initialization of paramters, when the application is launched or the patch is erased 
        private void InitializeParams(type TypeOfPatch, int _nDegM, int _nDegN, int _nSamples, bool? _DisplayIsoCurves, double _hlS, double _hlT)
        {
            Patches.Clear();

            nSamples = _nSamples;
            nDegM = _nDegM;
            nDegN = _nDegN;
            DisplayIsoCurves = _DisplayIsoCurves;
            hlS = _hlS;
            hlT = _hlT;

            // Defining the patch  
            float[] color = { 0.42f, 0.5f, 0.286f }; // RGB color of the patch
            Patches.Add(new Patch(TypeOfPatch, nDegM, nDegN, nSamples, DisplayIsoCurves, hlS, hlT, color, placement.MIDDLE));
        }

        private void Sphere_Checked(object sender, RoutedEventArgs e)
        {
            // --------------- !!! TODO !!! -------------------
            // define the bidegree nDegM, nDegN for the sphere    
            // ------------------------------------------------

            InitializeParams(type.SPHERE, nDegM, nDegN, nSamples, DisplayIsoCurves, hlS, hlT);
            glControl.Invalidate();
        }

        private void Cylinder_Checked(object sender, RoutedEventArgs e)
        {
            // --------------- !!! TODO !!! -------------------
            // define the bidegree nDegM, nDegN for the cylinder 
            // ------------------------------------------------

            InitializeParams(type.CYLINDER, nDegM, nDegN, nSamples, DisplayIsoCurves, hlS, hlT);
            glControl.Invalidate();
        }

        private void Cone_Checked(object sender, RoutedEventArgs e)
        {
            // --------------- !!! TODO !!! -------------------
            // define the bidegree nDegM, nDegN for the cone 
            // ------------------------------------------------

            InitializeParams(type.CONE, nDegM, nDegN, nSamples, DisplayIsoCurves, hlS, hlT);
            glControl.Invalidate();
        }


        private void SliderS_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            hlS = SliderS.Value;
            LabelValueS.Content = "s = " + Convert.ToString(Math.Round(hlS, 3));

            if (SliderT != null)
            {
                SliderT.Value = hlT;
                LabelValueT.Content = "t = " + Convert.ToString(Math.Round(hlT, 3));
            }

            Patches[0].hlS = hlS;
            Patches[0].hlT = hlT;
            glControl.Invalidate();
        }

        private void SliderT_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            hlT = SliderT.Value;
            LabelValueT.Content = "t = " + Convert.ToString(Math.Round(hlT, 3));

            if (SliderS != null)
            {
                SliderS.Value = hlS;
                LabelValueS.Content = "s = " + Convert.ToString(Math.Round(hlS, 3));
            }

            Patches[0].hlS = hlS;
            Patches[0].hlT = hlT;
            glControl.Invalidate();
        }

        private void Mplus_Click(object sender, RoutedEventArgs e)
        {
            nDegM++;
            Mbox.Text = Convert.ToString(nDegM);

            InitializeParams(type.TENSOR, nDegM, nDegN, nSamples, DisplayIsoCurves, hlS, hlT);
            // redraw the scene
            glControl.Invalidate();
        }


        private void Mminus_Click(object sender, RoutedEventArgs e)
        {
            if (nDegM > 0) nDegM--;
            Mbox.Text = Convert.ToString(nDegM);

            InitializeParams(type.TENSOR, nDegM, nDegN, nSamples, DisplayIsoCurves, hlS, hlT);
            // redraw the scene
            glControl.Invalidate();
        }

        private void Nplus_Click(object sender, RoutedEventArgs e)
        {
            nDegN++;
            Nbox.Text = Convert.ToString(nDegN);

            InitializeParams(type.TENSOR, nDegM, nDegN, nSamples, DisplayIsoCurves, hlS, hlT);
            // redraw the scene
            glControl.Invalidate();
        }


        private void Nminus_Click(object sender, RoutedEventArgs e)
        {
            if (nDegN > 0) nDegN--;
            Nbox.Text = Convert.ToString(nDegN);

            InitializeParams(type.TENSOR, nDegM, nDegN, nSamples, DisplayIsoCurves, hlS, hlT);
            // redraw the scene
            glControl.Invalidate();
        }


        private void Uminus_Click(object sender, RoutedEventArgs e)
        {
            if (nSamples > 0) nSamples--;
            Ubox.Text = Convert.ToString(nSamples);
            InitializeParams(Patches[0].TypeOfPatch, nDegM, nDegN, nSamples, DisplayIsoCurves, hlS, hlT);
            glControl.Invalidate();
        }

        private void Uplus_Click(object sender, RoutedEventArgs e)
        {
            nSamples++;
            Ubox.Text = Convert.ToString(nSamples);
            InitializeParams(Patches[0].TypeOfPatch, nDegM, nDegN, nSamples, DisplayIsoCurves, hlS, hlT);
            glControl.Invalidate();
        }

        

        private void DisplayIso_Checked(object sender, RoutedEventArgs e)
        {
            SliderS.IsEnabled = true;
            LabelValueS.IsEnabled = true;
            SliderT.IsEnabled = true;
            LabelValueT.IsEnabled = true;

            DisplayIsoCurves = true;
            Patches[0].DisplayIsoCurves = DisplayIsoCurves;

            glControl.Invalidate();
        }

        private void DisplayIso_Unchecked(object sender, RoutedEventArgs e)
        {
            SliderS.IsEnabled = false;
            LabelValueS.IsEnabled = false;
            SliderT.IsEnabled = false;
            LabelValueT.IsEnabled = false;


            DisplayIsoCurves = false;
            Patches[0].DisplayIsoCurves = DisplayIsoCurves;

            glControl.Invalidate();
        }


        private void Tensor_Checked(object sender, RoutedEventArgs e)
        {
            nDegM = Convert.ToInt32(Mbox.Text);
            nDegN = Convert.ToInt32(Nbox.Text);
            InitializeParams(type.TENSOR, nDegM, nDegN, nSamples, DisplayIsoCurves, hlS, hlT);
            glControl.Invalidate();
        }









        //-----------------------------------------------------------------------------------------------------------------------

        /////////////////////////////////////////////////////////
        //                                                     //
        //                  DRAWING PROCEDURES                 //
        //                                                     //
        /////////////////////////////////////////////////////////





        //-----------------------------------------------------------------------------------------------------------------------

        // draw the coordinate axes
        private void DrawAxes()
        {
            GL.Begin(PrimitiveType.Lines);
            GL.Color3(1.0f, 0.0f, 0.0f);
            GL.Vertex3(0.0f, 0.0f, 0.0f);
            GL.Color3(1.0f, 0.0f, 0.0f);
            GL.Vertex3(2.0f, 0.0f, 0.0f);

            GL.Color3(0.0f, 1.0f, 0.0f);
            GL.Vertex3(0.0f, 0.0f, 0.0f);
            GL.Color3(0.0f, 1.0f, 0.0f);
            GL.Vertex3(0.0f, 2.0f, 0.0f);

            GL.Color3(0.0f, 0.0f, 1.0f);
            GL.Vertex3(0.0f, 0.0f, 0.0f);
            GL.Color3(0.0f, 0.0f, 1.0f);
            GL.Vertex3(0.0f, 0.0f, 2.0f);
            GL.End();
        }

        //-----------------------------------------------------------------------------------------------------------------------
        //                                                      DRAWING
        //-----------------------------------------------------------------------------------------------------------------------

        // drawing the patch
        private void DrawPatch(Patch _patch)
        {
            // drawing triangles / quadrilaterals 
            GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Fill); // enabble filling of shapes with color 

            // color -- !!! TODO !!! -- edit if you want something different :-) 
            float[] diffuse = { 0.9f, 0.9f, 0.9f, 1.0f };
            float[] specular = { 0.5f, 0.5f, 0.5f, 1.0f };


            GL.Material(MaterialFace.FrontAndBack, MaterialParameter.Diffuse, diffuse);
            GL.Material(MaterialFace.FrontAndBack, MaterialParameter.Specular, specular);

            GL.Material(MaterialFace.FrontAndBack, MaterialParameter.Shininess, 0.1f);

            PrimitiveType prim = new PrimitiveType();
            if (_patch.TypeOfPatch == type.TENSOR || _patch.TypeOfPatch == type.SPHERE || _patch.TypeOfPatch == type.CYLINDER || _patch.TypeOfPatch == type.CONE) prim = PrimitiveType.Quads;


            GL.Begin(prim); // !!! TODO !!! -- when drawing triangles, use "PrimitiveType.Triangles"
            for (int i = 0; i < _patch.Sampling.Indices.Count; i++)
            {

                GL.Material(MaterialFace.FrontAndBack, MaterialParameter.Ambient, _patch.Color);

                GL.Normal3(_patch.Sampling.Normals[_patch.Sampling.Indices[i]]); // !!! TODO !!! -- compute the coordinates of the unit normal vector for the point "PatchCoords[PatchIndices[i]]" and replace the current values
                GL.Vertex3(_patch.Sampling.Coordinates[_patch.Sampling.Indices[i]]);
            }
            GL.End();

            // drawing the wireframe model
            GL.Translate(0.0f, 0.0f, 0.01f);
            GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Line);

            float[] black = { 0.0f, 0.0f, 0.0f, 1.0f };
            GL.Material(MaterialFace.FrontAndBack, MaterialParameter.Specular, black);
            GL.Material(MaterialFace.FrontAndBack, MaterialParameter.Ambient, black);

            GL.LineWidth(0.5f);

            GL.Begin(prim); // !!! TODO !!! -- when drawing triangles, use "PrimitiveType.Triangles"
            for (int i = 0; i < _patch.Sampling.Indices.Count; i++)
                GL.Vertex3(_patch.Sampling.Coordinates[_patch.Sampling.Indices[i]]);
            GL.End();
        }

        //-----------------------------------------------------------------------------------------------------------------------

        // drawing the ControlNet
        private void DrawNet(Patch _patch)
        {
            // firstly, draw the wireframe of the control net
            GL.Translate(0.0f, 0.0f, 0.01f);
            GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Line); // zabezpeci vykreslenie drotoveho modelu

            GL.LineWidth(2.0f);
            GL.Color3(0.529f, 0.904f, 0.971f); // color of the wireframe net

            PrimitiveType prim = new PrimitiveType();
            if (_patch.TypeOfPatch == type.TENSOR || _patch.TypeOfPatch == type.SPHERE || _patch.TypeOfPatch == type.CYLINDER || _patch.TypeOfPatch == type.CONE) prim = PrimitiveType.Quads;
            

            GL.Begin(prim);
            for (int i = 0; i < _patch.ControlNet.Indices.Count; i++)
                GL.Vertex3(_patch.ControlNet.Coordinates[_patch.ControlNet.Indices[i]]);
            GL.End();


        }

        //-----------------------------------------------------------------------------------------------------------------------
        // drawing of the points of the patch
        private void DrawPoints(Patch _patch)
        {
            if (_patch.TypeOfPatch == type.TENSOR)
            {
                GL.PointSize(6.0f);
                GL.Color3(0.490f, 0.116f, 0.116f); // color of the control points

                GL.Begin(PrimitiveType.Points);
                for (int i = 0; i < _patch.InterpolatedPoints.Coordinates.Count; i++)
                    GL.Vertex3(_patch.InterpolatedPoints.Coordinates[i]);
                GL.End();
            }
        }

        //-----------------------------------------------------------------------------------------------------------------------

        //drawing the isocurves 
        private void DrawIsoLines(Patch _patch)
        {
            GL.LineWidth(2.0f);
            GL.Translate(0.0f, 0.0f, 0.001f);

            GL.Begin(PrimitiveType.Lines);


            //control polygon of the S-curve
            GL.Color3(1.0f, 0.145f, 0.5f); // color
                                           //if (_patch.TypeOfPatch == type.TENSOR)
            for (int i = 0; i < _patch.IsoS_ControlPolygon.Count - 1; i++)
            {
                GL.Vertex3(_patch.IsoS_ControlPolygon[i]);
                GL.Vertex3(_patch.IsoS_ControlPolygon[i + 1]);
            }

            // points of the S-curve
            GL.Color3(1.0f, 0.666f, 0.8f); // color
            for (int i = 0; i < _patch.IsoS_Sampling.Count - 1; i++)
            {
                GL.Vertex3(_patch.IsoS_Sampling[i]);
                GL.Vertex3(_patch.IsoS_Sampling[i + 1]);
            }

            //control polygon of the T-curve
            GL.Color3(0.7f, 0.831f, 0.165f); // color
                                             //if (_patch.TypeOfPatch == type.TENSOR)
            for (int i = 0; i < _patch.IsoT_ControlPolygon.Count - 1; i++)
            {
                GL.Vertex3(_patch.IsoT_ControlPolygon[i]);
                GL.Vertex3(_patch.IsoT_ControlPolygon[i + 1]);
            }

            // points of the T-curve
            GL.Color3(1.0f, 0.931f, 0.265f); // color
            for (int i = 0; i < _patch.IsoT_Sampling.Count - 1; i++)
            {
                GL.Vertex3(_patch.IsoT_Sampling[i]);
                GL.Vertex3(_patch.IsoT_Sampling[i + 1]);
            }

            GL.End();
        }

        //-----------------------------------------------------------------------------------------------------------------------

        // drawing 
        private void GLControl_Paint(object sender, PaintEventArgs e)
        {
            // Modelview matrix
            GL.MatrixMode(MatrixMode.Modelview);
            GL.LoadIdentity();
            Matrix4 matLook = Matrix4.LookAt((float)(Dist * Math.Cos(Theta) * Math.Cos(Phi)), (float)(Dist * Math.Sin(Phi) * Math.Cos(Theta)), (float)(Dist * Math.Sin(Theta)), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
            GL.LoadMatrix(ref matLook);

            // perspective projection
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadIdentity();
            Matrix4 matPers = Matrix4.CreatePerspectiveFieldOfView(0.785f, (float)glControl.Width / (float)glControl.Height, 0.1f, 10.5f);
            GL.LoadMatrix(ref matPers);

            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            // Check the compatibility of the two patches before recomputing the patch

            //if (C0.IsChecked == true) CheckCompatibility(0);
            //if (C1.IsChecked == true) CheckCompatibility(1);

            if (DrawCoordinateAxes) DrawAxes();

            for (int i = 0; i < Patches.Count; i++)
                Patches[i].RecomputePatch();

            GL.Enable(EnableCap.Lighting);
            GL.Enable(EnableCap.DepthTest);
            for (int i = 0; i < Patches.Count; i++)
                DrawPatch(Patches[i]);
            GL.Disable(EnableCap.Lighting);
            for (int i = 0; i < Patches.Count; i++)
            {
                //if (Patches[i].TypeOfPatch == type.TENSOR)
                DrawNet(Patches[i]);
                DrawPoints(Patches[i]);
                if (Patches[i].DisplayIsoCurves == true)
                {

                    DrawIsoLines(Patches[i]);
                }
            }

            // the buffers need to swapped, so the scene is drawn
            glControl.SwapBuffers();
        }

        //-----------------------------------------------------------------------------------------------------------------------

        // initialization of the window, where OpenTK drawing is used 
        private void WindowsFormsHost_Initialized(object sender, EventArgs e)
        {
            // Inicializacia OpenTK;
            OpenTK.Toolkit.Init();
            var flags = GraphicsContextFlags.Default;
            glControl = new GLControl(new GraphicsMode(32, 24), 2, 0, flags);
            glControl.MakeCurrent();
            glControl.Paint += GLControl_Paint;
            glControl.Dock = DockStyle.Fill;
            (sender as WindowsFormsHost).Child = glControl;

            // user controls
            glControl.MouseDown += GLControl_MouseDown;
            glControl.MouseMove += GLControl_MouseMove;
            glControl.MouseUp += GLControl_MouseUp;
            glControl.MouseWheel += GLControl_MouseWheel;

            // shading
            GL.ShadeModel(ShadingModel.Smooth);

            // color of the window
            GL.ClearColor(1.0f, 1.0f, 1.0f, 1.0f);


            GL.ClearDepth(1.0f);

            //enable z-buffering
            GL.Enable(EnableCap.DepthTest);
            GL.DepthFunc(DepthFunction.Lequal);
            GL.Hint(HintTarget.PerspectiveCorrectionHint, HintMode.Nicest);

            GL.Enable(EnableCap.Blend);
            GL.BlendFunc(BlendingFactorSrc.SrcAlpha, BlendingFactorDest.OneMinusSrcAlpha);

            //smoothing
            GL.Enable(EnableCap.LineSmooth);
            GL.Enable(EnableCap.PointSmooth);

            // illumination
            float[] light_ambient = { 0.5f, 0.5f, 0.5f, 1.0f };
            float[] light_diffuse = { 0.5f, 0.5f, 0.5f, 1.0f };
            float[] light_specular = { 0.2f, 0.2f, 0.2f, 1.0f };
            float[] light_position = { 10.0f, 10.0f, 200.0f };
            GL.Light(LightName.Light0, LightParameter.Ambient, light_ambient);
            GL.Light(LightName.Light0, LightParameter.Diffuse, light_diffuse);
            GL.Light(LightName.Light0, LightParameter.Specular, light_specular);
            GL.Light(LightName.Light0, LightParameter.ConstantAttenuation, 1.0f);
            GL.Light(LightName.Light0, LightParameter.Position, light_position);
            GL.Enable(EnableCap.Light0);

            // parameters for the camera
            Phi = 0.6f; Theta = 0.6f; Dist = 3.8f;


        }

        //-----------------------------------------------------------------------------------------------------------------------

        /////////////////////////////////////////////////////////
        //                                                     //
        //                 USER INTERFACE CONTROLS             //
        //                                                     //
        /////////////////////////////////////////////////////////
        private void GLControl_MouseDown(object sender, System.Windows.Forms.MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Right) // camera is adjusted using RMB
            {
                IsRightDown = true;
                RightX = e.X;
                RightY = e.Y;
                oPhi = Phi;
                oTheta = Theta;
            }
            else if (e.Button == MouseButtons.Left) // using LMB we search for the control point beneath the mouse cursor 
            {
                //the idea of the searching -- when I am doing the inverse projection, what points lie in the ray which is casted from the point beneath the cursor. If there are any, I choose the closest one. 

                Vector3 start, end;

                int[] viewport = new int[4];
                Matrix4 modelMatrix, projMatrix;

                GL.GetFloat(GetPName.ModelviewMatrix, out modelMatrix);
                GL.GetFloat(GetPName.ProjectionMatrix, out projMatrix);
                GL.GetInteger(GetPName.Viewport, viewport);

                start = UnProject(new Vector3(e.X, e.Y, 0.0f), projMatrix, modelMatrix, new Size(viewport[2], viewport[3]));
                end = UnProject(new Vector3(e.X, e.Y, 1.0f), projMatrix, modelMatrix, new Size(viewport[2], viewport[3]));

                double se = Math.Sqrt(Vector3.Dot(start - end, start - end));
                for (int k = 0; k < Patches.Count; k++)
                    for (int i = 0; i < Patches[k].InterpolatedPoints.Coordinates.Count; i++)
                    {
                        double sA = Math.Sqrt(Vector3.Dot(Patches[k].InterpolatedPoints.Coordinates[i] - start, Patches[k].InterpolatedPoints.Coordinates[i] - start));
                        double eA = Math.Sqrt(Vector3.Dot(Patches[k].InterpolatedPoints.Coordinates[i] - end, Patches[k].InterpolatedPoints.Coordinates[i] - end));

                        if (sA + eA > se - 0.001 && sA + eA < se + 0.001)
                        {
                            ActivePoint = i;
                            ActivePatch = k;
                            IsLeftDown = true;

                            RightX = e.X;
                            RightY = e.Y;
                        }
                    }
            }

            // redraw the scene
            glControl.Invalidate();
        }

        // Inverse projection
        public Vector3 UnProject(Vector3 mouse, Matrix4 projection, Matrix4 view, Size viewport)
        {
            Vector4 vec;

            vec.X = 2.0f * mouse.X / (float)viewport.Width - 1;
            vec.Y = -(2.0f * mouse.Y / (float)viewport.Height - 1);
            vec.Z = mouse.Z;
            vec.W = 1.0f;

            Matrix4 viewInv = Matrix4.Invert(view);
            Matrix4 projInv = Matrix4.Invert(projection);

            Vector4.Transform(ref vec, ref projInv, out vec);
            Vector4.Transform(ref vec, ref viewInv, out vec);

            if (vec.W > 0.000001f || vec.W < -0.000001f)
            {
                vec.X /= vec.W;
                vec.Y /= vec.W;
                vec.Z /= vec.W;
            }

            return vec.Xyz;
        }

        //-----------------------------------------------------------------------------------------------------------------------

        private void GLControl_MouseMove(object sender, System.Windows.Forms.MouseEventArgs e)
        {
            if (IsRightDown) // RMB - rotate the camera
            {
                IsRightDown = true;

                Phi = oPhi + (RightX - e.X) / 200.0f;
                Theta = oTheta + (e.Y - RightY) / 200.0f;
            }
            else if (IsLeftDown) // LMB - move the control vertex
            {
                IsLeftDown = true;

                float Scaling = 0.003f;

                if (IsX)
                    Patches[ActivePatch].InterpolatedPoints.Coordinates[ActivePoint] = new Vector3(Patches[ActivePatch].InterpolatedPoints.Coordinates[ActivePoint].X + Convert.ToSingle(RightX - e.X) * Scaling, Patches[ActivePatch].InterpolatedPoints.Coordinates[ActivePoint].Y - Convert.ToSingle(RightY - e.Y) * Scaling, Patches[ActivePatch].InterpolatedPoints.Coordinates[ActivePoint].Z);
                if (IsY)
                    Patches[ActivePatch].InterpolatedPoints.Coordinates[ActivePoint] = new Vector3(Patches[ActivePatch].InterpolatedPoints.Coordinates[ActivePoint].X - Convert.ToSingle(RightY - e.Y) * Scaling, Patches[ActivePatch].InterpolatedPoints.Coordinates[ActivePoint].Y - Convert.ToSingle(RightX - e.X) * Scaling, Patches[ActivePatch].InterpolatedPoints.Coordinates[ActivePoint].Z);
                if (IsZ)
                    Patches[ActivePatch].InterpolatedPoints.Coordinates[ActivePoint] = new Vector3(Patches[ActivePatch].InterpolatedPoints.Coordinates[ActivePoint].X, Patches[ActivePatch].InterpolatedPoints.Coordinates[ActivePoint].Y, Patches[ActivePatch].InterpolatedPoints.Coordinates[ActivePoint].Z + Convert.ToSingle(RightY - e.Y) * Scaling);

                RightY = e.Y;
                RightX = e.X;
            }

            // redraw the scene
            glControl.Invalidate();
        }

        //-----------------------------------------------------------------------------------------------------------------------

        private void GLControl_MouseUp(object sender, System.Windows.Forms.MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Right) IsRightDown = false;
            if (e.Button == MouseButtons.Left) IsLeftDown = false;
        }

        //-----------------------------------------------------------------------------------------------------------------------

        private void GLControl_MouseWheel(object sender, System.Windows.Forms.MouseEventArgs e)
        {
            Dist -= (double)e.Delta * 0.001; // zooming

            // redraw the scene
            glControl.Invalidate();
        }

        //-----------------------------------------------------------------------------------------------------------------------

        private void Grid_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            GL.Viewport(0, 0, glControl.Width, glControl.Height);

        }

        private void Window_KeyDown(object sender, System.Windows.Input.KeyEventArgs e)
        {
            if (e.Key == Key.X) // view from above 1
            {
                if (IsX)
                {
                    IsX = false;
                    Phi = prevPhi;
                    Theta = prevTheta;
                    Dist = prevDist;

                    XLabelY.Visibility = System.Windows.Visibility.Hidden;
                    XLabelX.Visibility = System.Windows.Visibility.Hidden;
                    XRectY.Visibility = System.Windows.Visibility.Hidden;
                    XRectX.Visibility = System.Windows.Visibility.Hidden;
                }
                else
                {
                    IsX = true;
                    IsY = false;
                    IsZ = false;
                    prevPhi = Phi;
                    prevTheta = Theta;
                    prevDist = Dist;
                    Phi = 1.57;
                    Theta = 1.24;
                    Dist = 3.5;

                    LabelZ.Visibility = System.Windows.Visibility.Hidden;
                    RectZ.Visibility = System.Windows.Visibility.Hidden;
                    XLabelY.Visibility = System.Windows.Visibility.Visible;
                    XLabelX.Visibility = System.Windows.Visibility.Visible;
                    YLabelY.Visibility = System.Windows.Visibility.Hidden;
                    YLabelX.Visibility = System.Windows.Visibility.Hidden;
                    XRectY.Visibility = System.Windows.Visibility.Visible;
                    XRectX.Visibility = System.Windows.Visibility.Visible;
                    YRectY.Visibility = System.Windows.Visibility.Hidden;
                    YRectX.Visibility = System.Windows.Visibility.Hidden;
                }
            }

            if (e.Key == Key.Y) // view from above 2
            {
                if (IsY)
                {
                    IsY = false;
                    Phi = prevPhi;
                    Theta = prevTheta;
                    Dist = prevDist;

                    YLabelY.Visibility = System.Windows.Visibility.Hidden;
                    YLabelX.Visibility = System.Windows.Visibility.Hidden;
                    YRectY.Visibility = System.Windows.Visibility.Hidden;
                    YRectX.Visibility = System.Windows.Visibility.Hidden;
                }
                else
                {
                    IsY = true;
                    IsX = false;
                    IsZ = false;
                    prevPhi = Phi;
                    prevTheta = Theta;
                    prevDist = Dist;
                    Phi = 0;
                    Theta = 1.3;
                    Dist = 3.5;

                    LabelZ.Visibility = System.Windows.Visibility.Hidden;
                    RectZ.Visibility = System.Windows.Visibility.Hidden;
                    XLabelY.Visibility = System.Windows.Visibility.Hidden;
                    XLabelX.Visibility = System.Windows.Visibility.Hidden;
                    YLabelY.Visibility = System.Windows.Visibility.Visible;
                    YLabelX.Visibility = System.Windows.Visibility.Visible;
                    XRectY.Visibility = System.Windows.Visibility.Hidden;
                    XRectX.Visibility = System.Windows.Visibility.Hidden;
                    YRectY.Visibility = System.Windows.Visibility.Visible;
                    YRectX.Visibility = System.Windows.Visibility.Visible;
                }

            }

            if (e.Key == Key.Z)
            {
                if (IsZ)
                {
                    IsZ = false;

                    LabelZ.Visibility = System.Windows.Visibility.Hidden;
                    RectZ.Visibility = System.Windows.Visibility.Hidden;
                }
                else
                {
                    IsZ = true;
                    IsY = false;
                    IsX = false;
                    LabelZ.Visibility = System.Windows.Visibility.Visible;
                    RectZ.Visibility = System.Windows.Visibility.Visible;
                    XLabelY.Visibility = System.Windows.Visibility.Hidden;
                    XLabelX.Visibility = System.Windows.Visibility.Hidden;
                    YLabelY.Visibility = System.Windows.Visibility.Hidden;
                    YLabelX.Visibility = System.Windows.Visibility.Hidden;
                    XRectY.Visibility = System.Windows.Visibility.Hidden;
                    XRectX.Visibility = System.Windows.Visibility.Hidden;
                    YRectY.Visibility = System.Windows.Visibility.Hidden;
                    YRectX.Visibility = System.Windows.Visibility.Hidden;
                }
            }

            // redraw the scene
            glControl.Invalidate();
        }


    }


}
