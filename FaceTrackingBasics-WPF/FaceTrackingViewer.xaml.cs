// --------------------------------------------------------------------------------------------------------------------
// <copyright file="FaceTrackingViewer.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
// --------------------------------------------------------------------------------------------------------------------

namespace FaceTrackingBasics
{
    using Microsoft.Kinect;
    using Microsoft.Kinect.Toolkit.FaceTracking;
    using System;
    using System.Collections;
    using System.Collections.Generic;
    using System.Diagnostics;
    using System.IO;
    using System.IO.MemoryMappedFiles;
    using System.Linq;
    using System.Net;
    using System.Net.Sockets;
    using System.Runtime.InteropServices;
    using System.Runtime.Serialization.Formatters.Binary;
    using System.Text;
    using System.Threading;
    using System.Windows;
    using System.Windows.Controls;
    using System.Windows.Media;
    using System.Windows.Media.Media3D;
    using Point = System.Windows.Point;

    /// <summary>
    /// Class that uses the Face Tracking SDK to display a face mask for
    /// tracked skeletons
    /// </summary>
/*
    ///  // Defines Max in an object enumerable
    public static class EnumerableExtensions
    {
        public static T MaxObject<T, U>(this IEnumerable<T> source, Func<T, U> selector)
          where U : IComparable<U>
        {
            if (source == null) throw new ArgumentNullException("source");
            bool first = true;
            T maxObj = default(T);
            U maxKey = default(U);
            foreach (var item in source)
            {
                if (first)
                {
                    maxObj = item;
                    maxKey = selector(maxObj);
                    first = false;
                }
                else
                {
                    U currentKey = selector(item);
                    if (currentKey.CompareTo(maxKey) > 0)
                    {
                        maxKey = currentKey;
                        maxObj = item;
                    }
                }
            }
            if (first) throw new InvalidOperationException("Sequence is empty.");
            return maxObj;
        }
    }
    // ends max defo
    */

    // SerializedStuct of vector3df


    // Define GetSlices
    public static class Ext
    {
        public static IEnumerable<T[]> GetSlices<T>(this IEnumerable<T> source, int n)
        {
            IEnumerable<T> it = source;
            T[] slice = it.Take(n).ToArray();
            it = it.Skip(n);
            while (slice.Length != 0)
            {
                yield return slice;
                slice = it.Take(n).ToArray();
                it = it.Skip(n);
            }
        }
    }
    //End GetSlices


    public partial class FaceTrackingViewer : UserControl, IDisposable
    {
        public static readonly DependencyProperty KinectProperty = DependencyProperty.Register(
            "Kinect", 
            typeof(KinectSensor), 
            typeof(FaceTrackingViewer), 
            new PropertyMetadata(
                null, (o, args) => ((FaceTrackingViewer)o).OnSensorChanged((KinectSensor)args.OldValue, (KinectSensor)args.NewValue)));

        private const uint MaxMissedFrames = 100;

        private readonly Dictionary<int, SkeletonFaceTracker> trackedSkeletons = new Dictionary<int, SkeletonFaceTracker>();

        private byte[] colorImage;

        private ColorImageFormat colorImageFormat = ColorImageFormat.Undefined;

        private short[] depthImage;

        private DepthImageFormat depthImageFormat = DepthImageFormat.Undefined;

        private bool disposed;

        private Skeleton[] skeletonData;
        /*
        // Access Physical Memory of Computer
        const int PROCESS_VM_WRITE = 0x0020;
        const int PROCESS_VM_OPERATION = 0x0008;
        [DllImport("kernel32.dll")]
        static extern IntPtr OpenProcess(ProcessAccessFlags dwDesiredAccess, [MarshalAs(UnmanagedType.Bool)] bool bInheritHandle, int dwProcessId);

        [DllImport("kernel32.dll", SetLastError = true)]
        static extern bool WriteProcessMemory(IntPtr hProcess, IntPtr lpBaseAddress, byte[] lpBuffer, uint nSize, out int lpNumberOfBytesWritten);

        [DllImport("kernel32.dll")]
        public static extern Int32 CloseHandle(IntPtr hProcess);
        // ends here
        */
        private Int32 trackingID = 0;
        private int frameNumber = 0;
        /*
        //Flags Defined
        [Flags]
        public enum ProcessAccessFlags : uint
        {
            All = 0x001F0FFF,
            Terminate = 0x00000001,
            CreateThread = 0x00000002,
            VMOperation = 0x00000008,
            VMRead = 0x00000010,
            VMWrite = 0x00000020,
            DupHandle = 0x00000040,
            SetInformation = 0x00000200,
            QueryInformation = 0x00000400,
            Synchronize = 0x00100000
        }
        // Flags defo ends here
        */
    /*    
        public static void WriteMem(Process p, int address, long v)
        {
            var hProc = OpenProcess(ProcessAccessFlags.All, false, p.Id);
            var val = new byte[] { (byte)v };

            int fubar = 0;
            WriteProcessMemory(hProc, new IntPtr(address), val, (UInt32)val.LongLength, out fubar);

            CloseHandle(hProc);
        }
*/
       
        public FaceTrackingViewer()
        {
            this.InitializeComponent();
        }

        ~FaceTrackingViewer()
        {
            this.Dispose(false);
        }

        public KinectSensor Kinect
        {
            get
            {
                return (KinectSensor)this.GetValue(KinectProperty);
            }

            set
            {
                this.SetValue(KinectProperty, value);
            }
        }

        public void Dispose()
        {
            this.Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
            if (!this.disposed)
            {
                this.ResetFaceTracking();

                this.disposed = true;
            }
        }

        protected override void OnRender(DrawingContext drawingContext)
        {
            base.OnRender(drawingContext);
            foreach (SkeletonFaceTracker faceInformation in this.trackedSkeletons.Values)
            {
                faceInformation.DrawFaceModel(drawingContext);
            }
        }
        // BEGIN:  // convert between bytes and float
        public unsafe byte[] FloatsToBytes(float[] floats)
        {

            fixed (void* pFloats = floats)
            {
                byte* pBytes = (byte*)pFloats;
                byte[] bytes = new byte[sizeof(float) * floats.Length];
                Marshal.Copy((IntPtr)pBytes, bytes, 0, floats.Length * sizeof(byte));
                return bytes;
            }
        }

        public unsafe float[] BytesToFloats(byte[] bytes)
        {

            fixed (void* pBytes = bytes)
            {
                float* pFloats = (float*)pBytes;
                float[] floats = new float[bytes.Length / sizeof(float)];
                Marshal.Copy((IntPtr)pFloats, floats, 0, bytes.Length / sizeof(float));
                return floats;
            }
        }
        // END: convert between bytes and float

        // TRY BYTES CONVERSION: _h_ttp://msdn.microsoft.com/en-us/library/yhwsaf3w(v=vs.110).aspx
        public static void GetBytesSingle(float argument)
        {
            byte[] byteArray = BitConverter.GetBytes(argument);
        }

        private void OnAllFramesReady(object sender, AllFramesReadyEventArgs allFramesReadyEventArgs)
        {
            ColorImageFrame colorImageFrame = null;
            DepthImageFrame depthImageFrame = null;

            try
            {
                depthImageFrame = allFramesReadyEventArgs.OpenDepthImageFrame();
                colorImageFrame = allFramesReadyEventArgs.OpenColorImageFrame();

                if (depthImageFrame == null)
                {
                    return;
                }

                if (colorImageFrame == null)
                {
                    return;
                }

                // Check for image format changes.  The FaceTracker doesn't
                // deal with that so we need to reset.
                if (this.depthImageFormat != depthImageFrame.Format)
                {
                    this.ResetFaceTracking();
                    this.depthImage = null;
                    this.depthImageFormat = depthImageFrame.Format;
                }

                if (this.colorImageFormat != colorImageFrame.Format)
                {
                    this.ResetFaceTracking();
                    this.colorImage = null;
                    this.colorImageFormat = colorImageFrame.Format;
                }

                // Create any buffers to store copies of the data we work with
                if (this.depthImage == null)
                {
                    this.depthImage = new short[depthImageFrame.PixelDataLength];
                }

                if (this.colorImage == null)
                {
                    this.colorImage = new byte[colorImageFrame.PixelDataLength];
                }

                depthImageFrame.CopyPixelDataTo(this.depthImage);
                colorImageFrame.CopyPixelDataTo(this.colorImage);

                if (!this.trackedSkeletons.ContainsKey(this.trackingID))
                {
                    this.trackedSkeletons.Add(this.trackingID, new SkeletonFaceTracker());
                }

                // Give each tracker the upated frame.
                SkeletonFaceTracker skeletonFaceTracker;
                if (this.trackedSkeletons.TryGetValue(this.trackingID, out skeletonFaceTracker))
                {
                    skeletonFaceTracker.OnFrameReady(this.Kinect, colorImageFormat, colorImage, depthImageFormat, depthImage);
                    skeletonFaceTracker.LastTrackedFrame = this.frameNumber;
                }

                this.RemoveOldTrackers(this.frameNumber);

                this.InvalidateVisual();
            }

            finally
            {
                if (depthImageFrame != null)
                {
                    depthImageFrame.Dispose();
                }

                if (colorImageFrame != null)
                {
                    colorImageFrame.Dispose();
                }
            }
        }

        
        private void OnSensorChanged(KinectSensor oldSensor, KinectSensor newSensor)
        {
            if (oldSensor != null)
            {
                oldSensor.AllFramesReady -= this.OnAllFramesReady;
                this.ResetFaceTracking();
            }

            if (newSensor != null)
            {
                newSensor.AllFramesReady += this.OnAllFramesReady;
            }
        }

        /// <summary>
        /// Clear out any trackers for skeletons we haven't heard from for a while
        /// </summary>
        private void RemoveOldTrackers(int currentFrameNumber)
        {
            var trackersToRemove = new List<int>();

            foreach (var tracker in this.trackedSkeletons)
            {
                uint missedFrames = (uint)currentFrameNumber - (uint)tracker.Value.LastTrackedFrame;
                if (missedFrames > MaxMissedFrames)
                {
                    // There have been too many frames since we last saw this skeleton
                    trackersToRemove.Add(tracker.Key);
                }
            }

            foreach (int trackingId in trackersToRemove)
            {
                this.RemoveTracker(trackingId);
            }
        }

        private void RemoveTracker(int trackingId)
        {
            this.trackedSkeletons[trackingId].Dispose();
            this.trackedSkeletons.Remove(trackingId);
        }

        private void ResetFaceTracking()
        {
            foreach (int trackingId in new List<int>(this.trackedSkeletons.Keys))
            {
                this.RemoveTracker(trackingId);
            }
        }

        private class SkeletonFaceTracker : IDisposable
        {
            private static FaceTriangle[] faceTriangles;

            private EnumIndexableCollection<FeaturePoint, PointF> facePoints;

            private EnumIndexableCollection<FeaturePoint, Vector3DF> facePoints3D;

            private FaceTracker faceTracker;

            private bool lastFaceTrackSucceeded;

           // private SkeletonTrackingState skeletonTrackingState;

            public int LastTrackedFrame { get; set; }

            public void Dispose()
            {
                if (this.faceTracker != null)
                {
                    this.faceTracker.Dispose();
                    this.faceTracker = null;
                }
            }

            public void DrawFaceModel(DrawingContext drawingContext)
            {
                if (!this.lastFaceTrackSucceeded)
                {
                    return;
                }
                
                var faceModelPts = new List<Point>();
                var faceModel = new List<FaceModelTriangle>();

                for (int i = 0; i < this.facePoints.Count; i++)
                {
                    faceModelPts.Add(new Point(this.facePoints[i].X + 0.5f, this.facePoints[i].Y + 0.5f));
                }

                foreach (var t in faceTriangles)
                {
                    var triangle = new FaceModelTriangle();
                    triangle.P1 = faceModelPts[t.First];
                    triangle.P2 = faceModelPts[t.Second];
                    triangle.P3 = faceModelPts[t.Third];
                    faceModel.Add(triangle);
                }

                var faceModelGroup = new GeometryGroup();
                for (int i = 0; i < faceModel.Count; i++)
                {
                    var faceTriangle = new GeometryGroup();
                    faceTriangle.Children.Add(new LineGeometry(faceModel[i].P1, faceModel[i].P2));
                    faceTriangle.Children.Add(new LineGeometry(faceModel[i].P2, faceModel[i].P3));
                    faceTriangle.Children.Add(new LineGeometry(faceModel[i].P3, faceModel[i].P1));
                    faceModelGroup.Children.Add(faceTriangle);
                }

                drawingContext.DrawGeometry(Brushes.LightYellow, new Pen(Brushes.LightYellow, 1.0), faceModelGroup);
            }

            
            /// <summary>
            /// Updates the face tracking information for this skeleton
            /// </summary>
            /// 
            internal void OnFrameReady(KinectSensor kinectSensor, ColorImageFormat colorImageFormat, byte[] colorImage, DepthImageFormat depthImageFormat, short[] depthImage)
            {
                if (this.faceTracker == null)
                {
                    try
                    {
                        this.faceTracker = new FaceTracker(kinectSensor);
                    }
                    catch (InvalidOperationException)
                    {
                        // During some shutdown scenarios the FaceTracker
                        // is unable to be instantiated.  Catch that exception
                        // and don't track a face.
                        Debug.WriteLine("AllFramesReady - creating a new FaceTracker threw an InvalidOperationException");
                        this.faceTracker = null;
                    }
                }

                if (this.faceTracker != null)
                {
                    FaceTrackFrame frame = this.faceTracker.Track(
                        colorImageFormat, colorImage, depthImageFormat, depthImage);

                    this.lastFaceTrackSucceeded = frame.TrackSuccessful;
                    if (this.lastFaceTrackSucceeded)
                    {
                        if (faceTriangles == null)
                        {
                            // only need to get this once.  It doesn't change.
                            faceTriangles = frame.GetTriangles();
                        }

                        this.facePoints = frame.GetProjected3DShape();
                        int n = 121;
                        this.facePoints3D = frame.Get3DShape();

                        // UDP Connection  :: Talker ::
                        Boolean done = false; 

                        Boolean exception_thrown = false;

                        Socket sending_socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);

                        IPAddress send_to_address = IPAddress.Parse("172.22.11.2");

                        IPEndPoint sending_end_point = new IPEndPoint(send_to_address, 80);

                        UdpClient listener = new UdpClient(80);

                        while (!done)
                        {
                          /*  int index = 0;
                            byte[] bytearray = new byte[facePoints3D.Count * 4];  */

                           foreach (Vector3DF vector in facePoints3D)
                            {
                                //arrange
                              /*   Array.Copy(BitConverter.GetBytes((float)vector.Z), 0, bytearray, index, 4);
                                 index += 4;   */
                            
                                float zvect = vector.Z;

                                byte[] bytearray = BitConverter.GetBytes(zvect);
                               
                              
                                //byte[] bytearray = Encoding.ASCII.GetBytes("The Little Boy is Good");
                               /*
                                if (BitConverter.IsLittleEndian)
                                    Array.Reverse(bytearray); 
                               */
                                // Remind the user of where this is going.
                                
                                Console.WriteLine("sending to address: {0} port: {1}", sending_end_point.Address,  sending_end_point.Port);
                                
                                try
                                {
                                    //sending_socket.SendTo(data, sending_end_point);
                                    sending_socket.SendTo(bytearray, sending_end_point);

                                    Console.WriteLine("Message was sent");
                                }

                                catch (Exception send_exception)
                                {
                                    exception_thrown = true;
                                    Console.WriteLine(" Exception {0}", send_exception.Message);
                                }

                                if (exception_thrown == false)
                                {
                                    Console.WriteLine("Message has been sent to the broadcast address");
                                }

                                else
                                {
                                    exception_thrown = false;
                                    Console.WriteLine("The exception indicates the message was not sent.");
                                }

                            }   //ends foreach statement

                            // start listening
                            
                         /*   byte[] bytes = listener.Receive(ref sending_end_point);  

                            Console.WriteLine("Received broadcast from {0} :\n {1}\n", sending_end_point.ToString(), Encoding.ASCII.GetString(bytes, 0, bytes.Length));   */
                           
                        }  //ends while(!done) statement

                        // ends udp talker


  
                        /*
                        using (MemoryStream stream = new MemoryStream())
                        {
                            var sw = new StreamWriter(stream);

                            int n = 121;

                            foreach (Vector3DF[] vector in facePoints3D.GetSlices(n))
                            {
                                //arrange
                                var copier = new VectorSerializer();

                                //act 
                                byte[] bytearray = copier.SerializeVectors(vector);
                                Vector3DF[] copiedVectors = copier.DeserializeVectors(bytearray);

                                //use MemoryFileManager Library
                                MemoryMappedFileCommunicator communicator = new MemoryMappedFileCommunicator("MemoryMappedShare", 10240);

                                // This process reads data that begins in the position 2000 and writes starting from the position 0.
                                communicator.ReadPosition = 2000;
                                communicator.WritePosition = 0; 
                                
                                // Creates an handler for the event that is raised when data are available in the MemoryMappedFile. 
                                //communicator.DataReceived += new EventHandler<MemoryMappedDataReceivedEventArgs>(communicator_DataReceived); 
                               // communicator.StartReader();
                                
                                //Write to the Shared File
                                communicator.Write(bytearray);
                            }

                        }
                        */
                    }
                }
            }
        
        private struct FaceModelTriangle
            {
                public Point P1;
                public Point P2;
                public Point P3;
            }
        }
    }
}