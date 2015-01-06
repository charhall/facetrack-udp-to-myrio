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
    using System.Collections.Generic;
    using System.Diagnostics;
    using System.IO;
    using System.Linq;
    using System.Runtime.InteropServices;
    using System.Net;
    using System.Net.Sockets;
    using System.Text;
    using System.Windows;
    using System.Windows.Controls;
    using System.Windows.Media;
    using Point = System.Windows.Point;

    /// <summary>
    /// Class that uses the Face Tracking SDK to display a face mask for
    /// tracked skeletons
    /// </summary>
    /// 

    //GetSlices Here
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

       // private Skeleton[] skeletonData;

        private Int32 trackingID = 0;

        private int frameNumber = 0;

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

                        int index = 1;

                        this.facePoints3D = frame.Get3DShape();


                        // UDP Connection  :: Talker ::
                        Boolean done = false;
                        Boolean exception_thrown = false;

                        Socket sending_socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);

                        IPAddress send_to_address = IPAddress.Parse("172.22.11.2");

                        IPEndPoint sending_end_point = new IPEndPoint(send_to_address, 80);

                        while (!done)
                        {
                            //foreach (Vector3DF[] vector in facePoints3D.GetSlices(n))
                          foreach (Vector3DF vector in facePoints3D)
                            {
                                //arrange
                                byte[] bytearray = BitConverter.GetBytes(vector.Z);
                                Console.WriteLine(bytearray);
                              /*
                                var copier = new VectorSerializer();

                                //act 
                                byte[] bytearray = copier.SerializeVectors(vector);
                                Vector3DF[] copiedVectors = copier.DeserializeVectors(bytearray);
                              */
                                //send_buffer = bytearray;
                                // Remind the user of where this is going.
                                Console.WriteLine("sending to address: {0} port: {1}",
                                sending_end_point.Address,
                                sending_end_point.Port);

                                try
                                {
                                    sending_socket.SendTo(bytearray, sending_end_point);
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
                        }  //ends while(!done) statement

                        // ends udp talker

                        /*
                        foreach (Vector3DF vector in facePoints3D)
                        {
                            //Console.WriteLine(string.Format("{0}: {1}, {2}, {3}" , index++, vector.X, vector.Y, vector.Z));
                            Console.WriteLine(vector.Z);
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
