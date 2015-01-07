/// <summary>
/// Summary description for Ext
/// </summary>
namespace Microsoft.Kinect.Toolkit.FaceTracking
{
    using System;
    using System.Globalization;
    using System.Runtime.InteropServices;


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

}
