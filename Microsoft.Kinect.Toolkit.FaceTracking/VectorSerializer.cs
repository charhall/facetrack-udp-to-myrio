using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
 
//namespace VectorCopyDemo
namespace Microsoft.Kinect.Toolkit.FaceTracking
{
    public class VectorSerializer
    {
 
        public byte[] SerializeVectors(Vector3DF[] vectors)
        {
            var formatter = new BinaryFormatter();
            using (var stream = new MemoryStream())
            {
                formatter.Serialize(stream, vectors);
                return stream.ToArray();
            }
        }
 
        public Vector3DF[] DeserializeVectors(byte[] vectorBuffer)
        {
            var formatter = new BinaryFormatter();
            using (var stream = new MemoryStream(vectorBuffer, false))
            {
                return (Vector3DF[])formatter.Deserialize(stream);
            }
        }
        /*
        public byte[] SerializeVectors(Vector3DF[][] vector)
        {
            throw new System.NotImplementedException();
        }
        */
    }
}