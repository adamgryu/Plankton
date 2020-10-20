using System;
using UnityEngine;

namespace Plankton {
    /// <summary>
    /// Represents a vertex in Plankton's halfedge mesh data structure.
    /// </summary>
    public partial class PlanktonVertex {

        public int OutgoingHalfedge;

        public PlanktonVertexData data;

        internal PlanktonVertex() {
            this.OutgoingHalfedge = -1;
        }

        internal PlanktonVertex(float x, float y, float z, PlanktonVertexData data)
        {
            OutgoingHalfedge = -1;
            this.X = x;
            this.Y = y;
            this.Z = z;
            this.data = data;
        }

        internal PlanktonVertex(float x, float y, float z)
            : this(x, y, z, new PlanktonVertexData())
        {

        }

        internal PlanktonVertex(double x, double y, double z)
            : this((float) x, (float) y, (float) z)
        {
            // empty
        }

        public float X { get; set; }
        
        public float Y { get; set; }
        
        public float Z { get; set; }

        public PlanktonXYZ ToXYZ()
        {
            return new PlanktonXYZ(this.X, this.Y, this.Z);
        }

        /// <summary>
        /// Gets an unset PlanktonVertex. Unset vertices have an outgoing halfedge index of -1.
        /// </summary>
        public static PlanktonVertex Unset
        {
            get { return new PlanktonVertex() { OutgoingHalfedge = -1 }; }
        }
        
        /// <summary>
        /// Whether or not the vertex is currently being referenced in the mesh.
        /// </summary>
        public bool IsUnused { get { return (this.OutgoingHalfedge < 0); } }
        
        [Obsolete()]
        public bool Dead { get { return this.IsUnused; } }

        #region unity

        public static implicit operator Vector3(PlanktonVertex v) => new Vector3(v.X, v.Y, v.Z);

        #endregion
    }
}
