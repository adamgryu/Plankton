using UnityEngine;

namespace Plankton {

    public struct PlanktonVertexData {
        public Vector2 UV;
        public Vector3 Normal;

        public static PlanktonVertexData operator *(PlanktonVertexData v, float t) {
            return new PlanktonVertexData() {
                UV = v.UV * t,
                Normal = v.Normal * t,
            };
        }

        public static PlanktonVertexData operator +(PlanktonVertexData a, PlanktonVertexData b) {
            return new PlanktonVertexData() {
                UV = a.UV + b.UV,
                Normal = a.Normal + b.Normal
            };
        }

        public PlanktonVertexData Normalize() {
            Normal = Normal.normalized;
            return this;
        }
    }
}
