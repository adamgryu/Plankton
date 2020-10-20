using UnityEngine;
using Plankton;
using System.Linq;
using System.Collections.Generic;
using System;

public static class GeometryTools {

    /// <summary>
    /// Returns true if the point is inside the 3D triangle.
    /// </summary>
    public static bool IsPointInTriangle(Vector3 pt, Vector3 v1, Vector3 v2, Vector3 v3) {
        Vector3 n = Vector3.Cross(v1 - v3, v2 - v3);

        float d1 = LineSide(pt, v1, v2, n);
        float d2 = LineSide(pt, v2, v3, n);
        float d3 = LineSide(pt, v3, v1, n);

        bool hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        bool hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        bool inside = !(hasNeg && hasPos);
        if (inside) {
            // Ensure that it's actually on the plane.
            if (Vector3.Project(pt - v1, n.normalized).sqrMagnitude < 0.01f) {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// With respect to a plane, checks to see if the vector to A is left or right of the vector to B.
    /// The sign of the returned float indicates the side.
    /// </summary>
    public static float LineSide(Vector3 from, Vector3 toA, Vector3 toB, Vector3 planeNormal) {
        return Vector3.Dot(planeNormal, Vector3.Cross(toA - from, toB - from));
    }

    /// <summary>
    /// Finds the intersection or closest intersection point between 3D lines A and B.
    /// </summary>
    public static bool IsIntersecting(Vector3 a1, Vector3 a2, Vector3 b1, Vector3 b2, out Vector3 intersection) {
        // From: http://paulbourke.net/geometry/pointlineplane/

        Vector3 v1 = a1;
        Vector3 v2 = a2;
        Vector3 v3 = b1;
        Vector3 v4 = b2;

        float mua = (d(v1, v3, v4, v3) * d(v4, v3, v2, v1) - d(v1, v3, v2, v1) * d(v4, v3, v4, v3))
            / (d(v2, v1, v2, v1) * d(v4, v3, v4, v3) - d(v4, v3, v2, v1) * d(v4, v3, v2, v1));

        intersection = v1 + mua * (v2 - v1);

        float mub = (d(v1, v3, v4, v3) + mua * d(v4, v3, v2, v1)) / d(v4, v3, v4, v3);
        return mua >= 0 && mua <= 1 && mub >= 0 && mub <= 1;
    }

    public static Vector3 FindIntersection(Vector3 a1, Vector3 a2, Vector3 b1, Vector3 b2) {
        IsIntersecting(a1, a2, b1, b2, out Vector3 hit);
        return hit;
    }

    private static float d(Vector3 m, Vector3 n, Vector3 o, Vector3 p) {
        return (m.x - n.x) * (o.x - p.x) + (m.y - n.y) * (o.y - p.y) + (m.z - n.z) * (o.z - p.z);
    }
}


public static class PlanktonTools {

    public const float FLOAT_ERROR_SNAP = 1f / 0.000001f;

    /// <summary>
    /// Returns all the holes on the specified mesh, ignoring seams along the UV edge.
    /// </summary>
    public static IEnumerable<List<int>> AllBorderLoops(this PlanktonMesh mesh) {
        HashSet<int> seenEdges = new HashSet<int>();
        List<int> startEdges = new List<int>();

        foreach (var edge in AllBorderEdges(mesh.Halfedges)) {
            if (!seenEdges.Contains(edge)) {
                startEdges.Add(edge);
                foreach (var borderEdge in TraceBorderFromEdge(mesh, edge)) {
                    seenEdges.Add(borderEdge);
                }
            }
        }

        foreach (var edge in startEdges) {
            foreach (var loop in CollapseSeamEdgeIntoLoops(mesh, TraceBorderFromEdge(mesh, edge).ToList())) {
                yield return loop;
            }
        }
    }

    /// <summary>
    /// Given a set of border edges, prune edges that are actually on the edge of a UV seam in the mesh.
    /// </summary>
    public static IEnumerable<List<int>> CollapseSeamEdgeIntoLoops(PlanktonMesh mesh, List<int> edges) {
        Dictionary<Vector3, int> pointPositions = new Dictionary<Vector3, int>();
        List<int> loopPoints = new List<int>();

        int searchIndex = 0;
        while (searchIndex < edges.Count) {
            int edgeIndex = edges[searchIndex];
            loopPoints.Add(edgeIndex);
            Vector3 position = mesh.Vertices[mesh.Halfedges[edgeIndex].StartVertex].ToUnity();
            position = SnapToInterval(position);

            if (!pointPositions.ContainsKey(position)) {
                pointPositions.Add(position, loopPoints.Count - 1);
            } else {
                int lastLoopIndex = pointPositions[position];
                if (lastLoopIndex == loopPoints.Count - 3) {
                    // Remove the flat leaf from the loop.
                    loopPoints.RemoveAt(loopPoints.Count - 3);
                    loopPoints.RemoveAt(loopPoints.Count - 2);
                } else {
                    // A hole has been detected. Cut it out and return it.
                    List<int> innerLoop = new List<int>();
                    int removeNumber = loopPoints.Count - 1 - lastLoopIndex;
                    for (int i = 0; i < removeNumber; i++) {
                        innerLoop.Add(loopPoints[lastLoopIndex]);
                        loopPoints.RemoveAt(lastLoopIndex);
                    }
                    yield return innerLoop;
                }
            }
            searchIndex++;
        }

        yield return loopPoints;
    }

    private static Vector3 SnapToInterval(Vector3 position) {
        return new Vector3(
            Mathf.Round(position.x * FLOAT_ERROR_SNAP) / FLOAT_ERROR_SNAP,
            Mathf.Round(position.y * FLOAT_ERROR_SNAP) / FLOAT_ERROR_SNAP,
            Mathf.Round(position.z * FLOAT_ERROR_SNAP) / FLOAT_ERROR_SNAP);
    }

    public static IEnumerable<int> AllBorderEdges(this PlanktonHalfEdgeList halfedges) {
        for (int i = 0; i < halfedges.Count; i++) {
            if (!halfedges[i].IsUnused && halfedges[i].AdjacentFace == -1) {
                yield return i;
            }
        }
    }

    public static HashSet<int> FindConnectedFaces(this PlanktonMesh mesh, int startFaceIndex, HashSet<int> blockedEdges) {
        HashSet<int> seenFaces = new HashSet<int>();
        Queue<int> queued = new Queue<int>();

        seenFaces.Add(startFaceIndex);
        queued.Enqueue(startFaceIndex);

        int[] foundEdges = new int[3];
        while (queued.Count > 0) {
            int face = queued.Dequeue();

            int edgeNum = mesh.Faces.GetHalfedgesNonAlloc(face, foundEdges);
            for (int i = 0; i < edgeNum; i++) {
                var pairEdge = mesh.Halfedges.GetPairHalfedge(foundEdges[i]);
                int pairFace = mesh.Halfedges[pairEdge].AdjacentFace;
                if (pairFace != -1 && !seenFaces.Contains(pairFace) && !blockedEdges.Contains(pairEdge)) {
                    queued.Enqueue(pairFace);
                    seenFaces.Add(pairFace);
                }
            }
        }

        return seenFaces;
    }

    public static float CalculateRegionArea(this PlanktonMesh mesh, IEnumerable<int> triangles) {
        float total = 0;
        int[] faceVertices = new int[3];
        foreach (var face in triangles) {
            mesh.Faces.GetFaceVerticesNonAlloc(face, faceVertices);
            Vector3 a = mesh.Vertices.GetPositionUnity(faceVertices[0]);
            Vector3 b = mesh.Vertices.GetPositionUnity(faceVertices[1]);
            Vector3 c = mesh.Vertices.GetPositionUnity(faceVertices[2]);
            total += Vector3.Cross(a - b, a - c).magnitude / 2f;
        }
        return total;
    }

    public static int InsertPointInTriangle(this PlanktonMesh mesh, int triIndex, Vector3 point) {
        // Remove the old face.
        var verts = mesh.Faces.GetFaceVertices(triIndex);
        mesh.Faces.RemoveFace(triIndex);

        // Add three new inner triangles
        int newVertIndex = mesh.Vertices.Add(point.x, point.y, point.z, BlendVertexDataInsideTriangle(mesh, point, verts));
        mesh.Faces.AddFace(verts[0], verts[1], newVertIndex);
        mesh.Faces.AddFace(verts[1], verts[2], newVertIndex);
        mesh.Faces.AddFace(verts[2], verts[0], newVertIndex);
        return newVertIndex;
    }

    /// <summary>
    /// Searches every triangle on the mesh to find one that contains the given position.
    /// </summary>
    public static int FindTriangle(this PlanktonMesh mesh, Vector3 position) {
        int[] faceVertices = new int[3];
        for (int i = 0; i < mesh.Faces.Count; i++) {
            if (mesh.Faces.GetFaceVerticesNonAlloc(i, faceVertices) == 3) {
                bool inside = GeometryTools.IsPointInTriangle(
                    position,
                    mesh.Vertices.GetPositionUnity(faceVertices[0]),
                    mesh.Vertices.GetPositionUnity(faceVertices[1]),
                    mesh.Vertices.GetPositionUnity(faceVertices[2]));
                if (inside) {
                    return i;
                }
            }
        }
        new GameObject().transform.position = position;
        return -1;
    }

    public static PlanktonVertexData BlendVertexDataInsideTriangle(PlanktonMesh mesh, Vector3 point, int[] vertexIndices) {
        if (vertexIndices.Length != 3) {
            Debug.Log(vertexIndices.Length);
        }
        Vector3 p1 = mesh.Vertices.GetPositionUnity(vertexIndices[0]);
        Vector3 p2 = mesh.Vertices.GetPositionUnity(vertexIndices[1]);
        Vector3 p3 = mesh.Vertices.GetPositionUnity(vertexIndices[2]);

        Vector3 f1 = p1 - point;
        Vector3 f2 = p2 - point;
        Vector3 f3 = p3 - point;

        // Calculate the areas and factors (order of parameters doesn't matter):
        float a = Vector3.Cross(p1 - p2, p1 - p3).magnitude; // main triangle area a
        float a1 = Vector3.Cross(f2, f3).magnitude / a; // p1's triangle area / a
        float a2 = Vector3.Cross(f3, f1).magnitude / a; // p2's triangle area / a 
        float a3 = Vector3.Cross(f1, f2).magnitude / a; // p3's triangle area / a

        return (mesh.Vertices[vertexIndices[0]].data * a1
            + mesh.Vertices[vertexIndices[1]].data * a2
            + mesh.Vertices[vertexIndices[2]].data * a3).Normalize();
    }

    /// <summary>
    /// Searches from the given vertex along the mesh to trace the edge of the mesh.
    /// Enumerates through each half edge along the border.
    /// </summary>
    public static IEnumerable<int> TraceBorderFromVertex(this PlanktonMesh mesh, int startVertex) {
        // Find the starting edge from the vertex.
        int startingEdge = -1;
        foreach (var e in mesh.Vertices.GetHalfedges(startVertex)) {
            if (mesh.Halfedges[e].AdjacentFace == -1) {
                startingEdge = e;
            }
        }

        if (startingEdge == -1) {
            Debug.LogError("There is no border here!");
            yield break;
        }

        foreach (var e in TraceBorderFromEdge(mesh, startingEdge)) {
            yield return e;
        }
    }

    /// <summary>
    /// Searches from the given edge along the mesh to trace the border of the mesh.
    /// Enumerates through each half edge along the border.
    /// </summary>
    public static IEnumerable<int> TraceBorderFromEdge(this PlanktonMesh mesh, int startingEdge) {
        yield return startingEdge;
        int current = mesh.Halfedges[startingEdge].NextHalfedge;

        int count = 0;
        while (current != startingEdge && count < 19999) {
            yield return current;
            current = mesh.Halfedges[current].NextHalfedge;
            count++;
        }

        if (count == 19999) {
            Debug.LogError("Runaway border trace!");
        }
    }

    public static PlanktonMesh UnityToPlankton(this Mesh mesh) {
        var plankton = new PlanktonMesh();

        // Cache the mesh data.
        var vertices = mesh.vertices;
        var triangles = mesh.triangles;
        var uvs = mesh.uv;
        var normals = mesh.normals;

        for (int i = 0; i < vertices.Length; i++) {
            plankton.Vertices.Add(
                vertices[i].x,
                vertices[i].y,
                vertices[i].z,
                new PlanktonVertexData() { UV = uvs[i], Normal = normals[i] }
            );
        }

        int[] triIndices = new int[3];
        for (int i = 0; i < triangles.Length; i += 3) {
            triIndices[0] = triangles[i];
            triIndices[1] = triangles[i + 1];
            triIndices[2] = triangles[i + 2];
            plankton.Faces.AddFace(triIndices);
        }

        return plankton;
    }

    public static Mesh PlanktonToUnity(this PlanktonMesh plankton) {
        int vCount = plankton.Vertices.Count;
        var vertices = new Vector3[vCount];
        for (int i = 0; i < vCount; i++) {
            PlanktonVertex v = plankton.Vertices[i];
            vertices[i] = new Vector3(v.X, v.Y, v.Z);
        }

        int fCount = plankton.Faces.Count;
        var triangles = new int[fCount * 3];
        int[] faceVertices = new int[3];
        for (int i = 0; i < fCount; i++) {
            if (!plankton.Faces[i].IsUnused) {
                plankton.Faces.GetFaceVerticesNonAlloc(i, faceVertices);
                triangles[i * 3] = faceVertices[0];
                triangles[i * 3 + 1] = faceVertices[1];
                triangles[i * 3 + 2] = faceVertices[2];
            }
        }

        var mesh = new Mesh();
        mesh.vertices = vertices;
        mesh.uv = plankton.Vertices.Select(v => v.data.UV).ToArray();
        mesh.normals = plankton.Vertices.Select(v => v.data.Normal).ToArray();
        mesh.triangles = triangles;
        mesh.RecalculateTangents();
        return mesh;
    }

    public static PlanktonXYZ ToPlankton(this Vector3 v) {
        return new PlanktonXYZ(v.x, v.y, v.z);
    }

    public static Vector3 ToUnity(this PlanktonXYZ v) {
        return new Vector3(v.X, v.Y, v.Z);
    }

    public static Vector3 ToUnity(this PlanktonVertex v) {
        return new Vector3(v.X, v.Y, v.Z);
    }

    public static Vector3 GetPositionUnity(this PlanktonVertexList list, int i) {
        return list[i].ToUnity();
    }
}