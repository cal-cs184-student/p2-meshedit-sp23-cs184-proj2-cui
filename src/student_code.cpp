#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
      std::vector<Vector2D> res;
      for (int i = 1; i < points.size(); ++i) {
          Vector2D intermed = points[i-1]*(1-t) + points[i]*t;
          res.push_back(intermed);
      }
      return res;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
{
    // TODO Part 2.
    std::vector<Vector3D> res;
    for (int i = 1; i < points.size(); ++i) {
        Vector3D intermed = points[i-1]*(1-t) + points[i]*t;
        res.push_back(intermed);
    }
    return res;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      if (points.size() == 1) {
          return points[0];
      }
      std::vector<Vector3D> intermeds = evaluateStep(points, t);
      return evaluate1D(intermeds, t);
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
      std::vector<Vector3D> uPoints;
      for (int i = 0; i < controlPoints.size(); ++i) {
          uPoints.push_back(evaluate1D(controlPoints[i], u));
      }
      return evaluate1D(uPoints, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
      HalfedgeCIter h = halfedge();
      std::vector<Vector3D> edgeNormals;
      do {
          HalfedgeCIter cur = h;
          std::vector<Vector3D> vertices;
          do {
              vertices.push_back(cur->vertex()->position);
              cur = cur->next();
          } while (cur != h);
          Vector3D crossProduct = 0.5 * cross((vertices[1]-vertices[0]), (vertices[2]-vertices[0]));
          edgeNormals.push_back(crossProduct);
          HalfedgeCIter h_twin = h->twin();
          h = h_twin->next();
      } while (h != halfedge());
      Vector3D sum = Vector3D();
      for (int i = 0; i < edgeNormals.size(); ++i) {
          sum += edgeNormals[i];
      }
      sum.normalize();
      return sum;
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
      // collect elements
      if (e0->isBoundary()) {
          return EdgeIter();
      }
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h1->twin();
      HalfedgeIter h7 = h2->twin();
      HalfedgeIter h8 = h4->twin();
      HalfedgeIter h9 = h5->twin();
      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h6->vertex();
      VertexIter v3 = h8->vertex();
      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();
      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();
      // edit elements
      h0->vertex() = v3;
      h1->twin() = h7;
      h1->vertex() = v2;
      h1->edge() = e2;
      h2->twin() = h8;
      h2->vertex() = v0;
      h2->edge() = e3;
      h3->vertex() = v2;
      h4->twin() = h9;
      h4->vertex() = v3;
      h4->edge() = e4;
      h5->twin() = h6;
      h5->vertex() = v1;
      h5->edge() = e1;
      h6->twin() = h5;
      h7->twin() = h1;
      h8->twin() = h2;
      h9->twin() = h4;
      v0->halfedge() = h2;
      v1->halfedge() = h5;
      v2->halfedge() = h1;
      v3->halfedge() = h0;
      e1->halfedge() = h5;
      e2->halfedge() = h1;
      e3->halfedge() = h2;
      e4->halfedge() = h4;
      return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
      if (e0->isBoundary()) {
          return;
      }
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      VertexIter v0 = h0->vertex();
      VertexIter v1 = h1->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();
      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h5->edge();
      EdgeIter e4 = h4->edge();
      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();
      
      HalfedgeIter h6 = newHalfedge();
      HalfedgeIter h7 = newHalfedge();
      HalfedgeIter h8 = newHalfedge();
      HalfedgeIter h9 = newHalfedge();
      HalfedgeIter h10 = newHalfedge();
      HalfedgeIter h11 = newHalfedge();
      VertexIter v4 = newVertex();
      EdgeIter e5 = newEdge();
      EdgeIter e6 = newEdge();
      EdgeIter e7 = newEdge();
      FaceIter f2 = newFace();
      FaceIter f3 = newFace();
      
      v4->isNew = true;
      e5->isNew = true;
      e7->isNew = true;
      
      h0->next() = h9;
      h1->next() = h10;
      h1->face() = f3;
      // no changes to h2
      h3->vertex() = v4;
      h4->next() = h8;
      h5->next() = h6;
      h5->face() = f2;
      
      h6->twin() = h11;
      h6->next() = h7;
      h6->vertex() = v1;
      h6->edge() = e6;
      h6->face() = f2;
      
      h7->twin() = h8;
      h7->next() = h5;
      h7->vertex() = v4;
      h7->edge() = e7;
      h7->face() = f2;
      
      h8->twin() = h7;
      h8->next() = h3;
      h8->vertex() = v3;
      h8->edge() = e7;
      h8->face() = f1;
      
      h9->twin() = h10;
      h9->next() = h2;
      h9->vertex() = v4;
      h9->edge() = e5;
      h9->face() = f0;
      
      h10->twin() = h9;
      h10->next() = h11;
      h10->vertex() = v2;
      h10->edge() = e5;
      h10->face() = f3;
      
      h11->twin() = h6;
      h11->next() = h1;
      h11->vertex() = v4;
      h11->edge() = e6;
      h11->face() = f3;
      
      v4->position = (v0->position + v1->position) * 0.5;
      v4->halfedge() = h3;
      
      e5->halfedge() = h9;
      e6->halfedge() = h6;
      e7->halfedge() = h7;
      
      f0->halfedge() = h0;
      f1->halfedge() = h3;
      f2->halfedge() = h5;
      f3->halfedge() = h1;
      
      return v4;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.
    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          v->isNew = false;
          float n = (float) v->degree();
          float u = (n == 3.0) ? (3.0/16.0) : (3.0/(8.0*n));
          HalfedgeCIter h = v->halfedge();
          Vector3D neighborSum = Vector3D();
          do {
              HalfedgeCIter h_twin = h->twin();
              VertexCIter curV = h_twin->vertex();
              neighborSum += curV->position;
              h = h->twin()->next();
          } while (h != v->halfedge());
          v->newPosition = (1.0 - n * u) * v->position + u * neighborSum;
      }
      
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
          e->isNew = false;
          HalfedgeIter h0 = e->halfedge();
          HalfedgeIter h1 = h0->next();
          HalfedgeIter h2 = h1->next();
          HalfedgeIter h3 = h0->twin();
          HalfedgeIter h4 = h3->next();
          HalfedgeIter h5 = h4->next();
          VertexIter A = h0->vertex();
          VertexIter B = h3->vertex();
          VertexIter C = h2->vertex();
          VertexIter D = h5->vertex();
          e->newPosition = (3.0/8.0) * (A->position + B->position) + (1.0/8.0) * (C->position + D->position);
      }
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
          VertexIter v1 = e->halfedge()->vertex();
          VertexIter v2 = e->halfedge()->twin()->vertex();
          if (v1->isNew == false && v2->isNew == false) {
              VertexIter v = mesh.splitEdge(e);
              v->newPosition = e->newPosition;
          }
      }
    // 4. Flip any new edge that connects an old and new vertex.
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
          VertexIter v1 = e->halfedge()->vertex();
          VertexIter v2 = e->halfedge()->twin()->vertex();
          if (e->isNew && (v1->isNew != v2->isNew)) {
              mesh.flipEdge(e);
          }
      }
    // 5. Copy the new vertex positions into final Vertex::position.
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          v->isNew = false;
          if (v->isBoundary()) {
              continue;
          }
          v->position = v->newPosition;
      }
  }
}
