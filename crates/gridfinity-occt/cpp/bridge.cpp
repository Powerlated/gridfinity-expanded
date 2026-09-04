#include "gridfinity_occt.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_Line.hxx>
#include <Geom_Plane.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_Surface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS_Vertex.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeHalfSpace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <GeomLProp_SLProps.hxx>
#include <TopoDS_Shell.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Vec.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopAbs_Orientation.hxx>
#include <gp_Pnt.hxx>

#include <Precision.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>

#include <cmath>
#include <algorithm>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

struct GfOcctShape { TopoDS_Shape value; };
namespace {
thread_local std::string last_error;
struct Mesh { std::vector<double> p, n; std::vector<uint32_t> i; };

template<class F> auto guarded(F&& f) -> decltype(f()) {
  last_error.clear();
  try { return f(); }
  catch (const Standard_Failure& e) { last_error = e.GetMessageString(); }
  catch (const std::exception& e) { last_error = e.what(); }
  catch (...) { last_error = "unknown C++ exception"; }
  using R = decltype(f());
  return R{};
}


constexpr size_t SEGMENT_STRIDE = 10;
constexpr size_t EDGE_STRIDE = 6;

/* The wire of `count` segments read from `segments`, laid in the plane z. */
TopoDS_Wire wire_of(const double* segments, size_t count, double z) {
  if (count == 0) throw std::invalid_argument("a loop needs at least one segment");
  BRepBuilderAPI_MakeWire wire;
  for (size_t s = 0; s < count; ++s) {
    const double* v = segments + s * SEGMENT_STRIDE;
    const gp_Pnt a(v[1], v[2], z);
    const gp_Pnt b(v[3], v[4], z);
    if (v[0] == 0.0) {
      if (a.IsEqual(b, Precision::Confusion())) continue;
      wire.Add(BRepBuilderAPI_MakeEdge(a, b).Edge());
      continue;
    }
    const bool counter_clockwise = v[9] >= v[8];
    const gp_Dir axis(0.0, 0.0, counter_clockwise ? 1.0 : -1.0);
    const gp_Circ circle(gp_Ax2(gp_Pnt(v[5], v[6], z), axis), v[7]);
    wire.Add(BRepBuilderAPI_MakeEdge(circle, a, b).Edge());
  }
  if (!wire.IsDone()) throw std::runtime_error("the segments of a loop do not form a wire");
  const TopoDS_Wire built = wire.Wire();
  if (!built.Closed()) throw std::runtime_error("a profile loop does not close");
  return built;
}

/* The wires of every loop `loops` describes, read in order from `segments`. */
std::vector<TopoDS_Wire> wires_of(const double* segments, const size_t* loops, size_t loop_count,
                                  double z) {
  if (!segments || !loops || loop_count == 0) throw std::invalid_argument("a profile needs a loop");
  std::vector<TopoDS_Wire> out;
  size_t offset = 0;
  for (size_t l = 0; l < loop_count; ++l) {
    out.push_back(wire_of(segments + offset * SEGMENT_STRIDE, loops[l], z));
    offset += loops[l];
  }
  return out;
}

/* The planar face at height z bounded by the first wire and holed by the rest. */
TopoDS_Face face_of(const std::vector<TopoDS_Wire>& wires, double z) {
  BRepBuilderAPI_MakeFace face(gp_Pln(gp_Pnt(0.0, 0.0, z), gp_Dir(0.0, 0.0, 1.0)), wires.front());
  for (size_t w = 1; w < wires.size(); ++w) face.Add(TopoDS::Wire(wires[w].Reversed()));
  if (!face.IsDone()) throw std::runtime_error("a profile does not bound a planar face");
  return face.Face();
}

/* The solid lofted through one loop of every section, in section order. */
TopoDS_Shape loft_through(const std::vector<TopoDS_Wire>& sections) {
  if (sections.size() < 2) throw std::invalid_argument("a loft needs at least two sections");
  BRepOffsetAPI_ThruSections loft(true, true);
  for (const TopoDS_Wire& wire : sections) loft.AddWire(wire);
  loft.Build();
  if (!loft.IsDone()) throw std::runtime_error("OCCT could not loft through the profiles");
  return loft.Shape();
}

/* The midpoint of `edge`, which is what an edge is named by across this ABI:
   a Gridfinity blend selects the edges it rounds by where they run, and no
   index into an OCCT explorer survives the operation that precedes it. */
gp_Pnt edge_midpoint(const TopoDS_Edge& edge) {
  BRepAdaptor_Curve curve(edge);
  return curve.Value((curve.FirstParameter() + curve.LastParameter()) / 2.0);
}


/* The B-rep of one shape, in the analytic forms a transmit file names. */
struct Topology {
  std::vector<double> vertices, edges, faces, charts;
  std::vector<size_t> loops_per_face, fins_per_loop;
  std::vector<int64_t> fins;
};

/* Chart points across an edge's own parameter range. A chart names which branch
   of two surfaces' intersection an edge is, so it wants to be unambiguous
   rather than fine: the reader recomputes the curve from the exact surfaces. */
constexpr int CHART_POINTS = 9;

void put3(std::vector<double>& out, const gp_Pnt& p) {
  out.insert(out.end(), {p.X(), p.Y(), p.Z()});
}

void put3(std::vector<double>& out, const gp_Dir& d) {
  out.insert(out.end(), {d.X(), d.Y(), d.Z()});
}

/* `record` padded out to `stride`, which keeps every record the same width
   however few numbers its own kind needs. */
void pad_to(std::vector<double>& out, size_t start, size_t stride) {
  if (out.size() - start > stride) throw std::runtime_error("a topology record overran its stride");
  out.resize(start + stride, 0.0);
}

/* One edge's curve appended to `out`, refusing anything the format cannot
   state exactly. */
void write_curve(std::vector<double>& out, const TopoDS_Edge& edge, double& first,
                 double& last) {
  const Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
  if (curve.IsNull()) throw std::runtime_error("an edge with no curve cannot be transmitted");
  if (Handle(Geom_Line) line = Handle(Geom_Line)::DownCast(curve)) {
    out.push_back(0.0);
    put3(out, line->Position().Location());
    put3(out, line->Position().Direction());
    return;
  }
  if (Handle(Geom_Circle) circle = Handle(Geom_Circle)::DownCast(curve)) {
    out.push_back(1.0);
    put3(out, circle->Position().Location());
    put3(out, circle->Position().Direction());
    put3(out, circle->Position().XDirection());
    out.push_back(circle->Radius());
    return;
  }
  if (Handle(Geom_Ellipse) ellipse = Handle(Geom_Ellipse)::DownCast(curve)) {
    out.push_back(2.0);
    put3(out, ellipse->Position().Location());
    put3(out, ellipse->Position().Direction());
    put3(out, ellipse->Position().XDirection());
    out.push_back(ellipse->MajorRadius());
    out.push_back(ellipse->MinorRadius());
    return;
  }
  return;
}

/* `edge` sampled evenly across its parameter range, as the chart of a curve the
   analytic set cannot name. */
std::vector<double> chart_of(const TopoDS_Edge& edge) {
  BRepAdaptor_Curve adaptor(edge);
  const double first = adaptor.FirstParameter();
  const double last = adaptor.LastParameter();
  std::vector<double> out;
  for (int i = 0; i < CHART_POINTS; ++i) {
    const double t = first + (last - first) * (static_cast<double>(i) / (CHART_POINTS - 1));
    const gp_Pnt p = adaptor.Value(t);
    out.insert(out.end(), {p.X(), p.Y(), p.Z()});
  }
  return out;
}

/* One face's surface appended to `out`, refusing anything the format cannot
   state exactly. The sense follows the face's orientation: a REVERSED face is
   one whose surface normal points into the material. */
void write_surface(std::vector<double>& out, const TopoDS_Face& face) {
  const Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
  if (surface.IsNull()) throw std::runtime_error("a face with no surface cannot be transmitted");
  const double sense = face.Orientation() == TopAbs_REVERSED ? 0.0 : 1.0;
  if (Handle(Geom_Plane) plane = Handle(Geom_Plane)::DownCast(surface)) {
    out.insert(out.end(), {0.0, sense});
    put3(out, plane->Position().Location());
    put3(out, plane->Position().Direction());
    put3(out, plane->Position().XDirection());
    return;
  }
  if (Handle(Geom_CylindricalSurface) cylinder =
          Handle(Geom_CylindricalSurface)::DownCast(surface)) {
    out.insert(out.end(), {1.0, sense});
    put3(out, cylinder->Position().Location());
    put3(out, cylinder->Position().Direction());
    put3(out, cylinder->Position().XDirection());
    out.push_back(cylinder->Radius());
    return;
  }
  if (Handle(Geom_ConicalSurface) cone = Handle(Geom_ConicalSurface)::DownCast(surface)) {
    out.insert(out.end(), {2.0, sense});
    put3(out, cone->Position().Location());
    put3(out, cone->Position().Direction());
    put3(out, cone->Position().XDirection());
    out.push_back(cone->RefRadius());
    out.push_back(cone->SemiAngle());
    return;
  }
  if (Handle(Geom_ToroidalSurface) torus = Handle(Geom_ToroidalSurface)::DownCast(surface)) {
    out.insert(out.end(), {3.0, sense});
    put3(out, torus->Position().Location());
    put3(out, torus->Position().Direction());
    put3(out, torus->Position().XDirection());
    out.push_back(torus->MajorRadius());
    out.push_back(torus->MinorRadius());
    return;
  }
  if (Handle(Geom_SphericalSurface) sphere = Handle(Geom_SphericalSurface)::DownCast(surface)) {
    out.insert(out.end(), {4.0, sense});
    put3(out, sphere->Position().Location());
    put3(out, sphere->Position().Direction());
    put3(out, sphere->Position().XDirection());
    out.push_back(sphere->Radius());
    return;
  }
  throw std::runtime_error(
      "only a plane, cylinder, cone, torus or sphere can be transmitted, and this face carries "
      "none of them -- a lofted or swept surface has to be modelled analytically first");
}

Topology topology_of(const TopoDS_Shape& shape) {
  TopTools_IndexedMapOfShape vertex_map, edge_map;
  TopExp::MapShapes(shape, TopAbs_VERTEX, vertex_map);
  TopExp::MapShapes(shape, TopAbs_EDGE, edge_map);

  Topology out;
  for (int v = 1; v <= vertex_map.Extent(); ++v) {
    put3(out.vertices, BRep_Tool::Pnt(TopoDS::Vertex(vertex_map(v))));
  }
  for (int e = 1; e <= edge_map.Extent(); ++e) {
    const TopoDS_Edge edge = TopoDS::Edge(edge_map(e).Oriented(TopAbs_FORWARD));
    const size_t start = out.edges.size();
    double first = 0.0, last = 0.0;
    std::vector<double> curve;
    write_curve(curve, edge, first, last);
    const TopoDS_Vertex v0 = TopExp::FirstVertex(edge);
    const TopoDS_Vertex v1 = TopExp::LastVertex(edge);
    if (v0.IsNull() || v1.IsNull())
      throw std::runtime_error("an edge without both vertices cannot be transmitted");
    const bool sectioned = curve.empty();
    if (sectioned) {
      const std::vector<double> chart = chart_of(edge);
      curve = {3.0, static_cast<double>(out.charts.size() / 3),
               static_cast<double>(chart.size() / 3)};
      out.charts.insert(out.charts.end(), chart.begin(), chart.end());
      first = 0.0;
      last = 1.0;
    }
    out.edges.push_back(curve.front());
    out.edges.push_back(first);
    out.edges.push_back(last);
    out.edges.push_back(static_cast<double>(vertex_map.FindIndex(v0) - 1));
    out.edges.push_back(static_cast<double>(vertex_map.FindIndex(v1) - 1));
    out.edges.insert(out.edges.end(), curve.begin() + 1, curve.end());
    pad_to(out.edges, start, GF_OCCT_EDGE_STRIDE);
  }
  for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
    const TopoDS_Face face = TopoDS::Face(ex.Current());
    const size_t start = out.faces.size();
    write_surface(out.faces, face);
    pad_to(out.faces, start, GF_OCCT_FACE_STRIDE);

    const TopoDS_Wire outer = BRepTools::OuterWire(face);
    std::vector<TopoDS_Wire> wires{outer};
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
      const TopoDS_Wire wire = TopoDS::Wire(wx.Current());
      if (!wire.IsSame(outer)) wires.push_back(wire);
    }
    out.loops_per_face.push_back(wires.size());
    for (const TopoDS_Wire& wire : wires) {
      size_t fins = 0;
      std::vector<int> seen;
      for (BRepTools_WireExplorer we(wire, face); we.More(); we.Next()) {
        const TopoDS_Edge edge = we.Current();
        const int index = edge_map.FindIndex(edge);
        if (index == 0) throw std::runtime_error("a loop names an edge the shape does not have");
        if (std::find(seen.begin(), seen.end(), index) != seen.end())
          throw std::runtime_error(
              "a loop uses one edge twice, which is a seam on a closed surface and needs a "
              "split before it can be transmitted");
        seen.push_back(index);
        out.fins.push_back(index - 1);
        out.fins.push_back(edge.Orientation() == TopAbs_REVERSED ? 0 : 1);
        ++fins;
      }
      if (fins == 0) throw std::runtime_error("a loop with no edges cannot be transmitted");
      out.fins_per_loop.push_back(fins);
    }
  }
  if (out.faces.empty()) throw std::runtime_error("a body with no faces cannot be transmitted");
  return out;
}

Mesh mesh_of(const TopoDS_Shape& shape, double deflection) {
  BRepMesh_IncrementalMesh mesher(shape, deflection, false, 0.35, true);
  mesher.Perform();
  if (!mesher.IsDone()) throw std::runtime_error("OCCT tessellation failed");
  Mesh out;
  for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
    const TopoDS_Face face = TopoDS::Face(ex.Current());
    TopLoc_Location loc;
    const Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
    if (tri.IsNull()) continue;
    const uint32_t base = static_cast<uint32_t>(out.p.size() / 3);
    const gp_Trsf transform = loc.Transformation();
    const Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    const bool analytic = !surface.IsNull() && tri->HasUVNodes();
    const double flip = face.Orientation() == TopAbs_REVERSED ? -1.0 : 1.0;
    for (int node = 1; node <= tri->NbNodes(); ++node) {
      gp_Pnt p = tri->Node(node).Transformed(transform);
      out.p.insert(out.p.end(), {p.X(), p.Y(), p.Z()});
      gp_Vec normal(0.0, 0.0, 0.0);
      if (analytic) {
        const gp_Pnt2d uv = tri->UVNode(node);
        GeomLProp_SLProps props(surface, uv.X(), uv.Y(), 1, Precision::Confusion());
        if (props.IsNormalDefined()) normal = gp_Vec(props.Normal()).Transformed(transform) * flip;
      }
      out.n.insert(out.n.end(), {normal.X(), normal.Y(), normal.Z()});
    }
    for (int t = 1; t <= tri->NbTriangles(); ++t) {
      int a, b, c; tri->Triangle(t).Get(a, b, c);
      if (face.Orientation() == TopAbs_REVERSED) std::swap(b, c);
      uint32_t ia = base + static_cast<uint32_t>(a - 1);
      uint32_t ib = base + static_cast<uint32_t>(b - 1);
      uint32_t ic = base + static_cast<uint32_t>(c - 1);
      out.i.insert(out.i.end(), {ia, ib, ic});
      const double* pa = &out.p[3 * ia]; const double* pb = &out.p[3 * ib]; const double* pc = &out.p[3 * ic];
      double ux=pb[0]-pa[0], uy=pb[1]-pa[1], uz=pb[2]-pa[2];
      double vx=pc[0]-pa[0], vy=pc[1]-pa[1], vz=pc[2]-pa[2];
      double nx=uy*vz-uz*vy, ny=uz*vx-ux*vz, nz=ux*vy-uy*vx;
      double len=std::sqrt(nx*nx+ny*ny+nz*nz);
      if (len > 0) { nx/=len; ny/=len; nz/=len; }
      if (!analytic)
        for (uint32_t v : {ia, ib, ic}) { out.n[3*v]+=nx; out.n[3*v+1]+=ny; out.n[3*v+2]+=nz; }
    }
  }
  for (size_t v=0; v<out.n.size()/3; ++v) {
    double* n=&out.n[3*v]; double len=std::sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]);
    if (len > 0) { n[0]/=len; n[1]/=len; n[2]/=len; }
  }
  return out;
}
}

extern "C" const char* gf_occt_last_error(void) { return last_error.c_str(); }
extern "C" GfOcctShape* gf_occt_make_box(double dx,double dy,double dz) {
  return guarded([&]() -> GfOcctShape* { if(dx<=0||dy<=0||dz<=0) throw std::invalid_argument("box dimensions must be positive"); return new GfOcctShape{BRepPrimAPI_MakeBox(dx,dy,dz).Shape()}; });
}
extern "C" GfOcctShape* gf_occt_make_rounded_box(double dx,double dy,double dz,double radius) {
  return guarded([&]() -> GfOcctShape* {
    TopoDS_Shape box=BRepPrimAPI_MakeBox(dx,dy,dz).Shape();
    if(radius<=0) return new GfOcctShape{box};
    BRepFilletAPI_MakeFillet fillet(box);
    for(TopExp_Explorer ex(box,TopAbs_EDGE);ex.More();ex.Next()) fillet.Add(radius,TopoDS::Edge(ex.Current()));
    fillet.Build(); if(!fillet.IsDone()) throw std::runtime_error("OCCT fillet failed");
    return new GfOcctShape{fillet.Shape()};
  });
}
extern "C" GfOcctShape* gf_occt_make_cone(double r0, double r1, double h) {
  return guarded([&]() -> GfOcctShape* {
    if (h <= 0 || r0 < 0 || r1 < 0 || r0 == r1)
      throw std::invalid_argument("a cone needs a positive height and two different radii");
    return new GfOcctShape{BRepPrimAPI_MakeCone(r0, r1, h).Shape()};
  });
}
extern "C" void gf_occt_shape_free(GfOcctShape* p) { delete p; }
extern "C" int gf_occt_shape_is_valid(const GfOcctShape* p) { return guarded([&](){ return p && BRepCheck_Analyzer(p->value,true).IsValid() ? 1 : 0; }); }
extern "C" int gf_occt_mesh_counts(const GfOcctShape* p,double d,size_t* nv,size_t* ni) { return guarded([&](){ if(!p||!nv||!ni||d<=0) throw std::invalid_argument("invalid mesh arguments"); Mesh m=mesh_of(p->value,d); *nv=m.p.size()/3; *ni=m.i.size(); return 1; }); }
extern "C" int gf_occt_mesh_copy(const GfOcctShape* p,double d,double* pos,double* norm,uint32_t* idx,size_t nv,size_t ni) { return guarded([&](){ if(!p||!pos||!norm||!idx||d<=0) throw std::invalid_argument("invalid mesh arguments"); Mesh m=mesh_of(p->value,d); if(m.p.size()!=3*nv||m.i.size()!=ni) throw std::invalid_argument("mesh buffer size changed"); std::copy(m.p.begin(),m.p.end(),pos); std::copy(m.n.begin(),m.n.end(),norm); std::copy(m.i.begin(),m.i.end(),idx); return 1; }); }

extern "C" GfOcctShape* gf_occt_prism_from_loops(const double* segments, const size_t* loops,
                                                 size_t loop_count, double z, double dz) {
  return guarded([&]() -> GfOcctShape* {
    if (dz == 0.0) throw std::invalid_argument("a prism needs a non-zero height");
    const TopoDS_Face face = face_of(wires_of(segments, loops, loop_count, z), z);
    return new GfOcctShape{BRepPrimAPI_MakePrism(face, gp_Vec(0.0, 0.0, dz)).Shape()};
  });
}

extern "C" GfOcctShape* gf_occt_loft(const double* segments, const size_t* loops,
                                     const size_t* loops_per_section, const double* zs,
                                     size_t section_count) {
  return guarded([&]() -> GfOcctShape* {
    if (!segments || !loops || !loops_per_section || !zs || section_count < 2)
      throw std::invalid_argument("a loft needs at least two sections");
    const size_t per_section = loops_per_section[0];
    if (per_section == 0) throw std::invalid_argument("a loft section needs a loop");
    std::vector<std::vector<TopoDS_Wire>> sections;
    size_t loop_offset = 0;
    size_t segment_offset = 0;
    for (size_t s = 0; s < section_count; ++s) {
      if (loops_per_section[s] != per_section)
        throw std::invalid_argument("every loft section needs the same loop count");
      sections.push_back(
          wires_of(segments + segment_offset * SEGMENT_STRIDE, loops + loop_offset, per_section, zs[s]));
      for (size_t l = 0; l < per_section; ++l) segment_offset += loops[loop_offset + l];
      loop_offset += per_section;
    }
    std::vector<TopoDS_Wire> one;
    for (const std::vector<TopoDS_Wire>& section : sections) one.push_back(section[0]);
    TopoDS_Shape solid = loft_through(one);
    for (size_t l = 1; l < per_section; ++l) {
      std::vector<TopoDS_Wire> hole;
      for (const std::vector<TopoDS_Wire>& section : sections) hole.push_back(section[l]);
      BRepAlgoAPI_Cut cut(solid, loft_through(hole));
      if (!cut.IsDone()) throw std::runtime_error("OCCT could not cut a lofted hole");
      solid = cut.Shape();
    }
    return new GfOcctShape{solid};
  });
}

extern "C" GfOcctShape* gf_occt_cut_half_space(const GfOcctShape* shape, double ox, double oy,
                                              double oz, double nx, double ny, double nz) {
  return guarded([&]() -> GfOcctShape* {
    if (!shape) throw std::invalid_argument("a half-space cut needs a shape");
    const gp_Vec n(nx, ny, nz);
    if (n.Magnitude() <= Precision::Confusion())
      throw std::invalid_argument("a half-space cut needs a non-zero normal");
    const gp_Pnt origin(ox, oy, oz);
    const gp_Dir direction(n);
    const TopoDS_Face plane = BRepBuilderAPI_MakeFace(gp_Pln(origin, direction)).Face();
    /* The half-space contains its reference point, so a point one unit along
       the normal names the material to remove and the cut keeps the rest. */
    const gp_Pnt discarded = origin.Translated(gp_Vec(direction));
    const TopoDS_Shape half = BRepPrimAPI_MakeHalfSpace(plane, discarded).Solid();
    BRepAlgoAPI_Cut cut(shape->value, half);
    if (!cut.IsDone()) throw std::runtime_error("OCCT could not cut against a plane");
    return new GfOcctShape{cut.Shape()};
  });
}

extern "C" GfOcctShape* gf_occt_boolean(const GfOcctShape* a, const GfOcctShape* b, int op) {
  return guarded([&]() -> GfOcctShape* {
    if (!a || !b) throw std::invalid_argument("a boolean needs two shapes");
    switch (op) {
      case 0: {
        BRepAlgoAPI_Cut cut(a->value, b->value);
        if (!cut.IsDone()) throw std::runtime_error("OCCT could not subtract two shapes");
        return new GfOcctShape{cut.Shape()};
      }
      case 1: {
        BRepAlgoAPI_Fuse fuse(a->value, b->value);
        if (!fuse.IsDone()) throw std::runtime_error("OCCT could not unite two shapes");
        return new GfOcctShape{fuse.Shape()};
      }
      case 2: {
        BRepAlgoAPI_Common common(a->value, b->value);
        if (!common.IsDone()) throw std::runtime_error("OCCT could not intersect two shapes");
        return new GfOcctShape{common.Shape()};
      }
      default:
        throw std::invalid_argument("a boolean op is 0 (cut), 1 (fuse) or 2 (common)");
    }
  });
}

extern "C" GfOcctShape* gf_occt_fillet_edges(const GfOcctShape* shape, const double* edges,
                                             size_t count, double tolerance) {
  return guarded([&]() -> GfOcctShape* {
    if (!shape || !edges || count == 0) throw std::invalid_argument("a fillet needs edges");
    if (tolerance <= 0.0) throw std::invalid_argument("a fillet tolerance must be positive");
    BRepFilletAPI_MakeFillet fillet(shape->value);
    std::vector<bool> found(count, false);
    for (TopExp_Explorer ex(shape->value, TopAbs_EDGE); ex.More(); ex.Next()) {
      const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
      const gp_Pnt mid = edge_midpoint(edge);
      for (size_t e = 0; e < count; ++e) {
        const double* v = edges + e * EDGE_STRIDE;
        if (mid.Distance(gp_Pnt(v[0], v[1], v[2])) > tolerance) continue;
        fillet.Add(v[3], edge);
        found[e] = true;
        break;
      }
    }
    for (size_t e = 0; e < count; ++e) {
      if (!found[e]) throw std::runtime_error("a filleted edge is not an edge of the shape");
    }
    fillet.Build();
    if (!fillet.IsDone()) throw std::runtime_error("OCCT could not round every requested edge");
    return new GfOcctShape{fillet.Shape()};
  });
}

extern "C" int gf_occt_volume(const GfOcctShape* shape, double* volume) {
  return guarded([&]() {
    if (!shape || !volume) throw std::invalid_argument("invalid volume arguments");
    GProp_GProps props;
    BRepGProp::VolumeProperties(shape->value, props);
    *volume = props.Mass();
    return 1;
  });
}

extern "C" int gf_occt_bounds(const GfOcctShape* shape, double* bounds) {
  return guarded([&]() {
    if (!shape || !bounds) throw std::invalid_argument("invalid bounds arguments");
    Bnd_Box box;
    BRepBndLib::Add(shape->value, box);
    if (box.IsVoid()) throw std::runtime_error("an empty shape has no bounds");
    box.Get(bounds[0], bounds[1], bounds[2], bounds[3], bounds[4], bounds[5]);
    return 1;
  });
}

extern "C" int gf_occt_shell_count(const GfOcctShape* shape, size_t* shells) {
  return guarded([&]() {
    if (!shape || !shells) throw std::invalid_argument("invalid shell arguments");
    size_t found = 0;
    for (TopExp_Explorer ex(shape->value, TopAbs_SHELL); ex.More(); ex.Next()) ++found;
    *shells = found;
    return 1;
  });
}

extern "C" int gf_occt_shell_volumes(const GfOcctShape* shape, double* volumes, size_t count) {
  return guarded([&]() {
    if (!shape || !volumes) throw std::invalid_argument("invalid shell volume arguments");
    size_t found = 0;
    for (TopExp_Explorer ex(shape->value, TopAbs_SHELL); ex.More(); ex.Next()) {
      if (found == count) throw std::invalid_argument("the shape has more shells than the buffer");
      /* A shell carries its own orientation, and a solid made of it alone
         integrates that orientation: a shell bounding material measures its
         volume, a shell bounding a void measures the negative of the void's. */
      const TopoDS_Solid solid = BRepBuilderAPI_MakeSolid(TopoDS::Shell(ex.Current())).Solid();
      GProp_GProps props;
      BRepGProp::VolumeProperties(solid, props);
      volumes[found++] = props.Mass();
    }
    if (found != count) throw std::invalid_argument("the shape has fewer shells than the buffer");
    return 1;
  });
}

extern "C" int gf_occt_edge_count(const GfOcctShape* shape, size_t* edges) {
  return guarded([&]() {
    if (!shape || !edges) throw std::invalid_argument("invalid edge count arguments");
    TopTools_IndexedMapOfShape map;
    TopExp::MapShapes(shape->value, TopAbs_EDGE, map);
    *edges = static_cast<size_t>(map.Extent());
    return 1;
  });
}

extern "C" int gf_occt_edge_midpoints(const GfOcctShape* shape, double* midpoints, size_t count) {
  return guarded([&]() {
    if (!shape || !midpoints) throw std::invalid_argument("invalid edge midpoint arguments");
    TopTools_IndexedMapOfShape map;
    TopExp::MapShapes(shape->value, TopAbs_EDGE, map);
    if (static_cast<size_t>(map.Extent()) != count)
      throw std::invalid_argument("the shape's edge count changed between the two calls");
    for (size_t e = 0; e < count; ++e) {
      const gp_Pnt mid = edge_midpoint(TopoDS::Edge(map(static_cast<int>(e) + 1)));
      midpoints[3 * e] = mid.X();
      midpoints[3 * e + 1] = mid.Y();
      midpoints[3 * e + 2] = mid.Z();
    }
    return 1;
  });
}

extern "C" int gf_occt_topology_counts(const GfOcctShape* shape, size_t* vertices, size_t* edges,
                                       size_t* faces, size_t* loops, size_t* fins,
                                       size_t* chart_points) {
  return guarded([&]() {
    if (!shape || !vertices || !edges || !faces || !loops || !fins || !chart_points)
      throw std::invalid_argument("invalid topology arguments");
    const Topology t = topology_of(shape->value);
    *vertices = t.vertices.size() / 3;
    *edges = t.edges.size() / GF_OCCT_EDGE_STRIDE;
    *faces = t.faces.size() / GF_OCCT_FACE_STRIDE;
    *loops = t.fins_per_loop.size();
    *fins = t.fins.size() / 2;
    *chart_points = t.charts.size() / 3;
    return 1;
  });
}

extern "C" int gf_occt_topology_copy(const GfOcctShape* shape, double* vertices, double* edges,
                                     double* faces, size_t* loops_per_face, size_t* fins_per_loop,
                                     int64_t* fins, double* charts, size_t vertex_count,
                                     size_t edge_count, size_t face_count, size_t loop_count,
                                     size_t fin_count, size_t chart_point_count) {
  return guarded([&]() {
    if (!shape || !vertices || !edges || !faces || !loops_per_face || !fins_per_loop || !fins ||
        !charts)
      throw std::invalid_argument("invalid topology arguments");
    const Topology t = topology_of(shape->value);
    if (t.vertices.size() != 3 * vertex_count || t.edges.size() != GF_OCCT_EDGE_STRIDE * edge_count ||
        t.faces.size() != GF_OCCT_FACE_STRIDE * face_count ||
        t.loops_per_face.size() != face_count || t.fins_per_loop.size() != loop_count ||
        t.fins.size() != 2 * fin_count || t.charts.size() != 3 * chart_point_count)
      throw std::invalid_argument("topology buffer size changed");
    std::copy(t.charts.begin(), t.charts.end(), charts);
    std::copy(t.vertices.begin(), t.vertices.end(), vertices);
    std::copy(t.edges.begin(), t.edges.end(), edges);
    std::copy(t.faces.begin(), t.faces.end(), faces);
    std::copy(t.loops_per_face.begin(), t.loops_per_face.end(), loops_per_face);
    std::copy(t.fins_per_loop.begin(), t.fins_per_loop.end(), fins_per_loop);
    std::copy(t.fins.begin(), t.fins.end(), fins);
    return 1;
  });
}
