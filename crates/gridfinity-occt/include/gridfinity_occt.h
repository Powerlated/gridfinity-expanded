#ifndef GRIDFINITY_OCCT_H
#define GRIDFINITY_OCCT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct GfOcctShape GfOcctShape;

/* All constructors return NULL on failure. The error is thread-local and stays
   valid until the next bridge call on the same thread. */
const char* gf_occt_last_error(void);
GfOcctShape* gf_occt_make_box(double dx, double dy, double dz);
GfOcctShape* gf_occt_make_rounded_box(double dx, double dy, double dz, double radius);
/* A truncated cone about +Z, `r0` at z=0 and `r1` at z=h; the one analytic
   surface an extrusion never produces. */
GfOcctShape* gf_occt_make_cone(double r0, double r1, double h);
void gf_occt_shape_free(GfOcctShape* shape);
int gf_occt_shape_is_valid(const GfOcctShape* shape);

/* A profile crosses this ABI as a flat array of segments, ten doubles each:
   {kind, ax, ay, bx, by, cx, cy, radius, a0, a1}. kind 0 is a line from a to b
   and ignores the rest; kind 1 is an arc from a to b about centre c, swept from
   a0 to a1, counter-clockwise when a1 > a0 and clockwise otherwise. `loops`
   gives the segment count of each loop in order, the first being the outer
   boundary and the rest holes in it. Every loop must close. */
GfOcctShape* gf_occt_prism_from_loops(const double* segments, const size_t* loops,
                                      size_t loop_count, double z, double dz);

/* A solid lofted through `section_count` profiles, the i-th laid in the plane
   zs[i]. `loops` gives every loop's segment count across all sections in order
   and `loops_per_section` how many of those loops each section owns; sections
   must agree on that count, and their loops are lofted in the order held. This
   is the peg's four rings as much as any two-profile ruled wall. */
GfOcctShape* gf_occt_loft(const double* segments, const size_t* loops,
                          const size_t* loops_per_section, const double* zs,
                          size_t section_count);

/* op 0 subtracts `b` from `a`, 1 unions them, 2 intersects them. */
GfOcctShape* gf_occt_boolean(const GfOcctShape* a, const GfOcctShape* b, int op);

/* Rounds every edge whose midpoint lies within `tolerance` of one named in
   `edges`, six doubles each: {mx, my, mz, radius, unused, unused}. An edge no
   segment names is left sharp; a named edge the kernel cannot round fails. */
GfOcctShape* gf_occt_fillet_edges(const GfOcctShape* shape, const double* edges,
                                  size_t count, double tolerance);

/* Measured properties, for holding a migrated body to the one it replaces.
   `bounds` is {min_x, min_y, min_z, max_x, max_y, max_z}. */
int gf_occt_volume(const GfOcctShape* shape, double* volume);
int gf_occt_bounds(const GfOcctShape* shape, double* bounds);
int gf_occt_shell_count(const GfOcctShape* shape, size_t* shells);

/* Two-call topology API, the same shape as the mesh one: query counts, allocate
   in Rust, then copy. It states a shape's B-rep in the analytic forms Parasolid
   names, which is what lets a transmit file be written from it.

   `vertices` holds three doubles per vertex. `edges` holds GF_OCCT_EDGE_STRIDE
   doubles per edge: {kind, t0, t1, v0, v1, ...}, where kind 0 is a line
   (p0[3], dir[3]), 1 a circle (centre[3], axis[3], ref[3], radius) and 2 an
   ellipse (centre[3], axis[3], x_axis[3], major, minor), and 3 a section --
   a curve none of those name, carried as (chart_offset, chart_count) into
   `charts`, three doubles per point, running from v0 to v1. t0 and t1 are the
   curve's own parameters at v0 and v1, and 0 and 1 for a section. `faces` holds GF_OCCT_FACE_STRIDE
   doubles per face: {kind, sense, ...}, where kind 0 is a plane (origin[3],
   normal[3], x_axis[3]), 1 a cylinder (base[3], axis[3], ref[3], radius), 2 a
   cone (apex-vector[3], axis[3], ref[3], radius, half_angle), 3 a torus
   (centre[3], axis[3], ref[3], major, minor) and 4 a sphere (centre[3],
   axis[3], ref[3], radius); sense is 1 when the surface's own normal points out
   of the material and 0 when it points in.

   The loops of a face are given by `loops_per_face`, the outer one first; the
   fins of a loop by `fins_per_loop`; and each fin by two entries in `fins`: the
   edge's index, and 1 when the loop runs along the curve's own direction or 0
   when it runs against it.

   A *surface* none of those name -- a lofted or swept B-spline -- is refused
   rather than approximated, as is a loop using one edge twice as a seam on a
   closed surface. A *curve* is not: an edge is the intersection of the two
   faces meeting along it, which the transmit format can state exactly, so a
   section's chart only has to say which branch of that intersection is meant. */
#define GF_OCCT_EDGE_STRIDE 16
#define GF_OCCT_FACE_STRIDE 14
int gf_occt_topology_counts(const GfOcctShape* shape, size_t* vertices, size_t* edges,
                            size_t* faces, size_t* loops, size_t* fins, size_t* chart_points);
int gf_occt_topology_copy(const GfOcctShape* shape, double* vertices, double* edges,
                          double* faces, size_t* loops_per_face, size_t* fins_per_loop,
                          int64_t* fins, double* charts, size_t vertex_count, size_t edge_count,
                          size_t face_count, size_t loop_count, size_t fin_count,
                          size_t chart_point_count);

/* Two-call mesh API: query counts, allocate in Rust, then copy. Positions and
   normals contain three doubles per vertex; indices contain three uint32s per
   triangle. */
int gf_occt_mesh_counts(const GfOcctShape* shape, double deflection,
                        size_t* vertex_count, size_t* index_count);
int gf_occt_mesh_copy(const GfOcctShape* shape, double deflection,
                      double* positions, double* normals, uint32_t* indices,
                      size_t vertex_count, size_t index_count);

#ifdef __cplusplus
}
#endif
#endif
