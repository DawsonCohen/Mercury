#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Alpha_shape_3.h>
#include <CGAL/Alpha_shape_cell_base_3.h>
#include <CGAL/Alpha_shape_vertex_base_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>

#include <fstream>
#include <list>
#include <cassert>

#include "triangulation.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

typedef CGAL::Triangulation_vertex_base_with_info_3<uint16_t, K>    Vb;
typedef CGAL::Alpha_shape_vertex_base_3<K,Vb>                       Vb_alpha;
typedef CGAL::Alpha_shape_cell_base_3<K>                            Fb;
typedef CGAL::Triangulation_data_structure_3<Vb_alpha, Fb>          Tds;
typedef CGAL::Delaunay_triangulation_3<K, Tds>                      Delaunay_3;
typedef CGAL::Alpha_shape_3<Delaunay_3>                             Alpha_shape_3;

typedef K::Point_3                                       Point;
typedef Alpha_shape_3::Vertex                            Vertex;
typedef Alpha_shape_3::Edge                              Edge;
typedef Alpha_shape_3::Facet                             Facet;
typedef Alpha_shape_3::Alpha_iterator                    Alpha_iterator;
typedef Alpha_shape_3::FT                                FT;

typedef Tds::Cell_handle                                 Cell_handle;
typedef Tds::Vertex_handle                               Vertex_handle;

Triangulation::Mesh Triangulation::AlphaShape(const std::vector<Mass>& masses) {
    std::list<std::pair<Point, uint16_t>> lp;
    Point p;

    for(const auto& m : masses)
    {
        if(m.material == materials::air) continue;
        p = Point(m.pos.x(), m.pos.y(), m.pos.z());
        lp.push_back({p,m.id});
    }

    std::vector<Simplex::Edge> edges;
    std::vector<Simplex::Facet> facets;
    std::vector<Simplex::Cell> cells;
    std::vector<bool> isBoundaryVertexFlags(masses.size(), false);
    if(lp.size() < 4) {
        return {edges, facets, cells, isBoundaryVertexFlags};
    }

    for (const auto& point : lp) {
        assert(point.first.dimension() == 3);
    }

    // Create a Delaunay triangulation
    // Delaunay_3 dt(lp.begin(), lp.end());

    // Create an alpha shape
    Alpha_shape_3 as(lp.begin(), lp.end());
    assert(as.dimension() == 3);
    
    // find optimal alpha values
    Alpha_iterator opt = as.find_optimal_alpha(1);
    if(opt != as.alpha_end()) {
        as.set_alpha((*opt));
    }

    std::list<Edge>  as_edges;
    std::list<Facet> as_facets;
    std::list<Cell_handle>  as_cells;
    as.get_alpha_shape_edges(std::back_inserter(as_edges),
                        Alpha_shape_3::REGULAR);
    as.get_alpha_shape_edges(std::back_inserter(as_edges),
                        Alpha_shape_3::INTERIOR); 

    as.get_alpha_shape_facets(std::back_inserter(as_facets),
                        Alpha_shape_3::REGULAR);
                        
    as.get_alpha_shape_cells(std::back_inserter(as_cells),
                        Alpha_shape_3::INTERIOR);


    for(const auto& e : as_edges) {
        // std::cout << e.second << ", " << e.third << std::endl;
        uint16_t v0 = e.first->vertex(e.second)->info();
        uint16_t v1 = e.first->vertex(e.third)->info();
        float dist = (masses[v0].pos - masses[v1].pos).norm();
        Simplex::Edge edge = {v0, v1, dist};
        edges.push_back(edge);
    }

    for(const auto& f : as_facets) {
        // std::cout << e.second << ", " << e.third << std::endl;
        uint16_t v0 = f.first->vertex((f.second+1)%4)->info();
        uint16_t v1 = f.first->vertex((f.second+2)%4)->info();
        uint16_t v2 = f.first->vertex((f.second+3)%4)->info();

        isBoundaryVertexFlags[v0] = true;
        isBoundaryVertexFlags[v1] = true;
        isBoundaryVertexFlags[v2] = true;
        Simplex::Facet facet = {v0, v1, v2};
        facets.push_back(facet);
    }

    for(const auto& c : as_cells) {
        uint16_t v0 = c->vertex(0)->info();
        uint16_t v1 = c->vertex(1)->info();
        uint16_t v2 = c->vertex(2)->info();
        uint16_t v3 = c->vertex(3)->info();

        Simplex::Cell cell = {v0, v1, v2, v3};
        cells.push_back(cell);
    }

    return {edges, facets, cells, isBoundaryVertexFlags};
}
