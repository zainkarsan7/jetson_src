#include "hb_robot_perception/profile_mesh_builder.hpp"
#include <CGAL/Polygon_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/extrude.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/boost/graph/iterator.h>
#include <boost/graph/graph_traits.hpp>
#include <cmath>
#include <vector>
#include <variant>
#include <limits>
#include <cstdint>
#include <map>




namespace hb_perception{

    namespace {


        using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
        using Point2 = Kernel::Point_2;
        using Point3 = Kernel::Point_3;
        using Polygon2 = CGAL::Polygon_2<Kernel>;
        using SurfaceMesh = CGAL::Surface_mesh<Point3>;
        namespace PMP = CGAL::Polygon_mesh_processing;


        bool samePoint (const Eigen::Vector2f& a, const Eigen::Vector2f& b){
            constexpr float tol = 1e-6f;
            return (a- b).squaredNorm() < tol*tol;

        }
    }
    
    std::optional<shape_msgs::msg::Mesh> ProfileMeshBuilder::extrude( const ProfileModel& profile,
            double z_min,
            double z_max
        )const{

            if(!std::isfinite(z_min) || !std::isfinite(z_max)|| 
            z_max < z_min || profile.boundary.size()<3
        ){
                return std::nullopt;
            }
            Polygon2 polygon;
            std::vector<LineSegment2D> lines;
            lines.reserve(profile.boundary.size());

            for(const auto& l : lines){
                polygon.push_back(Point2(l.a.x(),l.a.y()));
            }
            if(!polygon.is_simple() || polygon.orientation() == CGAL::COLLINEAR){
                return std::nullopt;
            }
            if(polygon.is_clockwise_oriented()){
                polygon.reverse_orientation();
            }
            SurfaceMesh section;
            std::vector<SurfaceMesh::Vertex_index> boundary;
            boundary.reserve(polygon.size());
            for(auto it = polygon.vertices_begin(); it!=polygon.vertices_end(); ++it){
                boundary.push_back(section.add_vertex(Point3(it->x(),it->y(),z_min)));
            }
            if(section.add_face(boundary) == SurfaceMesh::null_face()){
                std::cout<<"null face couldnt add"<<std::endl;
                return std::nullopt;
            }
            if(!PMP::triangulate_faces(section)|| !CGAL::is_triangle_mesh(section)){
                return std::nullopt;
            }
            SurfaceMesh solid;
            PMP::extrude_mesh(section,solid, Kernel::Vector_3(0,0,z_max- z_min));
            if(!CGAL::is_closed(solid)||!CGAL::is_triangle_mesh(solid)) return std::nullopt;

            shape_msgs::msg::Mesh result;
            result.vertices.reserve(solid.number_of_vertices());
            result.triangles.reserve(solid.number_of_faces());


            std::map<SurfaceMesh::Vertex_index, std::uint32_t> vert_inds;
            for(const auto v:solid.vertices()){
                const auto& p = solid.point(v);
                geometry_msgs::msg::Point vertex;
                vertex.x = CGAL::to_double(p.x());
                vertex.y = CGAL::to_double(p.y());
                vertex.z = CGAL::to_double(p.z());

                vert_inds[v] =static_cast<std::uint32_t>(result.vertices.size());
                result.vertices.push_back(vertex);
            }
            for (const auto f: solid.faces()){
                shape_msgs::msg::MeshTriangle triangle;
                std::size_t i = 0;
                for (const auto v: CGAL::vertices_around_face(solid.halfedge(f),solid
            )){
                if(i>=3) return std::nullopt;
                triangle.vertex_indices[i++] = vert_inds.at(v);
            }
            if(i!=3)return std::nullopt;
            result.triangles.push_back(triangle);

            
            }
            return result;
        }
    
}