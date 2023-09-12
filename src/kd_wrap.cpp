//
// Created by fabia on 22.06.2023.
//

#include "kd_wrap.h"

void
KDWrap(std::vector<Kernel::Point_3> &pointCloud, Mesh &m) {

    if(pointCloud.empty()) return;

    std::vector<Kernel::Point_3> res;
    /** Building the KDtree **/
    Kd_tree kd_tree(pointCloud.begin(),pointCloud.end(),Kd_tree::Splitter(15));
    kd_tree.build();

    /** Solving the Problem here **/
    std::vector<CGAL::SM_Halfedge_index> borderEdges;


    Kernel::Point_3 origin(0,0,0);
    for(auto &p: kd_tree){
        origin = {origin.x()+p.x()/kd_tree.size(),origin.y()+p.y()/kd_tree.size(),origin.z()+p.z()/kd_tree.size()};
    }

    //find first face
    //CGAL::Kd_tree_rectangle<Traits::FT,CGAL::internal::Get_dimension_tag<Traits>::Dimension>
    const CGAL::Kd_tree_rectangle<double, Traits::Dimension>& bbox(kd_tree.bounding_box());
    std::pair<Kernel::Point_3,Kernel::Point_3> set = {
             {bbox.max_coord(0),bbox.max_coord(1),bbox.max_coord(2)}
            ,{bbox.max_coord(0),bbox.max_coord(1),bbox.min_coord(2)}
    }; //TODO replace with halfedge
    set.first = *findBoundaryPoint(kd_tree, set, BS_RIGHT, origin);
    set.second = *findBoundaryPoint(kd_tree, set, BS_RIGHT, origin);

    //add face to mesh
    CGAL::SM_Face_index f = m.add_face(m.add_vertex(set.first), m.add_vertex(set.second), m.add_vertex(*findBoundaryPoint(
            kd_tree, set, BS_RIGHT, origin)));
    for(auto &e: m.halfedges_around_face(m.halfedge(f))){
        if(m.is_border(m.opposite(e))) borderEdges.push_back(m.opposite(e));
    }
    //iterate over borderedges
    while(not borderEdges.empty()){
        auto h = borderEdges.back();
        borderEdges.pop_back();
        if (m.is_border(h)){
            auto t = m.point(m.target(h));
            auto s = m.point(m.source(h));

            auto p = findBoundaryPoint(kd_tree, {s, t}, BS_RIGHT, origin);

            if(p == nullptr) {//TODO shouldn't happen, throw exception (if it happens s and t arent on the convex hull or the program is broken)
                continue;
            }

            //find index on mesh; max O(h log(h)) with h = points on convexhull ?
            //TODO add indices to octree, when they are added to the mesh
            auto pos = std::find(m.points().begin(), m.points().end(), *p);
            if (pos == m.points().end()) { //point not on mesh
                f = m.add_face(m.add_vertex(*p), m.source(h), m.target(h));
            } else { //point already on mesh
                f = m.add_face(static_cast<CGAL::SM_Vertex_index>(pos - m.points().begin()),
                               m.source(h), m.target(h));

                if(f == Mesh::null_face() ){ //TODO if this case is reached, then the algorithm failed. Maybe add an exception
                    std::cout << s <<"\n";
                    std::cout << t <<"\n";
                    std::cout << *p <<"\n";

                    std::cout << static_cast<CGAL::SM_Vertex_index>(pos - m.points().begin()) <<"|"<< m.source(h) <<"|"<< m.target(h)<<"\n";
                    std::cout << "---------------"<<"\n";
                    for(auto &face: m.faces()){
                        for(auto &half: m.halfedges_around_face(m.halfedge(face))) {
                            std::cout << m.target(half) << "|" ;
                        }
                        std::cout << "\n";
                    }
                    break;
                }
            }

            //add border edges of new face
            for (auto &e: m.halfedges_around_face(m.halfedge(f))) {
                if (m.is_border(m.opposite(e))) borderEdges.push_back(m.opposite(e));
            }
        }
    }
}
