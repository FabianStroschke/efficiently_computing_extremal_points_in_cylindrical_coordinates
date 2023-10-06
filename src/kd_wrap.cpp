//
// Created by fabia on 22.06.2023.
//

#include "kd_wrap.h"

void
KDWrap(std::vector<Kernel::Point_3> &pointCloud, Mesh &m) {

    if(pointCloud.empty()) return;
    bool coplanarFace = false;

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
             {bbox.max_coord(0),bbox.max_coord(1)+1,bbox.max_coord(2)}
            ,{bbox.max_coord(0),bbox.max_coord(1)+1,bbox.min_coord(2)}
    }; //TODO replace with halfedge
    set.first = **findBoundaryPoint(kd_tree, set, BS_RIGHT, origin).begin();
    set.second = **findBoundaryPoint(kd_tree, set, BS_RIGHT, origin).begin();

    //add face to mesh
    CGAL::SM_Face_index face = m.add_face( m.add_vertex(set.second), m.add_vertex(set.first),m.add_vertex(**findBoundaryPoint(
            kd_tree, set, BS_RIGHT, origin).begin()));
    for(auto &e: m.halfedges_around_face(m.halfedge(face))){
        if(m.is_border(m.opposite(e))) borderEdges.push_back(m.opposite(e));
    }
    //iterate over borderedges
    while(not borderEdges.empty()){
        auto h = borderEdges.back();
        borderEdges.pop_back();
        if (m.is_border(h)){
            auto t = m.point(m.target(h));
            auto s = m.point(m.source(h));

            auto res = findBoundaryPoint(kd_tree, {t,s}, BS_RIGHT, origin);

            if(res.empty()) {//TODO shouldn't happen, throw exception (if it happens s and t arent on the convex hull or the program is broken)
                continue;
            }
            if(res.size()>1) {
                coplanarFace = true;
            }


            std::vector<CGAL::SM_Face_index> faces;
            CGAL::SM_Vertex_index v1 = m.source(h);
            CGAL::SM_Vertex_index v2 = m.target(h);

            Kernel::Vector_3 refVec(m.point(v1), m.point(v2));
            std::map<double, const Kernel::Point_3 *> map;
            for(auto p : res){
                Kernel::Vector_3 vec(*p,m.point(v2));
                map.emplace(acos(refVec * vec / (sqrt(refVec.squared_length()) * sqrt(vec.squared_length()))),p);
            }

            for(auto e: map){
                auto pos = std::find(m.points().begin(), m.points().end(), *e.second);
                CGAL::SM_Vertex_index v0 = (pos == m.points().end()) ? m.add_vertex(*e.second) : static_cast<CGAL::SM_Vertex_index>(pos - m.points().begin());
                faces.emplace_back(m.add_face(v0,v1,v2));
                if(faces.back() == Mesh::null_face() and false) {
                    std::cout << v0 << "|" << v1 << "|" << v2 << std::endl;
                    std::cout << m.add_face(v1,v0,v2) << std::endl;
                    std::cout << (pos == m.points().end() )<< std::endl;
                    for(auto &face: m.faces()){
                        for(auto &half: m.halfedges_around_face(m.halfedge(face))) {
                            if(m.target(half) == v0 or m.target(half) == v1 or m.target(half) == v2){
                                for (auto &half2: m.halfedges_around_face(half)) {
                                    std::cout << m.target(half2) << "|" ;
                                }
                                std::cout << "\n";
                            }
                        }
                    }
                }

                v1 = v0;
            }

            //add border edges of new face
            for(auto f : faces){
                if(f == Mesh::null_face() ){ //TODO if this case is reached, then the algorithm failed. Maybe add an exception
                    if(coplanarFace){
                        std::cout << "Coplanar | ";
                    }
                    std::cout << "err\n";
                    exit(1);
                }else{
                    for (auto &e: m.halfedges_around_face(m.halfedge(f))) {
                        if (m.is_border(m.opposite(e))) borderEdges.push_back(m.opposite(e));
                    }
                }
            }
        }
    }
    if(coplanarFace){
        std::cout << "Coplanar | ";
    }
}
