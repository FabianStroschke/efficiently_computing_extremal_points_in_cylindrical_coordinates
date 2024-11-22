//
// Created by fabia on 22.06.2023.
//

#include "octree_wrap.h"

void
octreeWrap(std::vector<Kernel::Point_3> &pointCloud, Mesh &m){
    /** Building the Octree **/

    Octree octree(pointCloud);
    octree.refine(50, 2);
    octreeWrap(octree, m);
}

void
octreeWrap(Octree &octree, Mesh &m) {

    if(octree.root().empty()) return;
    bool coplanarFace = false;

    std::vector<CGAL::SM_Halfedge_index> borderEdges;

    Kernel::Point_3 origin(0,0,0);
    for(auto &p: octree.root()){
        origin = {origin.x()+p.x()/octree.root().size(),origin.y()+p.y()/octree.root().size(),origin.z()+p.z()/octree.root().size()};
    }

    //find first face
    auto bbox = octree.bbox(octree.root());
    std::pair<Kernel::Point_3,Kernel::Point_3> set = {
            {bbox.xmax()*2,bbox.ymax()*2,bbox.zmax()},
            {bbox.xmax()*2,bbox.ymax()*2,bbox.zmin()}}; //TODO replace with halfedge
    set.first = **findExtremalPoint(octree, set, BS_RIGHT, origin).begin();
    set.second = **findExtremalPoint(octree, set, BS_RIGHT, origin).begin();

    //add edge to mesh
    CGAL::SM_Halfedge_index initialEdge = m.add_edge( m.add_vertex(set.second), m.add_vertex(set.first));
    borderEdges.push_back(initialEdge);

    //iterate over borderedges
    while(not borderEdges.empty()){
        auto h = borderEdges.back();
        borderEdges.pop_back();
        if (m.is_border(h)){
            auto t = m.point(m.target(h));
            auto s = m.point(m.source(h));

            auto res = findExtremalPoint(octree, {t, s}, BS_RIGHT, origin);

            if(res.empty()) {//TODO shouldn't happen, throw exception (if it happens s and t arent on the convex hull or the program is broken)
                continue;
            }

            std::vector<CGAL::SM_Face_index> faces;
            if(res.size()>1) {
                coplanarFace = true;
                /*long count = 0;
                for (auto p: pointCloud) {
                    if (CGAL::coplanar(t, s, **res.begin(), p)) {
                        count++;
                    }
                }
                if (count > res.size() + 2) {
                    std::cout << "err\n";
                    exit(1);
                    std::cout<<"_________________________"<<std::endl;

                    for (auto p: pointCloud) {
                        if (CGAL::coplanar(t, s, **res.begin(), p)) {
                            std::cout << p << std::endl;
                        }
                    }
                    auto resAlt = findExtremalPoint(kd_tree, {s, t}, BS_RIGHT, origin);
                    std::cout << count << "|" << res.size() + 2 << "|" << resAlt.size() << std::endl;
                }*/

                //calc base vectors
                Point_3 p0 = m.point(m.source(h));
                Point_3 p1 = m.point(m.target(h));
                Point_3 p2 = **res.begin();
                Kernel::Vector_3 A(p1, p0);
                Kernel::Vector_3 B(p2, p0);
                Kernel::Vector_3 N = CGAL::cross_product(A, B);
                A = A / sqrt(A.squared_length());
                B = CGAL::cross_product(A, N / sqrt(N.squared_length()));

                //create triangulation and a map between mesh and T
                std::map<Triangulation::Vertex_handle, CGAL::SM_Vertex_index> map;
                Triangulation T;

                //add p0 and p1 to triangulation
                auto pos = std::find(m.points().begin(), m.points().end(), p0);
                map.emplace(
                        T.insert({(p0 - CGAL::ORIGIN) * A, (p0 - CGAL::ORIGIN) * B}),
                        (pos == m.points().end()) ? m.add_vertex(p0) : static_cast<CGAL::SM_Vertex_index>(pos -
                                                                                                          m.points().begin()));
                pos = std::find(m.points().begin(), m.points().end(), p1);
                map.emplace(
                        T.insert({(p1 - CGAL::ORIGIN) * A, (p1 - CGAL::ORIGIN) * B}),
                        (pos == m.points().end()) ? m.add_vertex(p1) : static_cast<CGAL::SM_Vertex_index>(pos -
                                                                                                          m.points().begin()));
                //add res to triangulation
                for (auto p: res) {
                    pos = std::find(m.points().begin(), m.points().end(), *p);
                    auto x = (*p) - CGAL::ORIGIN;
                    Kernel::Point_2 P(x * A, x * B);
                    auto a = T.insert(P);
                    auto b = (pos == m.points().end()) ? m.add_vertex(*p) : static_cast<CGAL::SM_Vertex_index>(pos -
                                                                                                               m.points().begin());
                    map.emplace(
                            a, b
                    );
                }
                /*if (count > res.size() + 2) {
                    std::cout << "err\n";
                    exit(1);
                    std::cout << "..................." << "\n";
                    for (auto pair: map) {
                        std::cout << m.point(pair.second) << "\n";
                    }
                }*/
                //add triangles of Delaunay to mesh
                for(auto f : T.finite_face_handles()){
                    faces.emplace_back(m.add_face(map[f->vertex(1)],map[f->vertex(0)],map[f->vertex(2)]));
                    //std::cout << map[f->vertex(1)] << "|" << map[f->vertex(0)] << "|" << map[f->vertex(2)] << std::endl ;
                    //if(faces.back() ==Mesh::null_face()) std::cout << map[f->vertex(1)] << "|" << map[f->vertex(0)] << "|" << map[f->vertex(2)] << std::endl ;

                }
                //std::cout << "______________\n";
            }else{
                auto pos = std::find(m.points().begin(), m.points().end(), **res.begin());
                faces.emplace_back(m.add_face(m.source(h),m.target(h),(pos == m.points().end()) ? m.add_vertex(**res.begin()) : static_cast<CGAL::SM_Vertex_index>(pos - m.points().begin())));
            }



            //add border edges of new face
            for(auto f : faces){
                if(f == Mesh::null_face() ){ //TODO if this case is reached, then the algorithm failed. Maybe add an exception
                    if(coplanarFace){
                        //std::cout << "Coplanar | ";
                    }
                    std::cout << "err\n";
                    exit(1);

                    std::cout << "______________\n";

                    for(auto &face: m.faces()){
                        for(auto &half: m.halfedges_around_face(m.halfedge(face))) {
                            std::cout << m.target(half) << "|" ;
                        }
                        std::cout << "\n";
                    }

                }else{
                    for (auto &e: m.halfedges_around_face(m.halfedge(f))) {
                        if (m.is_border(m.opposite(e))) borderEdges.push_back(m.opposite(e));
                    }
                }
            }
        }
    }
    if(coplanarFace){
        //std::cout << "Coplanar | ";
    }
    //std::cout << m.vertices().size() << ";";
}