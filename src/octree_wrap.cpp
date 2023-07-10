//
// Created by fabia on 22.06.2023.
//

#include "octree_wrap.h"

void
octreeWrap(std::vector<Kernel::Point_3> &pointCloud, Mesh &m) {

    if(pointCloud.empty()) return;

    std::vector<Kernel::Point_3> res;
    /** Building the Octree **/

    Octree octree(pointCloud);
    octree.refine(10, 15);

    /** Solving the Problem here **/
    std::vector<CGAL::SM_Halfedge_index> borderEdges;

    //find first face
    auto bbox = octree.bbox(octree.root());
    std::pair<Kernel::Point_3,Kernel::Point_3> set = {{bbox.xmax(),bbox.ymax(),bbox.zmax()},{bbox.xmax(),bbox.ymax(),bbox.zmin()}}; //TODO replace with halfedge
    set.first = *findBoundaryPoint(octree, set, BS_RIGHT);
    set.second = *findBoundaryPoint(octree, set, BS_RIGHT);

    //add face to mesh
    CGAL::SM_Face_index f = m.add_face(m.add_vertex(set.first), m.add_vertex(set.second), m.add_vertex(*findBoundaryPoint(octree, set, BS_RIGHT)));
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

            auto p = findBoundaryPoint(octree, {s, t}, BS_RIGHT);

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

                if(f == m.null_face() ){ //TODO if this case is reached, then the algorithm failed. Maybe add an exception
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