#include <emscripten.h>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <iostream>
#include <fstream>
#include <utility>
#include "liblouis.h"

extern "C" {

EMSCRIPTEN_KEEPALIVE
void generateCubeSTL() {
    std::cout << "Generating 10x10x10 cube..." << std::endl;
    
    TopoDS_Shape cube = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
    BRepMesh_IncrementalMesh mesh(cube, 0.1); // Mesh the shape to export as STL
    
    std::ofstream out("cube.stl");
    out << "solid cube\n";
    
    TopExp_Explorer ex(cube, TopAbs_FACE);
    for (; ex.More(); ex.Next()) {
        TopoDS_Face face = TopoDS::Face(ex.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        
        if (!tri.IsNull()) {
            for (int i = 1; i <= tri->NbTriangles(); ++i) {
                int n1, n2, n3;
                tri->Triangle(i).Get(n1, n2, n3);
                
                // Preserve face orientation so normals face outward
                if (face.Orientation() == TopAbs_REVERSED) {
                    std::swap(n1, n2);
                }
                
                gp_Pnt p1 = tri->Node(n1).Transformed(loc);
                gp_Pnt p2 = tri->Node(n2).Transformed(loc);
                gp_Pnt p3 = tri->Node(n3).Transformed(loc);
                
                // Calculate surface normal
                gp_Vec u(p1, p2), v(p1, p3);
                gp_Vec norm = u.Crossed(v);
                if (norm.SquareMagnitude() > 0) norm.Normalize();
                
                out << "facet normal " << norm.X() << " " << norm.Y() << " " << norm.Z() << "\n";
                out << "  outer loop\n";
                out << "    vertex " << p1.X() << " " << p1.Y() << " " << p1.Z() << "\n";
                out << "    vertex " << p2.X() << " " << p2.Y() << " " << p2.Z() << "\n";
                out << "    vertex " << p3.X() << " " << p3.Y() << " " << p3.Z() << "\n";
                out << "  endloop\n";
                out << "endfacet\n";
            }
        }
    }
    out << "endsolid cube\n";
    out.close();
    
    std::cout << "Cube written to virtual filesystem as cube.stl" << std::endl;
}

} // extern "C"

int main() {
    std::cout << "BrailleGen 10 WebAssembly loaded." << std::endl;
    std::cout << "liblouis version: " << lou_version() << std::endl;
    return 0;
}