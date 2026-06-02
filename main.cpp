#include <emscripten.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <utility>
#include <cstdlib>

// OpenCASCADE Includes
#include <gp_Trsf.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>

#include "liblouis.h"

// --- Helper: Exact Braille Dot Profile ---
TopoDS_Shape createExactBrailleDot(double radius, double braille_height) {
    gp_Pnt p1(0.0, 0.0, 0.0);                                  
    gp_Pnt p2(radius, 0.0, 0.0);                               
    gp_Pnt p3(radius, 0.0, braille_height - 0.3);              
    gp_Pnt p4(0.4, 0.0, braille_height);                       
    gp_Pnt p5(0.0, 0.0, braille_height);                       

    TopoDS_Edge e1 = BRepBuilderAPI_MakeEdge(p1, p2);
    TopoDS_Edge e2 = BRepBuilderAPI_MakeEdge(p2, p3);
    TopoDS_Edge e3 = BRepBuilderAPI_MakeEdge(p3, p4);
    TopoDS_Edge e4 = BRepBuilderAPI_MakeEdge(p4, p5);
    TopoDS_Edge e5 = BRepBuilderAPI_MakeEdge(p5, p1); 

    BRepBuilderAPI_MakeWire wire_maker;
    wire_maker.Add(e1); wire_maker.Add(e2); wire_maker.Add(e3);
    wire_maker.Add(e4); wire_maker.Add(e5);

    TopoDS_Face profile_face = BRepBuilderAPI_MakeFace(wire_maker.Wire());
    gp_Ax1 z_axis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    return BRepPrimAPI_MakeRevol(profile_face, z_axis).Shape();
}

// --- Helper: STL Exporter ---
void ExportShapeToSTL(const TopoDS_Shape& shape, const std::string& filename) {
    std::cout << "Meshing shape..." << std::endl;
    BRepMesh_IncrementalMesh mesh(shape, 0.1); 
    
    std::ofstream out(filename);
    out << "solid braille_plate\n";
    TopExp_Explorer ex(shape, TopAbs_FACE);
    for (; ex.More(); ex.Next()) {
        TopoDS_Face face = TopoDS::Face(ex.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        
        if (!tri.IsNull()) {
            for (int i = 1; i <= tri->NbTriangles(); ++i) {
                int n1, n2, n3;
                tri->Triangle(i).Get(n1, n2, n3);
                if (face.Orientation() == TopAbs_REVERSED) std::swap(n1, n2);
                
                gp_Pnt p1 = tri->Node(n1).Transformed(loc);
                gp_Pnt p2 = tri->Node(n2).Transformed(loc);
                gp_Pnt p3 = tri->Node(n3).Transformed(loc);
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
    out << "endsolid braille_plate\n";
    out.close();
    std::cout << "Shape written to " << filename << std::endl;
}

// --- Logic: Unicode to 2x3 Matrix via Bitwise Operations ---
std::vector<std::vector<int>> getBrailleMatrix(widechar codepoint) {
    std::vector<std::vector<int>> cell(2, std::vector<int>(3, 0));
    
    // Check if character is a valid Unicode Braille symbol (U+2800 to U+28FF)
    if (codepoint >= 0x2800 && codepoint <= 0x28FF) {
        uint32_t offset = codepoint - 0x2800;
        
        cell[0][0] = (offset & (1 << 0)) ? 1 : 0; // Dot 1
        cell[0][1] = (offset & (1 << 1)) ? 1 : 0; // Dot 2
        cell[0][2] = (offset & (1 << 2)) ? 1 : 0; // Dot 3
        cell[1][0] = (offset & (1 << 3)) ? 1 : 0; // Dot 4
        cell[1][1] = (offset & (1 << 4)) ? 1 : 0; // Dot 5
        cell[1][2] = (offset & (1 << 5)) ? 1 : 0; // Dot 6
    }
    return cell;
}

extern "C" {

// Callback to pipe liblouis errors to the JS console
void louLogCallback(logLevels level, const char *message) {
    std::cerr << "liblouis: " << message << std::endl;
}

EMSCRIPTEN_KEEPALIVE
void generateBrailleSTL(const char* raw_text, int max_chars_per_line, const char* braille_table,
                        double braille_height, double plate_height, double line_spacing,
                        double margin_size, double stl_scale, bool slab_mode, bool vertical_export) {
    std::cout << "Starting text translation & STL generation..." << std::endl;
    
    // Print out the parameters to verify they are received correctly from JS
    std::cout << "--- Parameters Received ---" << std::endl;
    std::cout << "braille_height: " << braille_height << " mm" << std::endl;
    std::cout << "plate_height: " << plate_height << " mm" << std::endl;
    std::cout << "line_spacing: " << line_spacing << " mm" << std::endl;
    std::cout << "margin_size: " << margin_size << " mm" << std::endl;
    std::cout << "stl_scale: " << stl_scale << std::endl;
    std::cout << "slab_mode: " << (slab_mode ? "true" : "false") << std::endl;
    std::cout << "vertical_export: " << (vertical_export ? "true" : "false") << std::endl;

    // 1. Liblouis Text Translation
    std::string input_text(raw_text);
    
    // Convert basic char* to liblouis widechar format
    std::vector<widechar> inbuf(input_text.begin(), input_text.end());
    int inlen = inbuf.size();
    
    // Allocate buffer for output (allow ample space for braille expansion)
    std::vector<widechar> outbuf(inlen * 4 + 100);
    int outlen = outbuf.size();
    
    // Configure liblouis to find embedded tables
    lou_registerLogCallback(louLogCallback);
    setenv("LOUIS_TABLEPATH", "/tables", 1);
    
    // Construct table string
    std::string tables_str = std::string(braille_table) + ",braille-patterns.cti";
    std::cout << "Using liblouis tables: " << tables_str << std::endl;
    
    int success = lou_translateString(tables_str.c_str(), inbuf.data(), &inlen, outbuf.data(), &outlen, nullptr, nullptr, 0);
    if (!success) {
        std::cerr << "Liblouis translation failed!" << std::endl;
        return;
    }
    std::cout << "Translation successful! " << outlen << " characters generated." << std::endl;

    // 2. Wrap Text based on max_chars_per_line
    std::vector<std::vector<widechar>> wrapped_lines;
    std::vector<widechar> current_line;
    
    for (int i = 0; i < outlen; ++i) {
        // Space in braille unicode is often U+2800 (Blank cell)
        if (outbuf[i] == '\n' || current_line.size() >= max_chars_per_line) {
            wrapped_lines.push_back(current_line);
            current_line.clear();
        }
        if (outbuf[i] != '\n') {
            current_line.push_back(outbuf[i]);
        }
    }
    if (!current_line.empty()) {
        wrapped_lines.push_back(current_line);
    }

    // 3. Convert mapped strings to our 4D Vector Matrix
    std::vector<std::vector<std::vector<std::vector<int>>>> the_matrix;
    size_t longest_length = 0;
    
    for (const auto& line : wrapped_lines) {
        std::vector<std::vector<std::vector<int>>> mapped_line;
        for (widechar c : line) {
            mapped_line.push_back(getBrailleMatrix(c));
        }
        the_matrix.push_back(mapped_line);
        if (mapped_line.size() > longest_length) {
            longest_length = mapped_line.size();
        }
    }

    // 4. OpenCASCADE Parametric Plate Generation
    double diameter = 1.6; 
    double radius = diameter / 2.0;
    double spacing = 2.34;
    double distance = 6.2;

    double plate_depth = spacing * 2.0 + diameter + line_spacing;
    double max_plate_length = (distance * longest_length) + margin_size * 2.0 - (distance - spacing - diameter);
    double total_width = (plate_depth * the_matrix.size()) + margin_size;

    // Create Base Slab
    TopoDS_Shape plate;
    if (plate_height > 0.0) {
        plate = BRepPrimAPI_MakeBox(total_width, max_plate_length, plate_height).Shape();

        if (!slab_mode) {
            // Corner Cutter
            TopoDS_Shape corner_cutter = BRepPrimAPI_MakeBox(5.0, 5.0, plate_height + 2.0).Shape();
            gp_Trsf cut_trsf, trans_trsf;
            cut_trsf.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(0,0,1)), 45.0 * (M_PI / 180.0));
            trans_trsf.SetTranslation(gp_Vec(-1.0, max_plate_length - 3.0, -1.0));
            cut_trsf.Multiply(trans_trsf); 
            BRepBuilderAPI_Transform xform_cutter(corner_cutter, cut_trsf);
            plate = BRepAlgoAPI_Cut(plate, xform_cutter.Shape()).Shape();
        }
    }

    // 5. Build the Compound Group (Batched Dots)
    TopoDS_Compound all_dots;
    BRep_Builder compound_builder;
    compound_builder.MakeCompound(all_dots);
    bool has_dots = false;

    for (size_t line = 0; line < the_matrix.size(); ++line) {
        for (size_t char_index = 0; char_index < the_matrix[line].size(); ++char_index) {
            for (int col = 0; col < 2; ++col) {
                for (int row = 0; row < 3; ++row) {
                    
                    if (the_matrix[line][char_index][col][row] == 1) {
                        double x_pos = (plate_depth * line) + (spacing * row + radius) + margin_size;
                        double y_pos = (distance * char_index) + margin_size + (spacing * col + radius);
                        
                        // Embed the dot slightly into the plate to ensure safe slicing/boolean intersection
                        double embed_depth = (plate_height > 0.0) ? 0.01 : 0.0;
                        double z_pos = plate_height - embed_depth;
                        
                        // Generate the exact braille profile
                        TopoDS_Shape dot = createExactBrailleDot(radius, braille_height + embed_depth);
                        
                        gp_Trsf dot_trsf;
                        dot_trsf.SetTranslation(gp_Vec(x_pos, y_pos, z_pos));
                        BRepBuilderAPI_Transform positioned_dot(dot, dot_trsf);

                        // Add to batch compound
                        compound_builder.Add(all_dots, positioned_dot.Shape());
                        has_dots = true;
                    }
                }
            }
        }
    }

    // 6. Build the Final Compound (Skip fusing to drastically speed up generation and avoid coincident face errors)
    TopoDS_Compound final_model;
    BRep_Builder final_builder;
    final_builder.MakeCompound(final_model);
    
    if (plate_height > 0.0) {
        final_builder.Add(final_model, plate);
    }
    
    if (has_dots) {
        std::cout << "Adding dots to the plate compound..." << std::endl;
        final_builder.Add(final_model, all_dots);
    } else {
        if (plate_height > 0.0) {
            std::cout << "No valid dots generated. Outputting blank plate." << std::endl;
        } else {
            std::cout << "No valid dots generated and no plate. Outputting empty model." << std::endl;
        }
    }

    TopoDS_Shape export_shape = final_model;

    // 7. Post-Transformations
    gp_Trsf final_trsf;
    bool apply_trsf = false;
    
    if (vertical_export) {
        gp_Trsf rot_trsf;
        rot_trsf.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(1,0,0)), 90.0 * (M_PI / 180.0));
        final_trsf.Multiply(rot_trsf);
        apply_trsf = true;
    }
    if (stl_scale != 1.0) {
        gp_Trsf scale_trsf;
        scale_trsf.SetScale(gp_Pnt(0,0,0), stl_scale);
        final_trsf.Multiply(scale_trsf);
        apply_trsf = true;
    }
    if (apply_trsf) {
        BRepBuilderAPI_Transform apply_final(export_shape, final_trsf);
        export_shape = apply_final.Shape();
    }
    
    ExportShapeToSTL(export_shape, "braille_plate.stl");
}

} // extern "C"

int main() {
    std::cout << "BrailleGen WebAssembly Core loaded." << std::endl;
    std::cout << "liblouis version: " << lou_version() << std::endl;
    return 0;
}