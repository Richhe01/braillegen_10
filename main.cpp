#include <emscripten.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <utility>
#include <cstdlib>
#include <algorithm>

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
#include <Standard_Version.hxx>

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
    
    // Axis of revolution explicitly formed by p1 (0,0,0) and p5 (0,0,braille_height)
    gp_Ax1 rev_axis(p1, gp_Dir(gp_Vec(p1, p5)));
    return BRepPrimAPI_MakeRevol(profile_face, rev_axis).Shape();
}

// --- Helper: STL Exporter ---
void ExportShapeToSTL(const TopoDS_Shape& shape, const std::string& filename) {
    std::cout << "Meshing shape..." << std::endl;
    // Decrease linear deflection (parameter 2) and angular deflection (parameter 4) for higher mesh quality
    BRepMesh_IncrementalMesh mesh(shape, 0.08, false, 0.2); 
    
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

// Helper to split string by newline and trim right
std::vector<std::string> split_text_at_new_line(const std::string& str) {
    std::vector<std::string> result;
    size_t start = 0;
    size_t end = str.find('\n');
    while (end != std::string::npos) {
        std::string s = str.substr(start, end - start);
        s.erase(s.find_last_not_of(" \n\r\t") + 1);
        result.push_back(s);
        start = end + 1;
        end = str.find('\n', start);
    }
    if (start <= str.size()) {
        std::string s = str.substr(start);
        s.erase(s.find_last_not_of(" \n\r\t") + 1);
        result.push_back(s);
    }
    return result;
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
    
    // 1. Liblouis Text Translation
    std::string input_text(raw_text);
    std::vector<std::string> lines = split_text_at_new_line(input_text);
    
    // Configure liblouis to find embedded tables
    lou_registerLogCallback(louLogCallback);
    setenv("LOUIS_TABLEPATH", "/tables", 1);
    
    // Construct table string
    std::string tables_str = "braille-patterns.cti," + std::string(braille_table);
    std::cout << "Using liblouis tables: " << tables_str << std::endl;
    
    std::vector<std::vector<widechar>> translated_lines;
    for (const auto& line : lines) {
        if (line.empty()) {
            translated_lines.push_back({});
            continue;
        }
        
        std::vector<widechar> inbuf(line.begin(), line.end());
        int inlen = inbuf.size();
        std::vector<widechar> outbuf(inlen * 4 + 100);
        int outlen = outbuf.size();
        
        int success = lou_translateString(tables_str.c_str(), inbuf.data(), &inlen, outbuf.data(), &outlen, nullptr, nullptr, 0);
        std::cout << "success: " << success << std::endl;
        
        if (success) {
            outbuf.resize(outlen);
            translated_lines.push_back(outbuf);
        } else {
            translated_lines.push_back({});
        }
    }

    // 2. Wrap Text based on max_chars_per_line
    std::vector<std::vector<widechar>> wrapped_lines;
    for (const auto& braille_line : translated_lines) {
        std::vector<std::vector<widechar>> words;
        std::vector<widechar> current_word;
        
        for (widechar c : braille_line) {
            if (c == 0x2800) { // '⠀'
                words.push_back(current_word);
                current_word.clear();
            } else {
                current_word.push_back(c);
            }
        }
        words.push_back(current_word);

        std::vector<widechar> current_line;
        for (size_t i = 0; i < words.size(); ++i) {
            const auto& word = words[i];
            int required_len = current_line.empty() ? word.size() : (current_line.size() + 1 + word.size());
            
            if (required_len > max_chars_per_line && !current_line.empty()) {
                wrapped_lines.push_back(current_line);
                current_line = word;
            } else {
                if (!current_line.empty()) {
                    current_line.push_back(0x2800);
                }
                current_line.insert(current_line.end(), word.begin(), word.end());
            }
        }
        if (!current_line.empty()) {
            wrapped_lines.push_back(current_line);
        }
    }
    
    if (wrapped_lines.empty()) {
        std::cerr << "No braille text generated (empty input)." << std::endl;
        return;
    }
    // Debug: Print wrapped lines and their character counts
    std::cout << "--- Wrapped Braille Output ---" << std::endl;
    for (size_t i = 0; i < wrapped_lines.size(); ++i) {
        std::cout << "Line " << i + 1 << " (" << wrapped_lines[i].size() << " chars): ";
        for (widechar c : wrapped_lines[i]) {
            if (c <= 0x7F) {
                std::cout << static_cast<char>(c);
            } else if (c <= 0x7FF) {
                std::cout << static_cast<char>(0xC0 | (c >> 6))
                          << static_cast<char>(0x80 | (c & 0x3F));
            } else {
                std::cout << static_cast<char>(0xE0 | (c >> 12))
                          << static_cast<char>(0x80 | ((c >> 6) & 0x3F))
                          << static_cast<char>(0x80 | (c & 0x3F));
            }
        }
        std::cout << std::endl;
    }

    // 3. Convert mapped strings to our 4D Vector Matrix
    std::vector<std::vector<std::vector<std::vector<int>>>> the_matrix;
    size_t longest_length = 0;
    
    for (const auto& wline : wrapped_lines) {
        std::vector<std::vector<std::vector<int>>> mapped_line;
        for (widechar c : wline) {
            mapped_line.push_back(getBrailleMatrix(c));
        }
        the_matrix.push_back(mapped_line);
        if (mapped_line.size() > longest_length) {
            longest_length = mapped_line.size();
        }
    }
    
    if (longest_length == 0) {
        std::cerr << "No valid braille characters generated." << std::endl;
        return;
    }

    // 4. OpenCASCADE Parametric Plate Generation
    double diameter = 1.6; 
    double radius = diameter / 2.0;
    double spacing = 2.34;
    double distance = 6.2;

    double plate_depth = spacing * 2.0 + diameter + line_spacing;
    double total_plate_depth = plate_depth * the_matrix.size() - line_spacing;
    
    auto calc_plate_length = [&](size_t len) {
        return std::max(0.1, (distance * len) + margin_size * 2.0 - (distance - spacing - diameter));
    };
    double max_plate_length = calc_plate_length(longest_length);

    // Create Base Slab
    TopoDS_Shape plate;
    if (plate_height > 0.0) {
        if (slab_mode) {
            double slab_width = margin_size + total_plate_depth + margin_size;
            plate = BRepPrimAPI_MakeBox(gp_Pnt(-margin_size, 0, 0), gp_Pnt(slab_width - margin_size, max_plate_length, plate_height)).Shape();
            
            // Corner Cutter
            double cut_depth = std::min(margin_size, 2.0) * 1.2;
            TopoDS_Shape corner_cutter = BRepPrimAPI_MakeBox(5.0, 5.0, plate_height + 2.0).Shape();
            gp_Trsf cut_trsf, trans_trsf;
            cut_trsf.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(0,0,1)), 45.0 * (M_PI / 180.0));
            trans_trsf.SetTranslation(gp_Vec(-margin_size, max_plate_length - cut_depth, -1.0));
            trans_trsf.Multiply(cut_trsf); 
            BRepBuilderAPI_Transform xform_cutter(corner_cutter, trans_trsf);
            plate = BRepAlgoAPI_Cut(plate, xform_cutter.Shape()).Shape();
        } else {
            bool first = true;
            TopoDS_Shape plate_shape;
            
            double top_plate_length = calc_plate_length(the_matrix[0].size());
            double bottom_plate_length = calc_plate_length(the_matrix.back().size());
            
            if (margin_size > 0.0) {
                TopoDS_Shape top_margin = BRepPrimAPI_MakeBox(gp_Pnt(-margin_size, 0, 0), gp_Pnt(0, top_plate_length, plate_height)).Shape();
                plate_shape = top_margin;
                first = false;
            }
            
            for (size_t line = 0; line < the_matrix.size(); ++line) {
                double plate_length = calc_plate_length(the_matrix[line].size());
                double actual_depth = (line == the_matrix.size() - 1) ? (plate_depth - line_spacing) : plate_depth;
                if (actual_depth > 0.0 && plate_length > 0.0) {
                    TopoDS_Shape line_box = BRepPrimAPI_MakeBox(gp_Pnt(plate_depth * line, 0, 0), gp_Pnt(plate_depth * line + actual_depth, plate_length, plate_height)).Shape();
                    if (first) {
                        plate_shape = line_box;
                        first = false;
                    } else {
                        plate_shape = BRepAlgoAPI_Fuse(plate_shape, line_box).Shape();
                    }
                }
            }
            
            if (margin_size > 0.0) {
                TopoDS_Shape bottom_margin = BRepPrimAPI_MakeBox(gp_Pnt(total_plate_depth, 0, 0), gp_Pnt(total_plate_depth + margin_size, bottom_plate_length, plate_height)).Shape();
                if (first) {
                    plate_shape = bottom_margin;
                } else {
                    plate_shape = BRepAlgoAPI_Fuse(plate_shape, bottom_margin).Shape();
                }
            }
            
            plate = plate_shape;
        }
    }

    // 5. Build the Compound Group (Batched Dots)
    TopoDS_Compound all_dots;
    BRep_Builder compound_builder;
    compound_builder.MakeCompound(all_dots);
    bool has_dots = false;

    // Pre-calculate the dot shape since it's exactly the same for every dot
    double embed_depth = (plate_height > 0.0) ? 0.01 : 0.0;
    TopoDS_Shape base_dot = createExactBrailleDot(radius, braille_height + embed_depth);

    for (size_t line = 0; line < the_matrix.size(); ++line) {
        for (size_t char_index = 0; char_index < the_matrix[line].size(); ++char_index) {
            for (int col = 0; col < 2; ++col) {
                for (int row = 0; row < 3; ++row) {
                    
                    if (the_matrix[line][char_index][col][row] == 1) {
                        double x_pos = (plate_depth * line) + (spacing * row + radius);
                        double y_pos = (distance * char_index) + margin_size + (spacing * col + radius);
                        double z_pos = plate_height - embed_depth;
                        
                        gp_Trsf dot_trsf;
                        dot_trsf.SetTranslation(gp_Vec(x_pos, y_pos, z_pos));
                        BRepBuilderAPI_Transform positioned_dot(base_dot, dot_trsf);

                        // Add to batch compound
                        compound_builder.Add(all_dots, positioned_dot.Shape());
                        has_dots = true;
                    }
                }
            }
        }
    }

    // 6. Build the Final Shape by fusing
    TopoDS_Shape export_shape;

    if (plate_height > 0.0 && has_dots) {
        std::cout << "Fusing geometry..." << std::endl;
        export_shape = BRepAlgoAPI_Fuse(plate, all_dots).Shape();
    } else if (plate_height > 0.0) {
        std::cout << "No valid dots generated. Outputting blank plate." << std::endl;
        export_shape = plate;
    } else if (has_dots) {
        std::cout << "No plate generated. Outputting dots only." << std::endl;
        export_shape = all_dots;
    } else {
        std::cout << "No valid dots generated and no plate. Outputting empty model." << std::endl;
        TopoDS_Compound empty_model;
        BRep_Builder().MakeCompound(empty_model);
        export_shape = empty_model;
    }

    // 7. Post-Transformations
    gp_Trsf rot_z, rot_x, scale_trsf;
    rot_z.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(0,0,1)), -90.0 * (M_PI / 180.0));
    rot_x.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(1,0,0)), (vertical_export ? 90.0 : 0.0) * (M_PI / 180.0));
    scale_trsf.SetScale(gp_Pnt(0,0,0), stl_scale);
    
    gp_Trsf final_trsf = scale_trsf;
    final_trsf.Multiply(rot_x);
    final_trsf.Multiply(rot_z);
    
    BRepBuilderAPI_Transform apply_final(export_shape, final_trsf);
    export_shape = apply_final.Shape();
    
    ExportShapeToSTL(export_shape, "braille_plate.stl");
}

} // extern "C"

int main() {
    std::cout << "BrailleGen WASM Core loaded." << std::endl;
    std::cout << "Liblouis version: " << lou_version() << std::endl;
    std::cout << "OpenCASCADE version: " << OCC_VERSION_STRING << std::endl;
    return 0;
}