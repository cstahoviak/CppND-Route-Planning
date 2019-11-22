#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);     // move does not require pointers or references to copy data
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.

    std::vector<std::string> coord_str {"start_x", "start_y", "end_x", "end_y"};
    std::vector<float> coords;
    float num;
    for( int i = 0; i < coord_str.size(); i++ ) {
        do {
            std::cout << "Enter " << coord_str[i] << ": ";
            std::cin >> num;

            if( num < 0.0f || num > 100 ) {
                std::cout << "coordinate values must be in [0,100]" << "\n";
            }
        } while ( num < 0.0f || num > 100 );

        // if( num < 0.0f) {
        //     std::cout << "coordinate values must be in [0,100]" << "\n";
        //     std::cout << "setting " << coord_str[i] << " to 0" << "\n";
        //     num = 0.0f;
        // }
        // else if( num > 100.0 ) {
        //     std::cout << "coordinate values must be in [0,100]" << "\n";
        //     std::cout << "setting " << coord_str[i] << " to 100" << "\n";
        //     num = 100.0;
        // }

        coords.push_back(num);
    }
    std::vector<float> init{coords[0], coords[1]};
    std::vector<float> goal{coords[2], coords[3]};

    // Build Model - create RouteModel object
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    // RoutePlanner route_planner{model, 10, 10, 90, 90};
    RoutePlanner route_planner{model, init[0], init[1], goal[0], goal[1]};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // create Render object and render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
