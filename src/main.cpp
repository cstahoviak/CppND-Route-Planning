#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <io2d.h>

#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;
using Duration = std::chrono::microseconds;

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
                std::cout << "coordinate values must be in [0,100]\n";
            }
        } while ( num < 0.0f || num > 100 );

        coords.push_back(num);
    }
    std::vector<float> init{coords[0], coords[1]};
    std::vector<float> goal{coords[2], coords[3]};

    // Build Model - create RouteModel object
    RouteModel model{osm_data};
    RouteModel model_dijkstra{osm_data};

    // Create RoutePlanner object
    RoutePlanner route_planner{model, init[0], init[1], goal[0], goal[1]};
    RoutePlanner route_planner_dijkstra{model_dijkstra, init[0], init[1], goal[0], goal[1]};

    std::vector<Duration> time;
    std::vector<float> dist;

    // perform Dijkstra search
    auto start = std::chrono::high_resolution_clock::now();
    route_planner_dijkstra.Dijkstra();
    auto stop = std::chrono::high_resolution_clock::now();
    
    time.push_back(std::chrono::duration_cast<Duration>(stop - start));
    dist.push_back(route_planner_dijkstra.GetDistance());

    // perform A* search
    start = std::chrono::high_resolution_clock::now();
    route_planner.AStarSearch();
    stop = std::chrono::high_resolution_clock::now();

    time.push_back(std::chrono::duration_cast<Duration>(stop - start));
    dist.push_back(route_planner.GetDistance());

    // display results and relative algorithm performance 
    std::cout << "\nDijkstra Route Distance:\t" << dist[0] << " [m]\n";
    std::cout << "A* Route Distance:\t\t" << dist[1] << " [m]\n\n";

    std::cout << "Dijkstra Execution Time:\t" << time[0].count() << " [us]\n";
    std::cout << "A* Execution time:\t\t" << time[1].count() << " [us]\n";

    // create Render object and render A* Search result.
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
