#include "color_mapping.h"

int main(int argc, char** argv) {
    std::string config_file = argv[1];
    ColorMapping map(config_file);
    map.MapBuild();
    return 0;
}
