#include <pluginlib/class_loader.hpp>
#include <nav2_core/global_planner.hpp>
#include <memory>
#include <iostream>

int main(int argc, char** argv)
{
    try {
        pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core", "nav2_core::GlobalPlanner");

        std::shared_ptr<nav2_core::GlobalPlanner> planner = 
            loader.createSharedInstance("nav2_edge_aware_planner/EdgeAwarePlanner");

        std::cout << "Plugin loaded successfully!" << std::endl;
    } catch (pluginlib::PluginlibException& ex) {
        std::cout << "Failed to load plugin: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
