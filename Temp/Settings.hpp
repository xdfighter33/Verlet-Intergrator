#pragma once 

#include <iostream>
#include <SFML/Graphics.hpp>
#include "../Physics/Particle.hpp"

struct Settings{

    public:
    sf::Vector2u Window_Size{1980,1000};


    

};



struct StreamProperties {
    sf::Vector2f spawn_position;
    float spawn_delay;
    uint32_t max_object_count;
    float time_for_next_object;
    sf::Clock clock;
};



