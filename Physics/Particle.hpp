#pragma once

#include <SFML/Graphics.hpp>
struct particle {
    float radius = 5.0f; 

    sf::Vector2f pos;
    sf::Vector2f old_pos;
    sf::Vector2f accel = {0,0};
    sf::Color color = sf::Color::White;
    uint32_t index;
   particle() : index(0) {}

    particle(sf::Vector2f position_, float radius_)
    : pos{position_}
    , index(0)
    , old_pos{position_}
    ,accel{0.0f,0.0f}
    ,radius{radius_}
{}

    particle(sf::Vector2f position_, float radius_, int idx_)
    : pos{position_}
    , index(idx_)
    , old_pos{position_}
    ,accel{0.0f,0.0f}
    ,radius{radius_}
{}


void set_color(sf::Color Color){
    color = Color;
}


void updatePosition(float dt)
{


    sf::Vector2f Vel = pos - old_pos;
    old_pos = pos;
    pos = pos + Vel + accel * dt * dt; 
    accel = {};
}

void set_obx_idx(uint32_t id){
index = id;
}


void accerlate(sf::Vector2f force)
{
    accel += force;
}

void setVelo(sf::Vector2f v, float dt)
{
    old_pos = pos - (v * dt );
}

void addVelocity(sf::Vector2f v, float dt)
{
    old_pos -= v * dt;
}

sf::Vector2f getPos()
{
    return pos;

}

sf::Vector2f GetVelocity(float dt)
{
    return(pos - old_pos) / dt;
}

uint32_t return_atom_idx(){

    return index;
}

};
