#pragma once
#include <SFML/Graphics.hpp>


struct Line
{
sf::Vector2f pos; 
sf::Vector2f old_pos;
sf::Vector2f Accel{0,0};
float rotation_speed;
sf::Vector2f size; 



Line(sf::Vector2f Pos, float speed, sf::Vector2f Size)
: pos{Pos} 
, rotation_speed{speed}
, Accel{0,0}
, size{Size}
{

}


public: 






void updatePosition(float dt)
{
    sf::Vector2f Vel = pos - old_pos;
    old_pos = pos;
    pos = pos + Vel + Accel * dt * dt; 
    Accel = {};
}



void accerlate(sf::Vector2f force)
{
    Accel += force;
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



};


