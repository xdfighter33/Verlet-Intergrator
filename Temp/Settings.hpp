#pragma once 

#include <iostream>
#include <SFML/Graphics.hpp>
#include "../Physics/Particle.hpp"

struct Settings{

    public:
    sf::Vector2u Window_Size{1980,1000};


    

};



class Stream{



    
    
    private:
    std::vector<particle> object_stream;
    int num_of_streams;
    int atom_idx = 0;
    int object_count;
    float spawn_delay;
    sf::Vector2f InitialPosition;
    float radius;

    
    void addObjectsToStream(){
        for(int i{0}; i < object_count; i++){
            if(i % 2 != 0){
                InitialPosition.y += 50;
            }
            addObject(InitialPosition,radius,atom_idx);
            atom_idx++;
        }

    
    }


particle& addObject(sf::Vector2f position, float radius, float idx){

    particle newParticle(position, radius,idx);
    
    object_stream.push_back(newParticle);

    return object_stream.back();
}

};

