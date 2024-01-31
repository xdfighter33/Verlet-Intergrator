#include <iostream>
#include <Sfml/Graphics.hpp>

#include "physics.hpp"
#include "render.hpp"

const int window_width = 1000;
const int window_height = 1000; 

bool spawn_delayz(sf::Time clock,float time){

    if(clock.asSeconds() >= time ){

            return true;
    }

};
int main(){


    sf::RenderWindow window(sf::VideoMode(window_width,window_height), "Physics ");
    const uint32_t frame_rate = 60;
    window.setFramerateLimit(frame_rate);


    Simulator simulator;

    render renders{window};

    simulator.setSubsStepscount(8);
    simulator.setSimulationUpdateRate(frame_rate);
    const float x_spawn =  500;
    const float y_spawn =  500;
    
    sf::Vector2f object_spawn_position = {x_spawn, y_spawn};
    const sf::Vector2f object_initial_speed = {0.0f,0.0f};
    const float object_min_radius = 10.0f;
    const float object_max_radius = 25.0f;
    const float spawn_delay = 1.5f;
    const uint32_t max_object_count = 500;
    const float max_angle = 10.0f;







    sf::Clock clock;
    while(window.isOpen()){
sf::Event events{};
sf::Time clocks = clock.getElapsedTime();
while(window.pollEvent(events)){
    if (events.type == sf::Event::Closed || sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
    {
            window.close();
    }



}
if (simulator.getObjectCount() < max_object_count && spawn_delayz(clock.getElapsedTime(),spawn_delay) == true ){
     clock.restart();

    auto & object = simulator.addObject( sin(max_angle) * object_spawn_position, object_min_radius);



    simulator.setObjectVelocity(object,object_initial_speed);

}
simulator.update();
window.clear(sf::Color::Black);
renders.renders(simulator);
window.display();




}

return 0;


}




