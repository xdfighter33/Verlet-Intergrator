#include <iostream>
#include <Sfml/Graphics.hpp>

#include "physics.hpp"
#include "render.hpp"

const int window_width = 1000;
const int window_height = 1000;

float r = 0;
float b = 230;
float g = 0;

sf::Vector3f sphereColor(1.0f, 0.0f, 0.0f);


std::string fragmentShaderPath = std::string(SHADER_DIR) + "/vert.frag";


bool spawn_delayz(sf::Time clock,float time){

    if(clock.asSeconds() >= time ){

            return true;
    }

};




sf::Vector2f spawn_pos1(sf::Vector2f pos, float x, sf::Time time, float angle){

return pos; 
};


auto shader = sf::Shader{};





int main(){




    
   if (!shader.loadFromFile(fragmentShaderPath, sf::Shader::Fragment))
    {
        std::cerr << "Couldn't load vert shader\n";
        return -1;
    }

    sf::RenderWindow window(sf::VideoMode(window_width,window_height), "Physics ");
    const uint32_t frame_rate = 60;
    window.setFramerateLimit(frame_rate);



 
    auto shape = sf::RectangleShape{ sf::Vector2f{ window.getSize() } };


    Simulator simulator;

    render renders{window};

    simulator.setSubsStepscount(12);
    simulator.setSimulationUpdateRate(frame_rate);
    const float x_spawn =  500;
    const float y_spawn =  955;
    
    sf::Vector2f object_spawn_position = {x_spawn, y_spawn};
    const sf::Vector2f object_initial_speed = {5.0f,0.0f};
    const float object_min_radius = 10.0f;
    const float object_max_radius = 25.0f;
    const float spawn_delay = 0.5f;
    const uint32_t max_object_count = 500;
    const float max_angle = 5.0f;
    

    sf::Color test(r,g,b);



sf::Vector2f poz; 


    sf::Clock clock;
    while(window.isOpen()){


sf::Event events{};
sf::Time clocks = clock.getElapsedTime();


sf::RectangleShape line(sf::Vector2f(150, 5));

line.setPosition(450,450);



poz = spawn_pos1(object_spawn_position,5,clocks,max_angle);

while(window.pollEvent(events)){
    if (events.type == sf::Event::Closed || sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
    {
            window.close();
    }





}
  

if (simulator.getObjectCount() < max_object_count && spawn_delayz(clock.getElapsedTime(),spawn_delay) == false ){
    
    float time_spawn = clock.getElapsedTime().asSeconds();
     clock.restart();



    auto & object = simulator.addObject(poz, object_min_radius);
  
    object.color = sf::Color(sphereColor.x,sphereColor.y,sphereColor.z);
    simulator.setObjectVelocity(object,object_initial_speed);

}

//SHADER VARIABLES




shader.setUniform("u_resolution", sf::Glsl::Vec2{ window.getSize() });
shader.setUniform("u_time",simulator.return_time());
shader.setUniform("sphereColor",sphereColor);
simulator.update();
window.clear(sf::Color::Black);
renders.renders(simulator);
window.draw(shape,&shader);
window.display();




}

return 0;


}




