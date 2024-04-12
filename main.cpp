#include <iostream>
#include <SFML/Graphics.hpp>
#include <random>
#include "physics.hpp"
#include "render.hpp"
#include "tree.hpp"
const int window_width = 1000;
const int window_height = 1000;

float r = 0;
float b = 230;
float g = 0;


static sf::Color getRainbow(float t, float velx, float vely)
{
    // Adjust the frequency of the color oscillations for more variation
    const float frequency = 2.5f;
    
    // Apply sine and cosine functions with different frequencies and offsets
    const float r = sin(t * frequency + velx * 0.1f);
    const float g = cos((t + 0.33f) * frequency + vely * 0.1f);
    const float b = sin((t + 0.66f) * frequency + velx * vely * 0.05f);

    // Scale the color components to the range [0, 255]
    uint8_t red = static_cast<uint8_t>(127.0f * (1.0f + r));
    uint8_t green = static_cast<uint8_t>(127.0f * (1.0f + g));
    uint8_t blue = static_cast<uint8_t>(127.0f * (1.0f + b));

    // Apply additional transformations to create more vibrant colors
    red = (red + 255) / 2;
    green = (green + 255) / 2;
    blue = (blue + 255) / 2;

    return sf::Color(red, green, blue);
}


sf::Color velo_test(sf::Vector2f velo){
    float red = 5;
    float blue = 5 ;
    float green = 5 ;

    red *= velo.x;
    blue *= velo.y;
    green = 255;

    return sf::Color(red,green,blue);
}

//sf::Vector3f sphereColor(250.0f, 0.0f, 0.0f);


std::string fragmentShaderPath = std::string(SHADER_DIR) + "/vert.frag";


bool spawn_delayz(sf::Time clock,float time){

    if(clock.asSeconds() >= time ){

            return true;
    }
    
};




sf::Vector2f spawn_pos1(sf::Vector2f pos, float radius, float time, float angle)
{
    // Calculate the position of the spawn object on the circle
    float x = 200 + time + radius * cos(angle + time);
    float y = pos.y + radius * sin(angle + time);

    // Create and return the position vector
    return sf::Vector2f(x, y);
}

auto shader = sf::Shader{};


int getRandomNumber() {
    // Create a random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1, 1000);

    // Generate and return a random number
    return dis(gen);
}

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




    Simulator simulator{1000,1000};

    render renders{window};

    simulator.setSubsStepscount(2);
    simulator.setSimulationUpdateRate(frame_rate);
    const float x_spawn =  100;
    const float y_spawn =  0;
    
    sf::Vector2f object_spawn_position = {10, 10};
    const sf::Vector2f object_initial_speed = {100.0f,00.0f};
    const float object_min_radius = 3.0f;
    const float object_max_radius = 25.0f;
    const float spawn_delay = .025f;
    const uint32_t max_object_count = 1500;
    const float max_angle = 360.0f;
    

    sf::Color test(r,g,b);
    int atom_id = 0; 


sf::Vector2f poz; 


    sf::Clock clock;
    while(window.isOpen()){


sf::Event events{};
sf::Time clocks = clock.getElapsedTime();


sf::RectangleShape line(sf::Vector2f(150, 5));

line.setPosition(450,450);





while(window.pollEvent(events)){
    if (events.type == sf::Event::Closed || sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
    {
            window.close();
    }





}
  
poz = spawn_pos1(object_spawn_position,30,simulator.return_time(),max_angle);

if (simulator.getObjectCount() < max_object_count && spawn_delayz(clock.getElapsedTime(),spawn_delay) == true ){
    float time_spawn = clock.getElapsedTime().asSeconds();

     clock.restart();

    auto & object = simulator.addObject(object_spawn_position, object_min_radius,atom_id);


    object.color = getRainbow(simulator.return_time(),100,300);
    simulator.setObjectVelocity(object,object_initial_speed);
    atom_id++;


}
std::cout << simulator.getObjectCount() << std::endl;


//SHADER VARIABLES



shader.setUniform("u_resolution", sf::Glsl::Vec2{ window.getSize() });
shader.setUniform("u_time",simulator.return_time());
//shader.setUniform("sphereColor",sf::Glsl::Vec3(sphereColor));
simulator.update();
window.clear(sf::Color::Black);
renders.renders(simulator);
window.display();




}

return 0;


}




