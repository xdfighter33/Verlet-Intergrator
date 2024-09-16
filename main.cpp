#include <iostream>
#include <SFML/Graphics.hpp>
#include <random>
#include "Physics/physics.hpp"
#include "Renders/render.hpp"
#include "Temp/Settings.hpp"


const int window_width = constraints.Window_Size.x;
const int window_height = constraints.Window_Size.y;



float r = 20;
float b = 5;
float g = 10;

    static sf::Color getRainbow(float t)
    {
        const float r = sin(t);
        const float g = sin(t + 0.33f * 2.0f);
        const float b = sin(t + 0.66f * 2.0f);
    return sf::Color(255 * r * r ,255 * g * g ,255 * b * b);
    }

static sf::Color getRainbow(float t, float z)
{
    // Normalize t to the range [0, 1]
    t = std::fmod(t, 1.0f);
    if (t < 0) t += 1.0f;

    // Adjust these values to fine-tune the color effect
    constexpr uint8_t minBlue = 20;   // Darkest blue
    constexpr uint8_t maxBlue = 255;  // Lightest blue
    constexpr uint8_t minGreen = 50;  // Minimum green component
    constexpr uint8_t maxGreen = 250; // Maximum green component
    constexpr float contrast = 1.5f;  // Adjust for more pronounced effect

    // Calculate the blue component
    float blueValue = std::pow(t, contrast);  // Apply contrast
    uint8_t blue = static_cast<uint8_t>(minBlue + blueValue * (maxBlue - minBlue));

    // Calculate the green component (for a slight teal tint in lighter shades)
    uint8_t green = static_cast<uint8_t>(minGreen + blueValue * (maxGreen - minGreen));

    // Red is kept low for a true blue effect
    uint8_t red = static_cast<uint8_t>(blueValue * 20);  // Just a hint of red in lighter shades

    return sf::Color(red, green, blue);
}

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


// std::string fragmentShaderPath = std::string(SHADER_DIR) + "/vert.frag";


bool spawn_delayz(sf::Time clock,float time){

    if(clock.asSeconds() >= time ){

            return true;
    } else 
    return false;
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

   tp::ThreadPool thread_pool(10);


    
  /*  if (!shader.loadFromFile(fragmentShaderPath, sf::Shader::Fragment))
    {
       std::cerr << "Couldn't load vert shader\n";
        return -1;
    }
*/ 
    sf::RenderWindow window(sf::VideoMode(window_width,window_height), "Physics ");
    const uint32_t frame_rate = 60;
    window.setFramerateLimit(frame_rate);



 
    auto shape = sf::RectangleShape{ sf::Vector2f{ window.getSize() } };


   
    Simulator simulator{1000,1000,thread_pool};

    render renders{window,simulator,thread_pool};

    simulator.setSubsStepscount(20);
    simulator.setSimulationUpdateRate(frame_rate);
    const float x_spawn =  100;
    const float y_spawn =  0;
    

    //Box Spawn Calcations

    

    //Put this in its own contraint file
    sf::Vector2f Box_constraint(1920,1000);
    sf::Vector2f Box_Positions(simulator.getBoxConstraintPos());
    simulator.setBoxConstraint(Box_constraint);
    sf::Vector2f object_spawn_position = {simulator.getBoxConstraintPos().x - 5.0f, 25};
    sf::Vector2f object_spawn_position2 = {simulator.getBoxConstraintPos().x - 5.0f, 75};
    sf::Vector2f object_spawn_position3 = {simulator.getBoxConstraintPos().x - 5.0f, 100};
     sf::Vector2f object_spawn_position4 = {simulator.getBoxConstraintPos().x - 5.0f, 125};  // New spawn position
    const sf::Vector2f object_initial_speed = {6500.0, 0.0f};
    const float object_min_radius = 4.5f;
    const float object_max_radius = 25.0f;
    const float spawn_delay = .000025f;
    const float spawn_delay2 = .000025f;
    const float spawn_delay3 = .000025f;
    const float spawn_delay4 = .000025f;    // New spawn delay
    const uint32_t max_object_count  = 50000;
    const uint32_t max_object_count1 = 50000;
    const uint32_t max_object_count2 = 50000;
    const uint32_t max_object_count3 = 50000;  
    const float max_angle = 360.0f;



sf::Vector2f object_spawn_position5 = {simulator.getBoxConstraintPos().x - 5.0f, 150};
sf::Vector2f object_spawn_position6 = {simulator.getBoxConstraintPos().x - 5.0f, 175};
sf::Vector2f object_spawn_position7 = {simulator.getBoxConstraintPos().x - 5.0f, 200};
sf::Vector2f object_spawn_position8 = {simulator.getBoxConstraintPos().x - 5.0f, 225};
sf::Vector2f object_spawn_position9 = {simulator.getBoxConstraintPos().x - 5.0f, 250};

const float spawn_delay5 = .000025f;
const float spawn_delay6 = .000025f;
const float spawn_delay7 = .000025f;
const float spawn_delay8 = .000025f;
const float spawn_delay9 = .000025f;

const uint32_t max_object_count4 = 50000;
const uint32_t max_object_count5 = 50000;
const uint32_t max_object_count6 = 50000;
const uint32_t max_object_count7 = 50000;
const uint32_t max_object_count8 = 50000;

sf::Clock clock5;
sf::Clock clock6;
sf::Clock clock7;
sf::Clock clock8;
sf::Clock clock9;

  std::vector<StreamProperties> streams(12);
for (int i = 0; i < 12; ++i) {
    streams[i] = {
        sf::Vector2f(simulator.getBoxConstraintPos().x - 5.0f, 5.0f + i * 20.0f),
        0.000025f,
        25000,
        2.0f + i * 0.5f,
        clock5
    };
}



//simulator.Add_all_objects(sf::Vector2f(0,0),object_min_radius,2000);
    sf::Color test(r,g,b);
    int atom_id = 0; 

sf::Vector2f poz; 




    sf::Clock clock;
    sf::Clock clock2;
    sf::Clock clock3;
    sf::Clock clock4;
    sf::Clock global_time;

    float angle = 5;
    // simulator.add_center_line_with_line(sf::Vector2f(Box_constraint.x/2,Box_constraint.y / 2),sf::Vector2f{25,100},angle);
    bool add_objects = true;
    float time_for_next_object = 2.0f;
    while(window.isOpen()){



sf::Event events;
sf::Time clocks = clock.getElapsedTime();






while(window.pollEvent(events)){
    if (events.type == sf::Event::Closed)
    {
            window.close();
    }    else if (events.type == sf::Event::MouseWheelScrolled)
    {
        if (events.mouseWheelScroll.wheel == sf::Mouse::VerticalWheel)
        {
            renders.handleZoom(events.mouseWheelScroll.delta, sf::Mouse::getPosition(window));
        }
    }

    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Escape)){
            window.close();
    }
    



    if(sf::Keyboard::isKeyPressed(sf::Keyboard::A)){
        add_objects = true;
       angle += 100;

    }


}
  



bool add_object_vector = true;


if (add_object_vector) {
    for (int i = 0; i < streams.size(); ++i) {
        auto& stream = streams[i];
        if (simulator.getObjectCount() < stream.max_object_count && 
            spawn_delayz(global_time.getElapsedTime(), stream.time_for_next_object) && 
            spawn_delayz(stream.clock.getElapsedTime(), stream.spawn_delay)) {
        
            stream.clock.restart();
            auto& object = simulator.addObject(stream.spawn_position, object_min_radius, atom_id);
            object.color = getRainbow(simulator.return_time()); // Slightly different color for each stream
            simulator.setObjectVelocity(object, object_initial_speed);
            atom_id++;
        }
    }
} 

 if(add_objects == false) {
            // First stream
            if (simulator.getObjectCount() < max_object_count && spawn_delayz(clock.getElapsedTime(), spawn_delay) == true) {
                clock.restart();
                auto & object = simulator.addObject(object_spawn_position, object_min_radius, atom_id);
                object.color = getRainbow(simulator.return_time());
                simulator.setObjectVelocity(object, object_initial_speed);
                atom_id++; 
            }

            // Second stream
            if (simulator.getObjectCount() < max_object_count1 && spawn_delayz(global_time.getElapsedTime(), time_for_next_object) == true && spawn_delayz(clock2.getElapsedTime(), spawn_delay2) == true) {
                clock2.restart();
                auto& object2 = simulator.addObject(object_spawn_position2, object_min_radius, atom_id);
                object2.color = getRainbow(simulator.return_time());
                simulator.setObjectVelocity(object2, object_initial_speed);
                atom_id++;
            }

            // Third stream
            if (simulator.getObjectCount() < max_object_count2 && spawn_delayz(global_time.getElapsedTime(), time_for_next_object + 2.0) == true && spawn_delayz(clock3.getElapsedTime(), spawn_delay3) == true) {
                clock3.restart();
                auto& object3 = simulator.addObject(object_spawn_position3, object_min_radius, atom_id);
                object3.color = getRainbow(simulator.return_time());
                simulator.setObjectVelocity(object3, object_initial_speed);
                atom_id++;
            }


            if (simulator.getObjectCount() < max_object_count3 && spawn_delayz(global_time.getElapsedTime(), time_for_next_object + 4.0) == true && spawn_delayz(clock4.getElapsedTime(), spawn_delay4) == true) {
                clock4.restart();
                auto& object4 = simulator.addObject(object_spawn_position4, object_min_radius, atom_id);
                object4.color = getRainbow(simulator.return_time());
                simulator.setObjectVelocity(object4, object_initial_speed);
                atom_id++;
            }
            if (simulator.getObjectCount() < max_object_count4 && spawn_delayz(global_time.getElapsedTime(), time_for_next_object + 6.0) == true && spawn_delayz(clock5.getElapsedTime(), spawn_delay5) == true) {
        clock5.restart();
        auto& object5 = simulator.addObject(object_spawn_position5, object_min_radius, atom_id);
        object5.color = getRainbow(simulator.return_time());
        simulator.setObjectVelocity(object5, object_initial_speed);
        atom_id++;
    }

    // Sixth stream
    if (simulator.getObjectCount() < max_object_count5 && spawn_delayz(global_time.getElapsedTime(), time_for_next_object + 8.0) == true && spawn_delayz(clock6.getElapsedTime(), spawn_delay6) == true) {
        clock6.restart();
        auto& object6 = simulator.addObject(object_spawn_position6, object_min_radius, atom_id);
        object6.color = getRainbow(simulator.return_time());
        simulator.setObjectVelocity(object6, object_initial_speed);
        atom_id++;
    }

    // Seventh stream
    if (simulator.getObjectCount() < max_object_count6 && spawn_delayz(global_time.getElapsedTime(), time_for_next_object + 10.0) == true && spawn_delayz(clock7.getElapsedTime(), spawn_delay7) == true) {
        clock7.restart();
        auto& object7 = simulator.addObject(object_spawn_position7, object_min_radius, atom_id);
        object7.color = getRainbow(simulator.return_time());
        simulator.setObjectVelocity(object7, object_initial_speed);
        atom_id++;
    }

    // Eighth stream
    if (simulator.getObjectCount() < max_object_count7 && spawn_delayz(global_time.getElapsedTime(), time_for_next_object + 12.0) == true && spawn_delayz(clock8.getElapsedTime(), spawn_delay8) == true) {
        clock8.restart();
        auto& object8 = simulator.addObject(object_spawn_position8, object_min_radius, atom_id);
        object8.color = getRainbow(simulator.return_time());
        simulator.setObjectVelocity(object8, object_initial_speed);
        atom_id++;
    }

    // Ninth stream
    if (simulator.getObjectCount() < max_object_count8 && spawn_delayz(global_time.getElapsedTime(), time_for_next_object + 14.0) == true && spawn_delayz(clock9.getElapsedTime(), spawn_delay9) == true) {
        clock9.restart();
        auto& object9 = simulator.addObject(object_spawn_position9, object_min_radius, atom_id);
        object9.color = getRainbow(simulator.return_time());
        simulator.setObjectVelocity(object9, object_initial_speed);
        atom_id++;
    }
    

 }

    









//SHADER VARIABLES



simulator.update(60);
window.clear(sf::Color::Black);
// renders.renders_VBO(simulator);
renders.temp_render();
// renders.renders(simulator);
window.display();




}

return 0;
}

