#include <SFML/Graphics.hpp>
#include <iostream>
#include <math.h>
#include <cmath>

void getDistance(sf::Vector2f distance, sf::Vector2f object1, sf::Vector2f Object2)
{

    float distance_x = object1.x - Object2.x;
    float distance_y = object1.y - Object2.y;
    
    distance = {distance_x,distance_y};

};
int main()
{
    sf::Vector2f Window_Screen_Bounds = {1000,1000};
    sf::RenderWindow window(sf::VideoMode(1000, 1000), "Rotation test");
    window.setFramerateLimit(60);


sf::Vector2f pos = {450.0f,450.0f};
sf::Vector2f point = {500.0f,500.0f};
float vel;


//clock
sf::Clock clock;

//velocity math
float radius = 500;
float rotation_speed = M_PI / 6;

//Rectangle
sf::RectangleShape rectangle(sf::Vector2f(50.f, 120.f));
rectangle.setSize(sf::Vector2f(65.0f, 10.0f));
rectangle.setPosition((pos));

// distance varibles 
sf::Vector2f distance; 
float dy_arc_length = 0;
    while (window.isOpen())
    {
sf::Time time = clock.getElapsedTime();
vel +=  rotation_speed;
float angle = rectangle.getRotation();

dy_arc_length +=  M_PI * angle / 360; 


distance.x = dy_arc_length / angle * radius; 
distance.y = dy_arc_length /  angle;


        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed) window.close();

            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape)) window.close();
        }


        window.clear();
        
       rectangle.setRotation((vel));
        //draw calls
        window.draw(rectangle);
        window.display();
        // std::cerr << rectangle.getPosition().x << std::endl;
          std::cerr << distance.x << std::endl;
         // std::cerr << distance.y << std::endl;
        // std::cerr << dy_arc_length << std::endl;
        //  std::cerr << angle << std::endl;
        
    }



    return 0;
}
