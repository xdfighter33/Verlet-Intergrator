#include <sfml/Graphics.hpp>

sf::Vector2f Grav_source_pos = {800,450};

class GravitySource {
    sf::Vector2f position;
    float strength;
    sf::CircleShape s;



public:
    GravitySource(float position_x = Grav_source_pos.x, float position_y = Grav_source_pos.y, float strength = 2000) {
        position.x = position_x;
        position.y = position_y;
        this->strength = strength;

        s.setPosition(position);
        s.setFillColor(sf::Color::White);
        s.setRadius(strength * 0.003);
    }

    void render(sf::RenderWindow& wind) {
        s.setPosition(position);
        wind.draw(s);
    }

    void set_pos(sf::Vector2f posi)
    {
        position.x = posi.x;
        position.y = posi.y;
        s.setPosition(position);
    }

    sf::Vector2f get_position() {
        return position;
    }

    float get_strength() {
        return strength;
    }
};

class Particle {
    sf::Vector2f position;
    sf::Vector2f velocity;
    sf::CircleShape s;




public:
    Particle(float position_x = rand() % 160, float position_y = rand() % 100, float velocity_x = (((rand() & 100) * 0.02) - 1), float velocity_y = (((rand() & 100) * 0.02) - 1),sf::Color c = sf::Color(rand() % 255, rand() % 255, rand() % 255)) {
        position.x = position_x;
        position.y = position_y;

        velocity.x = velocity_x;
        velocity.y = velocity_y;

        s.setPosition(position);
        s.setFillColor(c);
        s.setRadius(3);
    }

    void render(sf::RenderWindow& wind) {
        s.setPosition(position);
        wind.draw(s);
    }

    void update_physics(GravitySource& s) {
        float distance_x = s.get_position().x - position.x;
        float distance_y = s.get_position().y - position.y;

        float distance = sqrt(distance_x * distance_x + distance_y * distance_y);

        float inverse_distance = 1.f / distance;

        float normalised_x = inverse_distance * distance_x;
        float normalised_y = inverse_distance * distance_y;

        float inverse_square_droppoff = inverse_distance * inverse_distance;

        float acceleration_x = normalised_x * s.get_strength() * inverse_square_droppoff;
        float acceleration_y = normalised_y * s.get_strength() * inverse_square_droppoff;

        velocity.x += acceleration_x;
        velocity.y += acceleration_y;

        position.x += velocity.x;
        position.y += velocity.y;


    }

};



int main()
{
    sf::RenderWindow window(sf::VideoMode(1600, 1000), "My Program");
    window.setFramerateLimit(60);

    int const num_gravity_sources = 1;
    int const num_particles = 3000;

    GravitySource source[num_gravity_sources];
    GravitySource source1[num_gravity_sources];
    Particle particle[num_particles];
    Particle particl2[num_particles];

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed) window.close();

            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape)) window.close();
        }


        window.clear();

        for (int i = 0; i < num_particles; i++) {
            for (int j = 0; j < num_gravity_sources; j++) {
                particle[i].update_physics(source[j]);
                particle[i].update_physics(source1[j]);
                particl2[i].update_physics(source1[j]); 
                particl2[i].update_physics(source[j]);
                }
            particle[i].render(window);
            //particl2[i].render(window);
            
        }
        

        for (int j = 0; j < num_gravity_sources; j++) {
            sf::Vector2f position_source2 = {600,450};
            

        

            source[j].render(window);
            source1[j].render(window);
            source[j].set_pos(position_source2);
            source1[j].set_pos(Grav_source_pos);
        

            
        }
        
        
        //draw calls
        window.display();
    }



    return 0;
}
