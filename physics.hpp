#pragma once

#include <vector>

#include <sfml/Graphics.hpp>


struct particle {

    float radius = 10.f; 

    sf::Vector2f pos;
    sf::Vector2f old_pos;
    sf::Vector2f accel = {0,0};
    sf::Color color = sf::Color::White;

    particle() = default;

    particle(sf::Vector2f position_, float radius_)
    : pos{position_}
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


void accerlate(sf::Vector2f force)
{
    accel+= force;
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


class Simulator {
    public:
    Simulator() = default;

    particle& addObject(sf::Vector2f position, float radius)
    {
        return m_objects.emplace_back(position,radius);
    }

    void setObjectVelocity(particle& object, sf::Vector2f v)
    {
        object.setVelo(v, getStepDt());
    }

    float return_time(){
    return m_time;
    }   


    const std::vector<particle>& getObject() const
    {
        return m_objects;
    }


void update()
{
    m_time += m_frame_dt;

    float step_dt = getStepDt();
    for(uint32_t i{m_sub_steps}; i--;)
    {
     //  AttractToCenter(2,sf::Vector2f(450,450),step_dt);
      applyGravity(step_dt);
    //   circukarMotion(step_dt);
    //    applyRotatoionGravity(step_dt);
        checkCollsion(step_dt);
        appplyConstraint(step_dt);
       // RotateGravity(step_dt);
       fricition(step_dt);
        updateObjects(step_dt);
    }
}

void setSimulationUpdateRate(uint32_t rate)
{
    m_frame_dt = 1.0f / static_cast<float>(rate);
}

void setSubsStepscount(uint32_t sub_substeps)
{
    m_sub_steps = sub_substeps;
}

float getStepDt() const{
    return m_frame_dt /static_cast<float>(m_sub_steps);
}


uint64_t getObjectCount() const{
    return m_objects.size();
}

void rotate_degrees(float degrees){
    rotation =  500 * sin(m_time);
}

float return_rotation(){
    return rotation;
}
 private:
std::vector<particle> m_objects;

sf::Vector2f m_gravity = {0,750.0f};
sf::Vector2f Mass = {500,500};
uint32_t m_sub_steps = 1;
float m_time = 0.0f;
float m_frame_dt = 0.0f;
float rotation = 0;


void AttractToCenter(float strength,sf::Vector2f Posi,float dt)
{
//itterate through all objects
//Very Costly on the system
const uint64_t object_count = m_objects.size();
for(uint64_t i{0}; i < object_count; ++i)
{
particle& object_1 = m_objects[i];

//get Distance from Posi 
float distance_x = Posi.x - object_1.pos.x;
float distance_y = Posi.y - object_1.pos.y; 
float distance = sqrt(distance_x * distance_x + distance_y * distance_y);

float inverse_distance = 1.f / distance;

float normalised_x = inverse_distance * distance_x;
float normalised_y = inverse_distance * distance_y;

float inverse_square_droppoff = inverse_distance * inverse_distance;

///Calulate  Accerlation
sf::Vector2f Accerlation; 

Accerlation.x = normalised_x * strength; 
Accerlation.y = normalised_y * strength; 

//set Accerlation

object_1.addVelocity(Accerlation,dt);


}
 
}


void test_centr(float dt)
{
    sf::Vector2f edge = {1000.0f,1000.0f};

    for (auto& obj : m_objects){
        const int64_t object_count = m_objects.size();
        for(uint64_t i(0); i < object_count; i++ ){
particle& object_1 = m_objects[1];

// Setting up formula 
float distance_x = edge.x - object_1.getPos().x;
float distance_y = edge.y - object_1.getPos().y;

float distance_x_squared = distance_x * distance_x;
float distance_y_squared = distance_y * distance_y;

float force1 = sqrt(distance_x_squared + distance_y_squared);


float tangent = distance_y / distance_x * 180;


sf::Vector2f force = {force1,force1};





object_1.setVelo(force,dt);






        }
    }
}
void applyRotatoionGravity(float dt)
{
    float rotate = m_time ;
    float test_x =  300 * sin(360 + rotate) - 300 * cos(360 + rotate);
    float test_y = 300 * sin( 360 + rotate) + 300 * cos(360 + rotate);
    sf::Vector2f Temp(test_x,test_y);
    m_gravity = Temp;
    for(auto& obj : m_objects){
        obj.accerlate(m_gravity);
    }
}

// Rotation around the boundaries 
void applyGravity(float dt)
{

      for(auto& obj : m_objects){
        obj.accerlate(m_gravity);
    }
 

}


void checkCollsion(float dt)
{
    const float response_coef = 0.75f;
    const uint64_t object_count = m_objects.size();
    for (uint64_t i{0}; i < object_count; ++i) {
            particle& object_1 = m_objects[i];
            // Iterate on object involved in new collision pairs
            for (uint64_t k{i + 1}; k < object_count; ++k) {
                particle&      object_2 = m_objects[k];
                const sf::Vector2f v        = object_1.pos- object_2.pos;
                const float        dist2    = v.x * v.x + v.y * v.y;
                const float        min_dist = object_1.radius + object_2.radius;
                // Check overlapping
                if (dist2 < min_dist * min_dist) {
                    const float        dist  = sqrt(dist2);
                    const sf::Vector2f n     = v / dist;
                    const float mass_ratio_1 = object_1.radius / (object_1.radius + object_2.radius);
                    const float mass_ratio_2 = object_2.radius / (object_1.radius + object_2.radius);
                    const float delta        = 0.5f * response_coef * (dist - min_dist);
                    // Update positions
                    object_1.pos -= n * (mass_ratio_2 * delta);
                    object_2.pos += n * (mass_ratio_1 * delta);
                }
            }
        }
}

void fricition(float dt)
{

for(auto& obj : m_objects){
  sf::Vector2f Friction_force = -.999f *  obj.GetVelocity(dt);

  obj.accerlate(Friction_force);


}
}

void drag_force(float dt){

    for(auto& obj : m_objects){
        
    }
}
void circukarMotion(float dt){
    

    float test_x =  300 * sin(360 + m_time) - 300 * cos(360 + m_time);
    float test_y = 300 * sin( 360 + m_time) + 300 * cos(360 + m_time);

 // angluar_accerlation = angluar_accerlation / rad ;
       const uint64_t object_count = m_objects.size();
    for (auto&  obj : m_objects) {

            obj.setVelo(sf::Vector2f(test_x,test_y),dt);

    }   

}

void centriPetalforce(float dt){
}


void appplyConstraint(float dt)
{
    for (auto& obj : m_objects){
        const uint64_t object_count = m_objects.size();

        for (uint64_t i{0}; i < object_count; i++)
        {
            particle& object = m_objects[i];
            if(object.getPos().y >= 800)
            {
                object.pos.y += -1;
            }
            if(object.getPos().y <= 0)
            {
                object.pos.y += 1;
            }
             if(object.getPos().x >= 950)
            {
                object.pos.x += -1;
            }
             if(object.getPos().x <= 25)
            {
                object.pos.x += 1;
            }
        }
    }
}

void updateObjects(float dt)
{
    for (auto&  obj : m_objects)
    {
        obj.updatePosition(dt);
    }
}
};

