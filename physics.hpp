
#pragma once

#include <vector>
#include <SFML/Graphics.hpp>
#include "collisionCells.hpp"
#include "tree.hpp"
#include "grid.hpp"
struct particle {
    float radius = 5.0f; 

    sf::Vector2f pos;
    sf::Vector2f old_pos;
    sf::Vector2f accel = {0,0};
    sf::Color color = sf::Color::White;

    //Grid index 
    uint32_t index;
   particle() : index(0) {}

    particle(sf::Vector2f position_, float radius_)
    : pos{position_}
    , index(0)
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

std::vector<particle> m_objects;



    public:
  //  Simulator() = default;
    
	Simulator(uint32_t width, uint32_t height)
    : grid(width,height)
    , world_size((float)width,(float)height)
    {
        grid.clear();
    };



  








particle& addObject(sf::Vector2f position, float radius)
{
    particle newParticle(position, radius);
    m_objects.push_back(newParticle);
    return m_objects.back();
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
 
    
    //     addObjectsToGrid();
         addObjToGrid();
      checkCollsion(step_dt);
    AttractToCenter(2,sf::Vector2f(250,250),step_dt);
            //    checkGridData();
        //  applyRotatoionGravity(step_dt);
         appplyConstraint(step_dt);
   //   fricition(step_dt);
      drag_force(step_dt);

    updateObjects(step_dt);
       // RotateGravity(step_dt);
 
        
   //     updateObjects(step_dt);
       

    
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
CollisionGrid grid;
SpatialHashing grid_struct;

sf::Vector2f world_size{100,100};
sf::Vector2f m_gravity = {0,750.0f};
sf::Vector2f Mass = {500,500};
uint32_t m_sub_steps = 1;
float m_time = 0.0f;
float m_frame_dt = 0.0f;
float rotation = 0;
float response_coef = 0.8f;
float vel_coef = 0.0025f;
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

void visualizeGrid(const CollisionGrid& grid) {
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            const uint32_t id = y * grid.width + x;
            if (grid.data[id].objects_count > 0) {
                std::cout << "# "; // Print '#' for cells with atoms
            } else {
                std::cout << ". "; // Print '.' for empty cells
            }
        }
        std::cout << std::endl;
    }
}


void checkCollsion(float dt)
{
    const float response_coef = 1.7f;
    const uint64_t object_count = m_objects.size();
    for (uint64_t i{0}; i < object_count; ++i) {
            particle& object_1 = m_objects[i];
            // Iterate on object involved in new collision pairs
            for (uint64_t k{i + 1}; k < object_count; ++k) {
                particle&      object_2 = m_objects[k];
                const sf::Vector2f v        = object_1.pos - object_2.pos;
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
  sf::Vector2f Friction_force = -.99f *  obj.GetVelocity(dt);

  obj.accerlate(Friction_force);


}
}

void drag_force(float dt){

float C = 0.0025;
sf::Vector2f velo;   
    for(auto& obj : m_objects){
            
            velo = obj.GetVelocity(dt);
            velo.x -= 0.5f;
            velo.y -= 0.5f;

            float speed = sqrt(velo.x * velo.x + velo.y * velo.y);

            float dragForceMagnitude = 0.5 * 2.5 * speed * speed * 0.5 * 0.5;

            sf::Vector2f dragForceDirection = -velo / speed;

    sf::Vector2f dragForce = dragForceMagnitude * dragForceDirection * C;
    obj.accerlate(dragForce);

    }
}
void circukarMotion(float dt){
    
    float strength = 360;
    float radius = 360;
    float speed = 1.0;
    float test_x =  strength * sin(radius + m_time * speed) + m_time - strength * cos(radius + m_time * speed );
    float test_y = strength * sin( radius + m_time * speed ) + strength * cos(radius + m_time * speed);

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
    //gravity is programed into this function
    for (auto&  obj : m_objects)
    {   
     //  obj.accerlate(m_gravity);
        obj.updatePosition(dt);
    }
}
 
    //QUAD TREE ADDS
    void solveContact(uint32_t atom_1_idx, uint32_t atom_2_idx)
    {

  

        constexpr float response_coef =  0.9;
        constexpr float eps           = 0.001f;
        particle& obj_1 = m_objects[atom_1_idx];
        particle& obj_2 = m_objects[atom_2_idx];
        const sf::Vector2f v        = obj_1.pos - obj_2.pos;
        const float        distance2    = v.x * v.x + v.y * v.y;
        const float        min_dist = obj_1.radius + obj_2.radius;
    
    
        const sf::Vector2f o2_o1  =  obj_1.pos - obj_2.pos;
        const float dist2 = o2_o1.x * o2_o1.x + o2_o1.y * o2_o1.y;
     const float min_dist_squared = (obj_1.radius + obj_2.radius) * (obj_1.radius + obj_2.radius);

    
            const float dist = sqrt(dist2);


     
        if (dist2 < min_dist * min_dist) {
         //  std::cout << "WORKING" << std::endl;
    
            const float overlap = min_dist_squared - dist2;
            const float delta  = .5 * response_coef * (1.0f - dist);
            const sf::Vector2f col_vec = (o2_o1 / dist) * delta;
            const sf::Vector2f n     = v / dist;
            const float mass_ratio_1 = obj_1.radius / (obj_1.radius + obj_2.radius);
             const float mass_ratio_2 = obj_2.radius / (obj_1.radius + obj_2.radius);

            obj_1.pos += n;
            obj_2.pos -= n;
        }
    }

      void checkAtomCellCollisions(uint32_t atom_idx, const CollisionCell& c)
    {

        for (uint32_t i{0}; i < c.objects_count; ++i) {

           
            solveContact(atom_idx, c.objects[i]);
            
        }
    }

        void processCell(const CollisionCell& c, uint32_t index)
    {
        for (uint32_t i{0}; i < c.objects_count; ++i) {
           
            const uint32_t atom_idx = c.objects[i];
     //      std::cout << "Checking atom index: " << atom_idx << std::endl;
     //      std::cout << "C Object I: " << c.objects[i] << std::endl;

    
            checkAtomCellCollisions(atom_idx, grid.data[index - 1]);
            checkAtomCellCollisions(atom_idx, grid.data[index]);        
            checkAtomCellCollisions(atom_idx, grid.data[index + 1]);
            checkAtomCellCollisions(atom_idx, grid.data[index + grid.height - 1]);
            checkAtomCellCollisions(atom_idx, grid.data[index + grid.height    ]);
            checkAtomCellCollisions(atom_idx, grid.data[index + grid.height + 1]);
            checkAtomCellCollisions(atom_idx, grid.data[index - grid.height - 1]);
            checkAtomCellCollisions(atom_idx, grid.data[index - grid.height    ]);
            checkAtomCellCollisions(atom_idx, grid.data[index - grid.height + 1]);
        }
    }

    void checkGridData() {
    for (int x = 0; x < 20; ++x) {
        for (int y = 0; y < 20; ++y) {
            // Access the CollisionCell at position (x, y) in the grid
            const CollisionCell& cell = grid.data[x + y * grid.width];

            // Print information about the cell
            std::cout << "Cell (" << x << ", " << y << "):" << std::endl;
            std::cout << "Objects count: " << cell.objects_count << std::endl;
            std::cout << "Objects: ";
            for (uint32_t i = 0; i < cell.cell_capacity; ++i) {
                std::cout << cell.objects[i] << " ";
            }
            std::cout << std::endl;
        }
    }
}
    void solveCollisions()
    {
        
        uint32_t i{1};
        for (auto& cell : grid.data) {
     
            processCell(cell, i);
            ++i;
        }
    }

void check_cell_collisions(CollisionCell& cell1, CollisionCell& cell2){
    for(auto& obj_indx_1  : cell1.objects){
        for(auto& obj_indx_2 : cell2.objects){
            if(obj_indx_1 != obj_indx_2){
                solveContact(obj_indx_1,obj_indx_2);
            }
        }
    }

}


void find_collision_grid(){
    for(int x{1}; x < grid.width - 1; x++){
         for(int y{1}; y < grid.height - 1; y++){
                auto& current_cell = grid.get(x,y);
                    for(int dx{-1}; dx <= 1; ++dx){
                         for(int dy{-1}; dy <= 1; ++dy){
                            auto& other_cell = grid.get(x + dx, y + dy);

                            check_cell_collisions(current_cell,other_cell);
                     }  
                }
           }
    }
}


    void addObjectsToGrid()
    {
        grid.clear();
        // Safety border to avoid adding object outside the grid
        uint32_t i{0};
        for (const particle& obj : m_objects) {
            if (obj.pos.x > 1.0f && obj.pos.x < world_size.x - 1.0f &&
                obj.pos.y > 1.0f && obj.pos.y < world_size.y - 1.0f) {
               // std::cout << "SUCCESS \n";
                grid.addAtom(obj.pos, i);
            }
            ++i;
        }
    }

    void addObjToGrid(){
        
        grid_struct.clear();
        uint32_t i{ 0 };
          for (const particle& obj : m_objects) {
            if (obj.pos.x > 1.0f && obj.pos.x < world_size.x - 1.0f &&
                obj.pos.y > 1.0f && obj.pos.y < world_size.y - 1.0f){

                    grid_struct.add_object(obj.pos,i);
                }
            ++i;
           }
        grid_struct.print_buckets();
     
    }


    void check_spatial_collision(float dt){
        
    for(auto& pairs : grid_struct.getGrids()){

    auto& objects_in_grid = pairs.second;    

        for(size_t i  = 0; i < objects_in_grid.size(); i++){
             for(size_t j = i + 1; j < objects_in_grid.size(); ++j)   
             {
                
             }

        }         
    
    }
    
    }
};



