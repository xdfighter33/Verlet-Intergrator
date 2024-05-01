#pragma once
#include <chrono>
#include <vector>
#include <thread>
#include <future>
#include <SFML/Graphics.hpp>
#include <line.hpp>
#include "../Spatial_Hash/collisionCells.hpp"
#include "../Spatial_Hash/tree.hpp"
#include "../Spatial_Hash/grid.hpp"
#include "../Temp/thread.hpp"
#include "Particle.hpp"



class Simulator {

std::vector<particle> m_objects;
std::vector<Line> m_objects_line;

    public:
	Simulator(uint32_t width, uint32_t height)
    : grid(width,height)
    , world_size((float)width,(float)height)
    {
        grid_struct.clear();
    };






// add_object with the particle ID
particle& addObject(sf::Vector2f position, float radius, float idx){

    particle newParticle(position, radius,idx);
    
    m_objects.push_back(newParticle);

    return m_objects.back();
}

// Add_object with no particle ID (No collsion with hash map)
particle& addObject(sf::Vector2f position, float radius)
{


    particle newParticle(position, radius);
    m_objects.push_back(newParticle);
    return m_objects.back();
}



// Add line object 
Line& add_Line_Object(sf::Vector2f pos, float rot_speed, sf::Vector2f Size ){

Line newLine(pos,rot_speed,Size);

m_objects_line.push_back(newLine);


return m_objects_line.back();

}

//Test function to add  center line with a line that stops at box constraint 
void add_center_line_with_line(sf::Vector2f spawn_pos, sf::Vector2f Size, float speed)
{


auto& obj_line = add_Line_Object(spawn_pos,speed,Size);



// Calculate size based on the box constraint
// First get box constraints size
// get the distance between center line and box constraint 

sf::Vector2f distance;

distance.x = spawn_pos.x * spawn_pos.x + box_constraint.x * box_constraint.x;
distance.y = spawn_pos.y * spawn_pos.y + box_constraint.y * box_constraint.y;
auto& line_segment = add_Line_Object(spawn_pos - sf::Vector2f(-5,-5),speed,sf::Vector2f(distance));
}


// Add center line to project
void add_center_line(sf::Vector2f spawn_pos, sf::Vector2f Size, float speed)
{


auto& obj_line = add_Line_Object(spawn_pos,speed,Size);


}


 //Add the amount of objects you eed 
 void Add_all_objects(sf::Vector2f spawn_pos, float radius, uint32_t count){
    for(int i = 0; i < count; i++){
        
        float off_set =  500 / i;
        float temp = std::clamp(i + 1 ,-500,0);
        spawn_pos.x = spawn_pos.x + off_set;
        spawn_pos.y -= box_constraint.y;
        auto& obj = addObject(spawn_pos,radius,i);
        obj.set_color(sf::Color::Blue);
       setObjectVelocity(obj,sf::Vector2f(-5,0));

    }
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
    const std::vector<Line>& getObjectLine() const 
    {
        return m_objects_line;
    }
void update(float dt)
{
    m_time += m_frame_dt;

    float step_dt = getStepDt();
    const float sub_dt = dt / static_cast<float>(m_sub_steps);
    for(uint32_t i{m_sub_steps}; i--;)
    {
 
    
    addObjToGrid();
    //checkCollsion(step_dt);
  //  check_spatial_collision();
   multi_thread_check_spatial_collision();
//   applyRotatoionGravity(step_dt);
  //  fricition(step_dt);
  //  drag_force(step_dt);
   Multi_updateConstraintObjects(step_dt);

  
 
       

    
    }
}

void update()
{
    m_time += m_frame_dt;

    float step_dt = getStepDt();

    for(uint32_t i{m_sub_steps}; i--;)
    {
 
    
    addObjToGrid();
    //checkCollsion(step_dt);
  //  check_spatial_collision();
   multi_thread_check_spatial_collision();

  //  fricition(step_dt);
  //  drag_force(step_dt);
   Multi_updateConstraintObjects(step_dt);

  
 
       

    
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

void setBoxConstraint(sf::Vector2f  dim){
    box_constraint.x = dim.x;
    box_constraint.y = dim.y;
}

float getStepDt() const{
    return m_frame_dt /static_cast<float>(m_sub_steps);
}


uint64_t getObjectCount() const{
    return m_objects.size();
}

sf::Vector2f getBoxConstraint() const {
    return box_constraint;
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

std::mutex m_objectMutex;
sf::Vector2f world_size{100,100};
sf::Vector2f m_gravity = {0,3050.0f};
sf::Vector2f Mass = {500,500};
sf::Vector2f box_constraint;
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
    float rotate = m_time;
    float speed = 50;
    float grav_speed = 1.05;
    float test_x =  speed * sin(360 + rotate) - speed * cos(360 + rotate);
    float test_y = speed  * sin(360 + rotate) + speed * cos(360 + rotate);
    sf::Vector2f Temp(test_x,test_y);
    m_gravity = grav_speed  *  Temp;
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

void check_collision_grid(uint32_t idx1, uint32_t idx2){

    const float response_coef = 1.0f;
    particle& obj_1 = m_objects[idx1];
    particle& obj_2 = m_objects[idx2];


 
    const sf::Vector2f v = obj_1.pos - obj_2.pos;
    
    const float dist2 = v.x * v.x + v.y * v.y;
    const float min_dit = obj_1.radius + obj_2.radius;

    const float  min_dist_square = min_dit * min_dit;


    if(dist2 < min_dist_square)
    {
                    const float        dist  = sqrt(dist2);
                    const sf::Vector2f n     = v / dist;
                    const float mass_ratio_1 = obj_1.radius / (obj_1.radius + obj_2.radius);
                    const float mass_ratio_2 = obj_2.radius / (obj_1.radius + obj_2.radius);
                    const float delta        = 0.25f * response_coef * (dist - min_dit);
                    // Update positions
                    // std::lock_guard<std::mutex> lock(m_objectMutex);
                    obj_1.pos -= n * (mass_ratio_2 * delta);
                    obj_2.pos += n * (mass_ratio_1 * delta);

      
    } else {
        return;
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
    
    float strength = 170;
    float radius = 360;
    float speed = 0.5;
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
             if(object.getPos().x >= 990)
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


//Updating m_objects_line
void updateLObjects(float dt){
    // Polar coordinates
    // X = (r) * sin (theta angle)
    // y = (r) * cos (theta angle)
    for(auto& obj : m_objects_line){

        obj.updatePosition(dt);
    }

}
void updateObjects(float dt)
{
    //gravity is programed into this function
    for (auto&  obj : m_objects)
    {   
       obj.accerlate(m_gravity);
      obj.updatePosition(dt);
    }
}


// Mutli threaded update_object 
void Multi_updateObjects(float dt)
{
    const size_t numObjects = m_objects.size();
    const size_t numThreads = std::thread::hardware_concurrency();

    std::vector<std::thread> threads;
    threads.reserve(numThreads);

    size_t chunkSize = numObjects / numThreads;
    size_t remainder = numObjects % numThreads;

    size_t start = 0;
    for (size_t i = 0; i < numThreads; ++i)
    {
        size_t end = start + chunkSize;
        if (i < remainder)
            ++end;

        threads.emplace_back([this, start, end, dt]()
        {
            for (size_t j = start; j < end; ++j)
            {
                auto& obj = m_objects[j];
                obj.accerlate(m_gravity);
                obj.updatePosition(dt);
                sf::Vector2f frictionForce = -5.15f * obj.GetVelocity(dt);
                obj.accerlate(frictionForce);

            //World boundary put in function later 
            if(obj.getPos().y >= 800)
            {
                obj.pos.y += -1;
            }
            if(obj.getPos().y <= 0){
                obj.pos.y += 1;
            }
             if(obj.getPos().x >= 970)
            {
                obj.pos.x += -1;
            }
             if(obj.getPos().x <= 25)
            {
                obj.pos.x += 1;
            }

            }
        });

        start = end;
    }

    for (auto& thread : threads)
        thread.join();
}

void Multi_updateConstraintObjects(float dt)
{
    const size_t numObjects = m_objects.size();
    const size_t numThreads = 5;

    std::vector<std::thread> threads;
    threads.reserve(numThreads);

    size_t chunkSize = numObjects / numThreads;
    size_t remainder = numObjects % numThreads;

    size_t start = 0;
    for (size_t i = 0; i < numThreads; ++i)
    {
        size_t end = start + chunkSize;
        if (i < remainder)
            ++end;

        threads.emplace_back([this, start, end, dt]()
        {
            for (size_t j = start; j < end; ++j)
            {
                auto& obj = m_objects[j];
                obj.accerlate(m_gravity);
                obj.updatePosition(dt);

                // Apply friciton force
                sf::Vector2f frictionForce = -1.15f * obj.GetVelocity(dt);
                obj.accerlate(frictionForce);


            //World boundary put in function later - Use bounding_box 
            float margin = 2.0f; 
            sf::Vector2f v = getBoxConstraint() - obj.pos;
            float dist = sqrt(v.x * v.x + v.y * v.y);
                 if (obj.pos.x > getBoxConstraint().x - margin) {
                    obj.pos.x = getBoxConstraint().x - margin;
                } 
                 else if (obj.pos.x < margin) {
                    obj.pos.x = margin;
                }
                if (obj.pos.y > getBoxConstraint().y - margin) {
                    obj.pos.y = getBoxConstraint().y - margin;
                } else if (obj.pos.y < margin) {
                    obj.pos.y =  margin;
                }

        }
        });

        start = end;
    }

    for (auto& thread : threads)
        thread.join();
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
             std::cout << i << std::endl;
            ++i;
        }
    }


    // Add spatial hashing function to the grid 
    void addObjToGrid(){

        grid_struct.clear();
          for (const auto& obj : m_objects) {
            if (obj.pos.x > 1.0f && obj.pos.x < getBoxConstraint().x - 1.0f &&
                obj.pos.y > 1.0f && obj.pos.y < getBoxConstraint().y - 1.0f){
                    grid_struct.add_object(obj.pos,obj.index);
                }    
           }
  
  // grid_struct.print_buckets();
 
    }

    void test_to_add_to_grid(){

        for(const auto& obj: m_objects){
           std::cout <<  "Testing add object " << grid_struct.getObjectID(obj.pos) << std::endl;
         //   grid_struct.print_atom_idx();
        }


    }

    void check_spatial_collision(){
        
    for(auto& pairs : grid_struct.getGrids()){
    const int cell_index = pairs.first; 
    auto& objects_in_grid = pairs.second;    
   
        for(size_t i  = 0; i < objects_in_grid.size(); i++){
            const auto& obj1 = objects_in_grid[i];
            uint32_t obj1Idx = obj1.second;


             for(size_t j = i + 1; j < objects_in_grid.size(); ++j)   
             {
            const auto& obj2 = objects_in_grid[j];
            uint32_t obj2Idx = obj2.second;

                check_collision_grid(obj1Idx,obj2Idx);

             }


        }         

             for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;

                int neighbor_x = cell_index % grid_struct.getWidth() + dx;
                int neighbor_y = cell_index / grid_struct.getWidth() + dy;

                // Wrap around the grid boundaries
                if (neighbor_x < 0) neighbor_x += grid_struct.getWidth();
                if (neighbor_x >= grid_struct.getWidth()) neighbor_x -= grid_struct.getWidth();
                if (neighbor_y < 0) neighbor_y += grid_struct.getWidth();
                if (neighbor_y >= grid_struct.getWidth()) neighbor_y -= grid_struct.getWidth();

                int neighbor_cell_index = neighbor_y * grid_struct.getWidth() + neighbor_x;

                if (neighbor_cell_index >= 0 && neighbor_cell_index < grid_struct.getGrids().size()) {
                    const auto& neighbor_objects = grid_struct.getGrid(neighbor_cell_index);
                    for (const auto& obj1 : objects_in_grid) {
                        uint32_t obj1IDX = obj1.second;
                        for (const auto& obj2 : neighbor_objects) {
                            uint32_t obj2IDX = obj2.second;
                            if (obj1IDX != obj2IDX) {
                                check_collision_grid(obj1IDX, obj2IDX);
                            }
                        }
                    }
                }
            }
        }
    }
    }

//  pass in GRID IDX of each item in the grid 


//Passing in grid data with start and end 
// each thread should look towards unorederd_maps  start - end using find 
void check_hash_map_collisions(std::unordered_map< int, std::vector<std::pair<sf::Vector2f, uint32_t>>> grids){
    m_objectMutex.lock();
    for(auto& pairs : grids){
        const int cell_index = pairs.first; 
        auto& objects_in_grid = pairs.second;
        for(int i{0}; i < objects_in_grid.size(); i++ ){
            
            const auto& obj1 = objects_in_grid[i];
            uint32_t obj1IDX = obj1.second;
        for(size_t j = i + 1; j < objects_in_grid.size(); ++j)   
             {
            const auto& obj2 = objects_in_grid[j];
            uint32_t obj2IDX = obj2.second;

               check_collision_grid(obj1IDX,obj2IDX);

             }    

        }
         for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
               
                int neighbor_x = cell_index % grid_struct.getWidth() + dx;
                int neighbor_y = cell_index / grid_struct.getWidth() + dy;
               
                // Wrap around the grid boundaries
                if (neighbor_x < 0) neighbor_x += grid_struct.getWidth();
                if (neighbor_x >= grid_struct.getWidth()) neighbor_x -= grid_struct.getWidth();
                if (neighbor_y < 0) neighbor_y += grid_struct.getWidth();
                if (neighbor_y >= grid_struct.getWidth()) neighbor_y -= grid_struct.getWidth();

                int neighbor_cell_index = neighbor_y * grid_struct.getWidth() + neighbor_x;

                if (neighbor_cell_index >= 0 && neighbor_cell_index < grid_struct.getGrids().size()) {
                    const auto& neighbor_objects = grid_struct.getGrid(neighbor_cell_index);
                    for (const auto& obj1 : objects_in_grid) {
                        uint32_t obj1IDX = obj1.second;
                        for (const auto& obj2 : neighbor_objects) {
                            uint32_t obj2IDX = obj2.second;
                            if (obj1IDX != obj2IDX) {
                                check_collision_grid(obj1IDX, obj2IDX);
                            }
                        }
                    }
                }
            }
        }
    }
    m_objectMutex.unlock();
}

//using thread_pool to implement problem 
// Call thread_pool 
// Split the grid up to the amount of threads I want to set to the task
/*
Either this is set at the start of the function 
or call it here to check the differnece between the threads 
*/

void multi_thread_check_spatial_collision(){


    //Discord variable 
    uint32_t num_of_threads = 4; 
    const auto& grids = grid_struct.getGrids();
    const uint32_t grid_width = grid_struct.getWidth();
    const uint32_t num_cells = grids.size();
    
    
    //Split grid into 2's
    auto First_Half_grid = grid_struct.copyHalfMap(grids,true);
    auto Second_Half_grid = grid_struct.copyHalfMap(grids,false);


    // Split grid into quarters 
    // Prob should make this into a function 
    auto First_quarter_grid = grid_struct.copyHalfMap(First_Half_grid,true);
    auto Second_quarter_grid = grid_struct.copyHalfMap(First_Half_grid,false);

    auto Third_quarter_grid = grid_struct.copyHalfMap(Second_Half_grid,true);
    auto Fourth_quarter_grid = grid_struct.copyHalfMap(Second_Half_grid,false);

    std::vector<std::thread> threads;

    std::vector<std::chrono::microseconds> thread_durations(num_of_threads);

    auto start_time = std::chrono::high_resolution_clock::now();



    //Threads in use 
    threads.push_back(std::thread([this, &First_quarter_grid, &thread_durations]() {

     auto start = std::chrono::high_resolution_clock::now();
    this->check_hash_map_collisions(First_quarter_grid);  
    auto end = std::chrono::high_resolution_clock::now();
    thread_durations[0] = std::chrono::duration_cast<std::chrono::microseconds>(end - start);;
    
}));
   threads.push_back(std::thread([this, &Second_quarter_grid, &thread_durations](){
    auto start = std::chrono::high_resolution_clock::now();
    this->check_hash_map_collisions(Second_quarter_grid);
    auto end = std::chrono::high_resolution_clock::now();
    thread_durations[1] = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
}));
    threads.push_back(std::thread([this, &Third_quarter_grid, &thread_durations]() {
    auto start =  std::chrono::high_resolution_clock::now();
    this->check_hash_map_collisions(Third_quarter_grid);
    auto end = std::chrono::high_resolution_clock::now();
    thread_durations[2] = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
}));
   threads.push_back(std::thread([this, &Fourth_quarter_grid, &thread_durations](){
    auto start = std::chrono::high_resolution_clock::now();
    this->check_hash_map_collisions(Fourth_quarter_grid);
    auto end = std::chrono::high_resolution_clock::now();
    thread_durations[3] = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
}));


    for(auto& thread : threads){
        thread.join();
    }

    bool debug_thread_time = false;
    if(debug_thread_time == true){
    auto end_time = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    std::cout << "Multi-threaded execution time: " << total_duration.count() << " microseconds" << std::endl;

    for (size_t i = 0; i < num_of_threads; ++i) {
        std::cout << "Thread " << i << " execution time: " << thread_durations[i].count() << " microseconds" << std::endl;
    }
}
}


void thread_pool_collision() {
    const auto& grids = grid_struct.getGrids();
    const uint32_t num_cells = grids.size();

    // Create a task queue
    std::queue<std::pair<int, int>> taskQueue;

    // Populate the task queue with cell ranges
    int chunk_size = 15;
    int num_chunks = (num_cells + chunk_size - 1) / chunk_size;
    for (int i = 0; i < num_chunks; ++i) {
        int start = i * chunk_size;
        int end = std::min((i + 1) * chunk_size - 1, static_cast<int>(num_cells) - 1);
        taskQueue.push({start, end});
    }

    // Create a mutex for synchronization
    std::mutex queueMutex;

    // Create worker threads
    std::vector<std::thread> threads;
    const uint32_t num_threads = 10;

    for (uint32_t i = 0; i < num_threads; ++i) {
        threads.emplace_back([this, &taskQueue, &queueMutex, &grids, num_cells]() {
            while (true) {
                std::pair<int, int> task;
                {
                    std::unique_lock<std::mutex> lock(queueMutex);
                    if (taskQueue.empty()) {
                        break;
                    }
                    task = taskQueue.front();
                    taskQueue.pop();
                }

                // Process the task
                for (int j = task.first; j <= task.second; ++j) {
                    auto it = grids.find(j);
                    if (it != grids.end()) {
                        const auto& cell = it->second;
                        for (const auto& obj1 : cell) {
                            uint32_t obj1Idx = obj1.second;
                            for (const auto& obj2 : cell) {
                                uint32_t obj2Idx = obj2.second;
                                if (obj1Idx < obj2Idx) {
                                    check_collision_grid(obj1Idx, obj2Idx);
                                }
                            }
                            // Check collisions with neighboring cells
                            // ...
                        }
                    }
                }
            }

            // Work stealing
            while (true) {
                std::pair<int, int> task;
                {
                    std::unique_lock<std::mutex> lock(queueMutex);
                    if (taskQueue.empty()) {
                        break;
                    }
                    task = taskQueue.front();
                    taskQueue.pop();
                }

                // Process the stolen task
                for (int j = task.first; j <= task.second; ++j) {
                    auto it = grids.find(j);
                    if (it != grids.end()) {
                        const auto& cell = it->second;
                        for (const auto& obj1 : cell) {
                            uint32_t obj1Idx = obj1.second;
                            for (const auto& obj2 : cell) {
                                uint32_t obj2Idx = obj2.second;
                                if (obj1Idx < obj2Idx) {
                                    check_collision_grid(obj1Idx, obj2Idx);
                                }
                            }
                            // Check collisions with neighboring cells
                            // ...
                        }
                    }
                }
            }
        });
    }

    // Join the threads
    for (auto& thread : threads) {
        thread.join();
    }
}

};

