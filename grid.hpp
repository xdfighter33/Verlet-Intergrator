#include <iostream>
#include <stdint.h>
#include <unordered_map>
class SpatialHashing{

private:
    int height;
    float cell_size = 25; 
    float min = 0;
    float max = 100;
    float distance_check = 5;
    int width = (max - min) / cell_size;
    int grid_cell;
    int buckets = width * width;
    float conversion_factor = float(1) / cell_size;
    std::unordered_map< int, std::vector<std::pair<sf::Vector2f, uint32_t>>> grids;
    std::vector<uint32_t> atom_idx;
public: 
   

    void set_grid_cell(int x, int y){

        grid_cell = floor(x/ cell_size )  / floor (y/ cell_size ) * width;
    }



    void set_grid_cell(sf::Vector2f pos){
     //   grid_cell = floor(pos.x / cell_size) + floor(pos.y / cell_size) * width; 

        grid_cell = (pos.x * conversion_factor) + (pos.y * conversion_factor) * width;
    }

  

    void add_object(sf::Vector2f test,uint32_t idx){
        set_grid_cell(test);
        
        atom_idx.push_back(idx);
        grids[grid_cell].emplace_back(test,idx);

    }   
    

    void clear(){

        for(auto& pairs : grids){

            pairs.second.clear();
        }
        grids.clear();

        std::vector<uint32_t>().swap(atom_idx);
    }
  
    //Calcualte the adjacent cells 
    void cell_in_range(){


        
    }


    //Test function 
void print_buckets() {
    for (const auto& pair : grids) {
        std::cout << "Bucket " << pair.first << " contains " << pair.second.size() << " objects:" << std::endl;
        for (const auto& obj_pair : pair.second) {
            std::cout << obj_pair.second << " " << obj_pair.first.x << ", " << obj_pair.first.y << std::endl;
        }
    }
}


    void print_atom_idx(){

        for(auto& obj : atom_idx){
            std::cout << obj << std::endl;
        }


    }

    void return_max_size(){
     std::cout << "size of the atoms is " << atom_idx.size() << std::endl;
    }


    const std::unordered_map<int, std::vector<std::pair<sf::Vector2f, uint32_t>>>& getGrids()  {
        return grids;
    }

    //ISSUE IS COMING FROM GET ATOM_IDX IS DIFFERENT THAN ACTUAL VECTOR OF OBJECTS 
    // THIS JUST ITEIRATES THROUGH THE OBJECTID OF THE FIRST INSTANCE 
uint32_t getObjectID(const sf::Vector2f& pos) {
    set_grid_cell(pos);
    const auto& bucket = grids[grid_cell];
    const auto& iter = std::find_if(bucket.begin(), bucket.end(), [&pos](const std::pair<sf::Vector2f, uint32_t>& pair) {
        return pair.first == pos;
    });

    if (iter != bucket.end()) {
        return iter->second;
    }

    return 0; // Return 0 if the position is not found (or any other appropriate value)
};

};
