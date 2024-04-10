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
    float conversion_factor = float(1) / 10000;
    std::unordered_map< int, std::vector<sf::Vector2f>> grids;
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
        grids[grid_cell].push_back(test);

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
            for (const auto& obj : pair.second) {
                std::cout << getObjectID(sf::Vector2f(obj.x,obj.y)) << "  " << obj.x << ", " << obj.y << std::endl;
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


    const std::unordered_map<int, std::vector<sf::Vector2f>>& getGrids()  {
        return grids;
    }


    uint32_t getObjectID(const sf::Vector2f& pos) {
    set_grid_cell(pos);
    const std::vector<sf::Vector2f>& bucket = grids[grid_cell];
    const auto& iter = std::find(bucket.begin(), bucket.end(), pos);
    if (iter != bucket.end()) {
        size_t index = std::distance(bucket.begin(), iter);
        return atom_idx[index];
    
        }
        
};

};
