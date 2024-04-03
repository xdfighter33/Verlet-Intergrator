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
    std::unordered_map< int, std::vector<sf::Vector2f>> grids;
    std::vector<uint32_t> atom_idx;
public: 
   

    void set_grid_cell(int x, int y){

        grid_cell = floor(x/cell_size) / floor(y/cell_size) * width;
    }



    void set_grid_cell(sf::Vector2f pos){

        grid_cell = floor(pos.x/cell_size) / floor(pos.y/cell_size) * width;
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
        atom_idx.clear();
    }
  
    //Calcualte the adjacent cells 
    void cell_in_range(){


        
    }


    //Test function 
    void print_buckets() {
        for (const auto& pair : grids) {
            std::cout << "Bucket " << pair.first << " contains " << pair.second.size() << " objects:" << std::endl;
            for (const auto& obj : pair.second) {
                std::cout << "  " << obj.x << ", " << obj.y << std::endl;
            }
        }

       std::cout << atom_idx.size() << std::endl;

        
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

    
};
