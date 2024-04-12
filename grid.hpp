#include <iostream>
#include <stdint.h>
#include <unordered_map>
class SpatialHashing{

private:
    int height = 1000;
    int cell_size = 80; 
    int min = 0;
    int max = 1000;
    float distance_check = 5;
    int width = (max - min) / cell_size;
    int grid_cell;
    int buckets = width * width;
    int conversion_factor = 1 / cell_size;
    std::unordered_map< int, std::vector<std::pair<sf::Vector2f, uint32_t>>> grids;
public: 
   

    void set_grid_cell(int x, int y){

        grid_cell = floor(x/ cell_size )  / floor (y/ cell_size ) * width;
    }



void set_grid_cell(sf::Vector2f pos) {
    int col = static_cast<int>(pos.x) / cell_size;
    int row = static_cast<int>(pos.y) / cell_size;
    grid_cell = row * width + col;
}

  

    void add_object(sf::Vector2f test,uint32_t idx){
        set_grid_cell(test);
 
        grids[grid_cell].emplace_back(test,idx);

    }   
    

    void clear(){

        for(auto& pairs : grids){

            pairs.second.clear();
        }
        grids.clear();
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





    const std::unordered_map<int, std::vector<std::pair<sf::Vector2f, uint32_t>>>& getGrids()  {
        return grids;
    }


//Object id uses a pair to POS and uint32_t IDX 

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

 int getWidth(){
    return width;
 }
const std::vector<std::pair<sf::Vector2f, uint32_t>>& getGrid(int index) {
    if (index >= 0 && index < static_cast<int>(grids.size())) {
        return grids[index];
    } else {
        // Handle out-of-bounds index
        static const std::vector<std::pair<sf::Vector2f, uint32_t>> empty_vector;
        return empty_vector;
    }
}

};
