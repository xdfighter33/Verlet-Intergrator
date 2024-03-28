#include <iostream>
#include <stdint.h>
#include <unordered_map>
class SpatialHashing{
<<<<<<< HEAD

private:
    int height;
    float cell_size = 25; 
    float min = 0;
    float max = 100;
    float distance_check = 5;
    int width = (max - min) / cell_size;
    int grid_cell;
    std::unordered_map< int, std::vector<sf::Vector2f>> grids;
public: 
   
    void set_grid_cell(int x, int y){

        grid_cell = floor(x/cell_size) / floor(y/cell_size) * width;
    }



    void set_grid_cell(sf::Vector2f pos){

        grid_cell = floor(pos.x/cell_size) / floor(pos.y/cell_size) * width;
    }


    void add_object(sf::Vector2f test){
        set_grid_cell(test);
            
        grids[grid_cell].push_back(test);

    }   
    

    void clear(){

        for(auto& pairs : grids){

            pairs.second.clear();
        }
        grids.clear();
    }
  

    //Test function 
    void print_buckets() {
        for (const auto& pair : grids) {
            std::cout << "Bucket " << pair.first << " contains " << pair.second.size() << " objects:" << std::endl;
            for (const auto& obj : pair.second) {
                std::cout << "  " << obj.x << ", " << obj.y << std::endl;
            }
        }
    }

    void check_particles(){
        for(const auto& pair : grids ){
            
            for(const auto& obj : pair.second){
                

            }
        }

    }

};
=======
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> main

private:
    int height;
    float cell_size = 25; 
    float min = 0;
    float max = 100;
    float distance_check = 5;
    int width = (max - min) / cell_size;
    int grid_cell;
    std::unordered_map< int, std::vector<sf::Vector2f>> grids;
public: 
   
    void set_grid_cell(int x, int y){

        grid_cell = floor(x/cell_size) / floor(y/cell_size) * width;
    }



    void set_grid_cell(sf::Vector2f pos){

        grid_cell = floor(pos.x/cell_size) / floor(pos.y/cell_size) * width;
    }


    void add_object(sf::Vector2f test){
        set_grid_cell(test);
            
        grids[grid_cell].push_back(test);

    }   
    

    void clear(){

        for(auto& pairs : grids){

            pairs.second.clear();
        }
        grids.clear();
    }
  

    //Test function 
    void print_buckets() {
        for (const auto& pair : grids) {
            std::cout << "Bucket " << pair.first << " contains " << pair.second.size() << " objects:" << std::endl;
            for (const auto& obj : pair.second) {
                std::cout << "  " << obj.x << ", " << obj.y << std::endl;
            }
        }
    }

    void check_particles(){
        for(const auto& pair : grids ){
            
            for(const auto& obj : pair.second){
                

            }
        }

    }

};
>>>>>>> bd89332 (Setting up GRID)

private:
    int height;
    float cell_size = 25; 
    float min = 0;
    float max = 100;
    float distance_check = 5;
    int width = (max - min) / cell_size;
    int grid_cell;
    std::unordered_map< int, std::vector<sf::Vector2f>> grids;
public: 
   
    void set_grid_cell(int x, int y){

        grid_cell = floor(x/cell_size) / floor(y/cell_size) * width;
    }



    void set_grid_cell(sf::Vector2f pos){

        grid_cell = floor(pos.x/cell_size) / floor(pos.y/cell_size) * width;
    }


    void add_object(sf::Vector2f test){
        set_grid_cell(test);
            
        grids[grid_cell].push_back(test);

    }   
    

    void clear(){

        for(auto& pairs : grids){

            pairs.second.clear();
        }
        grids.clear();
    }
  

    //Test function 
    void print_buckets() {
        for (const auto& pair : grids) {
            std::cout << "Bucket " << pair.first << " contains " << pair.second.size() << " objects:" << std::endl;
            for (const auto& obj : pair.second) {
                std::cout << "  " << obj.x << ", " << obj.y << std::endl;
            }
        }
    }

    void check_particles(){
        for(const auto& pair : grids ){
            
            for(const auto& obj : pair.second){
                

            }
        }

    }

};
>>>>>>> bd89332 (Setting up GRID)

private:
    int height;
    float cell_size = 25; 
    float min = 0;
    float max = 100;
    float distance_check = 5;
    int width = (max - min) / cell_size;
    int grid_cell;
    std::unordered_map< int, std::vector<sf::Vector2f>> grids;
public: 
   
    void set_grid_cell(int x, int y){

        grid_cell = floor(x/cell_size) / floor(y/cell_size) * width;
    }



    void set_grid_cell(sf::Vector2f pos){

        grid_cell = floor(pos.x/cell_size) / floor(pos.y/cell_size) * width;
    }


    void add_object(sf::Vector2f test){
        set_grid_cell(test);
            
        grids[grid_cell].push_back(test);

    }   
    

    void clear(){

        for(auto& pairs : grids){

            pairs.second.clear();
        }
        grids.clear();
    }
  

    //Test function 
    void print_buckets() {
        for (const auto& pair : grids) {
            std::cout << "Bucket " << pair.first << " contains " << pair.second.size() << " objects:" << std::endl;
            for (const auto& obj : pair.second) {
                std::cout << "  " << obj.x << ", " << obj.y << std::endl;
            }
        }
    }

    void check_particles(){
        for(const auto& pair : grids ){
            
            for(const auto& obj : pair.second){
                

            }
        }

    }

};

class Grids{
private:
	int width;
	int height; 
	std::vector<std::vector<char>> grid;


public: 

	Grids(int w, int h) : width(w), height(h), grid(h, std::vector<char>(w, '.')) {}
};