#pragma once

#include "../Physics/physics.hpp"
#include "SFML/Graphics.hpp"
#include "../Temp/thread.hpp"
class render
{
    public:
    sf::VertexArray object_va; 
    sf::VertexArray world_va;
    sf::Texture object_texture;
    sf::Image object_pixel;
    Simulator& sim;
    explicit
    render(sf::RenderTarget& target,Simulator& sims)
    :m_target{target}
    , sim{sims}
    , object_va{sf::Quads}

    {
        object_texture.loadFromFile(std::string(SHADER_DIR) + "/circle.png");
        object_texture.generateMipmap();
        object_texture.setSmooth(true);

      //  object_pixel = object_texture.copyToImage();
    
    }


// Pass in the objects for the line
// Maybe obj_line from physics.hpp?
// use for loop to iteriate through every line
// Use vertices after testing rectangle shape
// use rectangle shape to test
// Rotate the object 
void render_line(const Simulator& simulator) const
{


auto constraint = simulator.getBoxConstraint();


sf::RectangleShape line;
const auto& lines = simulator.getObjectLine();

for (const auto& obj_line : lines)
{
    
line.setPosition(obj_line.pos);
line.setSize(obj_line.size);
line.setFillColor(sf::Color::Blue);
line.setRotation(obj_line.rotation_speed);

m_target.draw(line);
}



}



void renders_VBO(const Simulator& simulator) {

    object_va.resize(4 * simulator.getObject().size());


// Prob should center this rectanlge in the center of the screen through a simple translation by *5 
    auto constraint = simulator.getBoxConstraint();
    sf::RectangleShape rect(constraint);

    rect.setFillColor(sf::Color::White);


    sf::Vector2u windowSize = m_target.getSize();  // Assuming m_target is your render window

    // Calculate the position to center the rectangle
    float xPos = (windowSize.x - constraint.x) / 2.0f;
    float yPos = (windowSize.y - constraint.y) / 2.0f;


    rect.setPosition(xPos,yPos);
    m_target.draw(rect);



    // Begin Animation on the particles 
    const float    texture_size  = 1024.0f;
    const float    radius        = 2.5f;



    

    for(int i{0}; i < simulator.getObjectCount(); i++){
    
      const auto& obj = simulator.getObject().at(i);


        const uint64_t idx = i << 2;

        //Setting up Posiitons 
        object_va[idx + 0].position = obj.pos + sf::Vector2f(-radius,-radius);
        object_va[idx + 1].position = obj.pos + sf::Vector2f(radius,-radius);
        object_va[idx + 2].position = obj.pos + sf::Vector2f(radius,radius);
        object_va[idx + 3].position = obj.pos + sf::Vector2f(-radius,radius);
        



        //Setting up Textures

        object_va[idx + 0].texCoords = {0.0f        , 0.0f};
        object_va[idx + 1].texCoords = {texture_size, 0.0f};
        object_va[idx + 2].texCoords = {texture_size, texture_size};
        object_va[idx + 3].texCoords = {0.0f        , texture_size};

        sf::Color obj_color = obj.color;

        object_va[idx + 0].color = obj.color;
        object_va[idx + 1].color = obj.color;
        object_va[idx + 2].color = obj.color;
        object_va[idx + 3].color = obj.color;



    }




    


}
void renders(const Simulator& simulator) const
{


    auto constraint = simulator.getBoxConstraint();

    sf::RectangleShape rect(constraint);
    rect.setFillColor(sf::Color::White);
     m_target.draw(rect);

    sf::CircleShape circle{1.0f};
    circle.setPointCount(50);
    circle.setOrigin(1.0f,1.0f);
    const auto& objects = simulator.getObject();
    for (const auto& obj : objects)
    {
        circle.setPosition(obj.pos);
        circle.setScale(obj.radius,obj.radius);
        circle.setFillColor(obj.color);
        circle.setOutlineColor(sf::Color::Black);
        circle.setOutlineThickness(0.5);
        m_target.draw(circle);
    }

// render_line(simulator);
    
}

void temp_function(){


    std::cout << "Pixel X " <<  object_pixel.getSize().x << std::endl;
    std::cout << " Pixel y " <<  object_pixel.getSize().y << std::endl;
}
//WIP
/* 

*/
void renders_texture(const Simulator& simulator) const
{
    const auto& objects = simulator.getObject();
    sf::Image textureImage = object_texture.copyToImage();

    std::vector<std::future<void>> futures;
    const std::size_t numThreads = 3;

    for (std::size_t i = 0; i < numThreads; ++i) {
        futures.emplace_back(std::async(std::launch::async, [this, &objects, &textureImage, i, numThreads]() {
            sf::CircleShape circle(1.0f);
            circle.setPointCount(50);
            circle.setOrigin(1.0f, 1.0f);
            circle.setOutlineColor(sf::Color::Black);
            circle.setOutlineThickness(0.25);

            sf::Vector2u textureSize = textureImage.getSize();
            const sf::Uint8* texturePixels = textureImage.getPixelsPtr();

            for (std::size_t j = i; j < objects.size(); j += numThreads) {
                const auto& obj = objects[j];
                circle.setPosition(obj.pos);
                circle.setScale(obj.radius, obj.radius);

                sf::FloatRect bounds = circle.getGlobalBounds();
                sf::Vector2i startPos(std::max(0, static_cast<int>(bounds.left)), std::max(0, static_cast<int>(bounds.top)));
                sf::Vector2i endPos(std::min(static_cast<int>(bounds.left + bounds.width), static_cast<int>(textureSize.x)),
                                    std::min(static_cast<int>(bounds.top + bounds.height), static_cast<int>(textureSize.y)));

                for (int y = startPos.y; y < endPos.y; ++y) {
                    for (int x = startPos.x; x < endPos.x; ++x) {
                        if (circle.getLocalBounds().contains(x - obj.pos.x, y - obj.pos.y)) {
                            std::size_t index = (y * textureSize.x + x) * 4;
                            sf::Color color(texturePixels[index], texturePixels[index + 1], texturePixels[index + 2], texturePixels[index + 3]);
                            circle.setFillColor(color);
                            m_target.draw(circle);
                        }
                    }
                }
            }
        }));
    }

    for (auto& future : futures) {
        future.wait();
    }
}



    

void temp_render(){
    sf::RenderStates states;

    states.texture = &object_texture;



    //Render Particles
    renders_VBO(sim);


    m_target.draw(object_va,states);

}

private:

sf::RenderTarget& m_target;
};