#pragma once

#include "../Physics/physics.hpp"
#include "SFML/Graphics.hpp"
#include "../Temp/thread.hpp"
#include <mutex>
class render
{
    public:
    sf::VertexArray object_va; 
    sf::VertexArray world_va;
    sf::Texture object_texture;
    mutable std::mutex m_mutex;
    explicit
    render(sf::RenderTarget& target)
    :m_target{target}
    {
        object_texture.loadFromFile(std::string(SHADER_DIR) + "/fixed.png");
        object_texture.generateMipmap();
        object_texture.setSmooth(true);


    
    }


void renders(const Simulator& simulator) const
{
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
        circle.setOutlineThickness(0.25);
        m_target.draw(circle);
    }


    
}
//WIP
/* 

*/
void renders_texture(const Simulator& simulator) const
{
    sf::Vector2u imageSize = object_texture.getSize();
    const auto& objects = simulator.getObject();

    // Determine the number of threads to use
    unsigned int numThreads = std::thread::hardware_concurrency();

    // Create a vector to store the threads
    std::vector<std::thread> threads;

    // Divide the objects among the threads
    size_t objectsPerThread = objects.size() / numThreads;
    size_t remainingObjects = objects.size() % numThreads;

    size_t startIndex = 0;
    for (unsigned int t = 0; t < numThreads; ++t) {
        size_t endIndex = startIndex + objectsPerThread;
        if (t == numThreads - 1)
            endIndex += remainingObjects;

        threads.emplace_back([this, &simulator, imageSize, startIndex, endIndex]() {
            sf::CircleShape circle{1.0f};
            circle.setPointCount(10);
            circle.setOrigin(1.0f, 1.0f);

            const auto& objects = simulator.getObject();

            for (size_t i = startIndex; i < endIndex; ++i) {
                auto& obj = objects[i];

                if (i >= imageSize.x * imageSize.y) {
                    circle.setFillColor(sf::Color::White);
                } else {
                    std::size_t x = i % imageSize.x;
                    std::size_t y = i / imageSize.x;
                    sf::Color Pixel_Color = object_texture.copyToImage().getPixel(x, y);
                    circle.setFillColor(Pixel_Color);
                }

                circle.setPosition(obj.pos);
                circle.setScale(obj.radius, obj.radius);
                circle.setOutlineColor(sf::Color::Black);
                circle.setOutlineThickness(0.25);

                // Acquire a lock before drawing to the target
                std::lock_guard<std::mutex> lock(m_mutex);
                m_target.draw(circle);
            }
        });

        startIndex = endIndex;
    }

    // Wait for all threads to finish
    for (auto& thread : threads)
        thread.join();
}

private:

sf::RenderTarget& m_target;
};