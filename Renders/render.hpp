#pragma once

#include "../Physics/physics.hpp"
#include "SFML/Graphics.hpp"
class render
{
    public:
    sf::VertexArray object_va; 
    sf::VertexArray world_va;
    sf::Texture object_texture; 
    explicit
    render(sf::RenderTarget& target)
    :m_target{target}
    {
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

private:

sf::RenderTarget& m_target;
};