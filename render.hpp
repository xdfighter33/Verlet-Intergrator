#pragma once

#include "physics.hpp"
class render
{
    public:
    explicit
    render(sf::RenderTarget& target)
    :m_target{target}
    {
    }
void renders(const Simulator& simulator) const
{
    sf::CircleShape circle{1.0f};
    circle.setPointCount(32);
    circle.setOrigin(1.0f,1.0f);
    const auto& objects = simulator.getObject();
    for (const auto& obj : objects)
    {
        circle.setPosition(obj.pos);
        circle.setScale(obj.radius,obj.radius);
        circle.setFillColor(obj.color);
        m_target.draw(circle);
    }


    
}


private:

sf::RenderTarget& m_target;
};