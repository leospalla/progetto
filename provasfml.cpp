#include <SFML/Graphics.hpp>


int main(){
    sf::RenderWindow window(sf::VideoMode(1280 , 720), "prova");
    window.setFramerateLimit(60);
    sf::RectangleShape rettangolo;

    sf::Vector2f posizione(600, 350);
   
    rettangolo.setSize(sf::Vector2f(100, 100));

    double xvel=3;
    double yvel=3;
    while(window.isOpen())//zona degli eventi frame per frame
    {
        sf::Event event;
        while(window.pollEvent(event)){
            if(event.type == sf::Event::Closed) window.close();
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape)) window.close();
        }

        if (posizione.x <0 || posizione.x > 1180) xvel *= -1;
        if (posizione.y <0 || posizione.y > 620) yvel *= -1;
    posizione.x *= xvel;
    posizione.y *= yvel;
     rettangolo.setPosition(posizione);
     
    
         //rendering
    window.clear();

    window.draw(rettangolo);

    window.display();
    }
   
    return 0;
}