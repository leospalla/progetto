#include "flocktelly.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/System/Time.hpp>



int main() 
{
    sf::RenderWindow window(sf::VideoMode(1280, 720), "prova",
                          sf::Style::Default);
  window.setFramerateLimit(60);
   sf::RectangleShape bird(sf::Vector2f(10. , 10.));
  Flock f;
  while (window.isOpen()) {  /// inizio while della finestra
    sf::Event event;
    while (window.pollEvent(event)) {
      switch (event.type) {
        case sf::Event::Closed:
          window.close();
          break;
        case sf::Event::TextEntered:
          if (event.text.unicode < 128) {
            printf("%c", event.text.unicode);
          }
          break;
      }
    }
    if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
      sf::Vector2i mousepos = sf::Mouse::getPosition(window);
    bird.setPosition(
          static_cast<double>(mousepos.x),
          static_cast<double>(mousepos.y));  // non va bene ma l'idea è questa
    Vector r(static_cast<double> (mousepos.x) , static_cast<double>(mousepos.y));
    Boid b(r);//this should give me a random speed component every time a boid is generated
    f.addBoid(b);
    window.draw(bird);
    window.display();
    window.clear();
    } else {
    }
    if(f.getnumboids() >= 2){
            int numSteps;
            f.simulate(numSteps = 17);//in uqesto modo ottengo ad ogni frame esattamente 17 azioni di questo ciclo in teoria, poi in pratica non so
    for(Boid &b : f.getmboids()){
        if(b.pos().xcomp() <0.0){
            b.setPosition(-b.pos().xcomp(), b.pos().ycomp());
        }
        if(b.pos().ycomp()>0.0){
            b.setPosition(b.pos().xcomp(), -b.pos().ycomp());
        }
        bird.move(static_cast<float>(b.vel().xcomp()), static_cast<float>(b.vel().ycomp()));
        window.clear();
        window.draw(bird);
        window.display();
    }
    }else{}
}  // fine ciclo della finestra

return 0;
}