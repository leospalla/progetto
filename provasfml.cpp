#include <SFML/Graphics.hpp>
// non funziona per qualche motivo
#include <iostream>

int main() {
  sf::RenderWindow window(sf::VideoMode(1280, 720), "prova",
                          sf::Style::Default);
  window.setFramerateLimit(120);
  sf::RectangleShape player(sf::Vector2f(100, 100));
  // player.setFillColor(sf::Color::Green);
  player.setOrigin(50.0f, 50.0f);
  sf::Texture playerTexture;
  playerTexture.loadFromFile("Questo PC/Immagini/download.jpeg");//devo mettere i file che voglio vengano sati come texture sulla macchina virtuale.
  player.setTexture(&playerTexture);
  while (window.isOpen())  // zona degli eventi frame per frame,non ho ancora
                           // capito bene sta cosa ma ok
  {
    sf::Event event;
    while (window.pollEvent(event)) {
      switch (event.type) {
        case sf::Event::Closed:
          window.close();
          break;
        case sf::Event::Resized:

          // std::cout <<event.size.width << event.size.height << std::endl;
          printf(
              "nuova larghezza della finestra: %i Nuova altezza della "
              "finestra: %i\n",
              event.size.width,
              event.size.height);  // questo è un classico printf, al prof non
                                   // piace ma è funzionale, se viene cambiata
                                   // la crandezza della finestra me lo notifica
          break;
        case sf::Event::TextEntered:  // questa è una cosa in più che mi
                                      // permette di vedere a schermo tutto
                                      // quello che scrivo frame per frame.
          if (event.text.unicode < 128) {
            printf("%c", event.text.unicode);
          }
          break;
      }
      if (event.type == sf::Event::Closed) window.close();
      if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape)) window.close();
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)) {
      player.move(-1.0f, 0.0f);
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W)) {
      player.move(0.0f, -1.0f);
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S)) {
      player.move(0.0f, 1.0f);
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)) {
      player.move(1.0f, 0.0f);
    }
    /*if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
      sf::Vector2i mousepos =
          sf::Mouse::getPosition(window);  // ci permette di avere la posizione
                                           // del mouse rispetto alla finestra
      player.setPosition(
          static_cast<float>(mousepos.x),
          static_cast<float>(
              mousepos.y));  // static cast mi permette di usare l'intero che ho
                             // per la posizione del mouse come un float per
                             // metterlo nella funzione setposition
    }*/
    window.clear();
    window.draw(player);  // disegnato nel backbuffer
    window.display();     // questo ci disegna questo quello che abiamo nel
                          // backbuffer
  }

  return 0;
}