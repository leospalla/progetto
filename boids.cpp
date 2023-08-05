#include "boids.hpp"

#include <SFML/Graphics.hpp>
#include <SFML/System/Time.hpp>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

boid
    birdgl;  // ho dato dei nomi a queste classi per semlicità di scrittura dopo
parameters par;
flyghtrules rules;

// il vettore numboids viene pushato con degli elementi che appartengono al tipo
// della classe boids, va fatto sempre nel ciclo di sfml

// bisonga immaginare che si aggiorna ogni frame, tutte queste regole vanno
// messe nel while della parte grafica, assieme alla definizione di numboids e
// del centro di massa, li viene anche sempre controllato che il vettore che
// contiene i boids sia semrpe diverso da zero
regole.getsep() {  // qua mi da errore su fr non so perchè
  for (int i = 0; i = numboids.size(); i++) {  // for on screenboids everyframe
    if (numboids.size() == 1) {  // meglio mettere uno switch al posto dell'if
      throw  // non mi ricordo esattamente il comando preciso ma poco importa
    } else {
      if (Vector::distance(numboids[i - 1].getpos(), numboids[i].getpos()) <
          par.getds()) {
        Vector dist{numboids[i - 1].getpos() - numboids[i].getpos()};
      } else {
        return 0;
      }
    }
    Vector sum{sum + dist};
    return sum;  // è un vettore
  }              // chiusura for
  sep_ = -par.gets() *
         sum;  // in teoria sto * dovrebbe essere quello della classe vector per
               // uno scalare poichè sum è un elemento della classe Vector
  return sep_;
};
regole.getall() {
  for (i = 0, i < numboids.size(), i++) {
    Vector sumal;
    sumal = numboids[i].getvel() - numboids[it].getvel();
    sumal = sumal + sumal;
    return sumal;
  }
  par.geta() * ((1 / (numboids.size() - 1)) * sumal);
}
regole.getcoe() { par.getc() * (cm - numboids[i].getpos()); };
// parte grafica, manca da mettere dentro le regole e di renderezzare le
// immagini dei boids
int main() {
  sf::RenderWindow window(sf::VideoMode(1280, 720), "prova",
                          sf::Style::Fullscreen);
  window.setFramerateLimit(120);
  sf::RectangleShape bird(sf::Vector2<double>(50, 50));
  while (window.isOpen()) {
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
      sf::Vector2i mousepos = sf::Mouse::getPosition(
          window);  // non va bene per un problema di tipi

      bird.setPosition(
          static_cast<double>(mousepos.x),
          static_cast<double>(mousepos.y));  // non va bene ma l'idea è questa
      birdgl.getpos().Set(
          static_cast<double>(mousepos.x),
          static_cast<double>(
              mousepos.y));  // ogni frame setta la posizione  iniziale di
                             // quell'esatto boid alle coordinate del mouse
      numboids.push_back(birdgl);

    } else {
    }
    if (numboids.size() = > 2) {
      Vector sumcm;
      for (i = 0, i <= numboids.size() - 1, i++) {
        sumcm = numboids[i].getpos() + sumcm;
        return sumcm;
      }
      Vector cm{static_cast<double>((1 / (numboids.size() - 1))) *
                sumcm};  // ho il centro di massa di tutti gli altri boids
                         // tranne quello appena aggiunto, dovrei mettere le
                         // regole di volo qui
    } else {
    }
  }  // fine ciclo della finestra

  return 0;
}