//prova della libreria boids, deve includere le tre regole, il programma che usa questa libreria dovrà essere un'ibrido tra un cpp e uno per il settaggio grafico di SFML
#ifndef BOIDS_HPP
#define BOIDS_HPP
#include <cmath>
#include <vector>
#include <iostream>
#include "vector.hpp"
//facciamo un template<type T> ?
class sb{//stato boid,possiamousarlo in generale per fare anche predatori{
    private:
//non saprei che mettere, in teoria le cose che non voglio vengano toccate, probabilemtne conviene mettere il vettore posizione e quello velocità
    public:
    Vector::Vector(double , double) posizione;
    Vector::Vector(double , double) velocità;
};
    void s;
    void ds;//distanza di attivazione
    void a;
    void c;
    //tutte queste quattro variabili devono essere una variabili di imput
    void time; //serve poi per creare il coso con sfml

int numboids;

void centrodimassa(numboids); const

if (numboids != 1 || numboids != 0){
    void separation(); 
    void allinegment(); 
    void Coesion();
} else {void};
//non so bene ancora dove mettere la parte del tempo per sfml
#endif