// prova della libreria boids, deve includere le tre regole, il programma che
// usa questa libreria dovrà essere un'ibrido tra un cpp e uno per il settaggio
// grafico di SFML
#ifndef BOIDS_HPP
#define BOIDS_HPP
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "vector.hpp"  //vettori costruiti da leo
// probabilmtente scritto male ma whatevs
class parameters {
  double a_;
  double s_;
  double c_;
  double d_;
  double ds_;

 public:
  parameters(double, double, double, double, double);
  double geta() const { return a_; }
  double gets() const { return s_; }
  double getc() const { return c_; }
  double getd() const { return d_; }
  double getds() const { return ds_; }
};

class boid {
  Vector velocity_;
  Vector position_;

 public:
  boid(const Vector&, const Vector&);
  Vector getvel() const { return velocity_; }
  Vector getpos() const { return position_; }
};

std::vector<boid> numboids{
    0};  // è un gran casino con i tipi zio incastrato nel catrame

class flyghtrules {
  Vector sep_;
  Vector all_;
  Vector coe_;

 public:
  flyghtrules(const Vector&, const Vector&, const Vector&);
  Vector getsep() const { return sep_; }
  Vector getall() const { return all_; }
  Vector getcoe() const { return coe_; }
};
#endif