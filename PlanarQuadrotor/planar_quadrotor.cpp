#include <iostream>
#include <random>

#include "planar_quadrotor.h"

PlanarQuadrotor::PlanarQuadrotor() {
    std::random_device r;
    std::default_random_engine generator(r());
    std::normal_distribution<float> distribution(0.0, 1.0);
    auto gaussian = [&] (int) {return distribution(generator);};

    z = Eigen::VectorXf::NullaryExpr(6, gaussian);
}

PlanarQuadrotor::PlanarQuadrotor(Eigen::VectorXf z): z(z) {}
//Funkcja void PlanarQuadrotor::SetGoal(Eigen::VectorXf z_goal) ustawia wektor 
//z_goal jako cel (docelowe położenie) dla układu kwadrokoptera.
void PlanarQuadrotor::SetGoal(Eigen::VectorXf z_goal) {
    this->z_goal = z_goal;
}
//Funkcja Eigen::VectorXf PlanarQuadrotor::GetState() służy do pobrania aktualnego stanu układu.
//Zwraca wektor stanu z, który przechowuje pozycję (x, y), orientację (theta) oraz prędkości liniowe
//i kątową (x_dot, y_dot, theta_dot) quadrotora
Eigen::VectorXf PlanarQuadrotor::GetState() {
    return z;
}
//Funkcja GetControlState zwraca wektor różnicy między wektorem stanu z a wektorem celu z_goal.
Eigen::VectorXf PlanarQuadrotor::GetControlState() {
    return z - z_goal;
}
//Funkcja GravityCompInput zwraca wektor sterowania propellerami potrzebnego do skompensowania wpływu siły grawitacji.
Eigen::Vector2f PlanarQuadrotor::GravityCompInput() {
    /* Extract parameters */
    float m = params[0];
    float I = params[1];
    float r = params[2];
    float g = params[3];

    return Eigen::Vector2f::Constant(m * g / 2);
}
//Funkcja Linearize służy do liniowej aproksymacji modelu kwadrokoptera w okolicy
//punktu równowagi i zwraca macierze A i B opisujące dynamikę systemu w postaci równania stanu x_dot = Ax + Bu.
std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> PlanarQuadrotor::Linearize() {
    /* Extract parameters */
    float m = params[0];
    float I = params[1];
    float r = params[2];
    float g = params[3];

    Eigen::VectorXf z_star = Eigen::VectorXf::Zero(6);
    float x = z_star[0];
    float y = z_star[1];
    float theta = z_star[2];
    float x_dot = z_star[3];
    float y_dot = z_star[4];
    float theta_dot = z_star[5];

    Eigen::Vector2f input_star = GravityCompInput();
    float u_1 = input_star[0];
    float u_2 = input_star[1];

    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B = Eigen::MatrixXf::Zero(6, 2);

    /* dfi_dzj */
    A.block(0, 3, 3, 3) = Eigen::MatrixXf::Identity(3, 3);
    A.block(3, 0, 1, 3) << 0, 0, -(u_1 + u_2) * cos(theta) / m;
    A.block(4, 0, 1, 3) << 0, 0, -(u_1 + u_2) * sin(theta) / m;

    /* dfi_du_j */
    B.row(3) = Eigen::Vector2f::Constant(-sin(theta) / m);
    B.row(4) = Eigen::Vector2f::Constant(cos(theta) / m);
    B.row(5) = Eigen::Vector2f(r / I, -r / I);

    return std::tuple(A, B);
}
//Funkcja DoCalcTimeDerivatives oblicza pochodną czasową wektora stanu z w danym 
//punkcie i przechowuje wynik w wektorze z_dot.
void PlanarQuadrotor::DoCalcTimeDerivatives() {
    /* Extract parameters */
    float m = params[0];
    float I = params[1];
    float r = params[2];
    float g = params[3];

    /* Extract state */
    float x = z[0];
    float y = z[1];
    float theta = z[2];
    float x_dot = z[3];
    float y_dot = z[4];
    float theta_dot = z[5];

    /* Extract propellers actuation */
    float u_1 = input[0];
    float u_2 = input[1];

    z_dot.block(0, 0, 3, 1) = z.block(3, 0, 3, 1);
    
    /* See http://underactuated.mit.edu/acrobot.html#section3 3.3.1 */
    float x_dotdot = -(u_1 + u_2) * sin(theta) / m;
    float y_dotdot = (u_1 + u_2) * cos(theta) / m - g;
    float theta_dotdot = r * (u_1 - u_2) / I;

    z_dot.block(3, 0, 3, 1) << x_dotdot, y_dotdot, theta_dotdot;
}
//Funkcja DoUpdateState aktualizuje stan układu na podstawie pochodnej czasowej 
//i wykorzystuje prostą metodę Eulera do całkowania numerycznego.
void PlanarQuadrotor::DoUpdateState(float dt) {
    /* Euler integration */
    z += dt * z_dot;
}
//Funkcja SetInput ustawia wektor wejściowy input, który odpowiada za sterowanie kwadrokopterem.
void PlanarQuadrotor::SetInput(Eigen::Vector2f input) {
    this->input = input;
}
//Funkcja Update aktualizuje stan układu i zwraca wektor stanu z.
Eigen::VectorXf PlanarQuadrotor::Update(Eigen::Vector2f &input, float dt) {
    SetInput(input);
    DoCalcTimeDerivatives();
    DoUpdateState(dt);

    return z;
}
//Funkcja Update aktualizuje stan układu i zwraca wektor stanu z.
Eigen::VectorXf PlanarQuadrotor::Update(float dt) {
    return Update(input, dt);
}