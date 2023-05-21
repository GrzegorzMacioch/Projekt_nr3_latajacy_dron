#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers (extra points)
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta, qs_x, qs_y, qs_theta;
    
    /* x, y, theta coordinates */
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];
   
    qs_x=(q_x/2 + 1)*1280/2 ;
    qs_y=(q_y/1.125 - 1)*720 / (-2) ;
    
   
    
    Sint16 vx[12] = { (Sint16)(qs_x+125), (Sint16)(qs_x+125), (Sint16)(qs_x+100), (Sint16)(qs_x+100), (Sint16)(qs_x+95), (Sint16)(qs_x+95), (Sint16)(qs_x-95), (Sint16)(qs_x-95), (Sint16)(qs_x-100), (Sint16)(qs_x-100), (Sint16)(qs_x-125), (Sint16)(qs_x-125) };
    Sint16 vy[12] = { (Sint16)(qs_y+15), (Sint16)(qs_y-15), (Sint16)(qs_y-15), (Sint16)(qs_y-15-70), (Sint16)(qs_y-15-70), (Sint16)(qs_y-15), (Sint16)(qs_y-15), (Sint16)(qs_y-15-70), (Sint16)(qs_y-15-70), (Sint16)(qs_y-15), (Sint16)(qs_y-15), (Sint16)(qs_y+15) };// Sint16 vx1[6] = { q_x+78,q_x+120,q_x+120,q_x+78,q_x+36,q_x+36};
    Sint16 vx1[3] = { (Sint16)(qs_x+97),(Sint16)(qs_x+97+40),(Sint16)(qs_x+97+40)};
    Sint16 vy1[3] = { (Sint16)(qs_y-15-70),(Sint16)(qs_y-15-70),(Sint16)(qs_y-15-70-20)};
    Sint16 vx2[3] = { (Sint16)(qs_x+97),(Sint16)(qs_x+97-40),(Sint16)(qs_x+97-40)};
    Sint16 vy2[3] = { (Sint16)(qs_y-15-70),(Sint16)(qs_y-15-70),(Sint16)(qs_y-15-70-20)};
    Sint16 vx3[3] = { (Sint16)(qs_x-97),(Sint16)(qs_x-97-40),(Sint16)(qs_x-97-40)};
    Sint16 vy3[3] = { (Sint16)(qs_y-15-70),(Sint16)(qs_y-15-70),(Sint16)(qs_y-15-70-20)};
    Sint16 vx4[3] = { (Sint16)(qs_x-97),(Sint16)(qs_x-97+40),(Sint16)(qs_x-97+40)};
    Sint16 vy4[3] = { (Sint16)(qs_y-15-70),(Sint16)(qs_y-15-70),(Sint16)(qs_y-15-70-20)};

    float cos_q_theta = cos(-q_theta);
    float sin_q_theta = sin(-q_theta);
    //point of reference for rotation
    int center_Px = qs_x;
    int center_Py = qs_y;


    for (int i = 0; i < 12; i++) 
    {
        float dx = vx[i] - center_Px;
        float dy = vy[i] - center_Py;
        vx[i] = cos_q_theta * dx - sin_q_theta * dy + center_Px;
        vy[i] = sin_q_theta * dx + cos_q_theta * dy + center_Py;
    }

    filledPolygonRGBA(gRenderer.get(), vx, vy, 12, 117,117,117,128 );


    for (int i = 0; i < 3; i++) 
    {
        float dx1 = vx1[i] - center_Px;
        float dy1 = vy1[i] - center_Py;
        vx1[i] = cos_q_theta * dx1 - sin_q_theta * dy1 + center_Px;
        vy1[i] = sin_q_theta * dx1 + cos_q_theta * dy1 + center_Py;
    }
    filledPolygonRGBA(gRenderer.get(), vx1, vy1, 3, 0,188,212,255 );

    for (int i = 0; i < 3; i++) 
    {
        float dx2 = vx2[i] - center_Px;
        float dy2 = vy2[i] - center_Py;
        vx2[i] = cos_q_theta * dx2 - sin_q_theta * dy2 + center_Px;
        vy2[i] = sin_q_theta * dx2 + cos_q_theta * dy2 + center_Py;
    }
    filledPolygonRGBA(gRenderer.get(), vx2, vy2, 3, 255,167,38,255 );

    for (int i = 0; i < 3; i++) 
    {
        float dx3 = vx3[i] - center_Px;
        float dy3 = vy3[i] - center_Py;
        vx3[i] = cos_q_theta * dx3 - sin_q_theta * dy3 + center_Px;
        vy3[i] = sin_q_theta * dx3 + cos_q_theta * dy3 + center_Py;
    }
    filledPolygonRGBA(gRenderer.get(), vx3, vy3, 3, 255,167,38,255 );

    for (int i = 0; i < 3; i++) 
    {
        float dx4 = vx4[i] - center_Px;
        float dy4 = vy4[i] - center_Py;
        vx4[i] = cos_q_theta * dx4 - sin_q_theta * dy4 + center_Px;
        vy4[i] = sin_q_theta * dx4 + cos_q_theta * dy4 + center_Py;
    }
    filledPolygonRGBA(gRenderer.get(), vx4, vy4, 3, 0,188,212,255 );
    
}