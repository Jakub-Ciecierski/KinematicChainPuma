#include "puma.h"

#include <object/game_object.h>
#include <object/render_object.h>
#include <glm/gtx/matrix_decompose.hpp>
#include <math/print_math.h>
#include <iostream>

Puma::Puma(PumaArms& puma_arms) : puma_arms_(puma_arms){
    state_.length2 = puma_arms_.arm2.length;

    last_effector_position_ = puma_arms.effector.position;
    last_effector_rotation_ = puma_arms.effector.rotation;
}

Puma::~Puma(){}

void Puma::ComputeDirect(){
    ComputeDirect(state_);

    UpdateGameObject();
}

void Puma::ComputeInverse(){
    state_ = ComputeOptimalState();

    ComputeDirect(state_);

    last_effector_position_ = puma_arms_.effector.position;
    last_effector_rotation_ = puma_arms_.effector.rotation;

    UpdateGameObject();
}

PumaState Puma::ComputeOptimalState(){
    auto in = PreperInput();

    auto state1 = ComputeInverse(in, 1);
    ComputeDirect(state1);

    auto distance1 = ifx::EuclideanDistance(
            last_effector_position_, puma_arms_.effector.position);

    auto state2 = ComputeInverse(in, -1);
    ComputeDirect(state2);

    auto distance2 = ifx::EuclideanDistance(
            last_effector_position_, puma_arms_.effector.position);

    if(HasNan(state1) && HasNan(state2))
        std::cout << "Really bad" << std::endl;
    if(HasNan(state1))
        return state2;
    if(HasNan(state2))
        return state1;
    if(distance1 < distance2)
        return state1;
    else
        return state2;
}

InverseKinematicsInput Puma::PreperInput(){
    InverseKinematicsInput in;
    in.p0 = glm::vec3(0,0,0);
    in.x0 = glm::vec3(1,0,0);
    in.y0 = glm::vec3(0,1,0);
    in.z0 = glm::vec3(0,0,1);

    auto r5 = puma_arms_.effector.rotation;
    in.p5 = puma_arms_.effector.position;

    auto rx4
            = glm::rotate(glm::mat4(1.0f),
                          glm::radians(r5.x),
                          glm::vec3(1.0f, 0.0f, 0.0f));
    auto ry4
            = glm::rotate(glm::mat4(1.0f),
                          glm::radians(r5.y),
                          glm::vec3(0.0f, 1.0f, 0.0f));
    auto rz4
            = glm::rotate(glm::mat4(1.0f),
                          glm::radians(r5.z),
                          glm::vec3(0.0f, 0.0f, 1.0f));
    auto r4 = rz4 * ry4 * rx4;
    auto r = glm::mat3(r4);

    r[0].x = glm::degrees(r[0].x);
    r[0].y = glm::degrees(r[0].y);
    r[0].z = glm::degrees(r[0].z);

    r[1].x = glm::degrees(r[1].x);
    r[1].y = glm::degrees(r[1].y);
    r[1].z = glm::degrees(r[1].z);

    r[2].x = glm::degrees(r[2].x);
    r[2].y = glm::degrees(r[2].y);
    r[2].z = glm::degrees(r[2].z);

    r[0] = glm::normalize(r[0]);
    r[1] = glm::normalize(r[1]);
    r[2] = glm::normalize(r[2]);


    in.x5 = r[0];
    in.y5 = r[1];
    in.z5 = r[2];
/*
    std::cout << "r5" << std::endl;
    ifx::PrintVec3(r5);

    std::cout << "rx" << std::endl;
    ifx::PrintMat3(rx);
    std::cout << "ry" << std::endl;
    ifx::PrintMat3(ry);
    std::cout << "rz" << std::endl;
    ifx::PrintMat3(rz);
    std::cout << "r4" << std::endl;
    ifx::PrintMat4(r4);
    std::cout << "r" << std::endl;
    ifx::PrintMat3(r);

    std::cout << "x5" << std::endl;
    ifx::PrintVec3(in.x5);
    std::cout << "y5" << std::endl;
    ifx::PrintVec3(in.y5);
    std::cout << "z5" << std::endl;
    ifx::PrintVec3(in.z5);

    if(std::isnan(in.y5.x))
        std::cout << "nan" << std::endl;
*/

    in.l1 = puma_arms_.arm1.length;
    in.l3 = puma_arms_.arm3.length;
    in.l4 = puma_arms_.arm4.length;

    return in;
}

PumaState Puma::ComputeInverse(InverseKinematicsInput in, float z4_multiplier){
    const float epsilon = 0.001;
    auto p1 = in.p0;
    auto p2 = in.p0 + (in.l1 * in.y0);
    auto p4 = in.p5 - (in.l4 * in.z5);
    p4 = epsilon + p4;
    p2 = epsilon + p2;

    // TODO check n == 0;
    auto n = glm::cross((p4 - p1), (p2 - p1));
    n = glm::normalize(n);

    auto z4 = ComputeZ4(in.z5, n);
    z4 = glm::normalize(z4);
    z4 = z4 * z4_multiplier;
    auto p3 = p4 + (in.l3 * z4);

    // TODO a1 coords
    PumaState state;
    state.alpha1 = glm::degrees(atan2(p4.y, p4.x));
    state.alpha1 += glm::degrees(M_PI_2);
    //state.alpha1 -= glm::degrees(M_PI);
    //state.alpha1 = glm::degrees(atan2(p4.x, p4.y));
    //state.alpha1 += -glm::degrees(M_PI_2);
    //state.alpha1 = 360 - state.alpha1;
    //state.alpha1 = -state.alpha1;
    //state.alpha1 = glm::degrees(atan2(p4.x, p4.z));

    //state.alpha2 = Angle(p2 - p1, p3 - p2) + glm::degrees(M_PI_2);
    state.alpha2 = Angle(p2 - p1, p3 - p2) - glm::degrees(M_PI_2);
    //state.alpha2 += -glm::degrees(M_PI_4);
    //state.alpha2 += glm::degrees(M_PI);
    //state.alpha2 = Angle(p3 - p2, p2 - p1) - glm::degrees(M_PI_2);

    //state.alpha3 = Angle(p3 - p2, p4 - p3) + glm::degrees(M_PI_2);
    state.alpha3 = Angle(p3 - p2, p4 - p3) - glm::degrees(M_PI_2);
    //state.alpha3 += -glm::degrees(M_PI_4);
    //state.alpha3 += glm::degrees(M_PI);

    //state.alpha4 = Angle(n, in.z5) + glm::degrees(M_PI_2);
    //state.alpha4 = Angle(n, in.z5) + glm::degrees(M_PI_2);
    state.alpha4 = Angle(n, in.z5) + glm::degrees(M_PI_2);
    //state.alpha4 += -glm::degrees(M_PI);
    //state.alpha4 = -state.alpha4;

    state.alpha5 = Angle(p3 - p4, in.y5);
    //state.alpha5 += -glm::degrees(M_PI_2);

    state.length2 = ifx::Magnitude(p3 - p2);

    // <print>
/*
    std::cout << " ------------ State ------------" << std::endl;

    std::cout << "x5" << std::endl;
    ifx::PrintVec3(in.x5);
    std::cout << "y5" << std::endl;
    ifx::PrintVec3(in.y5);
    std::cout << "z5" << std::endl;
    ifx::PrintVec3(in.z5);
    std::cout << "p2" << std::endl;
    ifx::PrintVec3(p2);
    std::cout << "p3" << std::endl;
    ifx::PrintVec3(p3);
    std::cout << "p4" << std::endl;
    ifx::PrintVec3(p4);
    std::cout << "p5" << std::endl;
    ifx::PrintVec3(in.p5);
    std::cout << "n" << std::endl;
    ifx::PrintVec3(n);
    std::cout << "z4" << std::endl;
    ifx::PrintVec3(z4);
    std::cout << "a1: " << state.alpha1 << std::endl;
    std::cout << "a2: " << state.alpha2 << std::endl;
    std::cout << "a3: " << state.alpha3 << std::endl;
    std::cout << "a4: " << state.alpha4 << std::endl;
    std::cout << "a5: " << state.alpha5 << std::endl;
    std::cout << "q2: " << state.length2 << std::endl;
    std::cout << " -------------------------------" << std::endl;
    std::cout << std::endl;
*/
    // </print>

    return state;
}

glm::vec3 Puma::ComputeZ4(const glm::vec3& x, const glm::vec3& n){

    auto n1 = n[0];
    auto n2 = n[1];
    auto n3 = n[2];

    auto x1 = x[0];
    auto x2 = x[1];
    auto x3 = x[2];

/*
    auto n1 = n[2];
    auto n2 = n[0];
    auto n3 = n[1];

    auto x1 = x[2];
    auto x2 = x[0];
    auto x3 = x[1];
*/
    auto n1_2 = n1*n1;
    auto n2_2 = n2*n2;
    auto n3_2 = n3*n3;
    auto x1_2 = x1*x1;
    auto x2_2 = x2*x2;
    auto x3_2 = x3*x3;

    float d = n3_2
                 * (x1_2 + x2_2) - 2.0f * n1* n3* x1* x3 - 2.0f * n2 * x2
                                                           * (n1 *x1 + n3* x3)
                 + n2_2 *(x1_2 + x3_2) + n1_2 *(x2_2 + x3_2);
    d = sqrt(d);
    auto z1 = (-n3 * x2 + n2 * x3) / d;
    auto z2 = (n3 * x1 - n1 * x3) / d;
    auto z3 = (-n2 * x1 + n1 * x2) / d;

    return glm::vec3(z1, z2, z3);

/*
    float z3_d = n3_2
                 * (x1_2 + x2_2) - 2
                                   * n1 * n3 * x1 * x3 - 2 * n2 * x2
                                                         * (n1 * x1 + n3 * x3)
                 + n2_2 * (x1_2 + x3_2) + n1_2 * (x2_2 + x3_2);
    float z2_d = n3_2
                 * (x1_2 + x2_2) - 2 * n1 * n3 * x1 * x3 - 2 * n2 * x2
                                                           * (n1 * x1 + n3 * x3)
                 + n2_2 * (x1_2 + x3_2) + n1_2 * (x2_2 + x3_2);*/
}

float Puma::Angle(const glm::vec3& v, const glm::vec3& w){
    const float epsilon = 0.01;
    auto v_norm = glm::normalize(v);
    auto w_norm = glm::normalize(w);
    //auto v_norm = v;
    //auto w_norm = w;
    auto dot = ifx::dot(v_norm, w_norm);
    if(dot == 1 || dot == -1){
        v_norm = glm::normalize(epsilon+v);
        dot = ifx::dot(v_norm, w_norm);
    }
/*
    if(dot > 1)
        dot = 1;
    if(dot < -1)
        dot = -1;
    auto cosa = acos(dot);
*/
    auto cross_vw = glm::cross(v_norm, w_norm);
    auto cross_vw_norm = cross_vw;
    auto mag_cross_vw_norm = ifx::Magnitude(cross_vw_norm);
/*
    if(mag_cross_vw_norm > 1)
        mag_cross_vw_norm = 1;
    if(mag_cross_vw_norm < -1)
        mag_cross_vw_norm = -1;
    auto sina = asin(mag_cross_vw_norm);

    auto at = atan2(sina, cosa);
    if(std::isnan(cosa))
        std::cout << "nan" << std::endl;
    if(std::isnan(sina))
        std::cout << "nan" << std::endl;
*/

    //return glm::degrees(atan(mag_cross_vw_norm/dot));
    return glm::degrees(atan2(mag_cross_vw_norm, dot));
    //return glm::degrees(at);
}

void Puma::ComputeDirect(PumaState& state){
    auto frames = CalculateDirectFrame(state);

    UpdateArm1(frames.F01);
    UpdateArm2(frames.F02, state_.length2);
    UpdateArm3(frames.F03);
    UpdateArm4(frames.F04);
    UpdateEffector(frames.F05);
}

void Puma::UpdateEffector(glm::mat4& F){
    auto decomposition = Decompose(F);
/*
    puma_arms_.effector.position = decomposition.position;
    puma_arms_.effector.rotation = decomposition.rotation;*/

    puma_arms_.effector.position
            = puma_arms_.effector.render_object->getPosition();
    puma_arms_.effector.rotation
            = puma_arms_.effector.render_object->getRotation();
}

void Puma::UpdateArm1(glm::mat4& F){
    auto decomposition = Decompose(F);

    //puma_arms_.arm1.connector_render_object->rotateTo(decomposition.rotation);
    //puma_arms_.arm1.connector_render_object->rotate(glm::vec3(0,0,90));
}
void Puma::UpdateArm2(glm::mat4& F, float q2){
    auto decomposition = Decompose(F);

    puma_arms_.arm2.position = decomposition.position;
    puma_arms_.arm2.rotation = decomposition.rotation;
}

void Puma::UpdateArm3(glm::mat4& F){
    auto decomposition = Decompose(F);

    puma_arms_.arm3.position = decomposition.position;
    puma_arms_.arm3.rotation = decomposition.rotation;
}
void Puma::UpdateArm4(glm::mat4& F){
    auto decomposition = Decompose(F);

    puma_arms_.arm4.position = decomposition.position;
    puma_arms_.arm4.rotation = decomposition.rotation;
}

PumaFrames Puma::CalculateDirectFrame(PumaState& state){
    PumaFrames frames;
    auto a1 = state.alpha1;
    auto a2 = state.alpha2;
    auto a3 = state.alpha3;
    auto a4 = state.alpha4;
    auto a5 = state.alpha5;
    auto q2 = state.length2;

    auto l1 = puma_arms_.arm1.length;
    auto l3 = puma_arms_.arm3.length;
    auto l4 = puma_arms_.arm4.length;

    auto F1 = Ry(a1) * Ty(l1);
    auto F2 = Rx(a2) * Tz(q2);
    auto F3 = Rx(a3) * Ty(-l3);
    auto F4 = Ry(a4) * Tz(l4);
    auto F5 = Rz(a5);

    frames.F01 = F1;
    frames.F02 = frames.F01 * F2;
    frames.F03 = frames.F02 * F3;
    frames.F04 = frames.F03 * F4;
    frames.F05 = frames.F04 * F5;

    return frames;
}

void Puma::UpdateGameObject(){
    // <2>
    puma_arms_.arm2.render_object->moveTo(puma_arms_.arm2.position);
    puma_arms_.arm2.render_object->rotateTo(puma_arms_.arm2.rotation);
    puma_arms_.arm2.render_object->rotate(glm::vec3(-90,0,0));
    puma_arms_.arm2.render_object->scale(glm::vec3(1, state_.length2, 1));

    puma_arms_.arm2.connector_render_object->moveTo(puma_arms_.arm2.position);

    // <3>
    puma_arms_.arm3.render_object->moveTo(puma_arms_.arm3.position);
    puma_arms_.arm3.render_object->rotateTo(puma_arms_.arm3.rotation);

    puma_arms_.arm3.connector_render_object->moveTo(puma_arms_.arm3.position);
    puma_arms_.arm3.connector_render_object->move(glm::vec3(0, -0.06, 0));

    // <4>
    puma_arms_.arm4.render_object->moveTo(puma_arms_.arm4.position);
    puma_arms_.arm4.render_object->rotateTo(puma_arms_.arm4.rotation);
    puma_arms_.arm4.render_object->rotate(glm::vec3(-90,0,0));

    // <5>
    puma_arms_.effector.render_object->moveTo(puma_arms_.effector.position);
    puma_arms_.effector.render_object->rotateTo(puma_arms_.effector.rotation);
}

bool Puma::HasNan(PumaState& state){
    if(std::isnan(state.alpha1))
        return true;
    if(std::isnan(state.alpha2))
        return true;
    if(std::isnan(state.alpha3))
        return true;
    if(std::isnan(state.alpha4))
        return true;
    if(std::isnan(state.alpha5))
        return true;
    if(std::isnan(state.length2))
        return true;
    return false;
}

Decomposition Puma::Decompose(glm::mat4& F){
    Decomposition decomposition;

    glm::vec3 scale;
    glm::quat rotation;
    glm::vec3 translation;
    glm::vec3 skew;
    glm::vec4 perspective;
    glm::decompose(F, scale, rotation, translation, skew,perspective);
    rotation=glm::conjugate(rotation);
    glm::vec3 euler = glm::degrees(glm::eulerAngles(rotation));
/*
    if(std::isnan(euler.x))
        euler.x = 0;
    if(std::isnan(euler.y))
        euler.y = 0;
    if(std::isnan(euler.z))
        euler.z = 0;
*/
    decomposition.position = translation;
    decomposition.rotation = euler;

    return decomposition;
}

glm::mat4 Puma::Tx(float x){
    return Translate(glm::vec3(x, 0, 0));
}
glm::mat4 Puma::Ty(float x){
    return Translate(glm::vec3(0, x, 0));
}
glm::mat4 Puma::Tz(float x){
    return Translate(glm::vec3(0, 0, x));
}

glm::mat4 Puma::Translate(const glm::vec3& position){
    return translate(glm::mat4(1.0f), position);
}

glm::mat4 Puma::Rx(float angle){
    return glm::rotate(glm::mat4(1.0f),
                       glm::radians(angle),
                       glm::vec3(1.0f, 0.0f, 0.0f));
}
glm::mat4 Puma::Ry(float angle){
    return glm::rotate(glm::mat4(1.0f),
                       glm::radians(angle),
                       glm::vec3(0.0f, 1.0f, 0.0f));
}
glm::mat4 Puma::Rz(float angle){
    return glm::rotate(glm::mat4(1.0f),
                       glm::radians(angle),
                       glm::vec3(0.0f, 0.0f, 1.0f));
}

