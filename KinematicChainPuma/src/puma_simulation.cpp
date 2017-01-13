#include "puma_simulation.h"

#include <puma.h>
#include <object/render_object.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <math/print_math.h>

PumaSimulation::PumaSimulation(
        std::shared_ptr<PumaSimulationCreateParams> params) :
        manipulate_mode_(true){
    Reset(params);
    Pause();
}
PumaSimulation::~PumaSimulation(){}

void PumaSimulation::Update(){
    if(!UpdateTime())
        return;
    if(manipulate_mode_){
        ManipulatePuma();
    }else{
        InterpolatePuma();
    }
    puma_->ComputeInverse();
}

void PumaSimulation::Reset(std::shared_ptr<PumaSimulationCreateParams> params){
    ifx::Simulation::Reset();

    time_data_.simulation_length = params->simulation_length;
    if(params->puma)
        puma_ = params->puma;

    ResetInterpolationData(params);
}

void PumaSimulation::ManipulatePuma(){
    auto& effector = puma_->effector();
    effector.position = effector.render_object->getPosition();
    effector.rotation = effector.render_object->getRotation();
}

void PumaSimulation::InterpolatePuma(){
    float t = time_data_.total_time / time_data_.simulation_length;

    auto& effector = puma_->effector();

    effector.position = InterpolatePosition(t);
    effector.rotation = InterpolateRotation(t);

    effector.render_object->moveTo(effector.position);
    effector.render_object->rotateTo(effector.rotation);
}

glm::vec3 PumaSimulation::InterpolatePosition(float t){
    glm::vec3 direction = interpolation_data_.position_end -
                          interpolation_data_.position_start;
    glm::vec3 interpolated_position
            = interpolation_data_.position_start+ direction * t;

    return interpolated_position;
}
glm::vec3 PumaSimulation::InterpolateRotation(float t){
    glm::quat q;

    q = glm::slerp(interpolation_data_.rotate_start,
                   interpolation_data_.rotate_end, t);
    q = glm::normalize(q);

    glm::vec3 euler = glm::degrees(glm::eulerAngles(q));
    return euler;
}

void PumaSimulation::ResetInterpolationData(
        std::shared_ptr<PumaSimulationCreateParams> params){
    destination_axis_ = params->destination_axis;
    auto& effector = puma_->effector();

    glm::quat q1(glm::radians(effector.render_object->getRotation()));
    q1 = glm::normalize(q1);

    glm::quat q2(glm::radians(destination_axis_->getRotation()));
    q2 = glm::normalize(q2);

    interpolation_data_.position_start = effector.render_object->getPosition();
    interpolation_data_.rotate_start = q1;

    interpolation_data_.position_end = destination_axis_->getPosition();
    interpolation_data_.rotate_end = q2;
}