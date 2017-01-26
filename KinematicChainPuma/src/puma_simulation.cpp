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
    //Pause();
}
PumaSimulation::~PumaSimulation(){}

void PumaSimulation::Update(){
    if(!UpdateTime())
        return;

    if(manipulate_mode_){
        ResetInterpolationData();
        ManipulatePuma();
        puma_->ComputeInverse();
    }else{
        // Right screen
        InterpolatePuma();

        // Left screen
        InterpolatePumaBasic();
    }


}

void PumaSimulation::Reset(std::shared_ptr<PumaSimulationCreateParams> params){
    ifx::Simulation::Reset();

    time_data_.simulation_length = params->simulation_length;
    if(params->puma)
        puma_ = params->puma;
    if(params->puma_basic)
        puma_basic_ = params->puma_basic;

    ResetInterpolationData(params);
}

void PumaSimulation::ManipulatePuma(){
    auto& effector = puma_->effector();
    effector.position = effector.render_object->getPosition();
    effector.rotation = effector.render_object->getRotation();
}

void PumaSimulation::InterpolatePuma(){
    float t = time_data_.total_time / time_data_.simulation_length;
    if(t > 1.0)
        return;

    auto& effector = puma_->effector();

    effector.position = InterpolatePosition(t);
    effector.rotation = InterpolateRotation(t);

    effector.render_object->moveTo(effector.position);
    effector.render_object->rotateTo(effector.rotation);

    puma_->ComputeInverse();
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
    //ResetInterpolationData();
}

void PumaSimulation::ResetInterpolationData(){
    auto& effector = puma_->effector();

    glm::quat q1(glm::radians(effector.render_object->getRotation()));
    q1 = glm::normalize(q1);

    glm::quat q2(glm::radians(destination_axis_->getRotation()));
    q2 = glm::normalize(q2);

    interpolation_data_.position_start = effector.render_object->getPosition();
    interpolation_data_.rotate_start = q1;

    interpolation_data_.position_end = destination_axis_->getPosition();
    interpolation_data_.rotate_end = q2;

    // Get start
    puma_basic_->puma_arms().effector.position
            = interpolation_data_.position_start;
    puma_basic_->puma_arms().effector.rotation
            = effector.render_object->getRotation();
    start_state_ = puma_basic_->ComputeOptimalState();

    // Get end
    puma_basic_->puma_arms().effector.position
            = interpolation_data_.position_end;
    puma_basic_->puma_arms().effector.rotation
            = destination_axis_->getRotation();
    end_state_ = puma_basic_->ComputeOptimalState();

    // Reset
    puma_basic_->puma_arms().effector.position
            = interpolation_data_.position_start;
    puma_basic_->puma_arms().effector.rotation
            = effector.render_object->getRotation();

}

bool PumaSimulation::UpdateTime(){
    time_data_.current_time = glfwGetTime();
    if(!running_){
        time_data_.last_time = time_data_.current_time;
        return false;
    }
    double elapsed = time_data_.current_time - time_data_.last_time;
    time_data_.time_since_last_update += elapsed;
    if(!manipulate_mode_)
        time_data_.total_time += elapsed;
    time_data_.last_time = time_data_.current_time;

    if(time_data_.time_since_last_update >= time_data_.time_delta){
        time_data_.time_since_last_update = 0.0f;
        return true;
    }
}

void PumaSimulation::InterpolatePumaBasic(){
    float t = time_data_.total_time / time_data_.simulation_length;
    if(t > 1.0)
        return;

    float diff1 = end_state_.alpha1 - start_state_.alpha1;
    float diff2 = end_state_.alpha2 - start_state_.alpha2;
    float diff3 = end_state_.alpha3 - start_state_.alpha3;
    float diff4 = end_state_.alpha4 - start_state_.alpha4;
    float diff5 = end_state_.alpha5 - start_state_.alpha5;
    float diff_length = end_state_.length2 - start_state_.length2;
    ClampDiffAngle(diff1);

    ClampDiffAngle(diff2);
    ClampDiffAngle(diff3);
    ClampDiffAngle(diff4);
    ClampDiffAngle(diff5);

    PumaState state;
    state.alpha1 = start_state_.alpha1 + diff1 * t;
    state.alpha2 = start_state_.alpha2 + diff2 * t;
    state.alpha3 = start_state_.alpha3 + diff3 * t;
    state.alpha4 = start_state_.alpha4 + diff4 * t;
    state.alpha5 = start_state_.alpha5 + diff5 * t;
    state.length2 = start_state_.length2 + diff_length * t;

    puma_basic_->ComputeDirectAndUpdate(state);
}

void PumaSimulation::ClampDiffAngle(float& diff){
    if(diff > 180)
        diff = -(360 - diff);
    if(diff < -180)
        diff = (360 + diff);
}