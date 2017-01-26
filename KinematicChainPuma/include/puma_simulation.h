#ifndef PROJECT_PUMA_SIMULATION_H
#define PROJECT_PUMA_SIMULATION_H

#include <vr/simulation.h>
#include <math/math_ifx.h>
#include <puma.h>

#include <memory>

//class Puma;

namespace ifx{
class RenderObject;
}

struct PumaSimulationCreateParams{
    float simulation_length = 10;
    std::shared_ptr<Puma> puma = nullptr;
    std::shared_ptr<Puma> puma_basic = nullptr;

    std::shared_ptr<ifx::RenderObject> destination_axis;
};

struct InterpolationData{
    glm::vec3 position_start;
    glm::vec3 position_end;

    glm::quat rotate_start;
    glm::quat rotate_end;
};

class PumaSimulation : public ifx::Simulation {
public:
    PumaSimulation(std::shared_ptr<PumaSimulationCreateParams> params);
    ~PumaSimulation();

    std::shared_ptr<Puma> puma() {return puma_;}
    std::shared_ptr<Puma> puma_basic() {return puma_basic_;}
    std::shared_ptr<ifx::RenderObject> destination_axis(){
        return destination_axis_;}
    bool* manipulate_mode(){return &manipulate_mode_;}

    virtual void Update() override;
    void Reset(std::shared_ptr<PumaSimulationCreateParams> params);
protected:
    virtual bool UpdateTime() override;

private:
    void ManipulatePuma();

    void InterpolatePuma();
    glm::vec3 InterpolatePosition(float t);
    glm::vec3 InterpolateRotation(float t);

    void ResetInterpolationData(std::shared_ptr<PumaSimulationCreateParams> params);
    void ResetInterpolationData();

    void InterpolatePumaBasic();
    void ClampDiffAngle(float& angle);

    std::shared_ptr<Puma> puma_;
    std::shared_ptr<Puma> puma_basic_;

    std::shared_ptr<ifx::RenderObject> destination_axis_;
    InterpolationData interpolation_data_;

    PumaState start_state_;
    PumaState end_state_;

    // True if is in manipulate mode
    bool manipulate_mode_;
};


#endif //PROJECT_PUMA_SIMULATION_H
