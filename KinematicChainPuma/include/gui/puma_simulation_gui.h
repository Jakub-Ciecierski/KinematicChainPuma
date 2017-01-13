#ifndef PROJECT_PUMA_SIMULATION_GUI_H
#define PROJECT_PUMA_SIMULATION_GUI_H

#include <memory>

class PumaSimulation;
struct PumaSimulationCreateParams;
struct PumaCreateParams;

namespace ifx {
class SceneContainer;
}

class PumaSimulationGUI {
public:

    PumaSimulationGUI(std::shared_ptr<PumaSimulation> simulation,
                      std::shared_ptr<ifx::SceneContainer> scene);
    ~PumaSimulationGUI();

    void Render();
private:
    void RenderSimulationInfo();

    void RenderPumaParams();
    void RenderPumaParamsInitial();
    void RenderPumaParamsState();

    void Reset();

    std::shared_ptr<PumaSimulation> simulation_;
    std::shared_ptr<ifx::SceneContainer> scene_;

    std::shared_ptr<PumaSimulationCreateParams> simulation_params_;
    std::shared_ptr<PumaCreateParams> puma_params_;
};


#endif //PROJECT_PUMA_SIMULATION_GUI_H
