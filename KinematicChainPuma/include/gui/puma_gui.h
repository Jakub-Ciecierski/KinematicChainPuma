#ifndef PROJECT_EXAMPLE_GUI_H
#define PROJECT_EXAMPLE_GUI_H

#include "gui/gui.h"

#include <memory>

class PumaSimulation;
class PumaSimulationGUI;

namespace ifx{
class EngineGUI;
class SceneContainer;
}

class PumaGUI : public ifx::GUI{
public:

    PumaGUI(GLFWwindow* window,
            std::shared_ptr<ifx::SceneContainer> scene,
            std::shared_ptr<PumaSimulation> simulation);
    ~PumaGUI();

    virtual void Render() override;
private:
    std::shared_ptr<ifx::EngineGUI> engine_gui_;

    std::shared_ptr<PumaSimulationGUI> puma_gui_;
};


#endif //PROJECT_EXAMPLE_GUI_H
