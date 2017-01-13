#include "gui/puma_gui.h"

#include "gui/puma_simulation_gui.h"
#include <engine_gui/factory/engine_gui_factory.h>
#include <engine_gui/engine_gui.h>
#include <gui/imgui/imgui.h>

PumaGUI::PumaGUI(GLFWwindow* window,
                 std::shared_ptr<ifx::SceneContainer> scene,
                 std::shared_ptr<PumaSimulation> simulation) :
        ifx::GUI(window){
    engine_gui_ = ifx::EngineGUIFactory().CreateEngineGUI(scene);
    puma_gui_ = std::shared_ptr<PumaSimulationGUI>(
            new PumaSimulationGUI(simulation, scene));
}
PumaGUI::~PumaGUI(){}

void PumaGUI::Render(){
    NewFrame();

    engine_gui_->Render();
    puma_gui_->Render();

    ImGui::Render();
}