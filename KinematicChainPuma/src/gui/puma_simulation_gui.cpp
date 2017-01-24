#include <gui/imgui/imgui.h>
#include "gui/puma_simulation_gui.h"

#include <puma_simulation.h>
#include <puma.h>
#include <factory/puma_factory.h>

#include <game/scene_container.h>

PumaSimulationGUI::PumaSimulationGUI(
        std::shared_ptr<PumaSimulation> simulation,
        std::shared_ptr<ifx::SceneContainer> scene) :
        simulation_(simulation),
        scene_(scene){
    simulation_params_ = std::shared_ptr<PumaSimulationCreateParams>
            (new PumaSimulationCreateParams());
    simulation_params_->puma = simulation_->puma();
    simulation_params_->destination_axis = simulation_->destination_axis();

    puma_params_ = std::shared_ptr<PumaCreateParams>(new PumaCreateParams());
}

PumaSimulationGUI::~PumaSimulationGUI(){}

void PumaSimulationGUI::Render(){
    ImGui::Begin("Puma");

    if(ImGui::CollapsingHeader("Simulation"), 1)
        RenderSimulationInfo();

    if(ImGui::CollapsingHeader("Puma"), 1)
        RenderPumaParams();

    ImGui::End();
}

void PumaSimulationGUI::RenderSimulationInfo(){
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);

    ImGui::Text("Time: %.2f [s]", simulation_->time_data().total_time);
    ImGui::SameLine();
    ImGui::PushItemWidth(100);
    ImGui::ProgressBar(simulation_->time_data().total_time /
                       simulation_->time_data().simulation_length);
    ImGui::PopItemWidth();

    ImGui::PushItemWidth(100);
    ImGui::InputFloat("Simulation Length [s]",
                      &(simulation_params_->simulation_length));
    ImGui::PopItemWidth();
    if (ImGui::Button("Reset")) {
        Reset();
    }
    ImGui::SameLine();

    std::string play_button_text;
    if (simulation_->IsRunning())
        play_button_text = "Pause";
    else
        play_button_text = "Play";

    if (ImGui::Button(play_button_text.c_str())) {
        simulation_->SetRunning(!simulation_->IsRunning());
    }
}

void PumaSimulationGUI::RenderPumaParams(){
    ImGui::Checkbox("Manipulate Mode", simulation_->manipulate_mode());

    if(ImGui::TreeNode("Initial")){
        RenderPumaParamsInitial();
        ImGui::TreePop();
    }

    if(ImGui::TreeNode("State")){
        RenderPumaParamsState();
        ImGui::TreePop();
    }

}

void PumaSimulationGUI::RenderPumaParamsInitial(){
    float lengths[4];
    lengths[0] = puma_params_->l1;
    lengths[1] = puma_params_->l2;
    lengths[2] = puma_params_->l3;
    lengths[3] = puma_params_->l4;

    ImGui::InputFloat4("Arms Length", lengths);

    puma_params_->l1 = lengths[0];
    puma_params_->l2 = lengths[1];
    puma_params_->l3 = lengths[2];
    puma_params_->l4 = lengths[3];
}
void PumaSimulationGUI::RenderPumaParamsState(){
    PumaState& state = simulation_->puma()->state();

    ImGui::SliderFloat("a1", &state.alpha1, 0, 360);
    ImGui::SliderFloat("a2", &state.alpha2, 0, 360);
    ImGui::SliderFloat("a3", &state.alpha3, 0, 360);
    ImGui::SliderFloat("a4", &state.alpha4, 0, 360);
    ImGui::SliderFloat("a5", &state.alpha5, 0, 360);
    ImGui::SliderFloat("q2", &state.length2, 0.1, 10);

    ImGui::SliderFloat("z4_multiplier",
                       simulation_->puma()->z4_multiplier(), -1, 1);
    ImGui::SliderFloat("angle_multiplier",
                       simulation_->puma()->angle_multiplier(), -1, 1);

}

void PumaSimulationGUI::Reset(){
    if(simulation_params_->puma)
        scene_->Remove(simulation_params_->puma->game_object());
    if(simulation_params_->puma_basic)
        scene_->Remove(simulation_params_->puma_basic->game_object());

    simulation_params_->puma = PumaFactory().CreatePuma(puma_params_, -2);
    simulation_params_->puma_basic = PumaFactory().CreatePuma(puma_params_, -1);

    scene_->Add(simulation_params_->puma->game_object());
    scene_->Add(simulation_params_->puma_basic->game_object());

    simulation_->Reset(simulation_params_);
}
