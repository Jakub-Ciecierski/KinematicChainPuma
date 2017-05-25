#include <game/game_loop.h>
#include <game/factory/game_loop_factory.h>
#include <graphics/factory/render_object_factory.h>
#include <graphics/rendering/renderer.h>
#include <game/factory/game_factory.h>
#include <game/game.h>
#include <game/scene_container.h>
#include <object/game_object.h>
#include <graphics/factory/scene_factory.h>
#include <graphics/lighting/light_source.h>
#include <graphics/lighting/types/light_directional.h>
#include <graphics/lighting/types/light_spotlight.h>

#include <graphics/rendering/camera/camera.h>
#include <engine_gui/factory/engine_gui_factory.h>
#include <graphics/rendering/renderer.h>
#include <engine_gui/engine_gui.h>
#include <gui/puma_gui.h>
#include <factory/puma_factory.h>
#include <object/game_component.h>
#include <object/render_object.h>
#include <puma_simulation.h>
#include <puma.h>

std::shared_ptr<ifx::LightDirectional> CreateDirectionalLight();
std::shared_ptr<ifx::LightSpotlight> CreateSpotLight();

void AddSimulation(std::shared_ptr<ifx::Game> game);

std::shared_ptr<ifx::LightDirectional> CreateDirectionalLight(){
    ifx::LightParams light;

    light.ambient = glm::vec3(0.3f, 0.3f, 0.3f);
    light.diffuse = glm::vec3(0.3f, 0.3f, 0.3f);
    light.specular = glm::vec3(1.0f, 1.0f, 1.0f);

    auto light_source = std::shared_ptr<ifx::LightDirectional>(
            new ifx::LightDirectional(light));
    light_source->rotateTo(glm::vec3(0, 270, 0));

    light_source->moveTo(glm::vec3(0.0f, 2.5f, 0.0f));
    light_source->LookAt(glm::vec3(0,0,0));

    return light_source;
}

std::shared_ptr<ifx::LightSpotlight> CreateSpotLight(){
    ifx::LightParams light;

    light.ambient = glm::vec3(0.5f, 0.5f, 0.5f);
    light.diffuse = glm::vec3(0.5f, 0.5f, 0.5f);
    light.specular = glm::vec3(1.0f, 1.0f, 1.0f);

    auto light_source = std::shared_ptr<ifx::LightSpotlight>(
            new ifx::LightSpotlight(light));
    light_source->rotateTo(glm::vec3(0, 270, 0));

    light_source->moveTo(glm::vec3(0.0f, 2.5f, 0.0f));
    light_source->LookAt(glm::vec3(0,0,0));

    return light_source;
}

void AddSimulation(std::shared_ptr<ifx::Game> game){
    auto axis = PumaFactory().CreateAxis();
    game->scene()->Add(axis);
    auto axis_render = axis->GetComponents();
    auto simulation_params = std::shared_ptr<PumaSimulationCreateParams>(new
        PumaSimulationCreateParams);

    simulation_params->destination_axis
            = std::static_pointer_cast<ifx::RenderObject>(axis_render[0]);
    auto params = std::shared_ptr<PumaCreateParams>(new PumaCreateParams());
    simulation_params->puma = PumaFactory().CreatePuma(params, -2);
    simulation_params->puma_basic = PumaFactory().CreatePuma(params, -1);

    game->scene()->Add(simulation_params->puma->game_object());
    game->scene()->Add(simulation_params->puma_basic->game_object());
    game->scene()->Add(simulation_params->puma->puma_arms().debug_points);

    auto simulation = std::shared_ptr<PumaSimulation>(
            new PumaSimulation(simulation_params));

    auto gui = std::shared_ptr<PumaGUI>(
            new PumaGUI(
                    game->game_loop()->renderer()->window()->getHandle(),
                    game->scene(),
                    simulation));

    game->game_loop()->AddSimulation(simulation);
    game->game_loop()->renderer()->SetGUI(gui);
}

int main() {
    auto game_factory
            = std::shared_ptr<ifx::GameFactory>(new ifx::GameFactory());
    auto game = game_factory->Create();

    auto game_object1 = std::shared_ptr<ifx::GameObject>(new ifx::GameObject());
    auto game_object2 = std::shared_ptr<ifx::GameObject>(new ifx::GameObject());
    auto game_object3 = std::shared_ptr<ifx::GameObject>(new ifx::GameObject());

    auto lamp = ifx::RenderObjectFactory().CreateLampObject();
    lamp->moveTo(glm::vec3(0.0f, 2.7f, 0.0f));

    game_object1->Add(ifx::RenderObjectFactory().CreateFloor());

    game_object2->Add(CreateSpotLight());
    game_object2->Add(CreateDirectionalLight());

    auto camera
            = ifx::SceneFactory().CreateCamera(
                    game->game_loop()->renderer()->window());
    camera->moveTo(glm::vec3(-4,1,0));
    game_object3->Add(camera);

    game->scene()->Add(game_object1);
    game->scene()->Add(game_object2);
    game->scene()->Add(game_object3);

    AddSimulation(game);

    game->Start();
}
