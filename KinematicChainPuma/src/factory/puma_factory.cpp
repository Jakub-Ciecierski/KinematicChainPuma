#include "factory/puma_factory.h"

#include <object/game_object.h>
#include <graphics/factory/model_factory.h>
#include <graphics/factory/program_factory.h>
#include <object/render_object.h>
#include <graphics/model_loader/model_loader.h>
#include <puma.h>
#include <graphics/factory/render_object_factory.h>

PumaFactory::PumaFactory() : id_(0){}
PumaFactory::~PumaFactory(){}

std::shared_ptr<ifx::GameObject> PumaFactory::CreateAxis(){
    auto render_object = std::shared_ptr<ifx::RenderObject>(
            new ifx::RenderObject(ObjectID(0), CreateAxisModel()));
    render_object->addProgram(ifx::ProgramFactory().LoadMainProgram());
    render_object->scale(0.3);
    render_object->moveTo(glm::vec3(0, 1, 0));
    render_object->rotateTo(glm::vec3(90, 0, 0));

    auto game_object = std::shared_ptr<ifx::GameObject>(new ifx::GameObject());
    game_object->Add(render_object);

    return game_object;
}

std::shared_ptr<Puma> PumaFactory::CreatePuma(
        std::shared_ptr<PumaCreateParams> params,
        int id){
    id_ = id;
    return CreatePuma(params);
}

std::shared_ptr<Puma> PumaFactory::CreatePuma(
        std::shared_ptr<PumaCreateParams> params){
    PumaArms arms;

    arms.arm1 = CreatePumaArm1(params);
    arms.arm2 = CreatePumaArm2(params);
    arms.arm3 = CreatePumaArm3(params);
    arms.arm4 = CreatePumaArm4(params);
    arms.effector = CreatePumaEffector(params);

    arms.game_object = std::shared_ptr<ifx::GameObject>(new ifx::GameObject());
    arms.game_object->Add(arms.arm1.render_object);
    arms.game_object->Add(arms.arm1.connector_render_object);
    arms.game_object->Add(arms.arm2.render_object);
    arms.game_object->Add(arms.arm2.connector_render_object);
    arms.game_object->Add(arms.arm3.render_object);
    arms.game_object->Add(arms.arm3.connector_render_object);
    arms.game_object->Add(arms.arm4.render_object);
    arms.game_object->Add(arms.effector.render_object);

    arms.debug_points = std::shared_ptr<ifx::GameObject>(new ifx::GameObject());
    arms.debug_points->Add(ifx::RenderObjectFactory().CreateLampObject());
    arms.debug_points->Add(ifx::RenderObjectFactory().CreateLampObject());
    arms.debug_points->Add(ifx::RenderObjectFactory().CreateLampObject());
    arms.debug_points->Add(ifx::RenderObjectFactory().CreateLampObject());
    arms.debug_points->Add(ifx::RenderObjectFactory().CreateLampObject());

    auto puma = std::shared_ptr<Puma>(new Puma(arms));

    return puma;
}

std::vector<std::shared_ptr<ifx::Model>> PumaFactory::CreateAxisModel(){
    std::string path_x
            = ifx::Resources::GetInstance().GetResourcePath(
                    "axis-obj/axis.obj", ifx::ResourceType::MODEL);

    std::vector<std::shared_ptr<ifx::Model>> models;
    models.push_back(ifx::ModelLoader(path_x).loadModel());

    return models;
}

PumaArm PumaFactory::CreatePumaArm1(
        std::shared_ptr<PumaCreateParams> params){
    PumaArm arm;
    arm.angle = 0;
    arm.position = glm::vec3(0,0,0);
    arm.length = params->l1;
    arm.render_object = CreatePumaArm(CreatePumaArmModelRed());
    arm.render_object->scale(glm::vec3(1, params->l1, 1));

    arm.connector_render_object = CreatePumaArmConnector();
    arm.connector_render_object->rotateTo(glm::vec3(0, 0, 90));
    arm.connector_render_object->move(glm::vec3(0, params->l1, 0));

    arm.connector_render_object->move(glm::vec3(0.06, 0, 0));

    return arm;
}
PumaArm PumaFactory::CreatePumaArm2(
        std::shared_ptr<PumaCreateParams> params){
    PumaArm arm;
    arm.angle = 0;
    arm.position = glm::vec3(0, params->l1,0);
    arm.length = params->l2;
    arm.render_object = CreatePumaArm(CreatePumaArmModelBlue());
    arm.connector_render_object = CreatePumaArmConnector();

    arm.render_object->scale(glm::vec3(1, params->l2, 1));
    arm.render_object->rotateTo(glm::vec3(90, 0,0));
    arm.render_object->moveTo(glm::vec3(0, params->l1,0));

    // move as a1
    arm.connector_render_object->rotateTo(glm::vec3(0, 0, 90));
    arm.connector_render_object->move(glm::vec3(0, params->l1, 0));
    arm.connector_render_object->move(glm::vec3(0.06, 0, 0));
    // move
    arm.connector_render_object->move(glm::vec3(0, 0, params->l2));

    return arm;
}
PumaArm PumaFactory::CreatePumaArm3(
        std::shared_ptr<PumaCreateParams> params){
    PumaArm arm;
    arm.angle = 0;
    arm.position = glm::vec3(0, params->l1, params->l2);
    arm.length = params->l3;
    arm.render_object = CreatePumaArm(CreatePumaArmModelGreen());
    arm.connector_render_object = CreatePumaArmConnector();

    // move to a2
    arm.render_object->rotateTo(glm::vec3(90, 0,0));
    arm.render_object->moveTo(glm::vec3(0, params->l1,0));

    arm.render_object->move(glm::vec3(0, 0, params->l2));
    arm.render_object->rotate(glm::vec3(90, 0,0));

    arm.render_object->scale(glm::vec3(1, params->l3, 1));

    // move as a2
    arm.connector_render_object->rotateTo(glm::vec3(0, 0, 90));
    arm.connector_render_object->move(glm::vec3(0, params->l1, 0));
    arm.connector_render_object->move(glm::vec3(0.06, 0, 0));
    arm.connector_render_object->move(glm::vec3(0, 0, params->l2));

    arm.connector_render_object->move(glm::vec3(0, -params->l3, 0));
    arm.connector_render_object->rotate(glm::vec3(0, 0, -90));
    arm.connector_render_object->move(glm::vec3(-0.06, 0, 0));
    arm.connector_render_object->move(glm::vec3(0, -0.06, 0));

    return arm;
}
PumaArm PumaFactory::CreatePumaArm4(
        std::shared_ptr<PumaCreateParams> params){
    PumaArm arm;
    arm.angle = 0;
    arm.position = glm::vec3(0, params->l1 - params->l3, params->l2);
    arm.length = params->l4;
    arm.render_object = CreatePumaArm(CreatePumaArmModelRed());
    arm.connector_render_object = nullptr;

    // move to a3
    arm.render_object->rotateTo(glm::vec3(90, 0,0));
    arm.render_object->moveTo(glm::vec3(0, params->l1,0));
    arm.render_object->move(glm::vec3(0, 0, params->l2));
    arm.render_object->rotate(glm::vec3(90, 0,0));

    arm.render_object->rotate(glm::vec3(-90, 0,0));
    arm.render_object->move(glm::vec3(0, -params->l3, 0));

    arm.render_object->scale(glm::vec3(1, params->l4, 1));

    return arm;
}

PumaEffector PumaFactory::CreatePumaEffector(
        std::shared_ptr<PumaCreateParams> params){
    PumaEffector effector;
    effector.angle = 0;
    effector.position = glm::vec3(0,
                                  params->l1 - params->l3,
                                  params->l2 + params->l4);
    effector.rotation = glm::vec3(0,0,0);
    effector.render_object = CreatePumaEffector();

    // move to a3
    //effector.render_object->rotateTo(glm::vec3(90, 0,0));
    effector.render_object->moveTo(glm::vec3(0, params->l1,0));
    effector.render_object->move(glm::vec3(0, 0, params->l2));
    //effector.render_object->rotate(glm::vec3(90, 0,0));
    //effector.render_object->rotate(glm::vec3(-90, 0,0));
    effector.render_object->move(glm::vec3(0, -params->l3, 0));

    effector.render_object->move(glm::vec3(0,0, params->l4));

    return effector;
}

std::shared_ptr<ifx::RenderObject> PumaFactory::CreatePumaArm(
        std::shared_ptr<ifx::Model> model){
    auto render_object = std::shared_ptr<ifx::RenderObject>(
            new ifx::RenderObject(ObjectID(id_), model));
    render_object->id().key_id(id_);
    render_object->addProgram(ifx::ProgramFactory().LoadMainProgram());

    return render_object;
}

std::shared_ptr<ifx::Model> PumaFactory::CreatePumaArmModelRed(){
    std::string path
            = ifx::Resources::GetInstance().GetResourcePath(
                    "rod-red/rod.obj", ifx::ResourceType::MODEL);

    return ifx::ModelLoader(path).loadModel();
}
std::shared_ptr<ifx::Model> PumaFactory::CreatePumaArmModelBlue(){
    std::string path
            = ifx::Resources::GetInstance().GetResourcePath(
                    "rod-blue/rod.obj", ifx::ResourceType::MODEL);

    return ifx::ModelLoader(path).loadModel();
}
std::shared_ptr<ifx::Model> PumaFactory::CreatePumaArmModelGreen(){
    std::string path
            = ifx::Resources::GetInstance().GetResourcePath(
                    "rod-green/rod.obj", ifx::ResourceType::MODEL);

    return ifx::ModelLoader(path).loadModel();
}

std::shared_ptr<ifx::RenderObject> PumaFactory::CreatePumaArmConnector(){
    auto render_object = std::shared_ptr<ifx::RenderObject>(
            new ifx::RenderObject(ObjectID(id_),
                                  CreatePumaArmModelConnector()));
    render_object->id().key_id(id_);
    render_object->addProgram(ifx::ProgramFactory().LoadMainProgram());
    render_object->scale(1.2);
    return render_object;
}

std::shared_ptr<ifx::Model> PumaFactory::CreatePumaArmModelConnector(){
    std::string path
            = ifx::Resources::GetInstance().GetResourcePath(
                    "rod-small-obj/rod-small.obj", ifx::ResourceType::MODEL);

    return ifx::ModelLoader(path).loadModel();
}

std::shared_ptr<ifx::RenderObject> PumaFactory::CreatePumaEffector(){
    auto render_object = std::shared_ptr<ifx::RenderObject>(
            new ifx::RenderObject(ObjectID(id_),
                                  CreateAxisModel()));
    render_object->id().key_id(id_);

    render_object->addProgram(ifx::ProgramFactory().LoadMainProgram());
    render_object->scale(0.2);
    return render_object;
}