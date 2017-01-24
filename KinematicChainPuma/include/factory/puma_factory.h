#ifndef PROJECT_PUMA_FACTORY_H
#define PROJECT_PUMA_FACTORY_H

#include <memory>
#include <vector>
#include <puma.h>

struct PumaCreateParams{
    float l1 = 1;
    float l2 = 1;
    float l3 = 0.5;
    float l4 = 0.5;
};

namespace ifx{
class GameObject;
class Model;
class RenderObject;
}

class PumaFactory {
public:
    PumaFactory();
    ~PumaFactory();

    std::shared_ptr<ifx::GameObject> CreateAxis();

    std::shared_ptr<Puma> CreatePuma(std::shared_ptr<PumaCreateParams> params);
    std::shared_ptr<Puma> CreatePuma(std::shared_ptr<PumaCreateParams> params,
                                     int id);
private:
    std::vector<std::shared_ptr<ifx::Model>> CreateAxisModel();

    PumaArm CreatePumaArm1(std::shared_ptr<PumaCreateParams> params);
    PumaArm CreatePumaArm2(std::shared_ptr<PumaCreateParams> params);
    PumaArm CreatePumaArm3(std::shared_ptr<PumaCreateParams> params);
    PumaArm CreatePumaArm4(std::shared_ptr<PumaCreateParams> params);
    PumaEffector CreatePumaEffector(std::shared_ptr<PumaCreateParams> params);

    std::shared_ptr<ifx::RenderObject> CreatePumaArm(
            std::shared_ptr<ifx::Model> model);
    std::shared_ptr<ifx::Model> CreatePumaArmModelRed();
    std::shared_ptr<ifx::Model> CreatePumaArmModelBlue();
    std::shared_ptr<ifx::Model> CreatePumaArmModelGreen();

    std::shared_ptr<ifx::RenderObject> CreatePumaArmConnector();
    std::shared_ptr<ifx::Model> CreatePumaArmModelConnector();

    std::shared_ptr<ifx::RenderObject> CreatePumaEffector();

    int id_;
};


#endif //PROJECT_PUMA_FACTORY_H
