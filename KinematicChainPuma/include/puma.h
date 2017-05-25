#ifndef PROJECT_PUMA_H
#define PROJECT_PUMA_H

#include <memory>
#include <math/math_ifx.h>

namespace ifx{
class GameObject;
class RenderObject;
}

struct PumaArm {
    std::shared_ptr<ifx::RenderObject> render_object;
    float angle; // alpha
    float length; // l
    glm::vec3 position; // p
    glm::vec3 rotation = glm::vec3(0,0,0); // x,y,z

    // The part connecting this arm with the next.
    std::shared_ptr<ifx::RenderObject> connector_render_object;
};

struct PumaEffector{
    std::shared_ptr<ifx::RenderObject> render_object;
    float angle; // alpha
    glm::vec3 position; // p
    glm::vec3 rotation = glm::vec3(0,0,0); // x,y,z
};

struct PumaArms{
    PumaArm arm1;
    PumaArm arm2;
    PumaArm arm3;
    PumaArm arm4;
    PumaEffector effector;

    std::shared_ptr<ifx::GameObject> game_object;
    std::shared_ptr<ifx::GameObject> debug_points;
};

struct PumaParts {
    PumaArm arm1;
    PumaArm arm2;
    PumaArm arm3;
    PumaArm arm4;
};

struct PumaState{
    float alpha1 = 0;
    float alpha2 = 0;
    float alpha3 = 0;
    float alpha4 = 0;
    float alpha5 = 0;

    float length2 = 1.0;
};

struct PumaFrames{
    glm::mat4 F01;
    glm::mat4 F02;
    glm::mat4 F03;
    glm::mat4 F04;
    glm::mat4 F05;
};

struct Decomposition{
    glm::vec3 rotation;
    glm::vec3 position;
};

struct InverseKinematicsInput{
    glm::vec3 p0;
    glm::vec3 x0;
    glm::vec3 y0;
    glm::vec3 z0;

    glm::vec3 p5;
    glm::vec3 x5;
    glm::vec3 y5;
    glm::vec3 z5;

    float l1;
    float l3;
    float l4;
};

/**
 * YZ axies - Y is up (As opposed to ZX)
 * Original / Mine
 * X : Z
 * Y : X
 * Z : Y
 */

/**
 * IFX : Paper
 * X : Y
 * Y : Z
 * Z : X
 *
 */
class Puma {
public:

    Puma(PumaArms& puma_arms);
    ~Puma();

    PumaArms& puma_arms(){return puma_arms_;}
    std::shared_ptr<ifx::GameObject> game_object(){
        return puma_arms_.game_object;}
    PumaEffector& effector(){return puma_arms_.effector;}
    PumaState& state(){return state_;}

    float* z4_multiplier(){return &z4_multiplier_;}
    float* angle_multiplier(){return &angle_multiplier_;}


    /**
     * Inverse Kinematics.
     * Given effector postion, compute state
     */
    void ComputeInverse();

    void ComputeDirect();
    void ComputeDirectAndUpdate(PumaState& state);

    PumaState ComputeOptimalState(float z_multiplier);
    PumaState ComputeOptimalState();
private:
    // <Inverse>
    InverseKinematicsInput PreperInput();

    PumaState ComputeInverseAlgeb(InverseKinematicsInput in);


    PumaState ComputeInverse(InverseKinematicsInput in,
                             float z4_multiplier);
    PumaState ComputeInverseWithAngleMultiplier(
            InverseKinematicsInput in,
            float z4_multiplier,
            float alpha4_multiplier,
            float alpha3_multiplier);

    glm::vec3 ComputeZ4(const glm::vec3& x5, const glm::vec3& n);
    float ComputeDistance(const PumaState& state1, const PumaState& state2);
    float AngleDistance(float angle1, float angle2);
    /**
     * In degrees
     */
    float Angle(const glm::vec3& v, const glm::vec3& w, bool info = false);

    float AngleNormal(const glm::vec3 &v, const glm::vec3 &w,
                      const glm::vec3 &normal);
    float Angle3(const glm::vec3& v, const glm::vec3& w, float multiplier);
    float Angle4(const glm::vec3& v, const glm::vec3& w, float multiplier);
    void ClampState(PumaState& state);
    void UpdateDebugPoints(const glm::vec3& p1, const glm::vec3& p2,
                           const glm::vec3& p3, const glm::vec3& p4,
                           const glm::vec3& p5);
    // </Inverse>

    // <Direction>
    /**
     * Direct solution.
     * Given state, compute effector position.
     */
    void ComputeDirect(PumaState& state, bool update_effector = false);
    PumaFrames CalculateDirectFrame(PumaState& state);

    void UpdateEffector(glm::mat4& F);
    void UpdateArm1(glm::mat4& F);
    void UpdateArm2(glm::mat4& F, float q2);
    void UpdateArm3(glm::mat4& F);
    void UpdateArm4(glm::mat4& F);
    // </Direction>

    void UpdateGameObject();

    bool HasNan(PumaState& state);

    // TODO Move the bellow functions.
    Decomposition Decompose(glm::mat4& F);

    glm::mat4 Tx(float x);
    glm::mat4 Ty(float x);
    glm::mat4 Tz(float x);
    glm::mat4 Translate(const glm::vec3& position);
    glm::mat4 Rx(float angle);
    glm::mat4 Ry(float angle);
    glm::mat4 Rz(float angle);

    PumaArms puma_arms_;
    PumaState state_;

    glm::vec3 last_effector_position_;
    glm::vec3 last_effector_rotation_;

    const glm::vec3 INIT = glm::vec3(999, 999, 999);
    glm::vec3 current_p3_;
    glm::vec3 last_p3_;

    glm::vec3 last_z4_;

    float z4_multiplier_;
    float angle_multiplier_;
};


#endif //PROJECT_PUMA_H
