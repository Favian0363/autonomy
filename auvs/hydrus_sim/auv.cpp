#include <cmath>
#include <math.h>

#include "auv.h"

#include "actuators/Actuator.h"
#include "actuators/Thruster.h"
#include "core/GraphicalSimulationApp.h"
#include "core/Robot.h"
#include "core/ScenarioParser.h"
#include "core/SimulationManager.h"
#include "entities/Entity.h"
#include "entities/MovingEntity.h"
#include "entities/StaticEntity.h"
#include "graphics/OpenGLDataStructs.h"
#include "sensors/Sensor.h"
#include "sensors/ScalarSensor.h"


#include <cstdint>
#include <filesystem>
#include <optional>
#include <print>
#include <string>
#include <string_view>
#include <unordered_map>


static std::filesystem::path dataPath = "auvs/hydrus_sim/data/";
static std::string scenario_file = "scenarios/hydrus_env.scn";
static constexpr sf::Scalar kStepsPerSecond = 200.0;

namespace
{
enum class ObjectCls : AuvObjectCls
{
    cube = 0,
    rect = 1,
    gate = 2,
};

std::optional<ObjectCls> ParseObjectClass(const char* text)
{
    if(text == nullptr || *text == '\0')
        return std::nullopt;

    const std::string_view cls = text;
    if(cls == "cube" || cls == "0")
        return ObjectCls::cube;
    if(cls == "rect" || cls == "1")
        return ObjectCls::rect;
    if(cls == "gate" || cls == "2")
        return ObjectCls::gate;
    return std::nullopt;
}

std::optional<ObjectCls> InferObjectClass(const std::string& name)
{
    if(name.find("gate") != std::string::npos)
        return ObjectCls::gate;
    if(name.find("rect") != std::string::npos)
        return ObjectCls::rect;
    if(name.find("cube") != std::string::npos)
        return ObjectCls::cube;
    return std::nullopt;
}

MathPose ToMathPose(const sf::Transform& transform)
{
    const sf::Vector3 pos = transform.getOrigin();
    const sf::Quaternion quat = transform.getRotation();
    MathPose pose{};
    pose.pos.buf[0] = static_cast<float>(pos.x());
    pose.pos.buf[1] = static_cast<float>(pos.y());
    pose.pos.buf[2] = static_cast<float>(pos.z());
    pose.quat.buf[0] = static_cast<float>(quat.x());
    pose.quat.buf[1] = static_cast<float>(quat.y());
    pose.quat.buf[2] = static_cast<float>(quat.z());
    pose.quat.buf[3] = static_cast<float>(quat.w());
    return pose;
}

MathPose IdentityPose()
{
    MathPose pose{};
    pose.quat.buf[3] = 1.f;
    return pose;
}

sf::Transform EntityWorldTransform(sf::Entity* entity)
{
    switch(entity->getType())
    {
        case sf::EntityType::STATIC:
            return static_cast<sf::StaticEntity*>(entity)->getTransform();
        case sf::EntityType::SOLID:
        case sf::EntityType::ANIMATED:
            return static_cast<sf::MovingEntity*>(entity)->getOTransform();
        default:
            return sf::I4();
    }
}

bool FillObjectBBox(sf::Entity* entity, MathBoundingBox& bbox)
{
    sf::Vector3 min;
    sf::Vector3 max;
    entity->getAABB(min, max);
    const sf::Vector3 size = max - min;
    if(size.x() <= 0 || size.y() <= 0 || size.z() <= 0)
        return false;

    const sf::Vector3 center = (min + max) * sf::Scalar(0.5);
    bbox = {};
    bbox.pose = ToMathPose(EntityWorldTransform(entity));
    bbox.pose.pos.buf[0] = static_cast<float>(center.x());
    bbox.pose.pos.buf[1] = static_cast<float>(center.y());
    bbox.pose.pos.buf[2] = static_cast<float>(center.z());
    bbox.size.buf[0] = static_cast<float>(size.x());
    bbox.size.buf[1] = static_cast<float>(size.y());
    bbox.size.buf[2] = static_cast<float>(size.z());
    return true;
}

class AuvScenarioParser : public sf::ScenarioParser
{
public:
    using ScenarioParser::ScenarioParser;
    std::unordered_map<std::string, ObjectCls> objectClasses;

protected:
    bool ParseStatic(XMLElement* element) override
    {
        const char* name = nullptr;
        element->QueryStringAttribute("name", &name);

        const char* cls = nullptr;
        if(element->QueryStringAttribute("cls", &cls) != XML_SUCCESS)
            element->QueryStringAttribute("clss", &cls);

        if(name != nullptr)
        {
            if(const auto mapped = ParseObjectClass(cls))
                objectClasses[name] = *mapped;
        }

        return ScenarioParser::ParseStatic(element);
    }
};

class SimManager : public sf::SimulationManager
{
public:
    SimManager(sf::Scalar stepsPerSecond, std::filesystem::path scenarioPath)
        : SimulationManager(stepsPerSecond),
          scenarioPath(std::move(scenarioPath))
    {
    }

    void BuildScenario() override
    {
        AuvScenarioParser parser(this);
        if(!parser.Parse(scenarioPath.string()))
        {
            std::println(stderr, "Failed to parse scenario {}", scenarioPath.string());
            return;
        }
        objectClasses = std::move(parser.objectClasses);

        odometry = nullptr;
        for(unsigned int i = 0; sf::Sensor* sensor = getSensor(i); ++i)
        {
            if(sensor->getType() != sf::SensorType::LINK)
                continue;
            auto* scalar = static_cast<sf::ScalarSensor*>(sensor);
            if(scalar->getScalarSensorType() == sf::ScalarSensorType::ODOM)
            {
                odometry = scalar;
                break;
            }
        }

        thrusters.clear();
        sf::Robot* robot = getRobot(0u);
        for(size_t i = 0; sf::Actuator* actuator = robot->getActuator(i); ++i)
            thrusters.push_back(static_cast<sf::Thruster*>(actuator));
    }

    void setThrustorValues(const float* thrustor_values, uint8_t thrustor_values_len)
    {
        for(uint8_t i = 0; i < thrustor_values_len; ++i)
            thrusters[i]->setSetpoint(thrustor_values[i]);
    }

    AuvFrame getAuvFrame()
    {
        AuvFrame frame{};
        frame.camera_pose = IdentityPose();
        frame.timestamp = static_cast<uint64_t>(getSimulationTime() * sf::Scalar(1e9));

        if(odometry != nullptr && odometry->getNumOfChannels() >= 10)
        {
            frame.camera_pose.pos.buf[0] = static_cast<float>(odometry->getLastValue(0));
            frame.camera_pose.pos.buf[1] = static_cast<float>(odometry->getLastValue(1));
            frame.camera_pose.pos.buf[2] = static_cast<float>(odometry->getLastValue(2));
            frame.camera_pose.quat.buf[0] = static_cast<float>(odometry->getLastValue(6));
            frame.camera_pose.quat.buf[1] = static_cast<float>(odometry->getLastValue(7));
            frame.camera_pose.quat.buf[2] = static_cast<float>(odometry->getLastValue(8));
            frame.camera_pose.quat.buf[3] = static_cast<float>(odometry->getLastValue(9));
        }

        for(unsigned int i = 0; sf::Entity* entity = getEntity(i); ++i)
        {
            if(frame.objects_len == AUV_FRAME_MAX_OBJECTS)
                break;
            if(entity->getType() == sf::EntityType::FORCEFIELD)
                continue;
            if(entity->getType() == sf::EntityType::STATIC)
            {
                auto* staticEntity = static_cast<sf::StaticEntity*>(entity);
                if(staticEntity->getStaticType() == sf::StaticEntityType::PLANE
                   || staticEntity->getStaticType() == sf::StaticEntityType::TERRAIN)
                    continue;
            }

            std::optional<ObjectCls> cls;
            if(auto it = objectClasses.find(entity->getName()); it != objectClasses.end())
                cls = it->second;
            else
                cls = InferObjectClass(entity->getName());
            if(!cls)
                continue;

            AuvObject object{};
            if(!FillObjectBBox(entity, object.bbox))
                continue;
            object.id = i;
            object.cls = static_cast<AuvObjectCls>(*cls);
            frame.objects[frame.objects_len++] = object;
        }

        return frame;
    }

private:
    std::filesystem::path scenarioPath;
    std::unordered_map<std::string, ObjectCls> objectClasses;
    sf::ScalarSensor* odometry = nullptr;
    std::vector<sf::Thruster*> thrusters;
};

class SimApp : public sf::GraphicalSimulationApp
{
public:
    SimApp(std::string dataPath, sf::RenderSettings render_setting, sf::HelperSettings helper_settings, sf::SimulationManager* sim_manager)
        : sf::GraphicalSimulationApp("hydrus_sim", dataPath, render_setting, helper_settings, sim_manager)
    {
    }

    void start(sf::Scalar timeStep)
    {
        autostep_ = false;
        timeStep_ = timeStep;
        Init();
        StartSimulation();
    }

    void pump()
    {
        if(getState() != sf::SimulationState::FINISHED)
            LoopInternal();
    }

    void shutdown()
    {
        if(cleaned)
            return;
        if(getState() != sf::SimulationState::FINISHED)
            Quit();
        CleanUp();
        cleaned = true;
    }

private:
    bool cleaned = false;
};

struct SimulationContext
{
    SimManager* sim = nullptr;
    SimApp* app = nullptr;
};

SimulationContext* g_simulation_context = nullptr;

sf::RenderSettings DefaultRenderSettings()
{
    sf::RenderSettings s;
    s.windowW = 1200;
    s.windowH = 900;
    s.aa = sf::RenderQuality::HIGH;
    s.shadows = sf::RenderQuality::HIGH;
    s.ao = sf::RenderQuality::HIGH;
    s.atmosphere = sf::RenderQuality::MEDIUM;
    s.ocean = sf::RenderQuality::HIGH;
    s.ssr = sf::RenderQuality::HIGH;
    return s;
}

sf::HelperSettings DefaultHelperSettings()
{
    sf::HelperSettings h;
    h.showFluidDynamics = false;
    h.showCoordSys = false;
    h.showBulletDebugInfo = false;
    h.showSensors = false;
    h.showActuators = false;
    h.showForces = false;
    return h;
}
}

void auv_init(void)
{
    auv_deinit();

    g_simulation_context = new SimulationContext();
    g_simulation_context->sim = new SimManager(kStepsPerSecond, dataPath / scenario_file);
    g_simulation_context->app = new SimApp(
        dataPath.string(), DefaultRenderSettings(), DefaultHelperSettings(), g_simulation_context->sim);
    g_simulation_context->app->start(sf::Scalar(1) / kStepsPerSecond);
}

void auv_yield_until_next_frame(AuvFrame* frame)
{
    g_simulation_context->app->StepSimulation();
    g_simulation_context->app->pump();
    if(g_simulation_context->app->getState() == sf::SimulationState::FINISHED)
    {
        std::println(stderr, "[hydrus_sim] simulation finished; restart for another run");
        auv_deinit();
        return;
    }
    *frame = g_simulation_context->sim->getAuvFrame();
}

void auv_set_thrustor_values(const float* thrustor_values, uint8_t thrustor_values_len)
{
    if(g_simulation_context == nullptr)
        return;
    g_simulation_context->sim->setThrustorValues(thrustor_values, thrustor_values_len);
}

void auv_deinit(void)
{
    if(g_simulation_context == nullptr)
        return;

    if(g_simulation_context->app)
        g_simulation_context->app->shutdown();

    delete g_simulation_context;
    g_simulation_context = nullptr;
}
