#include <cmath>
#include <cstdio>
#include <math.h>

#include "auv.h"

#include "actuators/Actuator.h"
#include "actuators/Thruster.h"
#include "core/ConsoleSimulationApp.h"
#include "core/GraphicalSimulationApp.h"
#include "core/Robot.h"
#include "core/ScenarioParser.h"
#include "core/SimulationManager.h"
#include "entities/Entity.h"
#include "entities/MovingEntity.h"
#include "entities/StaticEntity.h"
#include "graphics/OpenGLDataStructs.h"
#include "graphics/OpenGLPipeline.h"
#include "sensors/Sensor.h"
#include "sensors/ScalarSensor.h"


#include <cctype>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <print>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>


static std::filesystem::path dataPath = "auvs/hydrus_sim/data/";
static std::string scenario_file = "scenarios/hydrus_env.scn";
static constexpr sf::Scalar kStepsPerSecond = 360.0;
static constexpr sf::Scalar kPhysicsDt = sf::Scalar(1) / kStepsPerSecond;
static constexpr sf::Scalar kRealtimeFactorCap = 8.0;
static constexpr auto kMinRenderInterval = std::chrono::milliseconds(16);
static constexpr bool kConsoleApp = false;

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

    std::string cls{text};
    for(char& c : cls)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

    if(cls == "cube" || cls == "0")
        return ObjectCls::cube;
    if(cls == "rect" || cls == "rectangle" || cls == "1")
        return ObjectCls::rect;
    if(cls == "gate" || cls == "2")
        return ObjectCls::gate;
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
        const char* type = nullptr;
        element->QueryStringAttribute("type", &type);
        if(type != nullptr && (std::string(type) == "plane" || std::string(type) == "terrain"))
            return ScenarioParser::ParseStatic(element);

        const char* name = nullptr;
        element->QueryStringAttribute("name", &name);
        const char* display_name = name != nullptr ? name : "<unnamed>";

        const char* cls = nullptr;
        if(element->QueryStringAttribute("cls", &cls) != XML_SUCCESS)
            element->QueryStringAttribute("clss", &cls);

        if(cls == nullptr || *cls == '\0')
        {
            std::println(stderr, "[hydrus_sim] static object '{}' is missing a cls tag", display_name);
            return false;
        }

        const auto mapped = ParseObjectClass(cls);
        if(!mapped)
        {
            std::println(stderr, "[hydrus_sim] static object '{}' has unknown cls '{}'", display_name, cls);
            return false;
        }
        if(name == nullptr)
        {
            std::println(stderr, "[hydrus_sim] static object with cls '{}' is missing a name", cls);
            return false;
        }

        objectClasses[name] = *mapped;
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
        const auto objectClasses = std::move(parser.objectClasses);

        odometry = nullptr;
        have_odometry = false;
        for(unsigned int i = 0; sf::Sensor* sensor = getSensor(i); ++i)
        {
            if(sensor->getType() != sf::SensorType::LINK)
                continue;
            auto* scalar = static_cast<sf::ScalarSensor*>(sensor);
            if(scalar->getScalarSensorType() == sf::ScalarSensorType::ODOM)
            {
                odometry = scalar;
                have_odometry = odometry->getNumOfChannels() >= 10;
                break;
            }
        }

        tracked_objects.clear();
        for(unsigned int i = 0; sf::Entity* entity = getEntity(i); ++i)
        {
            if(tracked_objects.size() == AUV_FRAME_MAX_OBJECTS)
                break;
            auto it = objectClasses.find(entity->getName());
            if(it == objectClasses.end())
                continue;
            tracked_objects.push_back({entity, i, it->second});
        }

        thrusters.clear();
        sf::Robot* robot = getRobot(0u);
        if(robot == nullptr)
        {
            std::println(stdout, "[hydrus_sim] no robot in scenario; thrusters will not run");
            return;
        }
        for(size_t i = 0; sf::Actuator* actuator = robot->getActuator(i); ++i)
        {
            if(actuator->getType() != sf::ActuatorType::THRUSTER)
                continue;
            thrusters.push_back(static_cast<sf::Thruster*>(actuator));
        }
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

        if(have_odometry)
        {
            frame.camera_pose.pos.buf[0] = static_cast<float>(odometry->getLastValue(0));
            frame.camera_pose.pos.buf[1] = static_cast<float>(odometry->getLastValue(1));
            frame.camera_pose.pos.buf[2] = static_cast<float>(odometry->getLastValue(2));
            frame.camera_pose.quat.buf[0] = static_cast<float>(odometry->getLastValue(6));
            frame.camera_pose.quat.buf[1] = static_cast<float>(odometry->getLastValue(7));
            frame.camera_pose.quat.buf[2] = static_cast<float>(odometry->getLastValue(8));
            frame.camera_pose.quat.buf[3] = static_cast<float>(odometry->getLastValue(9));
        }

        for(const TrackedObject& tracked : tracked_objects)
        {
            AuvObject object{};
            if(!FillObjectBBox(tracked.entity, object.bbox))
                continue;
            object.id = tracked.id;
            object.cls = static_cast<AuvObjectCls>(tracked.cls);
            frame.objects[frame.objects_len++] = object;
        }

        return frame;
    }


    struct TrackedObject
    {
        sf::Entity* entity;
        uint32_t id;
        ObjectCls cls;
    };

    std::filesystem::path scenarioPath;
    bool have_odometry = false;
    sf::ScalarSensor* odometry = nullptr;
    std::vector<TrackedObject> tracked_objects;
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

    void updateGraphics()
    {
        if(getState() == sf::SimulationState::FINISHED)
            return;

        const auto now = std::chrono::steady_clock::now();
        if(lastRender.has_value() && now - *lastRender < kMinRenderInterval)
            return;
        else{
        lastRender = now;
        sf::OpenGLPipeline* pipeline = getGLPipeline();
        SDL_LockMutex(pipeline->getDrawingQueueMutex());
        pipeline->PurgeDrawingQueue();
        pipeline->PurgeSelectedDrawingQueue();
        getSimulationManager()->UpdateDrawingQueue();
        SDL_UnlockMutex(pipeline->getDrawingQueueMutex());
        LoopInternal();
        }
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
    std::optional<std::chrono::steady_clock::time_point> lastRender;
};

class ConsoleSimApp : public sf::ConsoleSimulationApp
{
public:
    ConsoleSimApp(std::string dataPath, sf::SimulationManager* sim_manager)
        : sf::ConsoleSimulationApp("hydrus_sim", dataPath, sim_manager)
    {
    }

    void start(sf::Scalar timeStep)
    {
        autostep_ = false;
        timeStep_ = timeStep;
        Init();
        StartSimulation();
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

struct RealtimeThrottle
{
    void wait(sf::Scalar sim_time)
    {
        if(kRealtimeFactorCap <= sf::Scalar(0))
            return;

        const auto wall = std::chrono::steady_clock::now();
        if(!base_set)
        {
            base_sim = sim_time;
            base_wall = wall;
            base_set = true;
            return;
        }

        const sf::Scalar sim_elapsed = sim_time - base_sim;
        const sf::Scalar wall_elapsed =
            std::chrono::duration<sf::Scalar>(wall - base_wall).count();
        const sf::Scalar wall_budget = sim_elapsed / kRealtimeFactorCap;

        if(wall_budget > wall_elapsed)
        {
            const sf::Scalar deficit = wall_budget - wall_elapsed;
            // Sub-ms sleeps lose more to the scheduler than they buy.
            if(deficit < sf::Scalar(0.001))
                return;
            std::this_thread::sleep_for(std::chrono::duration<sf::Scalar>(deficit));
        }
        else
        {
            base_sim = sim_time;
            base_wall = wall;
        }
    }

    bool base_set = false;
    sf::Scalar base_sim = 0;
    std::chrono::steady_clock::time_point base_wall{};
};

struct SimulationContext
{
    SimManager* sim = nullptr;
    SimApp* graphical = nullptr;
    ConsoleSimApp* console = nullptr;
    RealtimeThrottle realtime;
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
    if(kConsoleApp)
    {
        std::println(stdout, "[hydrus_sim] app=console");
        g_simulation_context->console = new ConsoleSimApp(dataPath.string(), g_simulation_context->sim);
        g_simulation_context->console->start(kPhysicsDt);
    }
    else
    {
        std::println(stdout, "[hydrus_sim] app=graphical");
        g_simulation_context->graphical = new SimApp(
            dataPath.string(), DefaultRenderSettings(), DefaultHelperSettings(), g_simulation_context->sim);
        g_simulation_context->graphical->start(kPhysicsDt);
    }
}

void auv_yield_until_next_frame(AuvFrame* frame)
{
    if(g_simulation_context == nullptr)
        return;

    g_simulation_context->sim->StepSimulation(kPhysicsDt);
    if(g_simulation_context->graphical != nullptr)
        g_simulation_context->graphical->updateGraphics();
    g_simulation_context->realtime.wait(g_simulation_context->sim->getSimulationTime());

    const bool finished =
        (g_simulation_context->graphical != nullptr
         && g_simulation_context->graphical->getState() == sf::SimulationState::FINISHED)
        || (g_simulation_context->console != nullptr
            && g_simulation_context->console->getState() == sf::SimulationState::FINISHED);
    if(finished)
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

    if(g_simulation_context->graphical != nullptr)
    {
        g_simulation_context->graphical->shutdown();
        delete g_simulation_context->graphical;
    }
    if(g_simulation_context->console != nullptr)
    {
        g_simulation_context->console->shutdown();
        delete g_simulation_context->console;
    }
    delete g_simulation_context->sim;

    delete g_simulation_context;
    g_simulation_context = nullptr;
}
