#include <cmath>
#include <cstdio>
#include <math.h>
#include <cstdlib>
#include "../../include/auv.h"
#include <exception>
#include <stdexcept>
static AuvFrameError initialization_error{};
static void sensor_init();
static void sensor_read(AuvFrame *frame);
static void sensor_close();

#include "Stonefish/core/ConsoleSimulationApp.h"
#include "Stonefish/core/GraphicalSimulationApp.h"
#include "Stonefish/core/Robot.h"
#include "Stonefish/core/ScenarioParser.h"
#include "Stonefish/core/SimulationManager.h"
#include "Stonefish/entities/Entity.h"
#include "Stonefish/entities/MovingEntity.h"
#include "Stonefish/entities/StaticEntity.h"
#include "Stonefish/graphics/OpenGLDataStructs.h"
#include "Stonefish/graphics/OpenGLPipeline.h"
#include "Stonefish/sensors/Sensor.h"
#include "Stonefish/sensors/ScalarSensor.h"
#include "Stonefish/sensors/vision/Camera.h"


#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

static constexpr auto kFrameTimeout = std::chrono::seconds(1);
static constexpr auto kFrameRetryInterval = std::chrono::milliseconds(1);

static std::filesystem::path dataPath = "auvs/stonefish_sim/data/";
static std::string scenario_file = "scenarios/hydrus_env.scn";
static constexpr sf::Scalar kStepsPerSecond = 360.0;
static constexpr sf::Scalar kPhysicsDt = sf::Scalar(1) / kStepsPerSecond;
static constexpr sf::Scalar kRealtimeFactorCap = 1.0;
static constexpr sf::Scalar kTrackingOkSeconds = 1000000000.0;
static constexpr auto kMinRenderInterval = std::chrono::milliseconds(16);
static constexpr bool kConsoleApp = false;
static sf::Scalar realtimeFactorCap = kRealtimeFactorCap;


namespace
{
// Scenario text for AuvRobosubCls. Local to the simulator.
struct RobosubTag
{
    const char* name;
    AuvRobosubCls cls;
};

const RobosubTag kRobosubTags[] = {
    {"gate", AUV_ROBOSUB_GATE},
    {"marker", AUV_ROBOSUB_MARKER},
    {"gate_opening", AUV_ROBOSUB_GATE_OPENING},
};
static_assert(std::size(kRobosubTags) == static_cast<size_t>(AUV_ROBOSUB_COUNT),
              "stonefish robosub tags must match AuvRobosubCls");

std::optional<AuvObjectCls> ParseObjectClass(const char* text)
{
    if(text == nullptr || *text == '\0')
        return std::nullopt;

    std::string cls{text};
    for(char& c : cls)
    {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if(c == ' ' || c == '-')
            c = '_';
    }

    for(const RobosubTag& tag : kRobosubTags)
        if(cls == tag.name)
            return static_cast<AuvObjectCls>(tag.cls);

    char* end = nullptr;
    const unsigned long value = std::strtoul(cls.c_str(), &end, 10);
    if(end != cls.c_str() && *end == '\0' && value < static_cast<unsigned long>(AUV_ROBOSUB_COUNT))
        return static_cast<AuvObjectCls>(value);
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

// Stonefish camera frame: +X right, +Y down, +Z along the optical axis.
bool ProjectObjectBox(
    sf::Camera* camera,
    unsigned int image_width,
    unsigned int image_height,
    float focal_px,
    sf::Entity* entity,
    AuvObject2d& box)
{
    if(camera == nullptr || image_width == 0 || image_height == 0 || focal_px <= 0.f)
        return false;

    sf::Vector3 min;
    sf::Vector3 max;
    entity->getAABB(min, max);
    const sf::Transform world_to_camera = camera->getSensorFrame().inverse();
    const float cx = static_cast<float>(image_width) * 0.5f;
    const float cy = static_cast<float>(image_height) * 0.5f;

    float u_min = 1.0e9f;
    float v_min = 1.0e9f;
    float u_max = -1.0e9f;
    float v_max = -1.0e9f;
    bool any_in_front = false;

    for(int corner = 0; corner < 8; ++corner)
    {
        const sf::Vector3 world(
            (corner & 1) != 0 ? max.x() : min.x(),
            (corner & 2) != 0 ? max.y() : min.y(),
            (corner & 4) != 0 ? max.z() : min.z());
        const sf::Vector3 local = world_to_camera * world;
        if(local.z() <= sf::Scalar(0.05))
            continue;

        const float u = cx + focal_px * static_cast<float>(local.x() / local.z());
        const float v = cy + focal_px * static_cast<float>(local.y() / local.z());
        u_min = std::min(u_min, u);
        v_min = std::min(v_min, v);
        u_max = std::max(u_max, u);
        v_max = std::max(v_max, v);
        any_in_front = true;
    }

    if(!any_in_front)
        return false;
    if(u_max < 0.f || v_max < 0.f
       || u_min >= static_cast<float>(image_width)
       || v_min >= static_cast<float>(image_height))
        return false;

    const auto clamp_pixel = [](float value, unsigned int limit) -> uint32_t {
        if(value <= 0.f)
            return 0;
        const float last = static_cast<float>(limit - 1);
        if(value >= last)
            return limit - 1;
        return static_cast<uint32_t>(value);
    };

    box.top_left.x = clamp_pixel(u_min, image_width);
    box.top_left.y = clamp_pixel(v_min, image_height);
    box.bottom_right.x = clamp_pixel(u_max, image_width);
    box.bottom_right.y = clamp_pixel(v_max, image_height);
    return box.bottom_right.x > box.top_left.x && box.bottom_right.y > box.top_left.y;
}

class AuvScenarioParser : public sf::ScenarioParser
{
public:
    using ScenarioParser::ScenarioParser;
    std::unordered_map<std::string, AuvObjectCls> objectClasses;

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
            std::cerr << "[stonefish_sim] static object '" << display_name << "' is missing a cls tag\n";
            return false;
        }

        const auto mapped = ParseObjectClass(cls);
        if(!mapped)
        {
            std::cerr << "[stonefish_sim] static object '" << display_name << "' has unknown cls '" << cls << "'\n";
            return false;
        }
        if(name == nullptr)
        {
            std::cerr << "[stonefish_sim] static object with cls '" << cls << "' is missing a name\n";
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
            std::cerr << "Failed to parse scenario " << scenarioPath.string() << '\n';
            return;
        }
        const auto objectClasses = std::move(parser.objectClasses);

        odometry = nullptr;
        have_odometry = false;
        camera = nullptr;
        image_width = 0;
        image_height = 0;
        focal_px = 0.f;
        for(unsigned int i = 0; sf::Sensor* sensor = getSensor(i); ++i)
        {
            if(sensor->getType() == sf::SensorType::VISION)
            {
                if(auto* color_camera = dynamic_cast<sf::Camera*>(sensor))
                {
                    camera = color_camera;
                    camera->getResolution(image_width, image_height);
                    const float half_width = static_cast<float>(image_width) * 0.5f;
                    const float half_hfov_rad =
                        static_cast<float>(camera->getHorizontalFOV()) * (static_cast<float>(M_PI) / 360.f);
                    if(half_width > 0.f && half_hfov_rad > 0.f)
                        focal_px = half_width / std::tan(half_hfov_rad);
                }
                continue;
            }

            if(sensor->getType() != sf::SensorType::LINK)
                continue;
            auto* scalar = static_cast<sf::ScalarSensor*>(sensor);
            if(scalar->getScalarSensorType() == sf::ScalarSensorType::ODOM)
            {
                odometry = scalar;
                have_odometry = odometry->getNumOfChannels() >= 10;
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
        for(const TrackedObject& tracked : tracked_objects)
        {
            if(tracked.cls != AUV_ROBOSUB_GATE_OPENING)
                continue;
            if(tracked.entity->getType() != sf::EntityType::STATIC)
                continue;
            auto* gate_opening = static_cast<sf::StaticEntity*>(tracked.entity);
            if(gate_opening->getRigidBody() == nullptr)
                continue;
            btRigidBody* body = gate_opening->getRigidBody();
            body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
        }

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

            if(frame.objects2d_len >= AUV_FRAME_MAX_OBJECTS)
                continue;
            AuvObject2d box{};
            if(!ProjectObjectBox(camera, image_width, image_height, focal_px, tracked.entity, box))
                continue;
            box.id = tracked.id;
            box.cls = static_cast<AuvObjectCls>(tracked.cls);
            frame.objects2d[frame.objects2d_len++] = box;
        }

        frame.image_width = image_width;
        frame.image_height = image_height;
        frame.position_ok = {true, true, true};
        frame.attitude_ok = {true, true, true};
        if(getSimulationTime() >= kTrackingOkSeconds)
        {
            frame.position_ok.x = false;
            frame.position_ok.y = false;
            if(!logged_tracking_loss)
            {
                logged_tracking_loss = true;
                std::cout << "[stonefish_sim] horizontal position invalid after " << kTrackingOkSeconds << " s\n";
            }
        }
        if(have_odometry)
        {
            // Scenario is NED, so odometry Z is depth, positive down.
            frame.pressure_depth = static_cast<float>(odometry->getLastValue(2));
            frame.pressure_depth_ok = true;
        }

        return frame;
    }


    struct TrackedObject
    {
        sf::Entity* entity;
        uint32_t id;
        AuvObjectCls cls;
    };

    std::filesystem::path scenarioPath;
    bool have_odometry = false;
    sf::ScalarSensor* odometry = nullptr;
    sf::Camera* camera = nullptr;
    unsigned int image_width = 0;
    unsigned int image_height = 0;
    float focal_px = 0.f;
    bool logged_tracking_loss = false;
    std::vector<TrackedObject> tracked_objects;
};

class SimApp : public sf::GraphicalSimulationApp
{
public:
    SimApp(std::string dataPath, sf::RenderSettings render_setting, sf::HelperSettings helper_settings, sf::SimulationManager* sim_manager)
        : sf::GraphicalSimulationApp("stonefish_sim", dataPath, render_setting, helper_settings, sim_manager)
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
        : sf::ConsoleSimulationApp("stonefish_sim", dataPath, sim_manager)
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

static void sensor_init(void)
{
    sensor_close();

    g_simulation_context = new SimulationContext();
    g_simulation_context->sim = new SimManager(kStepsPerSecond, dataPath / scenario_file);
    if(kConsoleApp)
    {
        std::cout << "[stonefish_sim] app=console\n";
        g_simulation_context->console = new ConsoleSimApp(dataPath.string(), g_simulation_context->sim);
        g_simulation_context->console->start(kPhysicsDt);
    }
    else
    {
        std::cout << "[stonefish_sim] app=graphical\n";
        g_simulation_context->graphical = new SimApp(
            dataPath.string(), DefaultRenderSettings(), DefaultHelperSettings(), g_simulation_context->sim);
        g_simulation_context->graphical->start(kPhysicsDt);
    }
}

static void sensor_read(AuvFrame* frame)
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
        std::cerr << "[stonefish_sim] simulation finished; restart for another run\n";
        sensor_close();
        throw std::runtime_error("simulation finished");
    }
    *frame = g_simulation_context->sim->getAuvFrame();
}





static void sensor_close(void)
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

// SDK calls run directly on the mission thread. Control is polled between calls
// by the host. Frame deadlines are measured here; SDK calls can still block.
void auv_init(void) {
  initialization_error = {};
  try {
    sensor_init();
  } catch (const std::exception &e) {
    initialization_error.code = AUV_FRAME_ERROR_INITIALIZATION;
    std::snprintf(initialization_error.message, sizeof(initialization_error.message), "%s", e.what());
  } catch (...) {
    initialization_error.code = AUV_FRAME_ERROR_INITIALIZATION;
    std::snprintf(initialization_error.message, sizeof(initialization_error.message), "unexpected sensor initialization exception");
  }
}
void auv_yield_until_next_frame(AuvFrame *frame) {
  *frame = {};
  if (initialization_error.code != AUV_FRAME_ERROR_NONE) {
    frame->frame_error = initialization_error;
    return;
  }
  try {
    const auto started = std::chrono::steady_clock::now();
    while (true) {
      if (std::chrono::steady_clock::now() - started >= kFrameTimeout) break;
      sensor_read(frame);
      if (frame->frame_error.code != AUV_FRAME_ERROR_NONE) return;
      if (std::chrono::steady_clock::now() - started >= kFrameTimeout) break;
      if (frame->timestamp) return;
      std::this_thread::sleep_for(kFrameRetryInterval);
    }
    frame->frame_error.code = AUV_FRAME_ERROR_TIMEOUT;
    std::snprintf(frame->frame_error.message, sizeof(frame->frame_error.message),
                  "Stonefish frame timed out after %lld ms",
                  static_cast<long long>(std::chrono::duration_cast<std::chrono::milliseconds>(kFrameTimeout).count()));
  } catch (const std::exception &e) {
    frame->frame_error.code = AUV_FRAME_ERROR_SENSOR;
    std::snprintf(frame->frame_error.message, sizeof(frame->frame_error.message), "%s", e.what());
  } catch (...) {
    frame->frame_error.code = AUV_FRAME_ERROR_SENSOR;
    std::snprintf(frame->frame_error.message, sizeof(frame->frame_error.message), "unexpected sensor frame exception");
  }
}
void auv_deinit(void) {
  try { sensor_close(); }
  catch (const std::exception &e) { std::fprintf(stderr, "sensor cleanup: %s\n", e.what()); }
  catch (...) { std::fputs("unexpected sensor cleanup exception\n", stderr); }
  initialization_error = {};
}
