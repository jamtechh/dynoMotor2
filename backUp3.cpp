// ===============================================================================================================================================================================
// ======== HEADERS ==============================================================================================================================================
// ===============================================================================================================================================================================
///   t his is changeeddddd in test 1111111
// ======== Multi-purposes headers ==============================================================================================================================================
#if defined(_DEBUG)
#undef _DEBUG
#include <Python.h>
#define _DEBUG
#else

#endif

#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <unordered_set>
#include <tuple>
#include <filesystem>
#include <exception>
#define _USE_MATH_DEFINES 
#include <cmath>

#include <future>
#include <sstream>

#include <nlohmann/json.hpp>
#include <thread>
#include <fstream>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/geometry/ChTriangleMeshSoup.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChRandom.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_irrlicht/ChIrrMeshTools.h"
#include <chrono/physics/ChSystem.h>
// ======== ChElectronics headers ==============================================================================================================================================
#include "chrono_powerelectronics/ChElectronicsCosimulation.h"
#include "chrono_powerelectronics/circuits/ChElectronicMotor.h"
#include "chrono_powerelectronics/circuits/ChElectronicCircuit.h"
#include "chrono_powerelectronics/circuits/ChElectronicGeneric.h"

// ===============================================================================================================================================================================
// ======== NAMESPACES ==============================================================================================================================================
// ===============================================================================================================================================================================
using namespace chrono;
using namespace chrono::irrlicht;
using namespace ::chrono::powerelectronics;
using json = nlohmann::json;

using namespace irr; // Use the main namespaces of Irrlicht
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

// ======== Class: allows to compute the integral in-between a single simulation time-step through the cumulative trapezoidal method ==============================================================================================================================================
class CumTrapezIntegration {
public:
    double Integrate(double& dt, double& f_new)
    {
        f_new1 = f_new;
        dt1 = dt;
        Integral_res += dt1 * ((f_old1 + f_new1) / 2);
        //std::cout << "\n!!!!! f_new1: " << f_new1 << " !!!!!\n";            // DEBUG: Scope some needed results
        //std::cout << "\n!!!!! f_old1: " << f_old1 << " !!!!!\n";            // DEBUG: Scope some needed results
        f_old1 = f_new1;
        return Integral_res;
    }
private:
    double Integral_res = 0.0;
    double f_old1 = 0.0;
    double f_new1;
    double dt1;
};

// ======== Method: calculate the effective Euler angular position of a body from the angular velocity along x-y-z- axis ==============================================================================================================================================
std::vector<double> GetEulerAngPos(std::shared_ptr<chrono::ChBody> body, double& t_step_mechanic)
{
    // Get the effective angular velocity along x-y-z axis
    ChVector3d body_Euler_Vel = body->GetAngVelLocal(); // Get the angular velocity 
    double Rotor_Euler_dt_Yaw = body_Euler_Vel[0];
    double Rotor_Euler_dt_Pitch = body_Euler_Vel[1];
    double Rotor_Euler_dt_Roll = body_Euler_Vel[2];

    // Create the object only once through a static variable (the static variable allows to initialize it only once during the execution of the entire code)
    static CumTrapezIntegration body_Euler_Yaw_Integrator;
    static CumTrapezIntegration body_Euler_Pitch_Integrator;
    static CumTrapezIntegration body_Euler_Roll_Integrator;

    // Compute the effective angular position along x-y-z axis
    double body_Euler_Yaw = body_Euler_Yaw_Integrator.Integrate(t_step_mechanic, body_Euler_Vel[0]);
    double body_Euler_Pitch = body_Euler_Pitch_Integrator.Integrate(t_step_mechanic, body_Euler_Vel[1]);
    double body_Euler_Roll = body_Euler_Roll_Integrator.Integrate(t_step_mechanic, body_Euler_Vel[2]);

    // Populate the result vector
    std::vector<double> Results = { body_Euler_Yaw , body_Euler_Pitch, body_Euler_Roll };

    return Results;
}

// ======== Method: converts all the characters in the input string to lowercase and returns the resulting string ==============================================================================================================================================
std::string toLowerCase(const std::string& str) {
    std::string lower_str = str;
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(),
                [](unsigned char c){ return std::tolower(c); });
    return lower_str;
}

enum class JointType {
    FIXED,
    REVOLUTE,
    PRISMATIC
};
ChSystemNSC GravetySetup(){
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    ChSystemNSC sys; // Create a Chrono physical system
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    
    ChVector3d gravity_acc = sys.GetGravitationalAcceleration(); 
    std::cout << "The gravity acceleration  vector is: " << gravity_acc << "\n\n";
    double gravity = 9.81e3; //[mm/s^2]
    sys.SetGravitationalAcceleration(ChVector3d(gravity, 0, 0));
    ChVector3d gravity_acc_new = sys.GetGravitationalAcceleration(); 
    std::cout << "The new gravity acceleration  vector is: " << gravity_acc_new << "\n\n";
    return sys;
}           
void AddVisualizationBall(ChSystemNSC& sys, const ChVector3d& position, const ChColor& color = ChColor(1,0,0), int rad = 8){
    // set a ball on the center of mass of the frame for visualization purposes        
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.4f);
    mat->SetCompliance(0.0);
    mat->SetComplianceT(0.0);
    mat->SetDampingF(0.2f);
    auto mrigidBall = chrono_types::make_shared<ChBodyEasySphere>(rad, 10, true, true, mat);
    mrigidBall->SetPos(position);
    mrigidBall->SetPosDt(ChVector3d(0, 0, 0));  // set initial speed
    auto sphere_shape = chrono_types::make_shared<ChVisualShapeSphere>(rad);
    sphere_shape->SetColor(color);
    mrigidBall->AddVisualShape(sphere_shape);
    mrigidBall->SetFixed(true);
    sys.Add(mrigidBall);
}
void AddAxis(ChSystemNSC& sys, const ChVector3d& position, float x = 2, float y = 2, float z = 2, const ChColor& color = ChColor(1.0f, 0.0f, 0.0f)){
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto x_axis = chrono_types::make_shared<ChBodyEasyBox>(
        x, y, z, 1000, true, false, mat);
    x_axis->SetPos(ChVector3d(position[0]-x/2, position[1]-y/2, position[2]-z/2));  // Position along X-axis
    x_axis->SetFixed(true);
    x_axis->GetVisualShape(0)->SetColor(color); // Red color
    sys.Add(x_axis);
}
void AddCylAxis(ChSystemNSC& sys, const ChVector3d& position, ChQuaternion<> jointOrientation, float rad = 0.5, float len = 100, const ChColor& color = ChColor(0.0f, 0.0f, 1.0f)){
    auto axisShape = chrono_types::make_shared<ChVisualShapeCylinder>(rad, len); // Radius = 2, Length = 50
    axisShape->SetColor(color);
    auto axisBody = chrono_types::make_shared<ChBody>();
    axisBody->SetPos(position);
    axisBody->SetFixed(true); // The axis is just for visualization
    axisBody->AddVisualShape(axisShape, ChFrame<>(ChVector3d(0, 0, 0), jointOrientation));
    sys.Add(axisBody);
}
void CreateJoint(ChSystemNSC& sys, std::shared_ptr<ChBody> bodyA, std::shared_ptr<ChBody> bodyB, JointType jointType, const ChVector3d& orinn, bool showAxis = false) {
    ChVector3d jointPosition(bodyA->GetPos());
    ChQuaternion<> jointOrientation;
    jointOrientation.SetFromAngleAxis(90.0 * (CH_PI / 180.0), orinn);    
    ChFrame<> jointFrame(jointPosition, jointOrientation);

    std::shared_ptr<ChLinkLock> Joint;
    switch (jointType) {
        case JointType::FIXED:      {Joint = chrono_types::make_shared<ChLinkLockLock>();      break;}
        case JointType::REVOLUTE:   {Joint = chrono_types::make_shared<ChLinkLockRevolute>();  break;}
        case JointType::PRISMATIC:  {Joint = chrono_types::make_shared<ChLinkLockPrismatic>(); break;}
        default:    throw std::invalid_argument("Invalid joint type.");
    }

    Joint->Initialize(bodyA, bodyB, jointFrame);  
    sys.AddLink(Joint);
    
    if(showAxis)AddCylAxis(sys, bodyA->GetPos(), jointOrientation);
}
ChFrame<> GetFramee(std::shared_ptr<ChBody> body, const ChVector3d& orinn = ChVector3d(1, 0, 0)){
    ChVector3d jointPos(body->GetPos());
    ChQuaternion<> jointOr;
    jointOr.SetFromAngleAxis(90.0 * (CH_PI / 180.0), orinn);    
    ChFrame<> jointFrame(jointPos, jointOr);
    return jointFrame;
}

class RigidBody {
    public:
        RigidBody(ChSystemNSC& sys, const std::string& file_name, double density, bool is_fixed = false)
            : system(sys), obj_file(file_name), density(density), is_fixed(is_fixed) {SetupRigidBody();}
    
        std::shared_ptr<ChBody> GetBody() const {return body;}
        ChVector3d GetCOG() const {return cog;}
        ChVector3d getPos() const {return body->GetPos();}
        std::tuple<std::shared_ptr<ChBody>, ChVector3d> GetBodyAndCOG() const {return std::make_tuple(body, cog);}
        void HideBody() {mesh->SetVisible(false);}
        void ShowCG() {AddVisualizationBall(system, body->GetPos());}
        void setPos(ChVector3d poss){body->SetPos(poss);}
        void setColor(ChColor colll){mesh->SetColor(colll);}
    
    private:
        ChSystemNSC& system;
        std::string obj_file;
        double density;
        bool is_fixed;
        std::shared_ptr<ChBody> body;
        std::shared_ptr<ChVisualShapeTriangleMesh> mesh;
        std::shared_ptr<ChVisualShapeTriangleMesh> coll_mesh;
        std::shared_ptr<ChCollisionModel> coll_model;
        ChVector3d cog;
    
        void SetupRigidBody() {
            // Load visualization mesh
            obj_file = std::string("my_project/CAD/View/") + obj_file + std::string(".obj");
            auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(obj_file));
    
            std::cout<<"\t\t\t1"<<std::endl;
            // Load collision mesh
            std::cout<<"\t\t"<<obj_file<<std::endl;
            std::string coll_file = obj_file;
            if (coll_file.find("body") == std::string::npos){
                std::cout<<"\t\t\treplacing!!!!"<<std::endl;
                coll_file.replace(coll_file.find("View"), 4, "Collision");
                coll_file.replace(coll_file.find("_OBJ"), 4, "_Collision_OBJ");
            }            
            auto coll_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(coll_file));
            // std::cout<<"\t\t\t2"<<std::endl;
    
            // Compute mass properties
            double volume;
            ChMatrix33<> geometric_inertia;
            trimesh->ComputeMassProperties(true, volume, cog, geometric_inertia);
            // std::cout<<"\t\t\t3"<<std::endl;
    
            // Calculate mass and inertia
            double mass = density * volume;
            ChMatrix33<> inertia = density * geometric_inertia;
    
            // Create rigid body
            body = chrono_types::make_shared<ChBody>();
            body->SetFixed(is_fixed);
            body->SetMass(mass);
            body->SetInertiaXX(ChVector3d(inertia(0, 0), inertia(1, 1), inertia(2, 2)));
            // body->SetPos(cog);
            
            // std::cout<<"\t\t\t4"<<std::endl;
            system.Add(body);
    
            // std::cout<<"\t\t\t5"<<std::endl;
            // Visualization
            mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            mesh->SetMesh(trimesh);
            // mesh->SetOpacity(0.5f);
            // mesh->SetBackfaceCull(true);
            // mesh->SetColor(colorr);
            body->AddVisualShape(mesh, ChFrame<>(ChVector3d(0,0,0), ChMatrix33<>(1)));
    
            // Collision
            coll_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            coll_mesh->SetMesh(coll_trimesh);
            coll_mesh->SetVisible(false);
            body->AddVisualShape(coll_mesh, ChFrame<>(ChVector3d(0,0,0), ChMatrix33<>(1)));
    
            // Setup collision model
            coll_model = chrono_types::make_shared<ChCollisionModel>();
            coll_model->SetSafeMargin(0.1f);
            coll_model->SetEnvelope(0.001f);
            coll_trimesh->Transform(-cog, ChMatrix33<>(1));
            auto coll_mat = chrono_types::make_shared<ChContactMaterialNSC>();
            coll_mat->SetFriction(0.30);
            coll_mat->SetRestitution(0.001);
            auto coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(coll_mat, coll_trimesh, false, false, 0.001);
            coll_model->AddShape(coll_shape, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
            body->AddCollisionModel(coll_model);
            body->EnableCollision(false);

            body->GetCollisionModel()->SetFamily(1);
            body->GetCollisionModel()->DisallowCollisionsWith(2);
        }
    };

// ===============================================================================================================================================================================
// ======== MAIN LOOP ==============================================================================================================================================
// ===============================================================================================================================================================================

int main(int argc, char* argv[]) {
    ChSystemNSC sys = GravetySetup();
    float len = 50, thickk = 2;
    AddAxis(sys, ChVector3b(0,0,0),len,thickk,thickk, ChColor(1,0,0));
    AddAxis(sys, ChVector3b(0,0,0),thickk,len,thickk, ChColor(0,1,0));
    AddAxis(sys, ChVector3b(0,0,0),thickk,thickk,len, ChColor(0,0,1));
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    
    vis_mat->SetKdTexture(GetChronoDataFile("textures/pinkwhite.png"));
    std::vector<std::string> file_names = {"" , // empty value to start with index 1
        "../dynoObj/body_1_1",     // Part1_Motor
        "../dynoObj/body_2_1",     // GearA_Driver
        "../dynoObj/body_3_1",     // GearF
        "../dynoObj/body_4_1",     // frame
        "../dynoObj/body_5_1",     // GearB
        "../dynoObj/body_6_1",     // GearC
        "../dynoObj/body_7_1",     // GearD
        "../dynoObj/body_8_1",     // GearE
        "../dynoObj/body_9_1",     // Part2_flywheel
        "../dynoObj/body_10_1"     // Part2_dyno
    };
    std::vector<std::unique_ptr<RigidBody>> bodies(file_names.size());
    std::vector<std::shared_ptr<ChBody>> body_ptrs(file_names.size());
    std::vector<ChVector3d> cogs(file_names.size());
    std::vector<ChVector3d> positions = {ChVector3d(0,0,0),
        ChVector3d(-153.681408502864,232.071341649174,257.256405065421),
        ChVector3d(-153.681408502864,316.571341649174,257.256405065421),
        ChVector3d(-153.681408502864,-62.0286583508263,257.256405065421),
        ChVector3d(10.0501781487224,127.271341649174,219.573424974887),
        ChVector3d(-183.793091163963,320.571341649174,254.366929798653),
        ChVector3d(-213.749821851278,320.571341649174,280.873424974888),
        ChVector3d(-213.749821851277,-66.0286583508263,280.873424974888),
        ChVector3d(-183.793091163963,-66.0286583508263,254.366929798653),
        ChVector3d(-213.749821851277,127.271341649174,280.873424974888),
        ChVector3d(-153.681408502864,15.4713416491736,257.256405065421)
    };
    std::vector<ChQuaternion<>> rotss = { ChQuaternion<>(0.0,0.0,0.0,0.0),
        ChQuaternion<>(0.344860417986101,-0.344860417986101,0.617309721376921,0.617309721376921),
        ChQuaternion<>(0.69219181681608,-0.69219181681608,0.144466220040721,0.144466220040721),
        ChQuaternion<>(0.682163121105785,0.682163121105785,0.186154441803611,-0.186154441803611),
        ChQuaternion<>(1,0,0,0),
        ChQuaternion<>(0.590890270601115,0.590890270601115,0.388392440849382,-0.388392440849382),
        ChQuaternion<>(-0.0817797748423181,-0.0817797748423181,0.702361778876627,-0.702361778876627),
        ChQuaternion<>(0.707106781186548,-0.707106781186547,-8.1335083852247e-17,0),
        ChQuaternion<>(-0.227257494306092,0.227257494306092,0.669592436696918,0.669592436696918),
        ChQuaternion<>(0.661716586109785,0.661716586109785,-0.249261227765594,0.249261227765594),
        ChQuaternion<>(0.679959230544171,0.679959230544171,0.194050108986773,-0.194050108986773)
    };
    // Initialize RigidBody objects and store values (starting from index 1)
    bodies[1] = std::make_unique<RigidBody>(sys, file_names[1], 7850.00 / (1e9), true);
    bodies[2] = std::make_unique<RigidBody>(sys, file_names[2], 7850.00 / (1e9));bodies[2]->HideBody();
    bodies[3] = std::make_unique<RigidBody>(sys, file_names[3], 7850.00 / (1e9), true);
    bodies[4] = std::make_unique<RigidBody>(sys, file_names[4], 7850.00 / (1e9), true);
    bodies[5] = std::make_unique<RigidBody>(sys, file_names[5], 7850.00 / (1e9), true);bodies[5]->setColor(ChColor(0,1,0));
    bodies[6] = std::make_unique<RigidBody>(sys, file_names[6], 7850.00 / (1e9), true);
    bodies[7] = std::make_unique<RigidBody>(sys, file_names[7], 7850.00 / (1e9), true);
    bodies[8] = std::make_unique<RigidBody>(sys, file_names[8], 7850.00 / (1e9), true);
    bodies[9] = std::make_unique<RigidBody>(sys, file_names[9], 7850.00 / (1e9));
    bodies[10] = std::make_unique<RigidBody>(sys, file_names[10], 7850.00 / (1e9), true);
    for (size_t i = 1; i < file_names.size(); ++i) {
        body_ptrs[i] = bodies[i]->GetBody();
        cogs[i] = bodies[i]->GetCOG();
        body_ptrs[i]->SetPos(positions[i]-positions[4]);
        body_ptrs[i]->SetRot(rotss[i]);
        if(i==1)bodies[1]->setColor(ChColor(1,0,0));
    }
    auto Stator_body = body_ptrs[1];
    auto Rotor_body = body_ptrs[2];
    auto Frame_body = body_ptrs[4];
    // Create all the rigid bodies.
    double radA = 10, radB = 20;
    auto mbody_gearA = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, radA, 0.5, 1000, true, false, mat);
    mbody_gearA->SetPos(positions[2]-positions[4]);
    mbody_gearA->SetRot(QuatFromAngleX(CH_PI_2));
    mbody_gearA->GetVisualShape(0)->SetMaterial(0, vis_mat);
    sys.Add(mbody_gearA);

    double interaxis12 = radA + radB;
    auto mbody_gearB = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, radB, 0.4, 1000, true, false, mat);
    mbody_gearB->SetPos(positions[5]-positions[4]);
    mbody_gearB->SetRot(QuatFromAngleX(CH_PI_2));
    mbody_gearB->GetVisualShape(0)->SetMaterial(0, vis_mat);
    sys.Add(mbody_gearB);

    auto mbody_gearC = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, radB, 0.4, 1000, true, false, mat);
    mbody_gearC->SetPos(positions[6]-positions[4]);
    mbody_gearC->SetRot(QuatFromAngleX(CH_PI_2));
    mbody_gearC->GetVisualShape(0)->SetMaterial(0, vis_mat);
    sys.Add(mbody_gearC);


    // ...impose rotation speed between the first gear and the fixed truss
    ChQuaternion<> jointOrientation;
    jointOrientation.SetFromAngleAxis(90.0 * (CH_PI / 180.0), ChVector3d(1, 0, 0));
    // AddCylAxis(sys, ChVector3d(0, 0, 0), jointOrientation, 0.5f, 50.0f, ChColor(1,0,0));

    auto link_motorA = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    link_motorA->Initialize(mbody_gearA, Frame_body, ChFrame<>(positions[2]-positions[4], jointOrientation));
    link_motorA->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(20));
    sys.AddLink(link_motorA);
    auto link_revoluteB = chrono_types::make_shared<ChLinkLockRevolute>();
    link_revoluteB->Initialize(mbody_gearB, Frame_body, ChFrame<>(positions[5]-positions[4], jointOrientation));
    sys.AddLink(link_revoluteB);
    auto link_revoluteC = chrono_types::make_shared<ChLinkLockRevolute>();
    link_revoluteC->Initialize(mbody_gearC, Frame_body, ChFrame<>(positions[6]-positions[4], jointOrientation));
    sys.AddLink(link_revoluteC);
    auto link_revolute_Flywheel = chrono_types::make_shared<ChLinkLockRevolute>();
    link_revolute_Flywheel->Initialize(body_ptrs[9], Frame_body, ChFrame<>(positions[9]-positions[4], jointOrientation));
    sys.AddLink(link_revolute_Flywheel);
    auto link_lock_Flywheel = chrono_types::make_shared<ChLinkLockLock>();
    link_lock_Flywheel->Initialize(body_ptrs[9], mbody_gearC, ChFrame<>(positions[9]-positions[4], jointOrientation));
    sys.AddLink(link_lock_Flywheel);

    auto link_gearAB = chrono_types::make_shared<ChLinkLockGear>();
    link_gearAB->Initialize(mbody_gearA, mbody_gearB, ChFrame<>());
    link_gearAB->SetFrameShaft1(ChFrame<>(VNULL, chrono::QuatFromAngleZ(CH_PI_2)));  // Shaft 1 at gear A
    link_gearAB->SetFrameShaft2(ChFrame<>(VNULL, chrono::QuatFromAngleZ(CH_PI_2)));  // Shaft 2 at gear B
    link_gearAB->SetTransmissionRatio(radA / radB);
    // link_gearAB->SetEnforcePhase(true);
    sys.AddLink(link_gearAB);
    auto link_gearBC = chrono_types::make_shared<ChLinkLockGear>();
    link_gearBC->Initialize(mbody_gearB, mbody_gearC, ChFrame<>());
    link_gearBC->SetFrameShaft1(ChFrame<>(VNULL, chrono::QuatFromAngleZ(CH_PI_2)));  // Shaft 1 at gear A
    link_gearBC->SetFrameShaft2(ChFrame<>(VNULL, chrono::QuatFromAngleZ(CH_PI_2)));  // Shaft 2 at gear B
    link_gearBC->SetTransmissionRatio(1);
    // link_gearBC->SetEnforcePhase(true);
    sys.AddLink(link_gearBC);

    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(1200, 800);
    vis->SetWindowTitle("Modeling a simplified trackjed vehicle");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(-300, 400, 0));
    // vis->AddCamera(ChVector3d(-10, 10, -20));
    vis->AddLight(ChVector3d(300.f, 300.f, -300.f), 3000, ChColor(0.1f, 0.1f, 0.1f));
    vis->AddLight(ChVector3d(300.f, 300.f, 300.f), 3000, ChColor(0.1f, 0.1f, 0.1f));
    vis->EnableBodyFrameDrawing(true);
    vis->EnableLinkFrameDrawing(true);

    // ===========================================================================================================================================================================================
    // ======== SOLVER SETTINGS ====================================================================================================================================================================
    // ===========================================================================================================================================================================================
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);
    //sys.SetTimestepperType(ChTimestepper::Type::RUNGEKUTTA45);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(1000.0);
    sys.SetMaxPenetrationRecoverySpeed(1000.1);
    sys.SetMinBounceSpeed(0.001);
    ChRealtimeStepTimer realtime_timer;

    // ===========================================================================================================================================================================================
    // ======== SET THE MULTI-PHYSICS SYMULATION PARAMETERS ====================================================================================================================================
    // ===========================================================================================================================================================================================
    // ======== Mechanical domain ====================================================================================================================================================================
    double f_ToSample_mechanic = 1.0e3;//1.0e5;//8.0e3;// 0.5e4; // [Hz]
    double t_step_mechanic = 1 / f_ToSample_mechanic; // [s]
    // ======== Electronic domain ====================================================================================================================================================================
    double f_ToSample_electronic = 1.0e3;//1.0e5;// 0.5e4; // [Hz]                              Frequency at which the electronic domain is called respect to the global time line
    double T_ToSample_electronic = 1 / f_ToSample_electronic;               // Period at which the electronic domain is called respect to the global time line
    double T_sampling_electronic = t_step_mechanic;                         // Time window of the electronic (SPICE) simulation
    double t_step_electronic = 1.0e-5;//1.0e-6; // [s]                                  Discretization of the electronic time window

    std::string Netlist_location = "../data/my_project/SPICE/Circuit_Netlist.cir";   
    
    ChElectronicGeneric Generic_Circuit(Netlist_location, t_step_electronic); 
    Generic_Circuit.Initialize(t_step_mechanic);

    std::map<std::string, double> PWLIn = {
        {"VmotorVAR", 0.0},
        {"VpwmVAR", 0.0}
    };
    std::map<std::string, double> FlowIn = {
        {"Rmotor", 0.5},
        {"Lmotor", 12.0 * 1.0e-6}
    };

    std::map<std::string, std::vector<double>> OutputMap;
    OutputMap["n1"] = {};
    OutputMap["n3"] = {};
    OutputMap["VmotorVAR"] = {};
    OutputMap["t_electronics"] = {};
    OutputMap["alpha"] = {};
    OutputMap["dalpha"] = {};
    OutputMap["t_mechanics"] = {};
    OutputMap["T_magnetic"] = {};
    OutputMap["T_motor"] = {};

    Generic_Circuit.InputDefinition(PWLIn, FlowIn);

    double t_simulation_STOP = 10.0;//400.0e-3; //[s]
    double t_sim_mechanics = 0.0; //[s] 
    double t_sim_electronics = 0.0; //[s]
    double t_sampling_electronic_counter = 0; //[s] This variable is needed to count the event at which the Electronic domain need to be called respect to the Global Time-line
    int brake_flag = 1; // Set a brake flag in the case you want to stop the simulation before: t_simulation_STOP
    double Imotor = 0.0;
    
    std::cout << "\n";
    std::cout << "===================================================" << "\n";
    std::cout << "======= AsASfsadfsdfsadfsadfsadfsadfsadfasdfsadfasdfasdfsadfsadf =======" << "\n"; 
    std::cout << "===================================================" << "\n";
    std::cout << "\n";
    
    double T_PWM = 0.04; //[s] PWM Period
    double Duty_PWM = 100.0 / 100; //[s] PWM Duty
    double t_PWM_counter = 0.0; //[s] PWM Period

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // tools::drawCircle(vis.get(), 20, ChCoordsys<>(link_gearAB->GetMarker2()->GetAbsCoordsys().pos, QUNIT), ChColor(1,0,0));
        // tools::drawCircle(vis.get(), 0.1, ChCoordsys<>(link_gearBC->GetMarker2()->GetAbsCoordsys().pos, QUNIT));

        sys.DoStepDynamics(t_step_mechanic);
        realtime_timer.Spin(t_step_mechanic);
    }

    return 0;
}
