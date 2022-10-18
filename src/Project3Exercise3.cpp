///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/tools/benchmark/Benchmark.h>


#include "omplapp/apps/SE3RigidBodyPlanning.h"
#include "omplapp/config.h"

// Your random tree planner
#include "RTP.h"
using namespace ompl;

void benchmarkApartment()
{
    app::SE3RigidBodyPlanning setup;
	std::string benchmark_name;
	double runtime_limit, memory_limit;
	int run_count;
    benchmark_name = std::string("apartment");

	std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_robot.dae";
	std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_env.dae";
	setup.setRobotMesh(robot_fname);
	setup.setEnvironmentMesh(env_fname);

	base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
	start->setX(241.81);
	start->setY(106.15);
	start->setZ(70.57);
	start->rotation().setAxisAngle(0., 0., -1., 3.12413936107);

	base::ScopedState<base::SE3StateSpace> goal(start);
	goal->setX(-31.19);
	goal->setY(-99.85);
	goal->setZ(36.46);
	goal->rotation().setAxisAngle(0., 0., -1., 3.12413936107);


    base::RealVectorBounds bounds(3);
    bounds.setHigh(0, 295.77);
    bounds.setHigh(1, 168.26);
    bounds.setHigh(2, 90.39);
    bounds.setLow(0, -73.76);
    bounds.setLow(1, -179.59);
    bounds.setLow(2, -0.03);
    setup.getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);

	setup.setStartAndGoalStates(start, goal);
	setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
	setup.setup();



	runtime_limit = 10.0;
	memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
	run_count = 50;


	tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
	tools::Benchmark b(setup, benchmark_name);


	b.addPlanner(std::make_shared<geometric::RRT>(setup.getSpaceInformation()));
	b.addPlanner(std::make_shared<geometric::EST>(setup.getSpaceInformation()));
	b.addPlanner(std::make_shared<geometric::PRM>(setup.getSpaceInformation()));
	b.addPlanner(std::make_shared<geometric::RTP>(setup.getSpaceInformation()));
	b.benchmark(request);
	b.saveResultsToFile();
    // TODO
}

void benchmarkHome()
{
    // TODO
    app::SE3RigidBodyPlanning setup;
	std::string benchmark_name;
	double runtime_limit, memory_limit;
	int run_count;
    benchmark_name = std::string("Home");

    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Home_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Home_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(252.95);
    start->setY(-214.95);
    start->setZ(46.19);
    start->rotation().setAxisAngle(0., 0., -1., 3.12413936107);

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(-320.00);
    goal->setY(75.05);
    goal->setZ(46.19);
    goal->rotation().setAxisAngle(0., 0., -1., 3.12413936107);

    base::RealVectorBounds bounds(3);
    bounds.setHigh(0, 325.00);
    bounds.setHigh(1, 337.89);
    bounds.setHigh(2, 146.19);
    bounds.setLow(0, -383.80);
    bounds.setLow(1, -371.47);
    bounds.setLow(2, -0.20);
    setup.getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    runtime_limit = 10.0;
    memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    run_count = 50;



    tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
	tools::Benchmark b(setup, benchmark_name);


	b.addPlanner(std::make_shared<geometric::RRT>(setup.getSpaceInformation()));
	b.addPlanner(std::make_shared<geometric::EST>(setup.getSpaceInformation()));
	b.addPlanner(std::make_shared<geometric::PRM>(setup.getSpaceInformation()));
	b.addPlanner(std::make_shared<geometric::RTP>(setup.getSpaceInformation()));
	b.benchmark(request);
	b.saveResultsToFile();
}

int main(int /* argc */, char ** /* argv */)
{
    int environment;

    do
    {
        std::cout << "Benchmark for: " << std::endl;
        std::cout << " (1) Apartment" << std::endl;
        std::cout << " (2) Home" << std::endl;

        std::cin >> environment;
    } while (environment < 1 || environment > 2);

    switch (environment)
    {
        case 1:
            benchmarkApartment();
            break;
        case 2:
            benchmarkHome();
            break;
        default:
            std::cerr << "Invalid Environment!" << std::endl;
            break;
    }

    return 0;
}
