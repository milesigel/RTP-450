#include <iostream>
#include <fstream>
#include <cmath>
#include <functional>
#include <memory>

// The collision checker produced in project 2
#include "CollisionChecking.h"

// Your random tree planner
#include "RTP.h"

// Including SimpleSetup will pull in MOST of what you need to plan
#include <ompl/geometric/SimpleSetup.h>

// Except for the state space definitions...
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>


// Use placeholder namespace for arguments to bound functions.
using namespace std::placeholders;


void planPoint(const std::vector<Rectangle> &obstacles )
{
	
	auto r2 = std::make_shared<ompl::base::RealVectorStateSpace>(2);
	ompl::base::RealVectorBounds bounds(2);
	bounds.setLow(-3); 
	bounds.setHigh(3); 
	r2->setBounds(bounds);

	ompl::geometric::SimpleSetup ss(r2);
	ss.setStateValidityChecker(std::bind(isValidStatePoint, _1, obstacles));

	ompl::base::ScopedState<> start(r2);
	start[0] = -2;
	start[1] = -2;

	ompl::base::ScopedState<> goal(r2);
	goal[0] = 2;
	goal[1] = 2;

	ss.setStartAndGoalStates(start, goal);

	auto planner = std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation());
	ss.setPlanner(planner);

	ompl::base::PlannerStatus solved = ss.solve(30);

	if (solved)
	{
		// print to terminal
		std::cout << "Solution found:" << std::endl;
		ompl::geometric::PathGeometric &path = ss.getSolutionPath();

		path.interpolate(50);
		path.printAsMatrix(std::cout);

		// print to file
		std::ofstream fout("pathpoint.txt");
		fout << "R2" << std::endl;

		path.printAsMatrix(fout);
		fout.close();
	}
	else
		std::cout << "No solution found" << std::endl;

}

void planBox(const std::vector<Rectangle> &  obstacles )
{


	ompl::base::StateSpacePtr se2;
	auto r2 = std::make_shared<ompl::base::RealVectorStateSpace>(2);

	ompl::base::RealVectorBounds bounds(2);
	bounds.setLow(-3);
	bounds.setHigh(3);

	r2->setBounds(bounds);

	auto so2 = std::make_shared<ompl::base::SO2StateSpace>();

	se2 = r2 + so2;

	ompl::geometric::SimpleSetup ss(se2);

	ss.setStateValidityChecker(std::bind(isValidStateSquare, _1, 0.5, obstacles));

	ompl::base::ScopedState<> start(se2);
	start[0] = -2;
	start[1] = -2;
	start[2] = 0.4;

	ompl::base::ScopedState<> goal(se2);
	goal[0] = 2;
	goal[1] = 2;
	goal[2] = 0.8;

	
	ss.setStartAndGoalStates(start, goal);

	auto planner = std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation());
	ss.setPlanner(planner);

	
	ompl::base::PlannerStatus solved = ss.solve(10.0);

	if (solved)
	{
		// print to terminal
		std::cout << "Solution found:" << std::endl;
		ompl::geometric::PathGeometric &path = ss.getSolutionPath();

		path.interpolate(50);
		path.printAsMatrix(std::cout);

		// print to file
		std::ofstream fout("pathbox.txt");
		fout << "SE2" << std::endl;

		path.printAsMatrix(fout);
		fout.close();
	}
	else
		std::cout << "No solution found" << std::endl;
}

void makeEnvironment1(std::vector<Rectangle> &obstacles )
{
	Rectangle ob1;
	ob1.x = -1.5;
	ob1.y = -3;
	ob1.width = 0.2;
	ob1.height = 3.6;
	obstacles.push_back(ob1);

	Rectangle ob2;
	ob2.x = 1.5;
	ob2.y = -2.5;
	ob2.width = 3.6;
	ob2.height = 0.2;
	obstacles.push_back(ob2);

}

void makeEnvironment2(std::vector<Rectangle> &obstacles )
{
	Rectangle ob1;
	ob1.x = -3;
	ob1.y = -1.5;
	ob1.width = 3;
	ob1.height = 0.1;
	obstacles.push_back(ob1);

	Rectangle ob2;
	ob2.x = 0;
	ob2.y = -2;
	ob2.width = 0.1;
	ob2.height = 1;
	obstacles.push_back(ob2);

	Rectangle ob3;
	ob3.x = 0;
	ob3.y = 1.5;
	ob3.width = 0.1;
	ob3.height = 3;
	obstacles.push_back(ob3);

	Rectangle ob4;
	ob4.x = 0;
	ob4.y = 1;
	ob4.width = 0.1;
	ob4.height = 1;
	obstacles.push_back(ob4);
}

int main(int /* argc */, char ** /* argv */)
{
	int robot, choice;
	std::vector<Rectangle> obstacles;

	do
	{
		std::cout << "Plan for: " << std::endl;
		std::cout << " (1) A point in 2D" << std::endl;
		std::cout << " (2) A rigid box in 2D" << std::endl;

		std::cin >> robot;
	} while (robot < 1 || robot > 2);

	do
	{
		std::cout << "In Environment: " << std::endl;
		std::cout << " (1) Two rectangles" << std::endl;
		std::cout << " (2) Four rectangles" << std::endl;

		std::cin >> choice;
	} while (choice < 1 || choice > 2);

	switch (choice)
	{
		case 1:
			std::cout<<"num obstacles before return "<<obstacles.size()<<std::endl;
			makeEnvironment1(obstacles);
			std::cout<<"num obstacles after return "<<obstacles.size()<<std::endl;
			break;
		case 2:
			makeEnvironment2(obstacles);
			break;
		default:
			std::cerr << "environment num error" << std::endl;
			break;
	}

	switch (robot)
	{
		case 1:
			std::cout<<"num obstacles before plan "<<obstacles.size()<<std::endl;
			planPoint(obstacles);
			break;
		case 2:
			planBox(obstacles);
			break;
		default:
			std::cerr << "Invalid Robot Type!" << std::endl;
			break;
	}

	return 0;
}