///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Miles Sigel, Alex Prucka, Connor Kelley
//////////////////////////////////////

#include "RTP.h"
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"


ompl::geometric::RTP::RTP(const ompl::base::SpaceInformationPtr &si) : ompl::base::Planner(si, "RTP") {
	specs_.approximateSolutions = true;
	specs_.directed = true;
	Planner::declareParam<double>("range", this, &RTP::setRange, &RTP::getRange, "0.:1.:10000.");
	Planner::declareParam<double>("goal_bias", this, &RTP::setGoalBias, &RTP::getGoalBias, "0.:.05:1.");
}

ompl::geometric::RTP::~RTP() {
	freeMemory();
}


void ompl::geometric::RTP::clear() {
	Planner::clear();
	sampler_.reset();
	freeMemory();
	allMotions.clear(); /// Clear our vector that holds the motions of our tree
	lastGoalMotion_ = nullptr;
}

void ompl::geometric::RTP::freeMemory()
{
	allMotions.clear();
}

void ompl::geometric::RTP::setup()
{
	Planner::setup();
	tools::SelfConfig sc(si_, getName());
	sc.configurePlannerRange(maxDistance_);
}

ompl::base::PlannerStatus ompl::geometric::RTP::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
	checkValidity();
	ompl::base::Goal *goal = pdef_->getGoal().get(); // get goal state from problem def
	auto *goal_s = dynamic_cast<ompl::base::GoalSampleableRegion *>(goal); // dynamic cast to goal sampleable region

	while (const ompl::base::State *st = pis_.nextStart()) // initalize all possible start states
	{
		auto *motion = new Motion(si_); // initalize new motion pointer, passing the state information, si
		si_->copyState(motion->state, st); // copy the state into the _si
		allMotions.push_back(motion); // add motion to the queue
	}

	if (allMotions.size() == 0) // If our tree is empty, that means we couldn't initialize any start states
	{
		OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
		return ompl::base::PlannerStatus::INVALID_START;
	} // code adapted from RRT

	if (!sampler_)
    {
		sampler_ = si_->allocStateSampler();
    }

	OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), allMotions.size());
	// Initialize both approximate and exact solution pointers
	Motion *solution = nullptr; 
	Motion *approxsol = nullptr;

	double approxdif = std::numeric_limits<double>::infinity();
	auto *rmotion = new Motion(si_);
	ompl::base::State *rstate = rmotion->state; // temp variable that will hold random states

	while (!ptc) // while the problem not solved
	{
		/* sample random state (with goal biasing) */
		if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
			goal_s->sampleGoal(rstate); // With small probability, sample the goal region
		else
			sampler_->sampleUniform(rstate); // Otherwise, sample a state uniformly

		/* Choose a random state from the tree */
        // std:rand() generates random number between 0 and max_rand
		auto rIndex = std::rand() % allMotions.size();
		Motion *newMotion = allMotions.at(rIndex); // gets random motion point

		if (si_->checkMotion(newMotion->state, rstate)) // compare the two states
		{
            // adapted from RRT code 

            auto *motion = new Motion(si_);
            si_->copyState(motion->state, rstate); // copy the random state in the new motion object
            motion->parent = newMotion; // set the parent to the new motion
            allMotions.push_back(motion); // add the motion back to the tree

            newMotion = motion;

			double dist = 0.0;
			bool sat = goal->isSatisfied(newMotion->state, &dist); // the end condition check

            // copied from RRT planner 
			if (sat) 
			{
				approxdif = dist;
				solution = newMotion;
				break;
			}
			if (dist < approxdif)
			{
				approxdif = dist;
				approxsol = newMotion;
			}
		}
	}

	bool solved = false;
	bool approximate = false;
	if (solution == nullptr)
	{
		solution = approxsol;
		approximate = true;
	}

	if (solution != nullptr) // exact solution
	{
		lastGoalMotion_ = solution;

		/* construct the solution path */
		std::vector<Motion *> mpath; 
		while (solution != nullptr) // loop through the path to create the solution path
		{
			mpath.push_back(solution);
			solution = solution->parent; // Keep moving up the tree of motions until nullptr (root)
		}

		/* set the solution path */
		auto path(std::make_shared<PathGeometric>(si_));
		for (int i = mpath.size() - 1; i >= 0; --i)
			path->append(mpath[i]->state);
		pdef_->addSolutionPath(path, approximate, approxdif, getName());
		solved = true;
	}

	if (rmotion->state != nullptr)
		si_->freeState(rmotion->state);
	delete rmotion;

	OMPL_INFORM("%s: Created %u states", getName().c_str(), allMotions.size());

	return {solved, approximate};
}


void ompl::geometric::RTP::getPlannerData(ompl::base::PlannerData &data) const
{
	Planner::getPlannerData(data);

	if (lastGoalMotion_ != nullptr)
		data.addGoalVertex(ompl::base::PlannerDataVertex(lastGoalMotion_->state));

	for (auto &motion : allMotions)
	{
		if (motion->parent == nullptr)
			data.addStartVertex(ompl::base::PlannerDataVertex(motion->state));
		else
			data.addEdge(ompl::base::PlannerDataVertex(motion->parent->state), ompl::base::PlannerDataVertex(motion->state));
	}
}

