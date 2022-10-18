///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Miles Sigel, Alex Prucka
//////////////////////////////////////

// majority of this code taken from RRT.h in the ompl github dir

#ifndef RANDOM_TREE_H
#define RANDOM_TREE_H

#include "ompl/geometric/planners/PlannerIncludes.h"

namespace ompl
{
	namespace geometric
	{

		class RTP : public base::Planner
		{
		public:
			RTP(const base::SpaceInformationPtr &si);
			~RTP() override;
			void getPlannerData(base::PlannerData &data) const override;
            void clear() override;
			base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
			
            /**
             * sets the goal bias for the system. Want this to be not too big or too small.
             **/
			void setGoalBias(double goalBias)
			{
				goalBias_ = goalBias;
			}

            /**
             * Returns the goal bias
             **/
			double getGoalBias() const
			{
				return goalBias_;
			}

            /**
             * Sets the range for the planner
             **/
			void setRange(double distance)
			{
				maxDistance_ = distance;
			}

            /**
             * returns the range
             **/
			double getRange() const
			{
				return maxDistance_;
			}
			
            /**
             * abstract override
             **/
			void setup() override;



		protected:
			
			class Motion // A motion contains a current state and its parent motion in the tree, basically an edge in the graph
			{
			public:
				Motion() = default;

				/** \brief Constructor that allocates memory for the state */
				Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
				{
				}

				~Motion() = default;

				/** \brief The state contained by the motion */
				base::State *state{nullptr};

				/** \brief The parent motion in the exploration tree */
				Motion *parent{nullptr};
			};

			/** \brief Free the memory allocated by this planner */
			void freeMemory();

			/** \brief Compute distance between motions (actually distance between contained states) */
			double distanceFunction(const Motion *a, const Motion *b) const
			{
				return si_->distance(a->state, b->state);
			}

			/** \brief State sampler */
			base::StateSamplerPtr sampler_;

			/** \brief tree of motions */
			std::vector<Motion *> motionTree; // Holds pointers to motions 

			/** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
			 * available) */
			double goalBias_{.05};

			/** \brief The maximum length of a motion to be added to a tree */
			double maxDistance_{0.};

			/** \brief The random number generator */
			RNG rng_;

			/** \brief The most recent goal motion.  Used for PlannerData computation */
			Motion *lastGoalMotion_{nullptr};

		};
	} 
} 

#endif

