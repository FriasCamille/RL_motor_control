#include <cmath>
#include <vector>

using namespace std;

class Handler
{
private:
    
    int state;
    int D;
    double U;
    double L;
    double action_value;
    vector<double> positions; 
    vector<double> velocities; 

public:

    Handler(int disc, double upper, double lower):D(disc), U(upper), L(lower), action_value(0.0) 
    {

        double range = upper-lower;
        double step = range/disc;

        for (int i =0; i<disc; i++)
        {
            positions.push_back(lower+(step*i));
        }

        range = 20;
        step = range/disc;

        for (int i =0; i<disc; i++)
        {
            velocities.push_back(lower+(step*i));
        }

    }
    
    double reward(double error)
    {
        double reward =0;
        reward--;
        reward+=cos(error)*100;
        return reward;
    }
    
   inline int get_state_index(double position, double velocity)
        {
            double diff = abs(position- L);
            int prox_position=0;

            for (long unsigned int i=0; i<positions.size(); i++)
            {
                if(diff> abs(position-positions[i]))
                {
                    diff = abs(position-positions[i]);
                    prox_position = i;
                }
            }

            diff = abs(velocity- 10);
            int prox_velocity=0;

            for (long unsigned int i=0; i<velocities.size(); i++)
            {
                if(diff> abs(velocity-velocities[i]))
                {
                    diff = abs(velocity-velocities[i]);
                    prox_velocity = i;
                }
            }
            int state = (prox_position*pow(D,2)) + (prox_velocity*pow(10,1));
            return state;
        }

    
    double action(int n)
    {
        action_value+=(n-1)*10;
        return action_value;
    }



    inline double random_pos() 
    {
        static random_device rd;      
        static mt19937 gen(rd());     
        uniform_int_distribution<> dist(0, positions.size());
        return positions[dist(gen)];
    }
    
};