#include <cmath>
#include <vector>

using namespace std;

class Handler
{
private:
    
    int state;
    int D_pos;
    int D_vel;
    double U;
    double L;
    double action_value;
    vector<double> positions; 
    vector<double> velocities; 

public:

    Handler(int disc_pos,int disc_vel, double upper, double lower):D_pos(disc_pos),D_vel(disc_vel), U(upper), L(lower), action_value(0.0) 
    {

        double range = upper-lower;
        double step = range/disc_pos;

        for (int i =0; i<disc_pos; i++)
        {
            positions.push_back(lower+(step*i));
        }

        range = 20;
        step = range/disc_vel;

        for (int i =0; i<disc_vel; i++)
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
            int state = (prox_position*D_vel + prox_velocity);
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