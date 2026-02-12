#include "include/QLearning.hpp"

QLearning::QLearning(size_t state_d, size_t action_n, double e, double a, double g, double d, std::vector<size_t> state_disc, std::vector<double> upper_limits, std::vector<double> lower_limits)
    : state_dimension(state_d),
      action_number(action_n),
      epsilon(e),
      alpha(a),
      gamma(g),
      decay(d),
      gen(std::random_device{}()),
      dis_eps(0.0f, 1.0f),
      dis_action(0, action_number - 1),
      state_discretization(state_disc),
      state_lower_limits(lower_limits),
      state_upper_limits(upper_limits)
{
    state_number=1;
    for (size_t i=0; i<state_disc.size();i++)
    {
        state_number*=state_disc[i];
    }
    Qtable.resize(state_number * action_number, 0.0f);

}


size_t QLearning::prox_value_iterator(double value, double upper, double lower, size_t disc)
{
    double delta = (upper - lower) / disc;
    int idx = static_cast<int>((value - lower) / delta);

    if (idx < 0) return 0;
    if (idx >= static_cast<int>(disc)) return disc - 1;
    return static_cast<size_t>(idx);

}

size_t QLearning::get_state_index(const std::vector<double>& state)
{
    size_t state_index = 0;
    size_t stride = 1;

    for (int i = static_cast<int>(state_dimension) - 1; i >= 0; --i)
    {
       size_t bit=prox_value_iterator
       (
            state[i], 
            state_upper_limits[i], 
            state_lower_limits[i], 
            state_discretization[i]
        );
        state_index+= bit*stride;
        stride*=state_discretization[i];

    }
    return state_index;
}

size_t QLearning::max_action_index_Q(size_t s)
{
    const double *row = &Qtable[s*action_number];
    double maxv = row[0];
    size_t max_iterator=0;
    for (size_t i=1; i<action_number; i++ )
    {
        if(row[i]>maxv) 
        {
            maxv = row[i];
            max_iterator=i;
        }
    }
    return max_iterator;
}
void QLearning::update(size_t s, size_t a, double R, size_t s1) noexcept
{
    double& qsa = Q(s,a);
    qsa+= alpha * (R+gamma*Q(s1,max_action_index_Q(s1)) -qsa);
}