#include <iostream>

#include<csignal>
#include<atomic>
#include<vector>
#include<fstream>
#include<random>
#include <yaml-cpp/yaml.h>

#include "include/QLearning.hpp"
#include "include/Environment.hpp"
#include "include/Viewer.hpp"

using namespace std;

atomic<bool> running(true);
double random_angle() 
{
    static std::random_device rd;  
    static std::mt19937 gen(rd()); 
    static std::uniform_real_distribution<double> dist(-M_PI, M_PI);

    return dist(gen);
}
inline void handler(int) {running =false;}

vector<double> loadVector(const string& filename) 
{
    ifstream in(filename, ios::binary);
    if(!in.is_open()) {
        return vector<double>();
    }
    size_t size;
    in.read(reinterpret_cast<char*>(&size), sizeof(size));
    
    vector<double> v(size);
    in.read(reinterpret_cast<char*>(v.data()), size * sizeof(double));
    return v;
}

void saveVector(const vector<double>& v, const string& filename) 
{
    ofstream out(filename, ios::binary);
    size_t size = v.size();
    out.write(reinterpret_cast<const char*>(&size), sizeof(size));
    out.write(reinterpret_cast<const char*>(v.data()), size * sizeof(double));
}

int main(int argc, char* argv[]) 
{
    struct sigaction sa;
    sa.sa_handler = handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags =0;
    sigaction(SIGINT, &sa, nullptr);
    
    YAML::Node config = YAML::LoadFile("resources/RL_parameters.yaml");
    if(!config["parameters"]) 
    {
        cerr << "Error en archivo YAML\n";
        return -1;
    }
    auto param = config["parameters"];
    auto p = param[0];
    double e = p["epsilon"].as<double>();
    const double a = p["learning_rate"].as<double>();
    const double gamma = p["gamma"].as<double>();
    const double d = p["discount"].as<double>();

    size_t disc_position =360;
    size_t disc_velocity = 10;
    vector<size_t> disc = {disc_position, disc_velocity};
    size_t state_dim = disc.size(), actions = 3;

    int succes = 0, steps = 0, episodes = 0;
    double average_reward = 0, episode_reward = 0, total_reward = 0, goal_angle, torque=0, stability=0, max_torque = 5.0;
    vector<double> u_lim ={M_PI, 10};
    vector<double> l_lim ={-M_PI, -10};
    vector<double> state;

    const char * trainment_path = (char*)"trainings/trainment";
    const char * model_path = (char*)"resources/SimpleActuator.xml";
    bool view = false;
    std::unique_ptr<Viewer> viewer;

    if(argc > 1)
    {
        if(argc > 2) model_path = (char*)argv[2];
        string arg1 = argv[1];
        if(arg1 == "true" || arg1 == "True" || arg1 == "yes" || arg1 == "y" || arg1 == "1") {
            view = true;
        }
    }

    long unsigned int present_state = 0, new_state = 0, action_taken = 0;
    double reward_given = 0;
    double motor_angle, motor_velocity; 
    bool goal_flag = false, step_flag = false, restart = false;

    QLearning agent(state_dim, actions, e, a, gamma, d, disc, u_lim, l_lim);
    vector<double> loaded_qtable = loadVector(trainment_path);
    cout<<"Cargando tabla..."<<endl;
    if(!loaded_qtable.empty() && loaded_qtable.size() == agent.get_table().size()) 
    {
        agent.set_table(loaded_qtable);  
        cout << "Q-table cargada exitosamente (" << loaded_qtable.size() << " valores)" << endl;
    } else if (!loaded_qtable.empty()) 
    {
        cout << "Q-table inexistente o de tamaño incorrecto. Esperado: " 
             << agent.get_table().size() << ", obtenido: " << loaded_qtable.size() << endl;
    }

     Environment env(model_path);
    
    if (view) 
    {
        viewer = std::make_unique<Viewer>(env.get_model(), env.get_data());
        cout << "Modo visualización ACTIVADO" << endl;
    }
    goal_angle = random_angle();
    cout << "\nNew goal:" << goal_angle << " ===" << endl;
    env.reset();
    env.sync();

    motor_angle = env.read_joint_position("hinge");
    motor_velocity = env.read_joint_velocity("hinge");
    state = {motor_angle, motor_velocity};
    present_state = agent.get_state_index(state);

    while(running)
    {
        if(restart)
        {
            episodes++;
            total_reward+=episode_reward;
            average_reward = total_reward / max(1, episodes);
            stability=0;
            goal_angle = random_angle();
            cout << "\nNew goal:" << goal_angle << " ===" << endl;
            if (episodes % 10 == 0) 
            {
                cout << "\n=== EPISODIO " << episodes << " ===" << endl;
                cout << "Recompensa episodio: " << episode_reward << endl;
                cout << "Recompensa promedio: " << average_reward << endl;
                cout << "Épsilon actual: " << agent.get_epsilon() << endl;
                cout << "Pasos: " << steps << endl;
                cout << "Éxitos totales: " << succes << endl;
                cout << "Tasa de éxito: " << (100.0 * succes / max(1, episodes)) << "%" << endl;
                
                
                if (episodes % 50 == 0) 
                {
                    saveVector(agent.get_table(), trainment_path);  
                    cout << "Q-table guardada como backup" << endl;
                }
            
                
            }

            env.reset();
            env.sync();
            motor_angle = env.read_joint_position("hinge"); //nombre declarado en el xml
            motor_velocity = env.read_joint_velocity("hinge"); //nombre declarado en el xml
            state = {motor_angle, motor_velocity};
            present_state = agent.get_state_index(state);

            steps = 0;
            episode_reward = 0;
            goal_flag = false;
            step_flag = false;
            
            agent.decay_e();
            
            restart = false;
        }
         // Selección de acción ε-greedy
        action_taken = agent.e_greedy(present_state);
        torque = (action_taken - 1) * max_torque;
        if(torque > max_torque) torque = max_torque;
        if(torque < -max_torque) torque = -max_torque;
        //aplicar fuerza
        env.step1();
        env.write_actuator_force("torque_motor", torque); //nombre del actuador declarado en el xml

        steps++;

        //avanzar simulación
        env.step2();
        if(view && (steps%10==0)) 
        {
            viewer->render();
            if(viewer->should_close()) 
            {
                running = false;
                break;
            }
        }
        // ver la recompenza
        motor_angle= env.read_joint_position("hinge");
        double velocity = env.read_joint_velocity("hinge");
        torque = env.read_actuator_force("torque_motor");
        double pos_error =motor_angle - goal_angle;
        reward_given = -abs(pos_error) - 0.1*abs(velocity) - 0.01*abs(torque);
        ((abs(pos_error)*100/goal_angle)<2)?stability +=1:stability = 0;
        episode_reward+= reward_given;

        // Obtener nuevo estado después del step
        state = {motor_angle, velocity};
        new_state = agent.get_state_index(state);

        // actualizar Q
        agent.update(present_state, action_taken, reward_given, new_state);

        // mover estado
        present_state = new_state;


        // Verificar condiciones de terminación
        step_flag = (steps > 5000);
        goal_flag = (stability>5);
        restart = (goal_flag || step_flag);
        if(goal_flag && restart) succes++;

    }   

     // Estadísticas finales
    cout << "\n=== ENTRENAMIENTO FINALIZADO ===" << endl;
    cout << "Episodios totales: " << episodes << endl;
    cout << "Éxitos: " << succes << endl;
    cout << "Tasa de éxito: " << (100.0 * succes / max(1, episodes)) << "%" << endl;
    cout << "Recompensa promedio: " << average_reward << endl;
    cout << "Épsilon final: " << agent.get_epsilon() << endl;
    
    // Guardar Q-table final
    saveVector(agent.get_table(), trainment_path);
    cout << "Q-table final guardada como 'trainment' en la carpeta trainings" << endl;
    
    return 0;
}