#include <iostream>

#include<csignal>
#include<atomic>
#include<vector>
#include<fstream>
#include<random>
#include <yaml-cpp/yaml.h>

#include "include/Qlearning.h"
#include "include/Environment.h"
#include "include/Viewer.h"
#include "include/Handler.h"

using namespace std;

atomic<bool> running(true);

inline void handler(int) {running =false;}

vector<float> loadVector(const string& filename) 
{
    ifstream in(filename, ios::binary);
    if(!in.is_open()) {
        return vector<float>();
    }
    size_t size;
    in.read(reinterpret_cast<char*>(&size), sizeof(size));
    
    vector<float> v(size);
    in.read(reinterpret_cast<char*>(v.data()), size * sizeof(float));
    return v;
}

void saveVector(const vector<float>& v, const string& filename) 
{
    ofstream out(filename, ios::binary);
    size_t size = v.size();
    out.write(reinterpret_cast<const char*>(&size), sizeof(size));
    out.write(reinterpret_cast<const char*>(v.data()), size * sizeof(float));
}

int main(int argc, char* argv[]) 
{
    struct sigaction sa;
    sa.sa_handler = handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags =0;
    sigaction(SIGINT, &sa, nullptr);
    
    YAML::Node config = YAML::LoadFile("resources/RL_parameters.yaml");
    auto param = config["parameters"];
    auto p = param[0];
    double e = p["epsilon"].as<double>();
    const double a = p["learning_rate"].as<double>();
    const double gamma = p["gamma"].as<double>();
    const double d = p["discount"].as<double>();



    int disc_position =360;
    int disc_velocity = 10;
    const long unsigned int states = disc_position*disc_velocity, actions = 3;

    int succes = 0, steps = 0, episodes = 0;
    double average_reward = 0, episode_reward = 0, total_reward = 0;

    const char * trainment_path = (char*)"trainings/trainment";
    const char * model_path = (char*)"resources/SimpleActuator.xml";
    bool view = false;
    Viewer * viewer = nullptr;

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
    double random_angle, motor_angle, motor_velocity; 
    bool first = true, goal_flag = false, step_flag = false, restart = false;

    QLearning table(states, actions, e, a, gamma, d);
    vector<float> loaded_qtable = loadVector(trainment_path);
    cout<<"Cargando tabla..."<<endl;
    if(!loaded_qtable.empty() && loaded_qtable.size() == states * actions) 
    {
        table.set_table(loaded_qtable);  
        cout << "Q-table cargada exitosamente (" << loaded_qtable.size() << " valores)" << endl;
    } else if (!loaded_qtable.empty()) 
    {
        cout << "Q-table inexistente o de tamaño incorrecto. Esperado: " 
             << states * actions << ", obtenido: " << loaded_qtable.size() << endl;
    }

     Environment env(model_path);
    
    if (view) 
    {
        viewer = new Viewer(env.get_model(), env.get_data());
        cout << "Modo visualización ACTIVADO" << endl;
    }
    
    env.forward();

    Handler agent(disc_position,disc_velocity,-M_PI, M_PI);

    while(running)
    {
        if (restart)
        {
            episodes++;
            total_reward += episode_reward;
            average_reward = total_reward / episodes;
            random_angle = agent.random_pos(); 
            
            
            if (episodes % 10 == 0) 
            {
                cout << "\n=== EPISODIO " << episodes << " ===" << endl;
                cout << "Recompensa episodio: " << episode_reward << endl;
                cout << "Recompensa promedio: " << average_reward << endl;
                cout << "Épsilon actual: " << table.get_epsilon() << endl;
                cout << "Pasos: " << steps << endl;
                cout << "Éxitos totales: " << succes << endl;
                cout << "Tasa de éxito: " << (100.0 * succes / max(1, episodes)) << "%" << endl;
                
                
                if (episodes % 50 == 0) 
                {
                    saveVector(table.get_table(), trainment_path);  
                    cout << "Q-table guardada como backup" << endl;
                }
            }
            
            if (goal_flag) 
            {
                succes++;
                cout << "¡ÉXITO! en el episodio " << episodes << endl;
            }
            
            env.reset();
            env.forward();
            
            first = true;
            steps = 0;
            episode_reward = 0;
            goal_flag = false;
            step_flag = false;
            
            table.decay_e();
            
            restart = false;
        }
        
        // Leer estado actual
        if(!first) 
        {
            motor_angle = env.read_joint_position("hinge"); //nombre declarado en el xml
            motor_velocity = env.read_joint_velocity("hinge"); //nombre declarado en el xml
            new_state = agent.get_state_index(motor_angle, motor_velocity);
            table.update(present_state, action_taken, reward_given, new_state);
            present_state = new_state;
        }
        else 
        {
            present_state = agent.get_state_index(motor_angle, motor_velocity);
            first = false;
        }

        // Selección de acción ε-greedy
        action_taken = table.e_greedy(present_state);

        //aplicar fuerza
        double force =agent.action(action_taken);
        env.write_joint_force("torque_motor", force); //nombre del actuador declarado en el xml

        steps++;

        //avanzar simulación

        env.simstep();
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
        double error =motor_angle - random_angle;
        reward_given = agent.reward(error);

        episode_reward+= reward_given;

        // Verificar condiciones de terminación
        step_flag = (steps > 500);
        goal_flag = (abs(error)<0.2 && step_flag);
        restart = (goal_flag || step_flag);

    }

    // Estadísticas finales
    cout << "\n=== ENTRENAMIENTO FINALIZADO ===" << endl;
    cout << "Episodios totales: " << episodes << endl;
    cout << "Éxitos: " << succes << endl;
    cout << "Tasa de éxito: " << (100.0 * succes / max(1, episodes)) << "%" << endl;
    cout << "Recompensa promedio: " << average_reward << endl;
    cout << "Épsilon final: " << table.get_epsilon() << endl;
    
    // Guardar Q-table final
    saveVector(table.get_table(), trainment_path);
    cout << "Q-table final guardada como 'trainment' en la carpeta trainings" << endl;
    
    // Limpieza
    if(view) delete viewer;
    
    return 0;
}
