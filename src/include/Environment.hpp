#include <unordered_map>
#include <string>
#include <mujoco/mujoco.h>

class Environment
{
    private:
        mjModel* m = nullptr;
        mjData* d = nullptr;


        std::unordered_map<std::string, int> actuator_index;
        std::unordered_map<std::string, int> joint_index;

    public:

        Environment(const char* model_path);
        ~Environment(){if(d)mj_deleteData(d); if(m)mj_deleteModel(m);}
        void reset(){ mj_resetData(m, d); mj_forward(m, d);}
        void sync() { mj_forward(m, d);}
        void step() { mj_step(m, d);}
        void step1(){ mj_step1(m, d);}
        void step2(){ mj_step2(m, d);}
        void write_joint_position(const char* jname, double val) const; 
        void write_joint_velocity(const char* jname, double val); 
        void write_actuator_force(const char* jname, double val);
        double read_joint_position(const char* jname) const; 
        double read_joint_velocity(const char* jname); 
        double read_actuator_force(const char* jname); 
        double time_now() const {return d->time;}
        mjModel* get_model() const { return m; }
        mjData* get_data() const { return d; } 

};