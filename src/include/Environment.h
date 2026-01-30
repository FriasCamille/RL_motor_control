#include <iostream>
#include <mujoco/mujoco.h>

#define ERROR ("\x1b[1;31m")

using namespace std;

class Environment
{
    private:

        const char* model;
        char errstr[1024]="";
        mjModel* m;
        mjData* d;

    public:
        Environment(const char* model_path)
            : model(model_path), m(mj_loadXML(model_path, NULL, errstr, sizeof(errstr))) 
        {
            if (!m) 
            {
                cerr << ERROR << "No se pudo cargar el modelo\n" << errstr << endl;
                exit(1);
            }
            d = mj_makeData(m);
        }

        ~Environment() 
        {
            mj_deleteData(d);
            mj_deleteModel(m);
        }

        void simstep() { mj_step(m, d); }
        void forward() { mj_forward(m, d); }
        void reset() { mj_resetData(m, d); mj_forward(m, d);}

        double read_joint_position(const char* jname) const 
        {
            int jid = mj_name2id(m, mjOBJ_JOINT, jname);
            if (jid < 0) { cerr << ERROR << "Joint no encontrado: " << jname << endl; return 0.0; }
            return d->qpos[m->jnt_qposadr[jid]];
        }

        double read_joint_velocity(const char* jname) 
        {
            int jid = mj_name2id(m, mjOBJ_JOINT, jname);
            if (jid < 0) { cout << ERROR << "Joint no encontrado: " << jname << endl; exit(1); }
            return d->qvel[m->jnt_dofadr[jid]];
        }

        double read_joint_force(const char* jname) 
        {
            int act_id = mj_name2id(m, mjOBJ_ACTUATOR, jname);
            if (act_id < 0) { cout << ERROR << "Joint actuator no encontrado: " << jname << endl; exit(1); }
            return d->ctrl[act_id];
        }

        void write_joint_position(const char* jname, double val) const 
        {
            int jid = mj_name2id(m, mjOBJ_JOINT, jname);
            if (jid < 0) { cerr << ERROR << "Joint no encontrado: " << jname << endl; return; }
            d->qpos[m->jnt_qposadr[jid]] = val;
        }

        void write_joint_velocity(const char* jname, double val) 
        {
            int jid = mj_name2id(m, mjOBJ_JOINT, jname);
            if (jid < 0) { cout << ERROR << "Joint no encontrado: " << jname << endl; return; }
            d->qvel[m->jnt_dofadr[jid]] = val;
        }

        void write_joint_force(const char* jname, double val) 
        {
            int act_id = mj_name2id(m, mjOBJ_ACTUATOR, jname);
            if (act_id < 0) { cout << ERROR << "Joint actuator no encontrado: " << jname << endl; return; }
            d->ctrl[act_id] = val;
        }
        double time_now() const { return d->time; }

        mjModel* get_model() const { return m; }
        mjData* get_data() const { return d; }
};