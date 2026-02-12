#include <iostream>
#include "include/Environment.hpp"


#define ERROR ("\x1b[1;31m")
#define RESET_FORMAT ("\x1b[1;0m \n")

char errstr[1024]="";

Environment::Environment(const char* model_path)
     :m(mj_loadXML(model_path, NULL, errstr, sizeof(errstr))) 
{
    if (!m) 
    {
        throw std::invalid_argument(std::string(ERROR) + "No se pudo cargar el modelo\n" + errstr + RESET_FORMAT);
    }
    d = mj_makeData(m);
    
    if (!d) 
    {
        mj_deleteModel(m);
        throw std::runtime_error("No se pudo crear mjData");    
    }

    for (int i = 0; i < m->njnt; i++) 
    {
        const char* joint_name = mj_id2name(m, mjOBJ_JOINT, i);
        if (joint_name){joint_index[std::string(joint_name)]=i;} 
    }
    for (int i = 0; i < m->nu; i++) 
    {
        const char* actuator_name = mj_id2name(m, mjOBJ_ACTUATOR, i);
        if(actuator_name){actuator_index[std::string(actuator_name)]=i;}
    }

}

double Environment::read_joint_position(const char* jname) const
{
    if((joint_index.find(jname) == joint_index.end())){throw std::invalid_argument(std::string(ERROR) + "No existe una joint con ese nombre: " + jname + RESET_FORMAT);}
    auto idx = joint_index.at(jname);
    double angle = d->qpos[m->jnt_qposadr[idx]];
    angle = std::fmod(angle + M_PI, 2*M_PI);
    if (angle < 0) angle += 2*M_PI;
    return angle - M_PI;
}

double Environment::read_joint_velocity(const char* jname)
{
    if((joint_index.find(jname) == joint_index.end())){throw std::invalid_argument(std::string(ERROR) + "No existe una joint con ese nombre: " + jname + RESET_FORMAT);}
    auto idx = joint_index.at(jname);
    return d->qvel[m->jnt_dofadr[idx]];
}

double Environment::read_actuator_force(const char* aname)
{
    if((actuator_index.find(aname) == actuator_index.end())){throw std::invalid_argument(std::string(ERROR) + "No existe un actuador con ese nombre: " + aname + RESET_FORMAT);}
    auto idx = actuator_index.at(aname);
    return d->actuator_force[idx];
}

void Environment::write_joint_position(const char* jname, double val) const
{
    if((joint_index.find(jname) == joint_index.end())){throw std::invalid_argument(std::string(ERROR) + "No existe una joint con ese nombre: " + jname + RESET_FORMAT);}
    auto idx = joint_index.at(jname);
    d->qpos[m->jnt_qposadr[idx]]=val;
}

void Environment::write_joint_velocity(const char* jname, double val)
{
    if((joint_index.find(jname) == joint_index.end())){throw std::invalid_argument(std::string(ERROR) + "No existe una joint con ese nombre: " + jname + RESET_FORMAT);}
    auto idx = joint_index.at(jname);
    d->qvel[m->jnt_dofadr[idx]] = val;
}

void Environment::write_actuator_force(const char* aname, double val)
{
    if((actuator_index.find(aname) == actuator_index.end())){throw std::invalid_argument(std::string(ERROR) + "No existe un actuador con ese nombre: " + aname + RESET_FORMAT);}
    auto idx = actuator_index.at(aname);
    d->ctrl[idx] = val;
}

