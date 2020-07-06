/* Include the controller definition */
#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* math library */
#include <cmath>

/****************************************/
/****************************************/

CFootBotDiffusion::CFootBotDiffusion() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   vmax(0.0f), // [REV] ArgOS uses prefix (e.g. m_f...), you should always try to stick to the defined formalism.
   km(0.0f), // [REV] Good class attribute redefinition
   kt(0.0f),
   right_speed(0.0f),
   left_speed(0.0f) {}

/****************************************/
/****************************************/

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   // [REV] Defining your parameters in scenario is a good practice
   GetNodeAttribute(t_node, "km", km); // [REV] Why not calling this function with the default value like in the example.
   GetNodeAttribute(t_node, "kt", kt);
   GetNodeAttribute(t_node, "vmax", vmax);
   left_speed = vmax;
   right_speed = vmax;
}

/****************************************/
/****************************************/

void CFootBotDiffusion::ControlStep() {
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Get average obstacle proximity vector */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   cAccumulator /= tProxReads.size();
   /* Compute repulsive force */
   cAccumulator *= -1; // [REV] Non necessary line.
   /* If no repulsive force, go at max speed in displacement direction (because no specific goals) */
   Real vM = (right_speed+left_speed)/2; /* vM is the translational component of robot velocity */
   if(cAccumulator.Length()==0){
      left_speed = vmax*copysign(1, vM);
      right_speed = vmax*copysign(1, vM);
   }
   else
   {
      /* Apply force to robot velocity (translational acceleration and rotation) */
      Real fX = cAccumulator.GetX();
      Real fY = cAccumulator.GetY();
      vM += fX*km; /* fX induces translational acceleration */
      Real vT = kt*fY*copysign(1, vM); /* fY induces rotation */
      /* fX also induces rotation if approaching obstacle */
      if(vM*fX < 0){vT += copysign(1, vT)*abs(kt*fX);}
      /* Recompute wheel speeds from translational and rotational components */
      right_speed = vM+vT;
      left_speed = vM-vT;
      /* Wheel speeds are bounded by vmax */
      if(abs(left_speed/vmax) > 1){left_speed/=abs(left_speed/vmax);}
      if(abs(right_speed/vmax) > 1){right_speed/=abs(right_speed/vmax);} 
   }
   /* Set computed wheel speeds */
   m_pcWheels->SetLinearVelocity(left_speed, right_speed);
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")
