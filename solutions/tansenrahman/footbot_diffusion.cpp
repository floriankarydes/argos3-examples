/* Include the controller definition */
#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/****************************************/
/****************************************/

CFootBotDiffusion::CFootBotDiffusion() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}

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
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
}

/****************************************/
/****************************************/

void CFootBotDiffusion::ControlStep() {
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   CVector2 cMax = CVector2(tProxReads[0].Value, tProxReads[0].Angle);

   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
      //get maximum length value
      if(cMax.Length() < tProxReads[i].Value){
        cMax = CVector2(tProxReads[i].Value, tProxReads[i].Angle);
      }

   }
   cAccumulator /= tProxReads.size();
   /* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
   CRadians cAngle = cAccumulator.Angle();

   //If no obstacle detected, go straight
   if(cAccumulator.Length() == 0.0f){
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
   }

   //Here, the obstacle is in front. Choose to rotate backwards left or right (decided randomly) in order to 
   //avoid a deadlock of constantly moving forward and back. Eventually the sum of angles will exit the bound
   else if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle)){
      int chance = rand() % 100;
      //0-49
      if (chance < 50){
        m_pcWheels->SetLinearVelocity(m_fWheelVelocity * -1, 0.0f);
      }
      //50-99
      else{
        m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity * -1);
      }      
   }  
   //In this case the obstacle detected is not in front, so just turn away from it, depending on the sign of the angle
   else{
      //obstacle is behind, to the right
      if(cAngle.GetValue() > atan(1)*2) {
        //turn right
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
      }
      //obstacle is behind, to the left
      else if(cAngle.GetValue() < -1 * atan(1)*2){
         //turn left
         m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity); 
      }
      //Use the maximum length of all the sensors to determine when to move away rather than the average
      //obstacle is in front, to the right. When close enough, turn backwards to the left
      else if(cMax.Length() > m_fDelta * 2 && cAngle.GetValue() >= 0){
        m_pcWheels->SetLinearVelocity(0.0f, -1 * m_fWheelVelocity);
      }
      //obstacle is in front, to the left. When close enough, turn backwards to the right
      else if(cMax.Length() > m_fDelta * 2 && cAngle.GetValue() <0){
         m_pcWheels->SetLinearVelocity(-1 * m_fWheelVelocity, 0.0f);
      }
      else{
      //otherwise do nothing -> keeps previous velocity/trajectory

      }
   } 
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
