// -*- C++ -*-
/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * National Institute of Advanced Industrial Science and Technology (AIST)
 * General Robotix Inc. 
 */
/*!
 * @file  SRController.cpp
 * @brief Controller to drive a mobile robot
 * @date $Date$
 *
 * $Id$
 */

#include "SRController.h"

// Module specification
// <rtc-template block="module_spec">
static const char* joystickcontroller_spec[] =
  {
    "implementation_id", "SRController",
    "type_name",         "SRController",
    "description",       "Controller to drive a mobile robot",
    "version",           "1.0.0",
    "vendor",            "AIST HRG",
    "category",          "OpenHRP Controller",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

SRController::SRController(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_buttonsIn("buttons", m_buttons),
    m_axesIn("axes", m_axes),
    m_anglesIn("angles", m_angles),
    m_velsIn("vels", m_vels),
    m_torqueOut("torque", m_torque),
    
    // </rtc-template>
	dummy(0)
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

}

SRController::~SRController()
{
}


RTC::ReturnCode_t SRController::onInitialize()
{
  // Set InPort buffers
  addInPort("buttons", m_buttonsIn);
  addInPort("axes", m_axesIn);
  addInPort("angles", m_anglesIn);
  addInPort("vels", m_velsIn);
  
  // Set OutPort buffer
  addOutPort("torque", m_torqueOut);

  // ポート初期化 //
  m_torque.data.length(6);
  for (size_t i=0; i<m_torque.data.length(); i++){
      m_torque.data[i] = 0;
  }

  for (int i=0; i<2; i++) m_qRef[i] = 0;
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SRController::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SRController::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SRController::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SRController::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SRController::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t SRController::onExecute(RTC::UniqueId ec_id)
{
  // ジョイスティック（ユーザ）からのデータ入力 //
  if (m_buttonsIn.isNew()) m_buttonsIn.read();
  if (m_axesIn.isNew()) m_axesIn.read();
  if (m_anglesIn.isNew()) m_anglesIn.read();
  if (m_velsIn.isNew()) m_velsIn.read();
  if (!m_axes.data.length()) return RTC::RTC_OK;

  double commandTorqueR = -m_axes.data[1] - 0.2*m_axes.data[2];
  double commandTorqueL = -m_axes.data[1] + 0.2*m_axes.data[2];

  // ロボットへのトルク出力 //
  m_torque.data[0] = m_torque.data[2] = commandTorqueR;
  m_torque.data[3] = m_torque.data[5] = commandTorqueL;

#if 0
  for (size_t i=0; i<m_buttons.data.length(); i++){
      std::cout << m_buttons.data[i] << " ";
  }
  std::cout << std::endl;
#endif
  if (m_buttons.data[9]){
      m_qRef[0] += 0.02;
  }else if(m_buttons.data[11]){
      m_qRef[0] -= 0.02;
  } 
  if (m_buttons.data[8]){
      m_qRef[1] += 0.02;
  }else if(m_buttons.data[10]){
      m_qRef[1] -= 0.02;
  } 
#define P 200.0
#define D -20.0
#define MaxTau 50.0
  double flipperTorqueR = P*(m_qRef[0] - m_angles.data[1])+D*m_vels.data[1];
  flipperTorqueR = std::min( MaxTau, flipperTorqueR);
  flipperTorqueR = std::max(-MaxTau, flipperTorqueR);
  double flipperTorqueL = P*(m_qRef[1] - m_angles.data[4])+D*m_vels.data[4];
  flipperTorqueL = std::min( MaxTau, flipperTorqueL);
  flipperTorqueL = std::max(-MaxTau, flipperTorqueL);
  //std::cout << m_qRef[0] << ", " << m_angles.data[1] << "," << flipperTorqueR << std::endl; 
  m_torque.data[1] = flipperTorqueR;
  m_torque.data[4] = flipperTorqueL;

  m_torqueOut.write(); 
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SRController::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SRController::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SRController::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SRController::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SRController::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void SRControllerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(joystickcontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<SRController>,
                             RTC::Delete<SRController>);
  }
  
};


