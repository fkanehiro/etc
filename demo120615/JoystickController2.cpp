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
 * @file  JoystickController2.cpp
 * @brief Controller to drive a mobile robot
 * @date $Date$
 *
 * $Id$
 */

#include "JoystickController2.h"

// Module specification
// <rtc-template block="module_spec">
static const char* joystickcontroller_spec[] =
  {
    "implementation_id", "JoystickController2",
    "type_name",         "JoystickController2",
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

JoystickController2::JoystickController2(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_commandIn("command", m_command),
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

JoystickController2::~JoystickController2()
{
}


RTC::ReturnCode_t JoystickController2::onInitialize()
{
  // Set InPort buffers
  addInPort("command", m_commandIn);
  
  // Set OutPort buffer
  addOutPort("torque", m_torqueOut);

  // ポート初期化 //
  m_command.data.vx = m_command.data.vy = m_command.data.va = 0.0;
  m_torque.data.length(2);
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t JoystickController2::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JoystickController2::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JoystickController2::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JoystickController2::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JoystickController2::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t JoystickController2::onExecute(RTC::UniqueId ec_id)
{
  // ジョイスティック（ユーザ）からのデータ入力 //
  if (m_commandIn.isNew()) m_commandIn.read();

  double commandTorqueR = -m_command.data.vy + 0.2*m_command.data.va;
  double commandTorqueL = -m_command.data.vy - 0.2*m_command.data.va;

  // ロボットへのトルク出力 //
  m_torque.data[0] = commandTorqueR;
  m_torque.data[1] = commandTorqueL;
  m_torqueOut.write(); 

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t JoystickController2::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JoystickController2::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JoystickController2::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JoystickController2::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JoystickController2::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void JoystickController2Init(RTC::Manager* manager)
  {
    RTC::Properties profile(joystickcontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<JoystickController2>,
                             RTC::Delete<JoystickController2>);
  }
  
};


