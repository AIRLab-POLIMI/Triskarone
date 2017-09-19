/*
 * DC_0
 */
#include <ModuleConfiguration.hpp>
#include <Module.hpp>

// --- MESSAGES ---------------------------------------------------------------
#include <core/common_msgs/Led.hpp>
#include <core/actuator_msgs/Setpoint_f32.hpp>

// --- NODES ------------------------------------------------------------------
#include <core/sensor_publisher/Publisher.hpp>
#include <core/actuator_subscriber/Speed.hpp>
#include <core/led/Subscriber.hpp>

// --- BOARD IMPL -------------------------------------------------------------
#include <core/QEI_driver/QEI.hpp>
#include <core/A4957_driver/A4957.hpp>

// *** DO NOT MOVE ***
Module module;

// --- TYPES ------------------------------------------------------------------
using QEI_Publisher  = core::sensor_publisher::Publisher<ModuleConfiguration::QEI_DELTA_DATATYPE>;
using PID_Subscriber = core::actuator_subscriber::Speed<float, core::actuator_msgs::Setpoint_f32>;

// --- NODES ------------------------------------------------------------------
core::led::Subscriber led_subscriber("led_subscriber", core::os::Thread::PriorityEnum::LOWEST);

QEI_Publisher  encoder("encoder", module.qei, core::os::Thread::PriorityEnum::NORMAL);
PID_Subscriber motor("speed_sub", module.hbridge_pwm, core::os::Thread::PriorityEnum::NORMAL);

// --- MAIN -------------------------------------------------------------------
extern "C" {
   int
   main()
   {
	  const float period = 10.0;
	  const float pwmMax = 1.0;

	  module.initialize();


      // Module configuration
      core::QEI_driver::QEI_DeltaConfiguration qei_configuration;
      qei_configuration.period = period;
      qei_configuration.ticks  = 2000*14.0f;//2000*26.0f;//2000*74.0f;
      qei_configuration.invert = 0;
      module.qei.setConfiguration(qei_configuration);

      // Nodes configuration
      core::led::SubscriberConfiguration led_subscriber_configuration;
      led_subscriber_configuration.topic = "led";
      led_subscriber.setConfiguration(led_subscriber_configuration);

      // tuned pid constants:
      // ultimate gain K_u: between 0.065 and 0.070
      // oscillation period at ultimate gain (0.070) : 0.028571429[s]
      core::actuator_subscriber::SpeedConfiguration pid_configuration;
      pid_configuration.kp = 0.065*0.45;
      pid_configuration.ti = 0.028571429/1.2;
//      pid_configuration.kp = 0.34/2/2.85*0.6;
//      pid_configuration.ti = 1.591*2*2.85/20;
      pid_configuration.td = 0;
      pid_configuration.ts = period/1000.0;
      pid_configuration.min = -pwmMax;
      pid_configuration.max = pwmMax;
      pid_configuration.encoder_topic = "encoder_0";
      pid_configuration.setpoint_topic = "speed_0";
      pid_configuration.idle = 0;
      pid_configuration.timeout = 500;
      motor.setConfiguration(pid_configuration);

      core::sensor_publisher::Configuration encoder_configuration;
      encoder_configuration.topic = "encoder_0";
      encoder.setConfiguration(encoder_configuration);

      // Add nodes to the node manager (== board)...
      module.add(led_subscriber);
      module.add(encoder);
      module.add(motor);

      // ... and let's play!
      module.setup();
      module.run();

      // Is everything going well?
      for (;;) {
         if (!module.isOk()) {
            module.halt("This must not happen!");
         }

         module.keepAlive();

         core::os::Thread::sleep(core::os::Time::ms(500));
      }

      return core::os::Thread::OK;
   } // main
}
