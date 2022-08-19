/**
 * @file io_behavior.h
 * @author Ethan J. Musser (emusser@seas.upenn.edu)
 * @brief Provides `kodlab::IOBehavior`, which extends `kodlab::Behavior` to
 * provide input and output via LCM.
 * @date 8/9/22
 * 
 * @copyright (c) Copyright 2022 The Trustees of the University of Pennsylvania.
 * All rights reserved.
 * 
 */

#pragma once

#include "lcm/lcm-cpp.hpp"
#include "VoidLcm.hpp"

#include "kodlab_mjbots_sdk/behavior.h"
#include "kodlab_mjbots_sdk/robot_base.h"
#include "kodlab_mjbots_sdk/lcm_message_handler.h"
#include "kodlab_mjbots_sdk/lcm_publisher.h"
#include "kodlab_mjbots_sdk/lcm_subscriber.h"

namespace kodlab {

/**
 * @brief Robot behavior object with LCM input and output (I/O)
 * @note To include additional LCM behavior inputs in child classes, simply
 * compose additional `LcmMessageHandler` objects and call
 * `subscriber_->AddSubscription` for each additional subscription when
 * initializing the behavior.
 * @note To include additional LCM  behavior outputs in child classes, simply
 * compose additional `LcmPublisher` objects and initialize them with the
 * behavior.
 * @tparam Robot[optional] `kodlab::RobotBase`-derived robot class
 * @tparam Input[optional] LCM input message class, default is `VoidLcm`
 * @tparam Output[optional] LCM output message class, default is `VoidLcm`
 */
template<class Robot = kodlab::RobotBase, class Input = VoidLcm,
    class Output = VoidLcm>
class IOBehavior : public Behavior<Robot> {

 public:

  /**
   * @brief Construct an I/O Behavior object
   * @param robot robot that behavior is executing on
   * @param subscriber realtime LCM subscriber to monitor for inputs
   * @param publish_lcm LCM object used for publishing
   * @param subscribe_channel LCM channel to subscribe to
   * @param publish_channel LCM channel to publish on
   * @param name behavior name
   */
  IOBehavior(std::shared_ptr<Robot> robot,
             std::shared_ptr<LcmSubscriber> subscriber,
             std::shared_ptr<lcm::LCM> publish_lcm,
             std::string subscribe_channel,
             std::string publish_channel,
             std::string name = "")
      : Behavior<Robot>(robot, std::move(name)),
        subscriber_(subscriber),
        publisher_(LcmPublisher<Output>(publish_lcm, publish_channel)),
        publisher_data_(publisher_.get_message_shared_ptr()) {
    subscriber_->AddSubscription<Input>(subscribe_channel,
                                        input_handler_);
  }

  /**
   * Virtual destructor for clean destruction of child class instances
   */
  virtual ~IOBehavior() = default;

  /**
   * @brief Processes input data, if available
   * @note This method simply returns if the `Input` type is `VoidLcm`.
   */
  void ThreadSafeProcessInput() final {
    if (std::is_same_v<VoidLcm, Input>) { return; }
    auto input_msg = input_handler_.GetDataIfNew();
    if (input_msg.has_value()) {
      ProcessInput(input_msg.value());
      input_data_cache_ = input_msg.value();
    }
  }

  /**
   * @brief Processes input data, if available
   * @details Virtual method to be implemented when utilizing inputs. Called
   * when new input data is available, this method processes the new data
   * provided in `input_msg`.
   * @note This method simply returns if the `Output` type is `VoidLcm`.
   * @param input_msg[in] input data to be processed
   */
  virtual void ProcessInput(const Input &input_msg) {}

  /**
   * @brief Prepares output data message
   */
  void ProcessOutput() final {
    if (std::is_same_v<VoidLcm, Output>) { return; }
    PrepareOutput(*publisher_data_);
    PublishOutput();
  }

  /**
   * @brief Prepares output data message
   * @details Virtual method to be implemented when utilizing outputs. Called
   * every update loop, this method records data to `publisher_data`, which is
   * then published via the composed LCM publisher.
   * @param output_msg[out] output message to be prepared
   */
  virtual void PrepareOutput(Output &output_msg) {}

  /**
   * @brief Publish an output message over LCM from internal `publisher_` data.
   * @param output_msg output message to be published
   */
  void PublishOutput() {
    publisher_.Publish();
  }

  /**
   * @brief Publish an output message over LCM.
   * @param output_msg output message to be published
   */
  void PublishOutput(const Output &output_msg) {
    *publisher_data_ = output_msg;
    PublishOutput();
  }

 protected:

  /**
   * @brief Shared pointer to realtime LCM subscriber object used that listens
   * for input.
   */
  std::shared_ptr<LcmSubscriber> subscriber_;

  /**
   * @brief LCM output publisher.
   */
  LcmPublisher<Output> publisher_;

  /**
   * @brief Message handler for LCM inputs.
   */
  LcmMessageHandler<Input> input_handler_;

  /**
   * @brief Cache storing the most recent input data.
   */
  [[maybe_unused]] Input input_data_cache_;

 private:

  /**
   * @brief Shared pointer to message data that gets published by `publisher_`.
   */
  std::shared_ptr<Output> publisher_data_;

};

} // kodlab
