//Created by:       Joachim Jamtvedt Børresen
//Date created:     May 5-12th 2023.
//Project:          LoneWolf Kongsberg 2023 Bachelor thesis. 
//Client:           Kongsberg Defence & Aerospace, Division Land Systems (KDA, DSL)

// This is a ROS2 Foxy package which runs the node /head. This node is responsible for relaying communicaton from user and system to the microcontrollers.
// This node also contains the gear jam circumvention algorithm (state machine) in case the gear get stuck while trying to change gear.

// Modified by: Sigurd Sætherø Spangelo 13.05.2023, to accomodate new microROS node. endringer er markert "// --- Sigurd" og "// ---" ved slutt


// C++ ROS2 libraries
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
// ROS2 message types
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int64.hpp" 
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"

#define BRAKE_WHILE_GEARING true

#define SET_THROTTLE_VALUE_FULL_BRAKE -100
#define SET_THROTTLE_VALUE_IDLE 0
#define SET_THROTTLE_VALUE_UNJAM 25

// --- Sigurd:
#define ENABLE_ROSSERIAL_COMPATABILITY_MODE true
// --

enum State : uint8_t {
    Idle,
    CheckPreviousGear,
    EngageBrake,
    GearShifting,
    RevertGearShifting,
    ThrottleControl,
    GearShiftingCancellation
};
 
class GearHead : public rclcpp::Node
{
    public:
        // Constructor
        GearHead() : Node("head"), m_current_state(State::Idle)
        {   
            // Creating publishers
            m_set_gear_publisher = create_publisher<std_msgs::msg::UInt8>(
                // --- Sigurd
                #if ENABLE_ROSSERIAL_COMPATABILITY_MODE
                    "set_gear",
                #else
                    rclcpp::extend_name_with_sub_namespace("set_gear", "lw_gear"),
                #endif
                // ---
                10
            );
            m_cancel_gear_publisher = create_publisher<std_msgs::msg::Empty>(
                // --- Sigurd
                #if ENABLE_ROSSERIAL_COMPATABILITY_MODE
                    "cancel_gear",
                #else
                    rclcpp::extend_name_with_sub_namespace("cancel_gear", "lw_gear"),
                #endif
                // ---
                10
            );
            m_set_throttle_publisher = create_publisher<std_msgs::msg::Int64>("setThrottle", 10);
            // --- Sigurd
            m_get_last_pass_gear_publisher = create_publisher<std_msgs::msg::Empty>(
                #if ENABLE_ROSSERIAL_COMPATABILITY_MODE
                    "get_last_pass_gear",
                #else
                    rclcpp::extend_name_with_sub_namespace("get_last_pass_gear", "lw_gear"),
                #endif
                10
            );
            // ---
            
            // Creating subscribers
            m_set_gear_response_subscriber = create_subscription<std_msgs::msg::UInt8>(
                rclcpp::extend_name_with_sub_namespace("set_gear_response", "lw_gear"), 10, std::bind(&GearHead::setGearResponseCallback, this, std::placeholders::_1));
            
            m_try_set_gear_subscriber = create_subscription<std_msgs::msg::UInt8>(
                rclcpp::extend_name_with_sub_namespace("trySetGear", "lw_gear"), 10, std::bind(&GearHead::trySetGearCallback, this, std::placeholders::_1));

            m_stuck_gear_subscriber = create_subscription<std_msgs::msg::Empty>(
                rclcpp::extend_name_with_sub_namespace("stuck_gear", "lw_gear"), 10, std::bind(&GearHead::stuckGearCallback, this, std::placeholders::_1));

            m_input_selector_subscriber = create_subscription<std_msgs::msg::Bool>(
                "inputSelector", 10, std::bind(&GearHead::inputSelectorCallback, this, std::placeholders::_1));

            m_ui_selector_subscriber = create_subscription<std_msgs::msg::Bool>(
                "uiSelector", 10, std::bind(&GearHead::uiSelectorCallback, this, std::placeholders::_1));
            
            m_try_set_throttle_subscriber = create_subscription<std_msgs::msg::Int64>(
                rclcpp::extend_name_with_sub_namespace("trySetThrottle", "lw_gear"), 10, std::bind(&GearHead::trySetThrottleCallback, this, std::placeholders::_1));

            m_pass_gear_subscriber = create_subscription<std_msgs::msg::UInt8>(
                rclcpp::extend_name_with_sub_namespace("pass_gear", "lw_gear"), 10, std::bind(&GearHead::passGearCallback, this, std::placeholders::_1));
            
            m_cancel_gear_subscriber = create_subscription<std_msgs::msg::Empty>(
                // --- Sigurd
                #if ENABLE_ROSSERIAL_COMPATABILITY_MODE
                    "cancel_gear",
                #else
                    rclcpp::extend_name_with_sub_namespace("cancel_gear", "lw_gear"),
                #endif
                // ---
                10,
                std::bind(&GearHead::cancelGearCallback, this, std::placeholders::_1)
            );

            // --- Sigurd
            m_get_last_pass_gear_response_subscriber = create_subscription<std_msgs::msg::UInt8>(
                rclcpp::extend_name_with_sub_namespace("get_last_pass_gear_response", "lw_gear"),
                10,
                std::bind(&GearHead::passGearCallback, this, std::placeholders::_1) // also calls passGearCallback, same as pass_gear
            );
            // ---

            // Start in the idle state
            transitionToState(State::Idle); 
        }

    private:
        // Functions to publish messages
        void publish_set_gear(uint8_t gear) {
            std_msgs::msg::UInt8 set_gear_message;
            set_gear_message.data = gear;
            m_set_gear_publisher->publish(set_gear_message);
            RCLCPP_INFO(get_logger(), "publishing: set_gear");
        }

        //-100 er full brems, 0 er hverken brems eller gass og 100 er full gass
        void publish_set_throttle(int value) {
            std_msgs::msg::Int64 set_throttle_message;
            set_throttle_message.data = value;
            m_set_throttle_publisher->publish(set_throttle_message);
            RCLCPP_INFO(get_logger(), "publishing: setThrottle %d", value);
        }

        void publish_cancel_gear() {
            m_cancel_gear_publisher->publish(std_msgs::msg::Empty());
            RCLCPP_INFO(get_logger(), "publishing: cancel_gear");
        }
        
        // --- Sigurd
        void publish_get_last_pass_gear() {
            m_get_last_pass_gear_publisher->publish(std_msgs::msg::Empty());
            RCLCPP_INFO(get_logger(), "publishing: get_last_pass_gear");
        }
        // ---

        // Callback functions
        void setGearResponseCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
            this->m_SetGearResponse = msg->data;
            RCLCPP_INFO(get_logger(), "Callback received: set_gear_response");
        }

        void trySetGearCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
            m_TrySetGear = msg->data;
            this->m_TryAgainCounter = 0;
            RCLCPP_INFO(get_logger(), "Callback received: trySetGear: %d", m_TrySetGear);

            // Transition to the CheckPreviousGear state
            if (m_InputSelector && m_UiSelector){
                transitionToState(State::CheckPreviousGear);
            }

        }

        void stuckGearCallback(const std_msgs::msg::Empty::SharedPtr) {
            RCLCPP_INFO(get_logger(), "Callback received: stuck_gear");

            if (this->m_current_state == State::GearShifting){
                this->transitionToState(State::RevertGearShifting);
            }
            if (this->m_current_state == State::RevertGearShifting){
                //this->transitionToState(State::CheckPreviousGear);
                if (this->m_TryAgainCounter++ < 2){
                    RCLCPP_INFO(get_logger(), "Attempting to gear after JAM, attempt: %d", m_TryAgainCounter);
                    this->transitionToState(State::ThrottleControl);
                }
                else {
                    RCLCPP_INFO(get_logger(), "Attempts exhausted, JAM circumvention failed");
                }
            }
            if (this->m_current_state == State::ThrottleControl){
                RCLCPP_INFO(get_logger(), "Throttling to rotate transmission");
                this->publish_set_throttle(SET_THROTTLE_VALUE_UNJAM);
                m_ThrottleTimer = create_wall_timer(std::chrono::seconds(1), std::bind(&GearHead::throttleTimerCallback, this));
            }
        }

        void inputSelectorCallback(const std_msgs::msg::Bool::SharedPtr msg) {
            m_InputSelector = msg->data;
        }

        void uiSelectorCallback(const std_msgs::msg::Bool::SharedPtr msg) {
            m_UiSelector = msg->data;
        }

        void trySetThrottleCallback(const std_msgs::msg::Int64::SharedPtr msg) {
            m_TrySetThrottle = msg->data;
            RCLCPP_INFO(get_logger(), "Callback received: trySetThrottle");
        }

        void passGearCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
            this->m_PreviousTargetGear = msg->data;
            //
            if (this->m_current_state == State::GearShifting && this->m_TargetGear == msg->data){
                this->transitionToState(State::Idle);
                this->publish_set_throttle(SET_THROTTLE_VALUE_IDLE);
                RCLCPP_INFO(get_logger(), "Brakes released");
            }
            //
            if (this->m_current_state == State::RevertGearShifting && this->m_RevertGear == msg->data){

                this->transitionToState(State::CheckPreviousGear);
            }
            if (this->m_PassGear != msg->data){
                this->m_PassGear = msg->data;
                RCLCPP_INFO(get_logger(), "Callback received: pass_gear");
            }
        }

        void cancelGearCallback(const std_msgs::msg::Empty::SharedPtr) {
            RCLCPP_INFO(get_logger(), "Callback received: cancel_gear");

            this->transitionToState(State::Idle);

        }


        void gearTimerCallback() {
           if (this->m_current_state != State::GearShifting) {
                return;
           }
           
            this->publish_set_gear(this->m_TargetGear);
        }

        void brakeTimerCallback(){
            if(this->m_current_state != State::EngageBrake){
                return;
            }
            RCLCPP_INFO(get_logger(), "F and R brakes set to FULL");
            transitionToState(State::GearShifting);
        }

        // --- Sigurd
        // This function is also available in gear.hpp
        // gear.hpp alternatives:
        //      bool gear::is_valid(uint8_t gear_index)
        //      bool gear::is_some(const OptionGear& maybe_gear)
        // ---
        static bool isGearValid(uint8_t gear) {
            if (gear <= 4) {
                return true;
            }
            else {
                return false;
            }
        }

        // --- Sigurd
        // This function is also available in gear.hpp
        // gear.hpp alternatives:
        //      OptionGear gear::get_first_revertable_towards(const Gear& gear, bool direction)
        //      OptionGear gear::get_first_revertable_towards(const OptionGear& maybe_gear, bool direction)
        // ---
        static uint8_t getClosestRevertableGear(uint8_t gear, bool direction){
            const int8_t increment = direction ? 1 : -1;
            for (int8_t i = gear; i >= 0 && i <= 4; i+=increment){
                
                if (isGearRevertable((uint8_t)i)){
                    return i;
                }
            }
            return 255;
        }

        // --- Sigurd
        // this function is also available in gear.hpp
        // gear.hpp alternatives:
        //      bool gear::is_revertable(const Gear& gear)
        //      bool gear::is_revertable(const OptionGear& maybe_gear)
        // ---
        static bool isGearRevertable(uint8_t gear){
            if (!isGearValid(gear)){
                return false;
            }
            else if (gear == 1 || gear == 3 || gear == 4){
                return true;
            }
            else {
                return false;
            }
        }
        
        void throttleTimerCallback() {
            if(this->m_current_state != State::ThrottleControl){
                return;
            }
            transitionToState(State::CheckPreviousGear);
        }

        // ROS2 publishers
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_set_gear_publisher;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr m_set_throttle_publisher;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_cancel_gear_publisher;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_get_last_pass_gear_publisher;


        // ROS2 subscribers        
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr m_set_gear_response_subscriber;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr m_try_set_gear_subscriber;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_stuck_gear_subscriber;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_input_selector_subscriber;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_ui_selector_subscriber;
        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr m_try_set_throttle_subscriber;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr m_pass_gear_subscriber;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_cancel_gear_subscriber;
        // --- Sigurd
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr m_get_last_pass_gear_response_subscriber;
        // ---
        
        // gear_jam_circumvention
        State m_current_state;
        void transitionToState(State new_state) {
            // --- Sigurd
            if(!isGearValid(m_PassGear))
            {
                this->publish_get_last_pass_gear();
            }
            // ---


            // Update the current state
            m_current_state = new_state;
            // Perform any necessary actions when transitioning to a new state
            switch (new_state) {
                case State::Idle:
                {
                    // State purpose: wait for instructions
                    RCLCPP_INFO(get_logger(), "State:Idle");
                    
                    break;
                }
                case State::CheckPreviousGear:
                {
                    // State purpose: Prepare for GearShifting state.
                    RCLCPP_INFO(get_logger(), "State:CheckPreviousGear");
                    // subscribe to set_gear_reponse to detect current gear
                    // subscribe to stuck_gear to detect jam
                    
                    if (!isGearValid(m_TrySetGear)){
                        RCLCPP_WARN(get_logger(), "CheckPreviousGear: m_TrySetGear is not valid");
                        transitionToState(State::GearShiftingCancellation);
                        break;
                    }
                    if (!isGearValid(m_PreviousTargetGear)){
                        RCLCPP_WARN(get_logger(), "CheckPreviousGear: m_PreviousTargetGear is not valid");
                        transitionToState(State::GearShiftingCancellation);
                        break;
                    }
                    if (m_TrySetGear == m_PreviousTargetGear){
                        RCLCPP_WARN(get_logger(), "CheckPreviousGear: m_TrySetGear == m_PreviousTargetGear");
                        transitionToState(State::GearShiftingCancellation);
                        break;
                    }

                    m_TargetGear = m_TrySetGear;
                    transitionToState(State::EngageBrake);

                    break;
                }
                case State::EngageBrake:
                {
                    //State purpose: Engage brake before gearshifting
                    RCLCPP_INFO(get_logger(), "State:EngageBrake");
                    #if BRAKE_WHILE_GEARING
                        this->publish_set_throttle(SET_THROTTLE_VALUE_FULL_BRAKE);
                        m_brakeTimer = create_wall_timer(std::chrono::seconds(3), std::bind(&GearHead::brakeTimerCallback, this));
                    #else
                        this->transitionToState(State::GearShifting);
                    #endif

                    break;
                }
                case State::GearShifting:
                {
                    // State purpose: Move the gear shift lever to target gear and verify
                    RCLCPP_INFO(get_logger(), "State:GearShifting");

                    if (!this->isGearValid(m_TargetGear)){
                        this->transitionToState(State::Idle);
                        break;
                    }

                    this->publish_set_gear(m_TargetGear);

                    //timeout?

                    break;
                }
                case State::RevertGearShifting:
                {
                    // State purpose: Return the gear shift lever back to last successfull gear position
                    RCLCPP_INFO(get_logger(), "State:RevertGearShifting");
            
                    if (!isGearValid(m_PassGear) || !isGearValid(m_TargetGear)) {
                        RCLCPP_WARN(get_logger(), "revertgear: Either m_PassGear or m_TargetGear is not valid");
                        transitionToState(State::GearShiftingCancellation);
                        break;
                    }
                    const bool direction = this->m_PassGear > this->m_TargetGear;
                    this->m_RevertGear = this->getClosestRevertableGear(m_TargetGear, direction);
                    if (!this->isGearValid(m_RevertGear)){
                        this->transitionToState(State::Idle);
                        break;
                    }
                    this->publish_set_gear(m_RevertGear);

                    //timeout?

                    break;
                }
                case State::ThrottleControl:
                {
                    // State purpose: Engage throttle to rotate transmission
                    RCLCPP_INFO(get_logger(), "State:ThrottleControl");
                    
                    //Fjerne alt dette
                    if (m_ApplyThrottleAfterPreviousGear) {
                        this->publish_set_throttle(SET_THROTTLE_VALUE_UNJAM);
                        m_ThrottleTimer = create_wall_timer(std::chrono::seconds(1), std::bind(&GearHead::throttleTimerCallback, this));
                    }
                
                    break;
                }
                case State::GearShiftingCancellation:
                {
                    // State purpose: Cancel gearprocess
                    RCLCPP_INFO(get_logger(), "State:GearShiftingCancellation");

                    // Pubilsh target gear on topic: set_gear
                    this->publish_cancel_gear();

                    break;
                }
            }
        }   


        // Other private member variables
        rclcpp::TimerBase::SharedPtr m_ThrottleTimer;
        rclcpp::TimerBase::SharedPtr m_brakeTimer;

        uint8_t m_SetGearResponse = 0; //Antar at vi starter i PARK
        uint8_t m_TrySetGear;
        bool m_InputSelector;
        bool m_UiSelector;
        int m_TrySetThrottle;
        uint8_t m_TargetGear;
        int m_SetThrottle;
        bool m_CancelGear;
        uint8_t m_PreviousTargetGear = 255;
        uint8_t m_PassGear = 255;
        uint8_t m_RevertGear = 255;
        uint8_t m_TryAgainCounter = 0;

        bool m_ApplyThrottleAfterPreviousGear = false;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GearHead>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}