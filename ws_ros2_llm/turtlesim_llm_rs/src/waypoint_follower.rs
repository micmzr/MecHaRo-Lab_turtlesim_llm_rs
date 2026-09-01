use anyhow::Result;
use rclrs::*;
// /turtle1/cmd_vel geometry_msgs/msg/Twist 
use ros_env::geometry_msgs::msg::Twist as TwistMsg;
// /turtle1/pose turtlesim_msgs/msg/Pose
use ros_env::turtlesim_msgs::msg::Pose as PoseMsg;

use std::f32::consts::PI;

use std::sync::{Arc, Mutex};
use std::{thread, time::Duration};

#[derive(Debug, Copy, PartialEq, Clone)]
struct Waypoint {
    x: f32,
    y: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum ControlStage {
    /// Stage 1: Rotate in place to align turtle heading (theta) with target waypoint
    RotateInPlace,
    /// Stage 2: Drive straight to target waypoint while keeping fine heading alignment
    DriveStraight,
    /// Waypoint reached
    Reached,
}

struct ControllerParams {
    kp_linear: f32,
    kp_angular: f32,
    max_linear_vel: f32,
    max_angular_vel: f32,
    linear_tolerance: f32,  // in meters (units)
    angular_tolerance: f32, // in radians
}

impl Default for ControllerParams {
    fn default() -> Self {
        Self {
            kp_linear: 1.2,
            kp_angular: 3.0,
            max_linear_vel: 2.0,
            max_angular_vel: 2.5,
            linear_tolerance: 0.1,    // 0.1 units distance tolerance
            angular_tolerance: 0.05,  // ~2.8 degrees angle tolerance
        }
    }
}

fn normalize_angle(mut angle: f32) -> f32 {
    while angle > PI {
        angle -= 2.0 * PI;
    }
    while angle < -PI {
        angle += 2.0 * PI;
    }
    angle
}

#[derive(Debug, Clone, Copy, Default)]
struct TurtlePose {
    x: f32,
    y: f32,
    theta: f32
}

#[derive(Debug, Clone, Copy, Default)]
struct TwistCommand {
    linear_x: f32,
    angular_z: f32,
}

struct WaypointController {
    params: ControllerParams,
    stage: ControlStage,
}

impl WaypointController {
    fn new(params: ControllerParams) -> Self {
        Self {
            params,
            stage: ControlStage::RotateInPlace,
        }
    }

    /// Compute velocity commands given current pose and target waypoint.
    /// Returns (TwistCommand, reached_target_bool).
    fn update(&mut self, current_pose: &TurtlePose, target: &Waypoint, node_name: &str) -> (TwistCommand, bool) {
        let dx = target.x - current_pose.x;
        let dy = target.y - current_pose.y;
        let distance = (dx * dx + dy * dy).sqrt();
        let target_heading = dy.atan2(dx);
        let heading_error = normalize_angle(target_heading - current_pose.theta);

        let mut cmd = TwistCommand::default();
        let mut reached = false;

        match self.stage {
            ControlStage::RotateInPlace => {
                // Stage 1: Only rotate, linear velocity = 0
                cmd.linear_x = 0.0;

                if heading_error.abs() <= self.params.angular_tolerance {
                    // Angle aligned! Switch to Stage 2: Drive straight
                    log_info!(node_name, "[Stage 1 -> Stage 2] Heading aligned! Error: {:.3} rad. Driving straight to ({:.2}, {:.2}).", heading_error, target.x, target.y);
                    self.stage = ControlStage::DriveStraight;
                    cmd.angular_z = 0.0;
                } else {
                    // Proportional angular control
                    let w = self.params.kp_angular * heading_error;
                    cmd.angular_z = w.clamp(-self.params.max_angular_vel, self.params.max_angular_vel);
                }
            }
            ControlStage::DriveStraight => {
                // Stage 2: Drive forward to waypoint while maintaining fine heading correction
                if distance <= self.params.linear_tolerance {
                    // Target reached!
                    log_info!(node_name, "[Stage 2 -> Reached] Target waypoint ({:.2}, {:.2}) reached! Distance error: {:.3}", target.x, target.y, distance);
                    self.stage = ControlStage::Reached;
                    cmd.linear_x = 0.0;
                    cmd.angular_z = 0.0;
                    reached = true;
                } else {
                    // Proportional linear speed and fine heading adjustment
                    let v = self.params.kp_linear * distance;
                    let w = self.params.kp_angular * heading_error;

                    cmd.linear_x = v.clamp(-self.params.max_linear_vel, self.params.max_linear_vel);
                    cmd.angular_z = w.clamp(-self.params.max_angular_vel, self.params.max_angular_vel);
                }
            }
            ControlStage::Reached => {
                cmd.linear_x = 0.0;
                cmd.angular_z = 0.0;
                reached = true;
            }
        }

        (cmd, reached)
    }

    fn reset_stage(&mut self) {
        self.stage = ControlStage::RotateInPlace;
    }
}

struct FollowerNodeState {
    current_pose: TurtlePose,
    pose_received: bool,
    waypoints: Vec<Waypoint>,
    current_wp_index: usize,
    controller: WaypointController,
    is_active: bool,
}

impl FollowerNodeState {
    fn new() -> Self {
        Self {
            current_pose: TurtlePose::default(),
            pose_received: false,
            waypoints: Vec::new(),
            current_wp_index: 0,
            controller: WaypointController::new(ControllerParams::default()),
            is_active: false,
        }
    }

    fn set_waypoints(&mut self, waypoints: Vec<Waypoint>) {
        self.waypoints = waypoints;
        self.current_wp_index = 0;
        self.controller.reset_stage();
        self.is_active = !self.waypoints.is_empty();
    }
}

struct TurtleCmdPublisherNode {
    #[allow(unused)]
    publisher: Publisher<TwistMsg>,
    node: Node,
}

impl TurtleCmdPublisherNode {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("waypoint_follower").unwrap();
        let publisher = node
            .create_publisher("/turtle1/cmd_vel")
            .unwrap();
        Ok(Self { publisher, node})
    }

    fn publish_data(&self, cmd_msg: TwistMsg) -> Result<(), RclrsError> {
        self.publisher.publish(cmd_msg).unwrap();        
        Ok(())
    }
}

fn main() -> Result<()> {
    let mut executor = Context::default_from_env().unwrap().create_basic_executor();
    let tcmd_pub = TurtleCmdPublisherNode::new(&executor).unwrap();
    let node_name = tcmd_pub.node.name().to_string();
    let worker = tcmd_pub.node.create_worker("waypoint_follower_worker");

    let state = Arc::new(Mutex::new(FollowerNodeState::new()));

    log!(tcmd_pub.node.logger().once(),
        "{} is starting...", node_name);

    let tcmd_pub_ptr = Arc::new(Mutex::new(tcmd_pub));
    let tcmd_pub_ptr_clone = Arc::clone(&tcmd_pub_ptr);

    let state_clone = Arc::clone(&state);
    
    let n_name = node_name.as_str().to_string();

    let n_name_clone = n_name.clone();

    let _pose_sub = worker
        .create_subscription(
            "/turtle1/pose",
            move |msg: PoseMsg| {
                let mut s = state_clone.lock().unwrap();
                s.current_pose = TurtlePose {
                    x: msg.x,
                    y: msg.y,
                    theta: msg.theta,
                };
                s.pose_received = true;

                if s.is_active && s.current_wp_index < s.waypoints.len() {
                    let target = s.waypoints[s.current_wp_index];
                    let current_pose = s.current_pose;

                    let (cmd, reached) = s.controller.update(&current_pose, &target, &n_name);

                    let mut twist = TwistMsg::default();
                    twist.linear.x = cmd.linear_x as f64;
                    twist.angular.z = cmd.angular_z as f64;

                    let _ = tcmd_pub_ptr_clone.try_lock().unwrap().publish_data(twist);

                    if reached {
                        s.current_wp_index += 1;
                        if s.current_wp_index < s.waypoints.len() {
                            log_info!(&n_name, "Moving to next waypoint ({}/{}): ({:.2}, {:.2})",
                                s.current_wp_index + 1,
                                s.waypoints.len(),
                                s.waypoints[s.current_wp_index].x,
                                s.waypoints[s.current_wp_index].y
                            );
                            s.controller.reset_stage();
                        } else {
                            log_info!(&n_name, "All waypoints reached. Stopping turtle.");
                            s.is_active = false;
                            let _ = tcmd_pub_ptr_clone.try_lock().unwrap().publish_data(TwistMsg::default());
                        }
                    }
                }
            },
        ).unwrap();

    let _action_wp_follower = tcmd_pub_ptr.try_lock().unwrap().node.clone()
        .create_action_server("/turtlesim_waypoint_follower/follow_cmd",
            move |goal_request: RequestedGoal<ros_env::turtlesim_llm_msgs::action::FollowWaypoints>| {
    
                log_info!(&n_name_clone, "Received new waypoints goal request.");
                
                let state_clone = Arc::clone(&state);

                async move {
                    
                    let accepting_goal = goal_request.accept();

                    let mut s = state_clone.lock().unwrap();

                    let executing_goal = accepting_goal.execute();  

                    let waypoints: Vec<Waypoint> = executing_goal.goal().waypoints.iter()
                        .map(|p| Waypoint { x: p.x as f32, y: p.y as f32 })
                        .collect();

                    let waypoints_completed: u32 = waypoints.len() as u32;
                    s.set_waypoints(waypoints);

                    drop(s); // Release the lock before entering the loop

                    loop {
                        // Wait for the action to complete
                        
                        let s = state_clone.lock().unwrap();

                        if s.is_active
                        {
                            let feedback = ros_env::turtlesim_llm_msgs::action::FollowWaypoints_Feedback {
                                current_waypoint_index: s.current_wp_index as u32,
                            };
                            drop(s);
                            executing_goal.publish_feedback(feedback);
                        }
                        else {
                            drop(s);
                            break;
                        }

                        std::thread::sleep(std::time::Duration::from_millis(1000));
                    }
           
                    let result = ros_env::turtlesim_llm_msgs::action::FollowWaypoints_Result { 
                        success: true,
                        waypoints_completed: waypoints_completed,
                    };

                    executing_goal.succeeded_with(result)
                }
            },
        ).unwrap(); 
    
    thread::spawn(move || loop {
        thread::sleep(Duration::from_secs(1));
        let _ = worker.run(|_| {
        });

    });
 
    log_info!(&node_name, "Turtle waypoint follower node is running. Waiting for waypoints...");

    // {
    //     let mut s = state.lock().unwrap();
    //     s.set_waypoints(vec![
    //         Waypoint { x: 2.0, y: 2.0 },
    //         Waypoint { x: 8.0, y: 2.0 },
    //         Waypoint { x: 8.0, y: 8.0 },
    //         Waypoint { x: 2.0, y: 8.0 },
    //         Waypoint { x: 5.5, y: 5.5 },
    //     ]);
    // }

    executor.spin(SpinOptions::default()).first_error()?;

    Ok(())
}
