use anyhow::Result;
use rclrs::*;

// /turtle1/pose turtlesim_msgs/msg/Pose
use ros_env::turtlesim_msgs::msg::Pose as PoseMsg;
// /llm_cmd std_msgs/msg/String
use ros_env::std_msgs::msg::String as StringMsg;

use ros_env::geometry_msgs::msg::Point as PointMsg;

use foundry_local_sdk::{ChatCompletionRequestMessage,
    ChatCompletionRequestUserMessage, ChatCompletionRequestSystemMessage,
    FoundryLocalConfig, FoundryLocalManager};

use std::sync::{Arc, Mutex};
use tokio::sync::mpsc;
use serde::Deserialize;

#[derive(Debug, Deserialize)]
struct Waypoint {
    x: f32,
    y: f32,
}

enum MsgType {
    Pose(PoseMsg),
    Cmd(StringMsg),
}

fn send_cmd_to_llm( mpsc_tx: mpsc::Sender<MsgType>, msg: MsgType) {
    tokio::spawn(async move {
        if let Err(e) = mpsc_tx.send(msg).await {   
            eprintln!("Failed to send message: {}", e);
        }
    });
}

fn strip_think(input: &str) -> String {
    if let (Some(start), Some(end)) = (input.find("<think>"), input.find("</think>")) {
        if start < end {
            let mut result = String::new();
            result.push_str(&input[end + 10..]); // 8 is the length of "</think> + endl"
            return result;
        }
    }
    input.to_string()
}

#[tokio::main]
async fn main() -> Result<()> {
    let mut executor = Context::default_from_env().unwrap().create_basic_executor();
    let tllm_node = executor.create_node("turtle_llm").unwrap();
    let node_name = tllm_node.name().to_string();

    log!(tllm_node.logger().once(),
        "{} is starting...", node_name);

    // MS Foundry Local SDK initialization - takes time to download and register the models, so we do it at the beginning
    // Based on: https://github.com/microsoft/foundry-local/tree/main/sdk/rust
    let manager = FoundryLocalManager::create(FoundryLocalConfig::new(node_name.clone()))?;

    // Doesn't work for some reasons, maybe in 1.2.4 will be fixed 
    // let eps = manager.discover_eps()?;
    // for ep in &eps {
    //     println!("{} — registered: {}", ep.name, ep.is_registered);
    // }
    // Download and register all available EPs
    let _result = manager.download_and_register_eps(None).await?;
    // println!("Success: {}, Status: {}", result.success, result.status);

    let catalog = manager.catalog();

    // List all available models
    let models = catalog.get_models().await?;
    for model in &models {
        log_debug!(&node_name, "{} (id: {})", model.alias(), model.id());
    }

    // let model_name = "qwen3.5-0.8b";
    let model_name = "qwen3.5-2b";
    // let model_name = "qwen3-vl-2b-instruct";

    let model = catalog.get_model(model_name).await?;

    log!(tllm_node.logger().once(),
        "Downloading model {}...", model_name);

    model.download_builder()
        .progress(|progress| {
            print!("\r{progress:.1}%");
        std::io::Write::flush(&mut std::io::stdout()).ok();
        })
        .run()
        .await?;

    // print!("\n");

    log!(tllm_node.logger().once(),
        "Loading model {}...", model_name);
    model.load().await?;

    let client = model.create_chat_client();
    // Configure generation settings (fluent builder)
        // .temperature(0.7)
        // .max_tokens(256)
        // .top_p(0.9)
        // .frequency_penalty(0.5);

    let tllm_node_ptr = Arc::new(Mutex::new(tllm_node));
    let tllm_node_ptr_clone = Arc::clone(&tllm_node_ptr);

    let (tx, mut rx) = mpsc::channel::<MsgType>(10);

    let tx_pose = tx.clone();

    let _pose_sub = tllm_node_ptr_clone.try_lock().unwrap().clone()
        .create_subscription(
            "/turtle1/pose",
            move |msg: PoseMsg| {
                send_cmd_to_llm(tx_pose.clone(), MsgType::Pose(msg));
            },
        ).unwrap();

    let action_wp_follower_cmd = tllm_node_ptr_clone.try_lock().unwrap().clone()
        .create_action_client::<ros_env::turtlesim_llm_msgs::action::FollowWaypoints>("/turtlesim_waypoint_follower/follow_cmd")?;

    let n_name = node_name.as_str().to_string();
 
    // Process user commands by llm in a separate task
    tokio::spawn(async move {
        let mut turtle_pose: Option<PoseMsg> = None;
        while let Some(data) = rx.recv().await {
            match data {
                MsgType::Cmd(StringMsg { data }) => {
                    if data.trim().is_empty() {
                        log_warn!(&n_name, "Received empty command, ignoring.");
                        continue;
                    } else {
                        println!("got user command: {}", data);
                        if !turtle_pose.is_none() {
                            let system_msg = format!(
"You are the precise path planner of the Turtlesim robot in ROS2.
Your only task is to transform the user's intent into a sequence of waypoints.

CURRENT ROBOT STATE:
- X position: {:.2} (range 0.0 to 11.0)
- Y position: {:.2} (range 0.0 to 11.0)
- Orientation (Theta): {:.2} radians (-pi to pi)

REPLY GENERATION RULES:
1. Respond ONLY with a valid JSON object that matches the schema.
2. Do not add any explanations, comments, or markdown tags (e.g., ```json).
3. If you are instructed to draw a shape, such as a triangle, always add a starting position at the end to close the shape. 
4. The movements in the sequence will be executed one after the other.

REQUIRED JSON SCHEMA:
[
    {{
        \"x\": float,    # x coordinate of the waypoint
        \"y\": float     # y coordinate of the waypoint
    }}
]",
                                turtle_pose.as_ref().unwrap().x,
                                turtle_pose.as_ref().unwrap().y,
                                turtle_pose.as_ref().unwrap().theta
                            );

                            log_debug!(&n_name, "Sending system message to LLM: {}", system_msg);

                            let messages: Vec<ChatCompletionRequestMessage> = vec![
                                ChatCompletionRequestSystemMessage::from(system_msg).into(),
                                ChatCompletionRequestUserMessage::from(data).into()
                            ];

                            let response = client.complete_chat(&messages, None).await.unwrap();

                            log_info!(&n_name, "LLM response: {}", response.choices[0].message.content.as_deref().unwrap_or(""));

                            let striped_response = strip_think(response.choices[0].message.content.as_deref().unwrap_or(""));

                            log_debug!(&n_name, "Stripped response: {}", &striped_response);

                            match serde_json::from_str::<Vec<Waypoint>>(&striped_response) {
                                Ok(points) => {
                                    log_info!(&n_name, "Correctly parsed JSON into waypoints:");
                                    for (i, point) in points.iter().enumerate() {
                                        log_info!(&n_name, "Waypoint {}: x = {}, y = {}", i, point.x, point.y);
                                    }

                                    let goal = ros_env::turtlesim_llm_msgs::action::FollowWaypoints_Goal { 
                                        waypoints: points.into_iter().map(|wp| PointMsg { x: wp.x as f64, y: wp.y as f64, z: 0.0 }).collect() };
                                
                                    if let Ok(_request) = action_wp_follower_cmd.try_request_goal(goal) {
                                        log_info!(&n_name, "Sent waypoints to the waypoint follower action server.");

                                        // let goald_id = request.goal_id().clone();

                                        // I supouse that should receive None when feedback channel is closed, but it doesn't work, so I commented it out for now.
                                        // let mut feedback = action_wp_follower_cmd.receive_feedback(goald_id.clone());
                                    
                                        // while let Some(feedback_msg) = feedback.recv().await {
                                        //     log_info!(&n_name, "Action feedback - current waypoint index: {:?}", feedback_msg.current_waypoint_index);

                                        //     std::thread::sleep(std::time::Duration::from_millis(1000));
                                        // }   
                                    } else {
                                        log_error!(&n_name, "Failed to send waypoints to the waypoint follower action server.");
                                    }
                                }
                                Err(e) => {
                                    log_error!(&n_name, "Error while parsing JSON: {}", e);
                                }
                            }
                        } else {
                            log_warn!(&n_name, "Pose is not available yet, cannot process command: {}", data);
                        }
                    }
                }
                MsgType::Pose(pose_msg) => {
                    turtle_pose = Some(pose_msg);
                }
            }
        }
    });

    log_info!(&node_name, "Turtle LLM command listener is available");

    let _cmd_sub = tllm_node_ptr_clone.try_lock().unwrap().clone()
        .create_subscription(
            "/llm_cmd",
            move |msg: StringMsg| {
                send_cmd_to_llm(tx.clone(), MsgType::Cmd(msg));
            },
        ).unwrap();

    executor.spin(SpinOptions::default()).first_error()?;

    model.unload().await?;
    Ok(())
}
