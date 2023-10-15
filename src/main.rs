use drobo_interfaces::srv::{SolenoidStateSrv, SolenoidStateSrv_Request};
use gpio_cdev::{Chip, LineRequestFlags};
use safe_drive::{context::Context, error::DynError, qos::Profile, service::client::Client};
use tokio::time::timeout;
use vl53l1x;

use core::panic;
use std::time::Duration;

#[tokio::main]
async fn main() -> Result<(), DynError> {
    let mut chip = Chip::new("/dev/gpiochip0")?;
    let front_sensor_pin = 105;
    let mid_sensor_pin = 106;
    let rear_sensor_pin = 43;

    let handle_front_sensor =
        chip.get_line(front_sensor_pin)?
            .request(LineRequestFlags::OUTPUT, 0, "distance_front")?;
    let handle_mid_sensor =
        chip.get_line(mid_sensor_pin)?
            .request(LineRequestFlags::OUTPUT, 0, "distance_mid")?;
    let handle_rear_sensor =
        chip.get_line(rear_sensor_pin)?
            .request(LineRequestFlags::OUTPUT, 0, "distance_rear")?;

    handle_rear_sensor.set_value(1)?;
    let mut vl_rear = vl53l1x::Vl53l1x::new(1, None)?;
    vl_rear.soft_reset()?;
    vl_rear.init()?;
    vl_rear.set_device_address(0x31)?;
    vl_rear.init()?;
    vl_rear.start_ranging(vl53l1x::DistanceMode::Mid)?;

    handle_mid_sensor.set_value(1)?;
    let mut vl_mid = vl53l1x::Vl53l1x::new(1, None)?;
    vl_mid.soft_reset()?;
    vl_mid.init()?;
    vl_mid.set_device_address(0x30)?;
    vl_mid.init()?;
    vl_mid.start_ranging(vl53l1x::DistanceMode::Mid)?;

    handle_front_sensor.set_value(1)?;
    let mut vl_front = vl53l1x::Vl53l1x::new(1, None)?;
    vl_front.soft_reset()?;
    vl_front.init()?;
    vl_front.start_ranging(vl53l1x::DistanceMode::Mid)?;

    let ctx = Context::new()?;
    let node = ctx.create_node("pole_detector", None, Default::default())?;
    let mut client = node.create_client::<drobo_interfaces::srv::SolenoidStateSrv>("solenoid_order", Some(Profile::default()))?;
    let mut msg = drobo_interfaces::srv::SolenoidStateSrv_Request::new().unwrap();    

    loop {
        let s_front = vl_front.read_sample()?.distance;
        let s_middle = vl_mid.read_sample()?.distance;
        let s_rear = vl_rear.read_sample()?.distance;

        if s_front < 1100 {
            // ここからタイヤの上げ下げ
            msg.axle_position = 0;
            msg.state = 1;
            client = send_solenoid_state_srv(client, &msg).await;
        }
        if s_middle < 1100 {
            msg.axle_position = 0;
            msg.state = 0;
            client = send_solenoid_state_srv(client, &msg).await;
            msg.axle_position = 1;
            msg.state = 1;
            client = send_solenoid_state_srv(client, &msg).await;
        }
        if s_rear < 1100 {
            msg.axle_position = 1;
            msg.state = 0;
            client = send_solenoid_state_srv(client, &msg).await;
            msg.axle_position = 2;
            msg.state = 1;
            client = send_solenoid_state_srv(client, &msg).await;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

async fn send_solenoid_state_srv(client: Client<SolenoidStateSrv>, msg: &SolenoidStateSrv_Request) -> Client<SolenoidStateSrv>{
    let client_rcv = client.send(&msg).unwrap();
    let mut receiver = client_rcv.recv();
    match timeout(Duration::from_secs(1), &mut receiver).await {
        Ok(Ok((c, response, _header))) => c,
        Ok(Err(e)) => panic!(),
        Err(elapsed) => receiver.give_up(),
    }
}