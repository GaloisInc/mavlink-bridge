//! Bridge betwwen multiple instances of PX4 simulator connected over separete UDP channels, and
//! ZMQ bus which is connected to the async task manager.
//! 
//! Sample usage for a single instance: `cargo run -- 1 tcp://127.0.0.1:5560 5561 -d`
//! 
//! To run multiple instances (which can be computationally heavy), do: 
//! - `/Firmware$ ./Tools/sitl_multiple_run.sh $n` where $n is a number of instances of PX4 autopilot
//! - for the first instance, do: `/Firmware$ ./Tools/jmavsim_run.sh -r 250`
//! - for each additional instance, do: `Firmware$ ./Tools/jmavsim_run.sh -p $(4560+k) -r 250`
//! where $k=1..$n. The first instance connects to the simulator at 4560, the second one
//! at 4561, the third at 4562 etc.
//! - start QGroundControl with `./QGroundControl.AppImage`
//! And then start mavlink bridge with the correct number of instances. Use `-d` for debugging.
//!
//! Note: -r flag for the simulator specifies run frequency, and the simulator doesn't seem
//! to run otherwise.
//! 
//! `5560` is the default port to receive ZMQ messages from (TX for the Task manager)
//! and `5561` is the default port to send ZMQ messages to (RX for the Task manager).
//! The RX port (5561) increments for each instance, because we have to bind each local
//! publisher to a new port.
//! 
//! Then start the task manager and make sure that it gets initilized to read from 
//! all Mavling bridge ZMQ ports (in this case 5561,5562,...5560+k; k=1..n)
//! 
//! *TODO:* Script this, so it is easier to start the multi-uav simulation
//! *TODO:* Make it worth with a remote machine (configurable IP)
#[macro_use]
extern crate clap;

use std::sync::Arc;
use std::thread;

use clap::App;

const BASE_DEVICE_ADDR: &str = "udpin:127.0.0.1";
const MAVLINK_BASE_PORT: usize = 14540;
const BASE_PUB_ADDR: &str = "tcp://127.0.0.1";

fn main() {
    // The YAML file is found relative to the current file, similar to how modules are found
    let yaml = load_yaml!("../cli.yml");
    let matches = App::from_yaml(yaml).get_matches();

    let num_connections: usize = matches.value_of("NUM_CONNECTIONS").unwrap().parse().unwrap();
    let base_port_pub: usize = matches.value_of("ZMQ_BASE_PORT_PUB").unwrap().parse().unwrap();
    let addr_sub = Arc::new(matches.value_of("ZMQ_ADDR_SUB").unwrap());

    let context = zmq::Context::new();

    let mut handles = vec![];
    for n in 0..num_connections {
        let device = BASE_DEVICE_ADDR.to_string() + ":" + &(MAVLINK_BASE_PORT+n).to_string();
        println!("#{}: Mavlink connecting to {}", n, device);
        let vehicle = Arc::new(mavlink::connect(&device).unwrap());

        let subscriber = context.socket(zmq::SUB).unwrap();
        let publisher = context.socket(zmq::PUB).unwrap();
        let addr_pub = BASE_PUB_ADDR.to_string() + ":" + &(base_port_pub + n).to_string();

        // ZMQ -> Mavlink thread
        // Rx data from ZMQ, TX data to Mavlink device
        let t = thread::spawn({
            let vehicle = vehicle.clone();
            let addr_sub = addr_sub.clone();
            println!("Subscriber {}: connecting to {}",n,addr_sub);
            subscriber.connect(&addr_sub).unwrap();
            assert!(subscriber.set_subscribe("".as_bytes()).is_ok());

            move || loop {
                let stream = subscriber.recv_bytes(0).unwrap();
                let frame = mavlink::MavFrame::deser(&stream).unwrap();
                vehicle.send_frame(&frame).unwrap();
            }
        });
        handles.push(t);

        // Mavlink -> Zmq thread
        // Rx data from Mavlink, TX data to ZMQ
        let t = thread::spawn({
            let debug = matches.is_present("debug");
            println!("Publisher {}: connecting to {}",n,addr_pub);
            publisher.bind(&addr_pub).unwrap();

            move || loop {
                if let Ok(frame) = vehicle.recv_frame() {
                    if debug {
                        println!("{:?}", frame);
                    }

                    publisher.send(&frame.ser(), 0).unwrap(); // send &w with 0 flags
                }
            }
        });
        handles.push(t);
    }
    println!("Running...");
    for handle in handles {
        handle.join().unwrap();
    };
}