extern crate clap;
extern crate serialport;
extern crate csv_import_general;
#[macro_use]
extern crate rosrust;

use csv_import_general::csv_parse;

use std::io::{self, Write};
use std::time::Duration;

use clap::{App, AppSettings, Arg};
use serialport::prelude::*;

mod msg {
    rosrust::rosmsg_include!(std_msgs/Float64);
}

fn ave_calc(datas :&Vec<i32>) -> f64{
    let mut sum = 0.0;
    for item in datas{
        sum += *item as f64;
    }

    sum/(datas.len() as f64)
}

fn normal_dist_calc(data :f64,ave : f64, sigma2 :f64) -> f64{
    let tempconstnum = 1.0/((2.0*std::f64::consts::PI).sqrt()*sigma2.sqrt());
    let tempexp = -((data-ave).powf(2.0)/(2.0*sigma2));
    tempconstnum*tempexp.exp()

}

fn sensordata_filter(data:f64,sensor_data_buffer :f64,filter_param : f64) -> f64{
    (filter_param * sensor_data_buffer + (1.0 - filter_param) * data)
    
}

fn main() {
    // Initialize node
    let nodename = "plane_analyzer_node";
    rosrust::init(nodename);
    let variance_param = rosrust::param(&format!("/{}/variance",nodename).to_string()).unwrap();
    let average_param = rosrust::param(&format!("/{}/average",nodename).to_string()).unwrap();
    let plane_probs_pub = rosrust::publish("plane_probability", 10).unwrap();
    

    //read args
    let matches = App::new("Serialport Example - Receive Data")
        .about("Reads data from a serial port and echoes it to stdout")
        .setting(AppSettings::DisableVersion)
        .arg(
            Arg::with_name("port")
                .help("The device path to a serial port")
                .use_delimiter(false)
                .required(true),
        )
        .arg(
            Arg::with_name("baud")
                .help("The baud rate to connect at")
                .use_delimiter(false)
                .required(true),
        )
        .arg(
            Arg::with_name("file_path")
                .help("The file path to read parameter")
                .use_delimiter(false)
                .required(true),
        )
        .arg(
            Arg::with_name("filter_parameter")
                .help("The parameter to sensordata filter (range 0.0 < x < 1.0)")
                .use_delimiter(false)
                .required(true),
        )
        .get_matches();
    let port_name = matches.value_of("port").unwrap();
    let baud_rate = matches.value_of("baud").unwrap();
    let file_path = matches.value_of("file_path").unwrap();
    let filter_parameter = matches.value_of("filter_parameter").unwrap();

    //init normal_dist_param
    let mut datas_raw = csv_parse::read_csv_data(file_path.to_string()).unwrap();
    let mut datas : Vec<f64> = Vec::new();
    for item in &datas_raw{
        datas.push(item.get(0).unwrap().parse::<f64>().unwrap());
    }
    let sigma2 = *datas.get(0).unwrap();
    let ave = *datas.get(1).unwrap();
    
    variance_param.set(&sigma2).unwrap();
    average_param.set(&ave).unwrap();

    println!("average = {}, sigma^2 = {}",&ave,&sigma2);

    //init serial
    let mut sensor_data_buffer : f64 = 0.0;
    let mut settings: SerialPortSettings = Default::default();
    settings.timeout = Duration::from_millis(10);
    if let Ok(rate) = baud_rate.parse::<u32>() {
        settings.baud_rate = rate.into();
    } else {
        eprintln!("Error: Invalid baud rate '{}' specified", baud_rate);
        ::std::process::exit(1);
    }

    match serialport::open_with_settings(&port_name, &settings) {
        Ok(mut port) => {
            let mut serial_buf: Vec<u8> = vec![0; 1000];
            println!("Receiving data on {} at {} baud:", &port_name, &baud_rate);
            while rosrust::is_ok() {
                let mut msg = msg::std_msgs::Float64::default();
                match port.read(serial_buf.as_mut_slice()) {
                    // Ok(t) => io::stdout().write_all(&serial_buf[..t]).unwrap(),//receive
                    Ok(t) => sensor_data_buffer = sensordata_filter(String::from_utf8((&serial_buf[..t-2]).to_vec()).unwrap().parse::<f64>().unwrap(),sensor_data_buffer,filter_parameter.parse::<f64>().unwrap()),
                    Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
                    Err(e) => eprintln!("{:?}", e),
                }

                msg.data = (normal_dist_calc(sensor_data_buffer,ave,sigma2)/normal_dist_calc(ave,ave,sigma2))*100.0;
                plane_probs_pub.send(msg).unwrap();

            }
        }
        Err(e) => {
            eprintln!("Failed to open \"{}\". Error: {}", port_name, e);
            ::std::process::exit(1);
        }
    }
}
