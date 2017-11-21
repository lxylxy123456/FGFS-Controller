//
//  ViewController.swift
//  FGFS Controller
//
//  Created by 李宵逸 on 2017/11/13.
//  Copyright © 2017年 李宵逸. All rights reserved.
//

import UIKit
import CoreMotion
import CoreLocation
import SwiftSocket

class ViewController: UIViewController, UITextFieldDelegate {

    let motionManager = CMMotionManager()
    let locationManager = CLLocationManager()
    var timer: Timer!
    var ax: Double = -0.8
    var ay: Double =  0.2
    var az: Double = -0.4
    var hdg: Double = 1.5
    var aileron_zero: Double = 0.0
    var elevator_zero: Double = 0.0
    var rudder_zero: Double = 0.0
    var client: UDPClient? = nil
    
    @IBOutlet weak var IP_Address: UITextField!
    @IBOutlet weak var Port: UITextField!
    @IBOutlet weak var Frq: UITextField!
    @IBOutlet weak var Aileron: UISlider!
    @IBOutlet weak var Elevator: UISlider!
    @IBOutlet weak var Rudder: UISlider!
    @IBOutlet weak var Throttle: UISlider!
    @IBOutlet weak var Ax: UILabel!
    @IBOutlet weak var Ay: UILabel!
    @IBOutlet weak var Az: UILabel!
    @IBOutlet weak var Hdg: UILabel!
    @IBOutlet weak var Info: UITextView!
    @IBOutlet weak var Aileron_factor: UITextField!
    @IBOutlet weak var Elevator_factor: UITextField!
    @IBOutlet weak var Rudder_factor: UITextField!
    @IBOutlet weak var Aileron_value: UILabel!
    @IBOutlet weak var Elevator_value: UILabel!
    @IBOutlet weak var Rudder_value: UILabel!
    @IBOutlet weak var Throttle_value: UILabel!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view, typically from a nib.
        func get_string(userDefaults: UserDefaults, forKey: String, default_val: String) -> String {
            let ans = userDefaults.string(forKey: forKey)
            if ans == nil {
                return default_val
            }
            else {
                return ans!
            }
        }
        Throttle.transform = CGAffineTransform(rotationAngle: -CGFloat.pi/2.0)
        
        let userDefaults: UserDefaults = UserDefaults.standard
        Frq.text = get_string(userDefaults: userDefaults, forKey: "Frq", default_val: "12")
        IP_Address.text = get_string(userDefaults: userDefaults, forKey: "IP_Address", default_val: "10.100.0.10")
        Port.text = get_string(userDefaults: userDefaults, forKey: "Port", default_val: "6789")
        Aileron_factor.text = get_string(userDefaults: userDefaults, forKey: "Aileron_factor", default_val: "1.0")
        Elevator_factor.text = get_string(userDefaults: userDefaults, forKey: "Elevator_factor", default_val: "1.0")
        Rudder_factor.text = get_string(userDefaults: userDefaults, forKey: "Rudder_factor", default_val: "1.5")
        
        IP_Address.delegate = self
        Port.delegate = self
        Frq.delegate = self
    }

    func textFieldShouldReturn(_ textField: UITextField) -> Bool{
        self.view.endEditing(true)
        return false
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }

    @objc func get_raw_data() -> [Double] {
        func v_neg(_ a: [Double]) -> [Double] {
            return [-a[0], -a[1], -a[2]]
        }
        func v_times(_ a: [Double], _ b: Double) -> [Double] {
            return [a[0] * b, a[1] * b, a[2] * b]
        }
        func v_plus(_ a: [Double], _ b: [Double]) -> [Double] {
            return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
        }
        func v_minus(_ a: [Double], _ b: [Double]) -> [Double] {
            return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
        }
        func v_dot(_ a: [Double], _ b: [Double]) -> Double {
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
        }
        func v_cross(_ a: [Double], _ b: [Double]) -> [Double] {
            return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
        }
        func v_abs(_ a: [Double]) -> Double {
            return sqrt(v_dot(a, a))
        }
        func v_hat(_ a: [Double]) -> [Double] {
            return v_times(a, 1.0 / v_abs(a))
        }
        func v_proj(_ a: [Double], _ n: [Double]) -> [Double] {
            // 将向量 a 投影到和 n 垂直的平面上
            // a_vec - n_vec * ((n_vec * a_vec) / (n_vec * n_vec))
            return v_minus(a, v_times(n, v_dot(n, a) / v_dot(n, n)))
        }
        func v_angle(_ a: [Double], _ b: [Double], _ c: [Double]) -> Double {
            // 从 a 到 b 按 c 进行右手定则旋转的角度，返回 [-Double.pi, Double.pi)
            // 需要保证 a 和 b 都和 c 垂直
            let angle: Double = acos(v_dot(a, b) / v_abs(a) / v_abs(b))
            if v_dot(v_cross(a, b), c) > 0 {
                return angle
            }
            else {
                return -angle
            }
        }
        if let headingData = locationManager.heading {
            hdg = headingData.magneticHeading
        }
        if let accelerometerData = motionManager.accelerometerData {
            ax = accelerometerData.acceleration.x
            ay = accelerometerData.acceleration.y
            az = accelerometerData.acceleration.z
        }
        let vw: [Double] = [ ax,  ay,  az]  // 重力相对于手机的方向，和地面垂直
        let vi: [Double] = [1.0, 0.0, 0.0]  // x 单位向量(相对手机固定)，很少平行地面
        let vj: [Double] = [0.0, 1.0, 0.0]  // y 单位向量(相对手机固定)，很少垂直地面
        let vk: [Double] = [0.0, 0.0, 1.0]  // z 单位向量(相对手机固定)
        
        let e_angle: Double = v_angle(v_proj(v_neg(vw), vj), vi, vj)
        let a_angle: Double = v_angle(vi, v_proj(v_neg(vw), vk), vk)
        let r_angle: Double = -(hdg * Double.pi / 180)
        
        // Output
        Ax.text = Float(ax).description
        Ay.text = Float(ay).description
        Az.text = Float(az).description
        Hdg.text = Float(hdg).description
        return [a_angle, e_angle, r_angle]
    }

    @objc func rel_angle(_ a: Double, _ b: Double) -> Double {
        // a 转到 b 的相对角度，返回 [-Double.pi, Double.pi)
        var answer: Double = b - a
        while answer >= Double.pi {
            answer -= 2 * Double.pi
        }
        while answer < -Double.pi {
            answer += 2 * Double.pi
        }
        return answer
    }

    @objc func udp_send(aileron: Float, elevator: Float, rudder: Float, throttle: Float) {
        var variables: [Float] = [aileron, elevator, rudder, throttle]
        var output_ui_obj: [UILabel] = [Aileron_value, Elevator_value, Rudder_value, Throttle_value]
        var data = Data(count: 0)
        for i in 0...3 {
            output_ui_obj[i].text = variables[i].description
            data.append(Data(Data(buffer: UnsafeBufferPointer(start: &variables[i], count: 1)).reversed())) // 通过 reversed 得到 big-endian 的结果
        }
        if let result: Result? = client?.send(data: data){
            Info.text = result?.error.debugDescription
        }
    }

    @objc func update() {
        let raw_data = get_raw_data()
        let a = Float(rel_angle(aileron_zero,  raw_data[0]) * 4 / Double.pi * Double(Aileron_factor.text!)!)
        let e = Float(rel_angle(elevator_zero, raw_data[1]) * 4 / Double.pi * Double(Elevator_factor.text!)!)
        let r = Float(rel_angle(rudder_zero,   raw_data[2]) * 4 / Double.pi * Double(Rudder_factor.text!)!)
        Aileron.value = a
        Elevator.value = e
        Rudder.value = r
        let t = Throttle.value
        udp_send(aileron: a, elevator: e, rudder: r, throttle: t)
    }

    @IBAction func AileronZero(_ sender: Any) {
        let raw_data = get_raw_data()
        aileron_zero = raw_data[0]
    }
    @IBAction func ElevatorZero(_ sender: Any) {
        let raw_data = get_raw_data()
        elevator_zero = raw_data[1]
    }
    @IBAction func RudderZero(_ sender: Any) {
        let raw_data = get_raw_data()
        rudder_zero = raw_data[2]
    }
    @IBAction func ThrottleZero(_ sender: Any) {
        UIApplication.shared.open(URL(string: "https://github.com/lxylxy123456/FGFS-controller/")!)
    }
    @IBAction func Send(_ sender: Any) {
        let frequency:Double? = Double(Frq.text!)
        let address:String? = IP_Address.text!
        let port:Int? = Int(Port.text!)
        
        let userDefaults: UserDefaults = UserDefaults.standard
        userDefaults.set(Frq.text!, forKey: "Frq")
        userDefaults.set(IP_Address.text!, forKey: "IP_Address")
        userDefaults.set(Port.text!, forKey: "Port")
        userDefaults.set(Aileron_factor.text!, forKey: "Aileron_factor")
        userDefaults.set(Elevator_factor.text!, forKey: "Elevator_factor")
        userDefaults.set(Rudder_factor.text!, forKey: "Rudder_factor")
        
        motionManager.startAccelerometerUpdates()
        motionManager.startGyroUpdates()
        // motionManager.startMagnetometerUpdates()
        motionManager.startDeviceMotionUpdates()
        locationManager.startUpdatingHeading()
        
        if timer != nil {
            timer.invalidate()
            client?.close()
        }
        
        client = UDPClient(address: address!, port: Int32(port!))
        
        timer = Timer.scheduledTimer(timeInterval: 1.0 / frequency!, target: self, selector: #selector(ViewController.update), userInfo: nil, repeats: true)
    }
    @IBAction func Stop(_ sender: Any) {
        if timer != nil {
            timer.invalidate()
            client?.close()
        }
    }
    
}

