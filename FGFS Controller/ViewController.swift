//
//  ViewController.swift
//  FGFS Controller
//
//  Created by 李宵逸 on 2017/11/13.
//  Copyright © 2017年 李宵逸. All rights reserved.
//

import UIKit
import CoreMotion
import SwiftSocket
// import CocoaAsyncSocket

class ViewController: UIViewController {

    let motionManager = CMMotionManager()
    var timer: Timer!
    var ax: Double = -0.8
    var ay: Double =  0.2
    var az: Double = -0.4
    var mx: Double = -36
    var my: Double = -14
    var mz: Double = -14
    var aileron_zero: Double = 0.0
    var elevator_zero: Double = 0.0
    var rudder_zero: Double = 0.0
    var client: UDPClient? = nil;
    
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
    @IBOutlet weak var Mx: UILabel!
    @IBOutlet weak var My: UILabel!
    @IBOutlet weak var Mz: UILabel!
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
        ax = ax - Double(0.1 * Float(arc4random()) / Float(UINT32_MAX))
        ay = ay - Double(0.1 * Float(arc4random()) / Float(UINT32_MAX))
        az = az - Double(0.1 * Float(arc4random()) / Float(UINT32_MAX))
        mx = mx - Double(Float(arc4random()) / Float(UINT32_MAX))
        my = my - Double(Float(arc4random()) / Float(UINT32_MAX))
        mz = mz - Double(Float(arc4random()) / Float(UINT32_MAX))
        if let accelerometerData = motionManager.accelerometerData {
            ax = accelerometerData.acceleration.x
            ay = accelerometerData.acceleration.y
            az = accelerometerData.acceleration.z
        }
        if let magnetometerData = motionManager.magnetometerData {
            mx = magnetometerData.magneticField.x
            my = magnetometerData.magneticField.y
            mz = magnetometerData.magneticField.z
        }
        // unused: motionManager.gyroData
        // unused: motionManager.deviceMotion
        let vw: [Double] = [ ax,  ay,  az]  // 重力相对于手机的方向，和地面垂直
        let vn: [Double] = [ mx,  my,  mz]  // 北方
        let vi: [Double] = [1.0, 0.0, 0.0]  // x 单位向量(相对手机固定)，很少平行地面
        let vj: [Double] = [0.0, 1.0, 0.0]  // y 单位向量(相对手机固定)，很少垂直地面
        let vk: [Double] = [0.0, 0.0, 1.0]  // z 单位向量(相对手机固定)
        let pn: [Double] = v_proj(vn, vw)   // vn 在地面的投影
        let new_i: [Double] = v_hat(pn)             // pn    -> i
        let new_k: [Double] = v_hat(v_neg(vw))      // -vw   -> k
        let new_j: [Double] = v_cross(new_k, new_i) // k x i -> j
        let rel_i: [Double] = [v_dot(vi, new_i), v_dot(vi, new_j), v_dot(vi, new_k)]
        let rel_j: [Double] = [v_dot(vj, new_i), v_dot(vj, new_j), v_dot(vj, new_k)]
        let rel_k: [Double] = [v_dot(vk, new_i), v_dot(vk, new_j), v_dot(vk, new_k)]
        
        let e_angle: Double = v_angle(v_proj(vk, rel_j), rel_i, rel_j)
        let a_angle: Double = v_angle(rel_i, v_proj(vk, rel_k), rel_k)
        let r_angle: Double = v_angle(vi, v_proj(rel_j, vk), vk)
        
        // Output
        Ax.text = ax.description
        Ay.text = ay.description
        Az.text = az.description
        Mx.text = mx.description
        My.text = my.description
        Mz.text = mz.description
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
        _ = client?.send(data: data)
    }

    @objc func update() {
        let raw_data = get_raw_data()
        let a = Float(rel_angle(aileron_zero,  raw_data[0]) * 4 / Double.pi)
        let e = Float(rel_angle(elevator_zero, raw_data[1]) * 4 / Double.pi)
        let r = Float(rel_angle(rudder_zero,   raw_data[2]) * 6 / Double.pi)
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
    @IBAction func Send(_ sender: Any) {
        let frequency:Double? = Double(Frq.text!)
        let address:String? = IP_Address.text!
        let port:Int? = Int(Port.text!)
        
        let userDefaults: UserDefaults = UserDefaults.standard
        userDefaults.set(Frq.text!, forKey: "Frq")
        userDefaults.set(IP_Address.text!, forKey: "IP_Address")
        userDefaults.set(Port.text!, forKey: "Port")
        
        motionManager.startAccelerometerUpdates()
        // motionManager.startGyroUpdates()
        motionManager.startMagnetometerUpdates()
        // motionManager.startDeviceMotionUpdates()
        
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

