//  
//  FGFS-Controller - Using an iOS Device to Control FGFS
//  Copyright (C) 2017  lxylxy123456
//  
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Affero General Public License as
//  published by the Free Software Foundation, either version 3 of the
//  License, or (at your option) any later version.
//  
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Affero General Public License for more details.
//  
//  You should have received a copy of the GNU Affero General Public License
//  along with this program.  If not, see <https://www.gnu.org/licenses/>.
//  

import UIKit
import CoreMotion
import CoreLocation
import SwiftSocket

class ViewController: UIViewController, UITextFieldDelegate {

    let motionManager = CMMotionManager()
    let locationManager = CLLocationManager()
    var timer: Timer!
    var ax:     Double = -0.8
    var ay:     Double =  0.2
    var az:     Double = -0.4
    var hdg:    Double =  150
    var aileron_zero:   Double = 0.0
    var elevator_zero:  Double = 0.0
    var rudder_zero:    Double = 0.0
    var aileron_factor:     Double = 1.0
    var elevator_factor:    Double = 1.0
    var rudder_factor:      Double = 1.0
    var aileron_copy:   Int = 1
    var elevator_copy:  Int = 1
    var rudder_copy:    Int = 1
    var throttle_copy:  Int = 1
    var r_angle: Double = 0
    var client: UDPClient? = nil
    var ui_text_field_obj_list: [UITextField] = []
    var ui_text_field_name_list: [String] = []
    var ui_stepper_obj_list: [UIStepper] = []
    var ui_stepper_name_list: [String] = []
    
    @IBOutlet weak var IP_Address: UITextField!
    @IBOutlet weak var Port: UITextField!
    @IBOutlet weak var Frq: UITextField!
    @IBOutlet weak var SendSwitch: UISwitch!
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
    @IBOutlet weak var Aileron_copy: UILabel!
    @IBOutlet weak var Elevator_copy: UILabel!
    @IBOutlet weak var Rudder_copy: UILabel!
    @IBOutlet weak var Throttle_copy: UILabel!
    @IBOutlet weak var Aileron_value: UILabel!
    @IBOutlet weak var Elevator_value: UILabel!
    @IBOutlet weak var Rudder_value: UILabel!
    @IBOutlet weak var Throttle_value: UILabel!
    @IBOutlet weak var Aileron_switch: UISwitch!
    @IBOutlet weak var Elevator_switch: UISwitch!
    @IBOutlet weak var Rudder_switch: UISwitch!
    @IBOutlet weak var Aileron_stepper: UIStepper!
    @IBOutlet weak var Elevator_stepper: UIStepper!
    @IBOutlet weak var Rudder_stepper: UIStepper!
    @IBOutlet weak var Throttle_stepper: UIStepper!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view, typically from a nib.
        ui_text_field_obj_list = [IP_Address, Port, Frq, Aileron_factor, Elevator_factor, Rudder_factor]
        ui_text_field_name_list = ["IP_Address", "Port", "Frq", "Aileron_factor", "Elevator_factor", "Rudder_factor"]
        ui_stepper_obj_list = [Aileron_stepper, Elevator_stepper, Rudder_stepper, Throttle_stepper]
        ui_stepper_name_list = ["Aileron_stepper", "Elevator_stepper", "Rudder_stepper", "Throttle_stepper"]
        let ui_stepper_ind_list = [AileronStep, ElevatorStep, RudderStep, ThrottleStep]
        
        let UD: UserDefaults = UserDefaults.standard
        for i in 0..<ui_text_field_obj_list.count {
            ui_text_field_obj_list[i].delegate = self
            if let ans = UD.string(forKey: ui_text_field_name_list[i]) {
                ui_text_field_obj_list[i].text = ans
            }
        }
        for i in 0..<ui_stepper_obj_list.count {
            if let ans = UD.string(forKey: ui_stepper_name_list[i]) {
                ui_stepper_obj_list[i].value = Double(ans)!
                ui_stepper_ind_list[i](ui_stepper_obj_list[i])
            }
        }
        
        Throttle.transform = CGAffineTransform(rotationAngle: -CGFloat.pi/2.0)
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
            let angle: Double = acos(v_dot(a, b) / v_abs(a) / v_abs(b))
            if v_dot(v_cross(a, b), c) > 0 {
                return angle
            }
            else {
                return -angle
            }
        }
        var vw: [Double] = [0.0, 0.0, 0.9]  // 重力相对于手机的方向，和地面垂直
        let vi: [Double] = [1.0, 0.0, 0.0]  // x 单位向量(相对手机固定)，很少平行地面
        let vj: [Double] = [0.0, 1.0, 0.0]  // y 单位向量(相对手机固定)，很少垂直地面
        let vk: [Double] = [0.0, 0.0, 1.0]  // z 单位向量(相对手机固定)
        if let motionData = motionManager.deviceMotion {
            vw = [motionData.gravity.x, motionData.gravity.y, motionData.gravity.z]
        }
        else if let accelerometerData = motionManager.accelerometerData {
            vw = [accelerometerData.acceleration.x, accelerometerData.acceleration.y, accelerometerData.acceleration.z]
        }
        if let headingData = locationManager.heading {
            hdg = headingData.magneticHeading
        }
        
        // let e_angle: Double = v_angle(v_proj(v_neg(vw), vj), vi, vj)
        let a_angle: Double = v_angle(vi, v_proj(v_neg(vw), vk), vk)
        let e_angle: Double = v_angle(v_proj(v_neg(vw), vj), vk, vj)
        var r_angle: Double = -(hdg * Double.pi / 180)
        r_angle = r_angle + Double(atan((sin(a_angle) * cos(e_angle + Double.pi / 2)) / cos(a_angle)))
        
        // Output
        Ax.text = Float(vw[0]).description
        Ay.text = Float(vw[1]).description
        Az.text = Float(vw[2]).description
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
        var copy_number: [Int] = [aileron_copy, elevator_copy, rudder_copy, throttle_copy]

        var data = Data(count: 0)
        for i in 0...3 {
            output_ui_obj[i].text = variables[i].description
            for _ in 0..<copy_number[i] {
                data.append(Data(Data(buffer: UnsafeBufferPointer(start: &variables[i], count: 1)).reversed())) // 通过 reversed 得到 big-endian 的结果
            }
        }
        Info.text = data.base64EncodedString()
        if let result: Result? = client?.send(data: data){
            let desc = result?.error.debugDescription
            if desc != "nil" {
                Info.text = desc
            }
        }
    }

    @objc func update() {
        let raw_data = get_raw_data()
        var a = Float(rel_angle(aileron_zero,  raw_data[0]) * 4 / Double.pi * aileron_factor)
        var e = Float(rel_angle(elevator_zero, raw_data[1]) * 4 / Double.pi * elevator_factor)
        var r = Float(rel_angle(rudder_zero,   raw_data[2]) * 4 / Double.pi * rudder_factor)
        if Aileron_switch.isOn {
            Aileron.value = a
        }
        else {
            a = Aileron.value
        }
        if Elevator_switch.isOn {
            Elevator.value = e
        }
        else {
            e = Elevator.value
        }
        if Rudder_switch.isOn {
            Rudder.value = r
        }
        else {
            r = Rudder.value
        }
        let t = Throttle.value
        udp_send(aileron: a, elevator: e, rudder: r, throttle: t)
    }

    @IBAction func AileronZero(_ sender: Any) {
        let raw_data = get_raw_data()
        aileron_zero = raw_data[0]
        Aileron.value = 0
    }
    @IBAction func ElevatorZero(_ sender: Any) {
        let raw_data = get_raw_data()
        elevator_zero = raw_data[1]
        Elevator.value = 0
    }
    @IBAction func RudderZero(_ sender: Any) {
        let raw_data = get_raw_data()
        rudder_zero = raw_data[2]
        Rudder.value = 0
    }
    @IBAction func ThrottleZero(_ sender: Any) {
        UIApplication.shared.open(URL(string: "https://github.com/lxylxy123456/FGFS-controller/")!)
    }
    @IBAction func AileronStep(_ sender: Any) {
        Aileron_copy.text = Int(Aileron_stepper.value).description
    }
    @IBAction func ElevatorStep(_ sender: Any) {
        Elevator_copy.text = Int(Elevator_stepper.value).description
    }
    @IBAction func RudderStep(_ sender: Any) {
        Rudder_copy.text = Int(Rudder_stepper.value).description
    }
    @IBAction func ThrottleStep(_ sender: Any) {
        Throttle_copy.text = Int(Throttle_stepper.value).description
    }
    @IBAction func Send_or_Stop(_ sender: UISwitch) {
        if (SendSwitch.isOn) {
            let frequency:Double? = Double(Frq.text!)
            let address:String? = IP_Address.text!
            let port:Int? = Int(Port.text!)
            
            let UD: UserDefaults = UserDefaults.standard
            
            for i in 0..<ui_text_field_obj_list.count {
                UD.set(ui_text_field_obj_list[i].text!, forKey: ui_text_field_name_list[i])
            }
            for i in 0..<ui_stepper_obj_list.count {
                UD.set(ui_stepper_obj_list[i].value.description, forKey: ui_stepper_name_list[i])
            }
            
            aileron_factor  = Double(Aileron_factor.text!)!
            elevator_factor = Double(Elevator_factor.text!)!
            rudder_factor   = Double(Rudder_factor.text!)!
            aileron_copy    = Int(Aileron_stepper.value)
            elevator_copy   = Int(Elevator_stepper.value)
            rudder_copy     = Int(Rudder_stepper.value)
            throttle_copy   = Int(Throttle_copy.text!)!
            
            if aileron_copy > 0 || elevator_copy > 0 {
                motionManager.startAccelerometerUpdates()
                motionManager.startDeviceMotionUpdates()
            }
            if aileron_copy == 0 {  Aileron_switch.isOn = false }
            if elevator_copy == 0 { Elevator_switch.isOn = false }
            if rudder_copy > 0 {
                locationManager.startUpdatingHeading()
            }
            else {
                Rudder_switch.isOn = false
            }
            
            if timer != nil {
                timer.invalidate()
                client?.close()
            }
            
            client = UDPClient(address: address!, port: Int32(port!))
            
            timer = Timer.scheduledTimer(timeInterval: 1.0 / frequency!, target: self, selector: #selector(ViewController.update), userInfo: nil, repeats: true)
            UIApplication.shared.isIdleTimerDisabled = true
        }
        else {
            if timer != nil {
                timer.invalidate()
                client?.close()
            }
            UIApplication.shared.isIdleTimerDisabled = false
        }
    }
}

