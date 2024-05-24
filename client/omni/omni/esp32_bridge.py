#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
import json
import serial
from std_msgs.msg import Float32MultiArray
import time

class Esp32_Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        self.pub_rpm_all = self.create_publisher(Float32MultiArray, 'rpm_all', 10)
        self.rpm_all = Float32MultiArray()
        self.timer = self.create_timer(0.05, self.pub_tick_callback)
        
        # Parameters
        self.PPR = 330
        
        # RPM counter parameters
        self.prevPFL = 0
        self.prevPFR = 0
        self.prevPRL = 0
        self.prevPRR = 0
        self.prevRPM_time = int(time.time() * 1000)  # millisecond
        
        self.measuredRPM_FL = 0.0
        self.measuredRPM_FR = 0.0
        self.measuredRPM_RL = 0.0
        self.measuredRPM_RR = 0.0
        
        # Serial port
        self.esp32 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    
    def pub_tick_callback(self):
        self.rpm_all.data = [
            self.measuredRPM_FL, 
            self.measuredRPM_FR, 
            self.measuredRPM_RL, 
            self.measuredRPM_RR
        ]
        self.pub_rpm_all.publish(self.rpm_all)
        self.send_json_and_receive_response()
        
    def send_json_and_receive_response(self):
        response = self.esp32.readline().decode('utf-8').strip()
        try:
            response_data = json.loads(response)
            print("Received response:")
            print(json.dumps(response_data, indent=2))

            # Access specific keys from the JSON object
            if 'FL_cFL' in response_data:
                realP_FL = response_data['FL_cFL']
                
            if 'FR_cFR' in response_data:
                realP_FR = response_data['FR_cFR']

            if 'RL_cRL' in response_data:
                realP_RL = response_data['RL_cRL']
                
            if 'RR_cRR' in response_data:
                realP_RR = response_data['RR_cRR']    
                        
            self.measureTick_FL = realP_FL
            self.measureTick_FR = realP_FR
            self.measureTick_RL = realP_RL
            self.measureTick_RR = realP_RR
            
            print(f"realP_FL : {realP_FL}, realP_FR : {realP_FR}, realP_RL : {realP_RL}, realP_RR : {realP_RR}")
            self.counterRPM(realP_FL, realP_FR, realP_RL, realP_RR)
            
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON response: {e}")
        
    def counterRPM(self, inpFL, inpFR, inpRL, inpRR):
        # Implement your RPM counter logic here...
        curPFL = inpFL
        curPFR = inpFR
        curPRL = inpRL
        curPRR = inpRR
        curRPM_time = int(time.time() * 1000)  # millisecond
        diffRPM_time = curRPM_time - self.prevRPM_time 
        RPM_Fl = round((((curPFL - self.prevPFL) / self.PPR) / (diffRPM_time * 0.001)) * 60, 2)  # RPM
        RPM_Fr = round((((curPFR - self.prevPFR) / self.PPR) / (diffRPM_time * 0.001)) * 60, 2)
        RPM_Rl = round((((curPRL - self.prevPRL) / self.PPR) / (diffRPM_time * 0.001)) * 60, 2)
        RPM_Rr = round((((curPRR - self.prevPRR) / self.PPR) / (diffRPM_time * 0.001)) * 60, 2)
        
        self.prevPFL = curPFL
        self.prevPFR = curPFR
        self.prevPRL = curPRL
        self.prevPRR = curPRR
        
        self.prevRPM_time = curRPM_time
        print(f'rpmFL : {RPM_Fl}, rpmFR : {RPM_Fr}, rpmRL : {RPM_Rl}, rpmRR : {RPM_Rr}')
        
        self.measuredRPM_FL = float(RPM_Fl)
        self.measuredRPM_FR = float(RPM_Fr)
        self.measuredRPM_RL = float(RPM_Rl)
        self.measuredRPM_RR = float(RPM_Rr)

def main(args=None):
    rclpy.init(args=args)
    esp32_bridge = Esp32_Bridge()
    print("esp32_bridge node started")
    try:
        rclpy.spin(esp32_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        esp32_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
