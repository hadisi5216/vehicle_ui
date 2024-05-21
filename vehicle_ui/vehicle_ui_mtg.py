"""
门头沟中关村园区观光车控制系统
"""

import sys
import threading
import time
import subprocess

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from ui.vehicle_mtg_ui import Ui_MainWindow

import rclpy
from rclpy.node import Node

from autoware_adapi_v1_msgs.msg import OperationModeState,RouteState,LocalizationInitializationState,MotionState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode,SetRoutePoints,ClearRoute
from geometry_msgs.msg import Pose

class ActNode(Node, QObject):
    signal_operation_mode = pyqtSignal(object)
    signal_route = pyqtSignal(int)
    signal_initialization = pyqtSignal(int)
    signal_motion = pyqtSignal(int)
    signal_result_station = pyqtSignal(int,object)
    signal_result_control = pyqtSignal(object)
    
    def __init__(self,name):
        super().__init__(name)
        QObject.__init__(self)  
        self.get_logger().info(f"__init__")
        
        self.state_operation_mode               = self.create_subscription(OperationModeState,"/api/operation_mode/state",self.state_operation_mode_callback,10) 
        self.state_route                        = self.create_subscription(RouteState,"/api/routing/state",self.state_route_callback,10) 
        self.state_initialization               = self.create_subscription(LocalizationInitializationState,"/api/localization/initialization_state",self.state_localization_callback,10) 
        self.state_motion                       = self.create_subscription(MotionState,"/api/motion/state",self.state_motion_callback,10) 

        self.client_operation_mode_auto         = self.create_client(ChangeOperationMode,"/api/operation_mode/change_to_autonomous")
        self.client_operation_mode_stop         = self.create_client(ChangeOperationMode,"/api/operation_mode/change_to_stop")
        self.client_operation_mode_local        = self.create_client(ChangeOperationMode,"/api/operation_mode/change_to_local")
        self.client_operation_mode_remote       = self.create_client(ChangeOperationMode,"/api/operation_mode/change_to_remote")
        self.client_operation_mode_enable       = self.create_client(ChangeOperationMode,"/api/operation_mode/enable_autoware_control")
        self.client_operation_mode_disable      = self.create_client(ChangeOperationMode,"/api/operation_mode/disable_autoware_control")
        self.client_operation_mode_disable      = self.create_client(ChangeOperationMode,"/api/operation_mode/disable_autoware_control")
        self.client_routing_set_route_points    = self.create_client(SetRoutePoints,"/api/routing/set_route_points")
        self.client_routing_clear_route         = self.create_client(ClearRoute,"/api/routing/clear_route")
  
    # 操作模式状态 mode：UNKNOWN = 0 | STOP = 1 | AUTONOMOUS = 2 | LOCAL = 3 | REMOTE = 4
    def state_operation_mode_callback(self, msg):
        self.get_logger().info(f"/api/operation_mode/state: {msg.mode}")
        self.signal_operation_mode.emit(msg)        
     
    # Routing状态 state：UNKNOWN = 0 | UNSET = 1 | SET = 2 | ARRIVED = 3 | CHANGING = 4
    def state_route_callback(self, msg):
        self.get_logger().info(f"/api/routing/state: {msg.state}")
        self.signal_route.emit(msg.state)
     
    # Localization状态 state：UNKNOWN = 0 | UNINITIALIZED = 1 | INITIALIZING = 2 | INITIALIZED = 3
    def state_localization_callback(self, msg):
        self.get_logger().info(f"/api/localization/initialization_state: {msg.state}")
        self.signal_initialization.emit(msg.state)
        
    # Motion车辆运动状态 state：UNKNOWN = 0 | STOPPED = 1 | STARTING = 2 | MOVING = 3
    def state_motion_callback(self, msg):
        self.get_logger().info(f"/api/motion/state: {msg.state}")
        self.signal_motion.emit(msg.state)
    
    # 改变操作模式    
    def change_operation_mode(self, key):
        self.get_logger().info(f"change_operation_mode: {key}")
        if(key == "auto"):
            while rclpy.ok() and self.client_operation_mode_auto.wait_for_service(1)==False:
                pass
            request = ChangeOperationMode.Request()
            self.client_operation_mode_auto.call_async(request).add_done_callback(self.result_callback)
        elif(key == "stop"):
            while rclpy.ok() and self.client_operation_mode_stop.wait_for_service(1)==False:
                pass
            request = ChangeOperationMode.Request()
            self.client_operation_mode_stop.call_async(request).add_done_callback(self.result_callback)
        elif(key == "local"):
            while rclpy.ok() and self.client_operation_mode_local.wait_for_service(1)==False:
                pass
            request = ChangeOperationMode.Request()
            self.client_operation_mode_local.call_async(request).add_done_callback(self.result_callback)
        elif(key == "remote"):
            while rclpy.ok() and self.client_operation_mode_remote.wait_for_service(1)==False:
                pass
            request = ChangeOperationMode.Request()
            self.client_operation_mode_remote.call_async(request).add_done_callback(self.result_callback)
        elif(key == "enable"):
            while rclpy.ok() and self.client_operation_mode_enable.wait_for_service(1)==False:
                pass
            request = ChangeOperationMode.Request()
            self.client_operation_mode_enable.call_async(request).add_done_callback(self.result_callback)
        elif(key == "disable"):
            while rclpy.ok() and self.client_operation_mode_disable.wait_for_service(1)==False:
                pass
            request = ChangeOperationMode.Request()
            self.client_operation_mode_disable.call_async(request).add_done_callback(self.result_callback)
             
    # 清除router        
    def clear_router(self):
        while rclpy.ok() and self.client_routing_clear_route.wait_for_service(1)==False:
            pass
        request = ClearRoute.Request()
        self.client_routing_clear_route.call_async(request).add_done_callback(self.result_callback)
            
    # 切换站点      
    def change_station(self, stationId):
        while rclpy.ok() and self.client_routing_set_route_points.wait_for_service(1)==False:
            self.get_logger().info(f"Wait for the server to go online....")
        request = SetRoutePoints.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = "map"
        request.option.allow_goal_modification = False
        goal = Pose()
        self.stationId = stationId
        if(stationId == 1):
            goal.position
            goal.position.x = 36.7334
            goal.position.y = 51.8838
            goal.position.z = 0.0
            goal.orientation.x = 0.0
            goal.orientation.y = 0.0
            goal.orientation.z = 0.0920319
            goal.orientation.w = 0.995756
        elif(stationId == 2):
            goal.position
            goal.position.x = 29.6334
            goal.position.y = 36.9838
            goal.position.z = 0.0
            goal.orientation.x = 0.0
            goal.orientation.y = 0.0
            goal.orientation.z = -0.625727
            goal.orientation.w = 0.780042
        elif(stationId == 3):
            goal.position
            goal.position.x = 87.0334
            goal.position.y = 15.9838
            goal.position.z = 0.0
            goal.orientation.x = 0.0
            goal.orientation.y = 0.0
            goal.orientation.z = -0.63399
            goal.orientation.w = 0.773342
        elif(stationId == 4):
            goal.position
            goal.position.x = -7.7666
            goal.position.y = -22.5162
            goal.position.z = 0.0
            goal.orientation.x = 0.0
            goal.orientation.y = 0.0
            goal.orientation.z = 0.751445
            goal.orientation.w = 0.659795
        request.goal = goal
        self.client_routing_set_route_points.call_async(request).add_done_callback(self.result_callback_station)
    
    # client请求结果    
    def result_callback(self, result_future):
        response = result_future.result()
        self.get_logger().info(f"收到返回结果：{response}")
        self.signal_result_control.emit(response.status)
    
    # 站点client请求结果    
    def result_callback_station(self, result_future):
        response = result_future.result()
        self.get_logger().info(f"收到返回结果：{self.stationId}--{response}")
        self.signal_result_station.emit(self.stationId, response.status)
        
        
class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, node):
        super(MainWindow, self).__init__()
        self.node = node
        self.setupUi(self)
        
        self.color_green = "background-color: rgb(51, 209, 122);"
        self.color_orange = "background-color: rgb(255, 120, 0);"
        self.color_yellow = "background-color: rgb(249, 240, 107);"
        self.color_white = "background-color: rgb(255, 255, 255);"
        
        node.signal_operation_mode.connect(self.set_opration_mode_state)
        node.signal_route.connect(self.set_route_state)
        node.signal_initialization.connect(self.set_initialization_state)
        node.signal_motion.connect(self.set_motion_state)
        node.signal_result_station.connect(self.set_station_state)
        node.signal_result_control.connect(self.set_logging_state)
        
        self.pushButton_auto.clicked.connect(lambda: self.node.change_operation_mode("auto"))
        self.pushButton_stop.clicked.connect(lambda: self.node.change_operation_mode("stop")) 
        self.pushButton_local.clicked.connect(lambda: self.node.change_operation_mode("local")) 
        self.pushButton_remote.clicked.connect(lambda: self.node.change_operation_mode("remote")) 
        self.pushButton_enable.clicked.connect(lambda: self.node.change_operation_mode("enable")) 
        self.pushButton_disable.clicked.connect(lambda: self.node.change_operation_mode("disable")) 
        
        self.pushButton_station_1.clicked.connect(lambda: self.node.change_station(1)) 
        self.pushButton_station_2.clicked.connect(lambda: self.node.change_station(2)) 
        self.pushButton_station_3.clicked.connect(lambda: self.node.change_station(3)) 
        self.pushButton_station_4.clicked.connect(lambda: self.node.change_station(4)) 
        
        self.pushButton_clear_router.clicked.connect(lambda: self.node.clear_router()) 
        
       
    # 设置OperationMode按钮及标签状态 
    def set_opration_mode_state(self, msg):
        self.pushButton_auto.setEnabled(msg.is_autonomous_mode_available) 
        self.pushButton_stop.setEnabled(msg.is_stop_mode_available) 
        self.pushButton_local.setEnabled(msg.is_local_mode_available) 
        self.pushButton_remote.setEnabled(msg.is_remote_mode_available) 
        self.pushButton_enable.setEnabled(not msg.is_autoware_control_enabled) 
        self.pushButton_disable.setEnabled(msg.is_autoware_control_enabled) 
        
        mode = msg.mode
        if(mode == 0):
            self.label_state_operation_mode.setText("UNKNOWN")
            self.label_state_operation_mode.setStyleSheet(self.color_yellow)
        elif(mode == 1):
            self.label_state_operation_mode.setText("STOP")
            self.label_state_operation_mode.setStyleSheet(self.color_orange)
            self.pushButton_stop.setEnabled(False) 
        elif(mode == 2):
            self.label_state_operation_mode.setText("AUTONOMOUS")
            self.label_state_operation_mode.setStyleSheet(self.color_green)
            self.pushButton_auto.setEnabled(False)
        elif(mode == 3):
            self.label_state_operation_mode.setText("LOCAL")
            self.label_state_operation_mode.setStyleSheet(self.color_yellow)
            self.pushButton_local.setEnabled(False)
        elif(mode == 4):
            self.label_state_operation_mode.setText("REMOTE")
            self.label_state_operation_mode.setStyleSheet(self.color_yellow)
            self.pushButton_remote.setEnabled(False)
        self.label_state_operation_mode.setAlignment(Qt.AlignCenter)
        
        # 设置AutowareControl按钮及标签状态 
        if(msg.is_autoware_control_enabled):
            self.label_state_autoware_control.setText("Enable")
            self.label_state_autoware_control.setStyleSheet(self.color_green)
        else:
            self.label_state_autoware_control.setText("Disable")
            self.label_state_autoware_control.setStyleSheet(self.color_yellow)
        self.label_state_autoware_control.setAlignment(Qt.AlignCenter)
      
    # 设置Routing按钮及标签状态 
    def set_route_state(self, state):
        self.pushButton_clear_router.setEnabled(True) 
        if(state == 0):
            self.label_state_routing.setText("UNKNOWN")
            self.label_state_routing.setStyleSheet(self.color_yellow)
        elif(state == 1):
            self.label_state_routing.setText("UNSET")
            self.label_state_routing.setStyleSheet(self.color_yellow)
        elif(state == 2):
            self.label_state_routing.setText("SET")
            self.label_state_routing.setStyleSheet(self.color_green)
            self.pushButton_clear_router.setEnabled(True) 
        elif(state == 3):
            self.label_state_routing.setText("ARRIVED")
            self.label_state_routing.setStyleSheet(self.color_orange)
            self.set_station_color(0)
        elif(state == 4):
            self.label_state_routing.setText("CHANGING")
            self.label_state_routing.setStyleSheet(self.color_yellow)
        self.label_state_routing.setAlignment(Qt.AlignCenter)
    
    # 设置Localization按钮及标签状态    
    def set_initialization_state(self, state):
        self.pushButton_init_gnss.setEnabled(True) 
        if(state == 0):
            self.label_state_localization.setText("UNKNOWN")
            self.label_state_localization.setStyleSheet(self.color_yellow)
        elif(state == 1):
            self.label_state_localization.setText("UNINITIALIZED")
            self.label_state_localization.setStyleSheet(self.color_yellow)
        elif(state == 2):
            self.label_state_localization.setText("INITIALIZING")
            self.label_state_localization.setStyleSheet(self.color_yellow)
            self.pushButton_init_gnss.setEnabled(False) 
        elif(state == 3):
            self.label_state_localization.setText("INITIALIZED")
            self.label_state_localization.setStyleSheet(self.color_green)
        self.label_state_localization.setAlignment(Qt.AlignCenter)
    
    # 设置Motion按钮及标签状态
    def set_motion_state(self, state):
        if(state == 0):
            self.label_state_motion.setText("UNKNOWN")
            self.label_state_motion.setStyleSheet(self.color_yellow)
        elif(state == 1):
            self.label_state_motion.setText("STOPPED")
            self.label_state_motion.setStyleSheet(self.color_orange)
        elif(state == 2):
            self.label_state_motion.setText("STARTING")
            self.label_state_motion.setStyleSheet(self.color_orange)
            self.pushButton_init_gnss.setEnabled(False) 
        elif(state == 3):
            self.label_state_motion.setText("MOVING")
            self.label_state_motion.setStyleSheet(self.color_green)
        self.label_state_motion.setAlignment(Qt.AlignCenter)
        
    # 设置站点状态
    def set_station_state(self, stationId, status):
        self.set_logging_state(status)
        if(stationId == 1 and status.success):
            self.set_station_color(1)
        elif(stationId == 2 and status.success):
            self.set_station_color(2)
        elif(stationId == 3 and status.success):
            self.set_station_color(3)
        elif(stationId == 4 and status.success):
            self.set_station_color(4)
    
    # 设置站点颜色显示，stationId=0为未选中
    def set_station_color(self, stationId):
        self.pushButton_station_1.setStyleSheet(self.color_white)
        self.pushButton_station_2.setStyleSheet(self.color_white)
        self.pushButton_station_3.setStyleSheet(self.color_white)
        self.pushButton_station_4.setStyleSheet(self.color_white)
        if(stationId == 1):
            self.pushButton_station_1.setStyleSheet(self.color_green)
        elif(stationId == 2):
            self.pushButton_station_2.setStyleSheet(self.color_green)
        elif(stationId == 3):
            self.pushButton_station_3.setStyleSheet(self.color_green)
        elif(stationId == 4):
            self.pushButton_station_4.setStyleSheet(self.color_green)
    
    # 更新日志状态        
    def set_logging_state(self, status):
        self.label_message_station.setText(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())+"："+str(status))

    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()
        
        
def ros_spin(node):
    while rclpy.ok():
        rclpy.spin_once(node)
        time.sleep(0.1)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = ActNode("vehicle_ui_mtg") 
    
    app = QApplication(sys.argv)
    main = MainWindow(node)
    main.show()
    
    thread_spin = threading.Thread(target=ros_spin, args=(node,))
    thread_spin.start()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
