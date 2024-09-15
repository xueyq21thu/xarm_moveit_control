import rclpy, numpy as np
from rclpy.node import Node
from xarm_msgs.srv import SetFloat32List, MoveCartesian, GetFloat32List, SetInt16, Call, SetTcpLoad
from scipy.spatial.transform import Rotation
from rclpy.callback_groups import ReentrantCallbackGroup
import time


p_g_tcp = np.array([0.0, 0.0, 65])

def rot_to_euler(R):
    # convert rotation matrix to euler angles
    # only in this shitty hole
    rpy = Rotation.from_matrix(R).as_euler('xyz', degrees=False)
    return np.array(rpy)

class ComSrv(Node):
    def __init__(self) -> None:
        super().__init__('com_srv')
        self.group = ReentrantCallbackGroup()
        # create service to receive ee pose and force and publish them
        self.ee_pose_srv = self.create_service(SetFloat32List, 'ee_pose', self.ee_pose_callback, callback_group=self.group)
        self.force_srv = self.create_service(SetFloat32List, 'force', self.force_callback, callback_group=self.group)
        self.reset_srv = self.create_service(Call, 'reset_pose', self.reset_callback, callback_group=self.group)
                
        # client for xarm error and warning
        self.warn_cli = self.create_client(Call,'xarm/clean_warn')
        self.error_cli = self.create_client(Call,'xarm/clean_error')
        self.set_state_cli = self.create_client(SetInt16,'xarm/set_state')
        
        # client for xarm cartesian movement
        self.pose_move_cli = self.create_client(MoveCartesian,'/xarm/set_position', callback_group=self.group)
        self.set_fapp_cli = self.create_client(SetInt16,'/xarm/ft_sensor_app_set', callback_group=self.group)

        
        # client for xarm force control
        self.set_fref_cli = self.create_client(SetFloat32List, '/update_force_ref', callback_group=self.group)
        self.start_cli = self.create_client(Call, '/start_move', callback_group=self.group)
        self.end_cli = self.create_client(Call, '/end_move', callback_group=self.group)
        
        #  command to be operated
        self.cmd_pose = None
        self.cmd_force = None
        
        # params
        self.spd = 300.0
        self.acc = 100.0
        self.mvtime = 0.0

        # configure the robot
        self.xarm_pose_request = MoveCartesian.Request()
        self.xarm_pose_request.speed = self.spd
        self.xarm_pose_request.acc = self.acc
        self.xarm_pose_request.mvtime = self.mvtime
        self.xarm_pose_request.pose = [ 472.631836, 0.174363, 538.114868, 3.140461, 0.025776, 1.586435 ]
            
            
        
    def init_robot(self):
        # set the robot to the initial pose
        # rclpy.spin_until_future_complete(self, self.error_cli.call_async(Call.Request()))
        # rclpy.spin_until_future_complete(self, self.warn_cli.call_async(Call.Request()))
        self.xarm_pose_request.pose = [ 472.631836, 0.174363, 538.114868, 3.140461, 0.025776, 1.586435 ]
        self.pose_move_cli.call_async(self.xarm_pose_request)
        print("Robot initialized!")
    

    def ee_pose_callback(self, request, response):
        self.set_fapp_cli.call_async(SetInt16.Request(data=0))
        # get ee pose from the quest3
        request_data = np.array(request.datas)
        print(request_data)
        xyz = request_data[0:3]
        R = np.array([-request_data[3:6], request_data[6:9], request_data[9:12]]).T
        rpy = rot_to_euler(R)
        
        p_tcp_base = np.dot(R, -p_g_tcp)
        xyz = xyz + p_tcp_base
        
        pose = np.concatenate((xyz, rpy))
        self.cmd_pose = pose
        self.xarm_pose_request.pose = self.cmd_pose.tolist()
        
        # end the force control
        # self.end_cli.call_async(Call.Request())

        self.set_state_cli.call_async(SetInt16.Request(data=0))
        # self.end_cli.call_async(Call.Request())
        
        # move the robot to the desired pose
        self.pose_move_cli.call_async(self.xarm_pose_request)
        # rclpy.spin_until_future_complete(self, f)
        # print(f.result())

        print(f"Robot moved to: {self.cmd_pose}")
        return response
        
    
    def force_callback(self, request, response):
        # get force from the quest3
        print(request.datas)
        request_data = request.datas
        self.cmd_force = request_data.tolist()
        
        # update the force reference
        force_req = SetFloat32List.Request()
        force_req.datas = request_data
        self.set_fref_cli.call_async(force_req)
        print(f"Force updated: {self.cmd_force}")

        # time.sleep(1)
        
        # start the force control
        # self.start_cli.call_async(Call.Request())
        response.ret = 1
        response.message = "Force control started!"
        return response
    
    def reset_callback(self, request, response):
        # clear warn and error
        self.warn_cli.call_async(Call.Request())
        self.error_cli.call_async(Call.Request())
        self.set_state_cli.call_async(SetInt16.Request(data=0))
        
        # reset the robot
        self.xarm_pose_request.pose = [ 472.631836, 0.174363, 538.114868, 3.140461, 0.025776, 1.586435 ]
        f = self.pose_move_cli.call_async(self.xarm_pose_request)
        # rclpy.spin_until_future_complete(self, f)
        print("Robot reset!")
        return response
    
def main():
    try:
        rclpy.init()
        com_srv = ComSrv()
        com_srv.init_robot()
        rclpy.spin(com_srv)
    finally:
        com_srv.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()        
