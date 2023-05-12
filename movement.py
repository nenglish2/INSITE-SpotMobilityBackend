#Narit Trikasemsak 2022
#Nick English 2022


"""Provides movement functions for Spot Robot."""

import argparse
import sys
import time
import cv2
import numpy as np

from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.api import geometry_pb2, world_object_pb2, manipulation_api_pb2, image_pb2
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand, blocking_sit, blocking_selfright, block_until_arm_arrives
import bosdyn.api.basic_command_pb2 as basic_command_pb2
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.time_sync import TimeSyncError
from bosdyn.client.estop import EstopEndpoint, EstopKeepAlive, EstopClient
import bosdyn.client.util
from bosdyn.client.image import ImageClient, build_image_request


from estop_nogui import EstopNoGui

VELOCITY_BASE_SPEED = 0.5  # m/s
VELOCITY_BASE_ANGULAR = 0.6284 # rad/sec
VELOCITY_CMD_DURATION = 0.5  # seconds
mouth_open = False
g_image_click = None
g_image_display = None

class Movement(): 
    """Handles robot movements."""

    def __init__(self, user, password, spotAddress, **kwargs):
        self._user = user
        self._password = password
        self.spotAddress = spotAddress 

        self._isPoweredOn = False
    
            
    def auth(self):
        """Build robot object and authenticate"""
        try:
            #create robot SDK
            sdk = bosdyn.client.create_standard_sdk("Spot BVI")
            #create robot object 
            self._robot = sdk.create_robot(self.spotAddress)
            #authenticate
            self._robot.authenticate(self._user, self._password)

            #Create robot state, robot command, and lease clients
            self._robot_state_client = self._robot.ensure_client(RobotStateClient.default_service_name)
            self._robot_command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)
            self._lease_client = self._robot.ensure_client(LeaseClient.default_service_name)
            self._lease = self._lease_client.acquire()
 
            #create estop
            self._create_estop()

            # Construct our lease keep-alive object, which begins RetainLease calls in a thread.
            self._lease_keep_alive = LeaseKeepAlive(self._lease_client)

            print('lease')

            return True
        except Exception as e:
            print(e)
            return False


    def shutdown(self):
        if self.estop_keepalive:
            self.estop_keepalive.shutdown

    def _create_estop(self):
        """Create an estop and assign endpoint"""
        self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
        self._estop_endpoint = bosdyn.client.estop.EstopEndpoint(client=self._estop_client, name='estopper', estop_timeout=9.0)
        self._estop_endpoint.force_simple_setup()
        self._estop_keep_alive = bosdyn.client.estop.EstopKeepAlive(self._estop_endpoint)
        
    def toggle_estop(self):
        """Toggle estop on/off. Initial state is ON"""
        if self._estop_client is not None and self._estop_endpoint is not None:
            if not self._estop_keep_alive:
                self._estop_keep_alive = EstopKeepAlive(self._estop_endpoint)
                print("estop on")
            else:
                self._estop_keep_alive.settle_then_cut()
                self._estop_keep_alive = None
                print("estop off")

    def toggle_lease(self):
        """Toggle lease acquisition. Initial state is acquired"""
        if self._lease_client is not None:
            if self._lease_keep_alive is None:
                self._lease = self._lease_client.acquire()
                self._lease_keep_alive = LeaseKeepAlive(self._lease_client)
            else:
                self._lease_client.return_lease(self._lease)
                self._lease_keep_alive.shutdown()
                self._lease_keep_alive = None

    def toggle_power(self):
        try:
            if self._isPoweredOn:
                self._robot.power_off(cut_immediately=False)
                self._isPoweredOn = False
            else:
                self._robot.power_on()
                self._isPoweredOn = True
            return self._isPoweredOn
        except:
            return self._isPoweredOn

    def _start_command(self, desc, command_proto, end_time_secs=None):
        current_cmd = self._robot_command_client.robot_command_async(lease=None, command=command_proto, end_time_secs=end_time_secs)
        time.sleep(VELOCITY_CMD_DURATION)

    def self_right(self):
        self._start_command('self_right', RobotCommandBuilder.selfright_command())

    def battery_change_pose(self):
        # Default HINT_RIGHT, maybe add option to choose direction?
        self._start_command(
            'battery_change_pose',
            RobotCommandBuilder.battery_change_pose_command(
                dir_hint=basic_command_pb2.BatteryChangePoseCommand.Request.HINT_RIGHT),
                end_time_secs=VELOCITY_CMD_DURATION)

    def sit(self):
        self._start_command('sit', RobotCommandBuilder.synchro_sit_command())

    def stand(self):
        self._start_command('stand', RobotCommandBuilder.synchro_stand_command())

    def move_forward(self):
        self._velocity_cmd_helper('move_forward', v_x=VELOCITY_BASE_SPEED)

    def move_backward(self):
        self._velocity_cmd_helper('move_backward', v_x=-VELOCITY_BASE_SPEED)

    def strafe_left(self):
        self._velocity_cmd_helper('strafe_left', v_y=VELOCITY_BASE_SPEED)

    def strafe_right(self):
        self._velocity_cmd_helper('strafe_right', v_y=-VELOCITY_BASE_SPEED)

    def turn_left(self):
        self._velocity_cmd_helper('turn_left', v_rot=VELOCITY_BASE_ANGULAR)

    def turn_right(self):
        self._velocity_cmd_helper('turn_right', v_rot=-VELOCITY_BASE_ANGULAR)

    def stop(self):
        self._start_command('stop', RobotCommandBuilder.stop_command())

    def _velocity_cmd_helper(self, desc='', v_x=0.0, v_y=0.0, v_rot=0.0):
        """Helper command for movement"""
        self._start_command(
            desc, RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot),
            end_time_secs=time.time() + VELOCITY_CMD_DURATION)
         
    def toggle_mouth(self):
    	global mouth_open
    	if mouth_open:
    		gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(0.0)
    	else:
    		gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)
    	mouth_open = not mouth_open
    	
    	command = RobotCommandBuilder.build_synchro_command(gripper_command)
    	self._robot_command_client.robot_command(command)
    	
    def increase_speed(self):
    	global VELOCITY_BASE_SPEED
    	if VELOCITY_BASE_SPEED < 1.5:
    		VELOCITY_BASE_SPEED *= 10.0
    		VELOCITY_BASE_SPEED = int(VELOCITY_BASE_SPEED)
    		VELOCITY_BASE_SPEED += 1.0
    		VELOCITY_BASE_SPEED /= 10.0
    
    def decrease_speed(self):
    	global VELOCITY_BASE_SPEED
    	if VELOCITY_BASE_SPEED < 1.5:
    		VELOCITY_BASE_SPEED *= 10.0
    		VELOCITY_BASE_SPEED = int(VELOCITY_BASE_SPEED)
    		VELOCITY_BASE_SPEED -= 1.0
    		VELOCITY_BASE_SPEED /= 10.0
    
    def increase_accuracy(self):
    	global VELOCITY_CMD_DURATION
    	if VELOCITY_CMD_DURATION < 1.5:
    		VELOCITY_CMD_DURATION *= 10.0
    		VELOCITY_CMD_DURATION = int(VELOCITY_CMD_DURATION)
    		VELOCITY_CMD_DURATION += 1.0
    		VELOCITY_CMD_DURATION /= 10.0
    
    def decrease_accuracy(self):
    	global VELOCITY_CMD_DURATION
    	if VELOCITY_CMD_DURATION < 1.5:
    		VELOCITY_CMD_DURATION *= 10.0
    		VELOCITY_CMD_DURATION = int(VELOCITY_CMD_DURATION)
    		VELOCITY_CMD_DURATION -= 1.0
    		VELOCITY_CMD_DURATION /= 10.0
    
    def set_command(self, data):
    	global VELOCITY_CMD_DURATION
    	VELOCITY_CMD_DURATION = float(data)
    
    def stow(self):
        self._start_command('stow', RobotCommandBuilder.arm_stow_command())

    def unstow(self):
        self._start_command('stow', RobotCommandBuilder.arm_ready_command())
        
    def fetch_tug(self):
    	global VELOCITY_CMD_DURATION
    	cmd = VELOCITY_CMD_DURATION
    	self.set_command(1.5)
    	self.arm_grasp()
    	self.move_backward()
    	self.toggle_mouth()
    	self.stow()
    	self.toggle_mouth()
    	self.set_command(cmd)
       
    def arm_grasp(self):
        image_client = self._robot.ensure_client(ImageClient.default_service_name)
        # Take a picture with a camera
        print('Getting an image from: frontleft_fisheye_image')
        image_responses = image_client.get_image_from_sources(['frontleft_fisheye_image'])

        if len(image_responses) != 1:
            print('Got invalid number of images: ' + str(len(image_responses)))
            print(image_responses)
            assert False

        image = image_responses[0]
        if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
            dtype = np.uint16
        else:
            dtype = np.uint8
        img = np.fromstring(image.shot.image.data, dtype=dtype)
        if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
            img = img.reshape(image.shot.image.rows, image.shot.image.cols)
        else:
            img = cv2.imdecode(img, -1)

        # Show the image to the user and wait for them to click on a pixel
        print('Click on an object to start grasping...')
        image_title = 'Click to grasp'
        cv2.namedWindow(image_title)
        cv2.setMouseCallback(image_title, self.cv_mouse_callback)
        cv2.startWindowThread()

        global g_image_click, g_image_display
        g_image_display = img
        cv2.imshow(image_title, g_image_display)
        while g_image_click is None:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q'):
                # Quit
                print('"q" pressed, exiting.')
                exit(0)
        

        cv2.destroyAllWindows()
        cv2.waitKey(1)

        print('Picking object at image location (' + str(g_image_click[0]) + ', ' +
                          str(g_image_click[1]) + ')')

        pick_vec = geometry_pb2.Vec2(x=g_image_click[0], y=g_image_click[1])

        # Build the proto
        grasp = manipulation_api_pb2.PickObjectInImage(
            pixel_xy=pick_vec, transforms_snapshot_for_camera=image.shot.transforms_snapshot,
            frame_name_image_sensor=image.shot.frame_name_image_sensor,
            camera_model=image.source.pinhole)

        # Optionally add a grasp constraint.  This lets you tell the robot you only want top-down grasps or side-on grasps.
        self.add_grasp_constraint(None, grasp, self._robot_state_client)

        # Ask the robot to pick up the object
        grasp_request = manipulation_api_pb2.ManipulationApiRequest(pick_object_in_image=grasp)

        manipulation_api_client = self._robot.ensure_client(ManipulationApiClient.default_service_name)
        # Send the request
        cmd_response = manipulation_api_client.manipulation_api_command(
            manipulation_api_request=grasp_request)

        # Get feedback from the robot
        while True:
            feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
                manipulation_cmd_id=cmd_response.manipulation_cmd_id)

            # Send the request
            response = manipulation_api_client.manipulation_api_feedback_command(
                manipulation_api_feedback_request=feedback_request)

            print('Current state: ',
                  manipulation_api_pb2.ManipulationFeedbackState.Name(response.current_state))

            if response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED or response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED:
                break

            time.sleep(0.25)

        print('Finished grasp.')
        g_image_click = None
    	
    	
    # Convert raw image file to cv2
    def image_to_opencv(image):
    	"""Convert an image proto message to an openCV image."""
    	num_channels = 1  # Assume a default of 1 byte encodings.
    	if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
       	    dtype = np.uint16
            extension = ".png"
    	else:
            dtype = np.uint8
            if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
            	num_channels = 3
            elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGBA_U8:
            	num_channels = 4
            elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
            	num_channels = 1
            elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U16:
            	num_channels = 1
            	dtype = np.uint16
            extension = ".jpg"
        
    	img = np.frombuffer(image.shot.image.data, dtype=dtype)
    	if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
            try:
            	# Attempt to reshape array into a RGB rows X cols shape.
            	img = img.reshape((image.shot.image.rows, image.shot.image.cols, num_channels))
            except ValueError:
           	# Unable to reshape the image data, trying a regular decode.
            	img = cv2.imdecode(img, -1)
    	else:
            img = cv2.imdecode(img, -1)

    	ROTATION_ANGLE = {'back_fisheye_image': 0, 'frontleft_fisheye_image': -78,
                      'frontright_fisheye_image': -102, 'left_fisheye_image': 0,
                      'right_fisheye_image': 180, 'hand_color_image': 0}
    	img = ndimage.rotate(img, ROTATION_ANGLE[image.source.name])

    	return img, extension
    	
    	
    def cv_mouse_callback(event, flags, x, y, param, param2=None):
        print(event, flags, x, y, param, param2)

        global g_image_click, g_image_display
        clone = g_image_display.copy()
        if event == cv2.EVENT_LBUTTONUP or flags == 1:
            print("\nCLICKED\n")
            g_image_click = (x, y)
        else:
            # Draw some lines on the image.
            #print('mouse', x, y)
            color = (30, 30, 30)
            thickness = 2
            image_title = 'Click to grasp'
            height = clone.shape[0]
            width = clone.shape[1]
            cv2.line(clone, (0, y), (width, y), color, thickness)
            cv2.line(clone, (x, 0), (x, height), color, thickness)
            cv2.imshow(image_title, clone)
    

    def add_grasp_constraint(self, config, grasp, robot_state_client):
        # There are 3 types of constraints:
        #   1. Vector alignment
        #   2. Full rotation
        #   3. Squeeze grasp
        #
        # You can specify more than one if you want and they will be OR'ed together.

        if config == None:
            use_vector_constraint = True
            axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=1, y=0, z=0)

            # The axis in the vision frame is the negative z-axis
            axis_to_align_with_ewrt_vo = geometry_pb2.Vec3(x=0, y=0, z=-1)
            return
        else:
            # For these options, we'll use a vector alignment constraint.
            use_vector_constraint = config.force_top_down_grasp or config.force_horizontal_grasp

        # Specify the frame we're using.
        grasp.grasp_params.grasp_params_frame_name = VISION_FRAME_NAME

        if use_vector_constraint:
            if config.force_top_down_grasp:
                # Add a constraint that requests that the x-axis of the gripper is pointing in the
                # negative-z direction in the vision frame.

                # The axis on the gripper is the x-axis.
                axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=1, y=0, z=0)

                # The axis in the vision frame is the negative z-axis
                axis_to_align_with_ewrt_vo = geometry_pb2.Vec3(x=0, y=0, z=-1)

            if config.force_horizontal_grasp:
                # Add a constraint that requests that the y-axis of the gripper is pointing in the
                # positive-z direction in the vision frame.  That means that the gripper is constrained to be rolled 90 degrees and pointed at the horizon.

                # The axis on the gripper is the y-axis.
                axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=0, y=1, z=0)

                # The axis in the vision frame is the positive z-axis
                axis_to_align_with_ewrt_vo = geometry_pb2.Vec3(x=0, y=0, z=1)

            # Add the vector constraint to our proto.
            constraint = grasp.grasp_params.allowable_orientation.add()
            constraint.vector_alignment_with_tolerance.axis_on_gripper_ewrt_gripper.CopyFrom(
                axis_on_gripper_ewrt_gripper)
            constraint.vector_alignment_with_tolerance.axis_to_align_with_ewrt_frame.CopyFrom(
                axis_to_align_with_ewrt_vo)

            # We'll take anything within about 10 degrees for top-down or horizontal grasps.
            constraint.vector_alignment_with_tolerance.threshold_radians = 0.17

        elif config.force_45_angle_grasp:
            # Demonstration of a RotationWithTolerance constraint.  This constraint allows you to
            # specify a full orientation you want the hand to be in, along with a threshold.
            #
            # You might want this feature when grasping an object with known geometry and you want to
            # make sure you grasp a specific part of it.
            #
            # Here, since we don't have anything in particular we want to grasp,  we'll specify an
            # orientation that will have the hand aligned with robot and rotated down 45 degrees as an
            # example.

            # First, get the robot's position in the world.
            robot_state = robot_state_client.get_robot_state()
            vision_T_body = get_vision_tform_body(robot_state.kinematic_state.transforms_snapshot)

            # Rotation from the body to our desired grasp.
            body_Q_grasp = math_helpers.Quat.from_pitch(0.785398)  # 45 degrees
            vision_Q_grasp = vision_T_body.rotation * body_Q_grasp

            # Turn into a proto
            constraint = grasp.grasp_params.allowable_orientation.add()
            constraint.rotation_with_tolerance.rotation_ewrt_frame.CopyFrom(vision_Q_grasp.to_proto())

            # We'll accept anything within +/- 10 degrees
            constraint.rotation_with_tolerance.threshold_radians = 0.17

        elif config.force_squeeze_grasp:
            # Tell the robot to just squeeze on the ground at the given point.
            constraint = grasp.grasp_params.allowable_orientation.add()
            constraint.squeeze_grasp.SetInParent()
        else:
            config.force_top_down_grasp = True
            add_grasp_constraint(config, grasp, robot_state_client)


if __name__ == "__main__":
	spot = Movement("user", "vd87k7o35nrs", "137.146.188.201")
    
   

