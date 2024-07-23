import rclpy
from rclpy.node import Node
from middleware_example1.Utils import Utils
from rclpy.executors import MultiThreadedExecutor
from middleware_example1.SkillManagerBase import SkillManagerBase, SkillStateEnum, SkillManagerType
from middleware_example1.SharedData import SharedData, VariableType
from rclpy.action import ActionClient, CancelResponse
from turtlesim.srv import SetPen
from turtlesim.msg import Pose


class Set_penExternalVariables(SharedData):
    def __init__(self, manager_node: SkillManagerBase, var_type: VariableType):
        super().__init__(manager_node, var_type)

    def init(self):
        parameters = self.manager_node.parameters
        persistent = self.manager_node.persistent
        termination_modes = self.manager_node.termination_modes
        volatiles = self.manager_node.volatiles
        external_variables = self.manager_node.external_variables


class Set_penPersistent(SharedData):
    def __init__(self, manager_node: SkillManagerBase, var_type: VariableType):
        super().__init__(manager_node, var_type)

    def init(self):
        parameters = self.manager_node.parameters
        persistent = self.manager_node.persistent
        termination_modes = self.manager_node.termination_modes
        volatiles = self.manager_node.volatiles
        external_variables = self.manager_node.external_variables
    @property
    def is_pen_on(self):
        """Getter method"""
        parameters = self.manager_node.parameters
        persistent = self.manager_node.persistent
        termination_modes = self.manager_node.termination_modes
        volatiles = self.manager_node.volatiles
        external_variables = self.manager_node.external_variables
        value = self.r.get(self.prefix + 'is_pen_on')
        if value is None:
            _default_value = -1
            #possible value-  -1: unknown, 0: pen is disabled, 1: pen is can enabled
            
            value = _default_value
        return int(value)
        
    @is_pen_on.setter
    def is_pen_on(self, value):
        """Setter method"""
        old_value = self.r.set(self.prefix + 'is_pen_on', int(value) , get=True)
        old_value = int(old_value)


class Set_penVolatile(SharedData):
    def __init__(self, manager_node: SkillManagerBase, var_type: VariableType):
        super().__init__(manager_node, var_type)

    def init(self):
        parameters = self.manager_node.parameters
        persistent = self.manager_node.persistent
        termination_modes = self.manager_node.termination_modes
        volatiles = self.manager_node.volatiles
        external_variables = self.manager_node.external_variables
        volatiles._in_area = False
        
        
    @property
    def in_area(self):
        """Getter method"""
        parameters = self.manager_node.parameters
        persistent = self.manager_node.persistent
        termination_modes = self.manager_node.termination_modes
        volatiles = self.manager_node.volatiles
        external_variables = self.manager_node.external_variables
        return int(self.r.get(self.prefix + 'in_area'))
    @in_area.setter
    def in_area(self, value):
        """Setter method"""
        old_value = self.r.set(self.prefix + 'in_area', int(value) , get=True)
        old_value = int(old_value)
        if self.manager_node.dont_monitor:
            return
                
        if int(old_value) != value:
            self.manager_node.invoke_action('set_pen_by_in_area')


class Set_penParameters(SharedData):
    def __init__(self, manager_node: SkillManagerBase, var_type: VariableType):
        super().__init__(manager_node, var_type)

    def init(self):
        parameters = self.manager_node.parameters
        persistent = self.manager_node.persistent
        termination_modes = self.manager_node.termination_modes
        volatiles = self.manager_node.volatiles
        external_variables = self.manager_node.external_variables
    @property
    def on(self):
        """Getter method"""
        parameters = self.manager_node.parameters
        persistent = self.manager_node.persistent
        termination_modes = self.manager_node.termination_modes
        volatiles = self.manager_node.volatiles
        external_variables = self.manager_node.external_variables
        return int(self.r.get(self.prefix + 'on'))
    @on.setter
    def on(self, value):
        """Setter method"""
        old_value = self.r.set(self.prefix + 'on', int(value) , get=True)
        old_value = int(old_value)


class Set_penTerminationModes(SharedData):
    def __init__(self, manager_node: SkillManagerBase, var_type: VariableType):
        super().__init__(manager_node, var_type)

    def init(self):
        parameters = self.manager_node.parameters
        persistent = self.manager_node.persistent
        termination_modes = self.manager_node.termination_modes
        volatiles = self.manager_node.volatiles
        external_variables = self.manager_node.external_variables
    @property
    def success(self):
        """Getter method"""
        parameters = self.manager_node.parameters
        persistent = self.manager_node.persistent
        termination_modes = self.manager_node.termination_modes
        volatiles = self.manager_node.volatiles
        external_variables = self.manager_node.external_variables
        value = self.r.get(self.prefix + 'success')
        if value is None:
            _default_value = -1
            
            value = _default_value
        return int(value)
        
    @success.setter
    def success(self, value):
        """Setter method"""
        old_value = self.r.set(self.prefix + 'success', int(value) , get=True)
        old_value = int(old_value)


class Manager_Set_pen(SkillManagerBase):
    def __init__(self):
        super().__init__(skill_name='set_pen', manager_id='1', skill_manager_type= SkillManagerType.Background)
        Actions.initialize(self) 
        self.actions = Actions.actions
        self.parameters = Set_penParameters(self, var_type= VariableType.PARAMETERS)
        self.persistent = Set_penPersistent(self, var_type= VariableType.PERSISTENT)
        self.termination_modes = Set_penTerminationModes(self, var_type= VariableType.TERMINATION_MODE)
        self.volatiles = Set_penVolatile(self, var_type= VariableType.VOLATILE)
        self.external_variables = Set_penExternalVariables(self, var_type=VariableType.EXTERNAL_STATE)

        self.parameters.init()
        self.persistent.init()
        self.termination_modes.init()
        self.volatiles.init()
        self.external_variables.init()

    def invoke_action(self, action, parameters=None):
        try: 
            if action in self.actions.keys():
                if parameters is None:
                    self.actions[action]()
                else:
                    self.actions[action](*parameters)
            else:
                self.get_logger().error(f"invoke_action() cannot find action:'{action}'")
        except Exception as e:
            self.get_logger().error(f"invoke_action() during action:{action} exception: {e}")
            raise e

    def set_parameters(self, parameters, enforce=False):
        try:
            self.get_logger().info(f'start set_parameter()[{parameters}], enforce:{enforce}')
            
            if 'on' in parameters:
                self.parameters.on = parameters['on']
            elif enforce:
                self.get_logger().error(f'Fatal error: Invoked skill:{self.skill_name} parameter:"on" was not set, trace:{Utils.get_trace()}')
                raise KeyError(f'Fatal error: Invoked skill:{self.skill_name} parameter: "on" was not set')
                
        except Exception as e:
            self.get_logger().error(f'set_parameters error failed:{e}')

    def start_execution(self, parameters=None):
        try:
            self.get_logger().info('start_execution() from redis message')
            self.set_parameters(parameters, enforce=True) 
            self.invoke_action('start_execution')
        except Exception as e:
            self.get_logger().error(f'start_execution() error failed:{e}')

    def start_monitoring(self, parameters=None):
        super().start_monitoring() 
        if parameters is not None:
            self.set_parameters(parameters, enforce=False) 
        if self.monitor_init: 
            return
        self.monitor_init = True

        self.topic_subscriptions['_turtle1_pose'] = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.topic_listener_turtle1_pose,
            10) 

    def topic_listener_turtle1_pose(self, msg):
        if self.dont_monitor:
            return
        self.invoke_action('topic_listener_turtle1_pose', [msg])
        
    def stop_monitoring(self):
        super().stop_monitoring()
    
    def stop_execution(self):
        super().stop_execution()



class Actions():
    is_initialized = False
    manager_node = None
    actions = None

    @classmethod
    def initialize(cls, manager_node: SkillManagerBase):
        cls.manager_node = manager_node
        cls.actions = { 'start_execution': Actions.start_execution,'topic_listener_turtle1_pose': Actions.topic_listener_turtle1_pose,'set_pen_by_in_area': Actions.set_pen_by_in_area,'_turtle1_set_pen': Actions._turtle1_set_pen }
        cls.is_initialized = True

    
    @classmethod
    def _turtle1_set_pen(cls, msg=None):
        try:
            cls.manager_node.skill_state = SkillStateEnum.Running
            parameters = cls.manager_node.parameters
            persistent = cls.manager_node.persistent
            termination_modes = cls.manager_node.termination_modes
            volatiles = cls.manager_node.volatiles
            external_variables = cls.manager_node.external_variables

            service_path = '/turtle1/set_pen'
            client = cls.manager_node.create_client(SetPen, service_path)
            while not client.wait_for_service(timeout_sec=1.0):
                cls.manager_node.get_logger().info('service not available, waiting again...')
            _request = SetPen.Request()
            _request.r = 255
            _request.g = 0
            _request.b = 0
            _request.width = 1
            if parameters.on:
                _request.off = 0
            else:
                _request.off = 1
            
            cls.manager_node.get_logger().info(f'_request:{{_request}}')
            def handle_service_response(future):
                try:
                    cls.manager_node.get_logger().info(f'set_pen service response start handle')
                    _response = future.result()
                    termination_modes.success = False
                    if _response is not None:
                        termination_modes.success = True
                        persistent.is_pen_on = 1 if parameters.on else 0
                        termination_modes.success = False
                    if _response is not None:
                        termination_modes.success = True
                    
                    cls.manager_node.get_logger().info(f'Service call {service_path} Result: {str(_response)}') 
                except Exception as e:
                    cls.manager_node.get_logger().info(f'Service call {service_path} failed %r' % (e,))
            
            cls.future = client.call_async(_request)
            cls.future.add_done_callback(handle_service_response)
        except Exception as e:
            cls.manager_node.get_logger().info(f'Service call {service_path} failed %r' % (e,))
    @classmethod
    def start_execution(cls, msg=None):
        Actions._turtle1_set_pen()
        cls.manager_node.stop_execution() 
    
    @classmethod
    def topic_listener_turtle1_pose(cls, msg=None):
        try:
            
            parameters = cls.manager_node.parameters
            persistent = cls.manager_node.persistent
            termination_modes = cls.manager_node.termination_modes
            volatiles = cls.manager_node.volatiles
            external_variables = cls.manager_node.external_variables

            in_area = True
            if msg.x > 4 and msg.x < 8 and msg.y > 4 and msg.y < 8:
                in_area = True
            else:
                in_area = False
            volatiles.in_area = in_area
            
            
        except Exception as e:
            cls.manager_node.get_logger().info(f"Action 'topic_listener_turtle1_pose' failed %r" % (e,))
        
    @classmethod
    def set_pen_by_in_area(cls, msg=None):
        try:
            
            parameters = cls.manager_node.parameters
            persistent = cls.manager_node.persistent
            termination_modes = cls.manager_node.termination_modes
            volatiles = cls.manager_node.volatiles
            external_variables = cls.manager_node.external_variables

            Utils.skill_message('set_pen', 'RUN', {'on':volatiles.in_area})
        except Exception as e:
            cls.manager_node.get_logger().info(f"Action 'set_pen_by_in_area' failed %r" % (e,))
        

def main(args=None):
    rclpy.init(args=args)
    skill_manager_node = Manager_Set_pen()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(skill_manager_node)

    try:
        executor.spin()
    finally:
        skill_manager_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
