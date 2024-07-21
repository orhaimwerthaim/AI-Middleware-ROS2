import enum
import re
import os
class VariableType(enum.Enum):
    TERMINATION_MODE = 0,
    PARAMETER = 1,
    VOLATILE = 2,
    PERSISTENT = 3,
    EXTERNAL_STATE = 4,
VariableTypeToDesc = {VariableType.TERMINATION_MODE:'termination_mode:', VariableType.PARAMETER:'parameter:', VariableType.VOLATILE:'volatile:', VariableType.PERSISTENT:'persistent:', VariableType.EXTERNAL_STATE:'external_state:',}
VariableTypeToClassDesc = {VariableType.TERMINATION_MODE:'TerminationModes:', VariableType.PARAMETER:'Parameters:', VariableType.VOLATILE:'Volatile:', VariableType.PERSISTENT:'Persistent:', VariableType.EXTERNAL_STATE:'ExternalVariables:',}
VariableTypeToCollectionName = {VariableType.TERMINATION_MODE:'termination_modes', VariableType.PARAMETER:'parameters', VariableType.VOLATILE:'volatiles', VariableType.PERSISTENT:'persistent', VariableType.EXTERNAL_STATE:'external_variables',}


class EventType(enum.Enum):
    TOPIC_LISTENER = 0,
    VARIABLE_VALUE_CHANGE = 1,

class ActionType(enum.Enum):
    SERVICE_ACTIVATION = 0,
    CODE = 1,

class SkillParser:
    def __init__(self, input_file):
        base_name = os.path.basename(input_file)
        self.skill_name, _ = os.path.splitext(base_name)
        self.config = {}
        self.config['managet_type:'] = 'reactive' #default
        self.input_file = input_file
        self.variables ={}
        self.events = {}
        self.actions = {}

    def parse(self):
        with open(self.input_file, 'r') as file:
            content = file.read()

        sections = self.extract_sections(content)
        # Parse Config
        if 'Config' in sections:
            self.parse_config(sections['Config'])
        # Parse variables
        if 'Variables' in sections:
            self.parse_variables(sections['Variables'])

        # Parse events
        self.parse_events(sections['Events'])

        # Parse actions
        self.parse_actions(sections['Actions'])



    def extract_sections(self, text):
        sections = {}
        section_name = None
        for line in text.splitlines():
            s_line = line.strip()
            if s_line == '[Variables]':
                section_name = 'Variables'
                sections[section_name] = []
                continue
            elif s_line == '[Actions]':
                section_name = 'Actions'
                sections[section_name] = []
                continue
            elif s_line == '[Events]':
                section_name = 'Events'
                sections[section_name] = []
                continue
            elif s_line == '[Config]':
                section_name = 'Config'
                sections[section_name] = []
                continue
            else:
                if section_name is not None:
                    sections[section_name].append(line)

        return sections

    def get_field(self, name, line):
        if line.startswith(name):
            return line[len(name):].strip()
        return None

    def parse_config(self, content):

        i = 0
        while i < len(content):
            line = content[i]
            i += 1
            if self.get_field('manager_type:', line) is not None:
                self.config['manager_type:'] = self.get_field('manager_type:', line.strip())
    def parse_variables(self, content):
        self.variables['external_state:'] = []
        self.variables['termination_mode:'] = []
        self.variables['persistent:'] = []
        self.variables['volatile:'] = []
        self.variables['parameter:'] = []
        variable_type = None
        variable = {}



        break_words = [key for key in self.variables]
        break_words.append('[Config]')
        break_words.append('[Events]')
        break_words.append('[Actions]')
        break_words.append('[Variables]')

        fields_by_type = {VariableType.TERMINATION_MODE: ['type:', 'name:'], VariableType.PERSISTENT:['type:', 'name:'], VariableType.VOLATILE:['type:', 'name:'], VariableType.PARAMETER:['type:', 'name:'], VariableType.EXTERNAL_STATE:['type:', 'name:']}
        code_fields_by_type = {VariableType.TERMINATION_MODE: ['default_code:'],
                          VariableType.PERSISTENT: ['default_code:'], VariableType.VOLATILE: ['init_code:'],
                          VariableType.PARAMETER: [], VariableType.EXTERNAL_STATE: []}

        i = 0
        while i < len(content):
            line = content[i]
            i+=1
            line = line.strip()
            if line == 'termination_mode:':
                variable = {}
                self.variables['termination_mode:'].append(variable)
                variable_type = VariableType.TERMINATION_MODE
            if line == 'persistent:':
                variable = {}
                self.variables['persistent:'].append(variable)
                variable_type = VariableType.PERSISTENT
            if line == 'volatile:':
                variable = {}
                self.variables['volatile:'].append(variable)
                variable_type = VariableType.VOLATILE
            if line == 'parameter:':
                variable = {}
                self.variables['parameter:'].append(variable)
                variable_type = VariableType.PARAMETER
            if line == 'external_state:':
                variable = {}
                self.variables['external_state:'].append(variable)
                variable_type = VariableType.EXTERNAL_STATE
            if variable_type is None:
                continue
            for field in code_fields_by_type[variable_type]:
                if self.get_field(field, line) is not None:
                    i + 1
                    code_field = []
                    while i < len(content) and all(not content[i].strip().startswith(item) for item in break_words):
                        code_field.append(content[i].strip())
                        i+=1
                    variable[field] = code_field
            for field in fields_by_type[variable_type]:
                if self.get_field(field, line) is not None:
                    variable[field] =  self.get_field(field, line)

    def parse_events(self, content):
        self.events['topic_listener:'] = []
        self.events['variable_value_change:'] = []

        event_type = None
        event = {}

        break_words = ['event:']
        break_words.append('[Config]')
        break_words.append('[Events]')
        break_words.append('[Actions]')
        break_words.append('[Variables]')

        fields_by_type = {EventType.TOPIC_LISTENER: ['topic:', 'imports:', 'actions:'],
                          EventType.VARIABLE_VALUE_CHANGE: ['actions:', 'variable:']}
        # code_fields_by_type = {VariableType.TERMINATION_MODE: ['default_code:'],
        #                        VariableType.PERSISTENT: ['default_code:'], VariableType.VOLATILE: ['init_code:'],
        #                        VariableType.PARAMETER: [], VariableType.EXTERNAL_STATE: []}

        i = 0
        while i < len(content):
            line = content[i]
            i += 1
            line = line.strip()
            value = self.get_field('event:',line)
            if value is not None:
                if value.strip() == 'topic_listener':
                    event_type = EventType.TOPIC_LISTENER
                    event = {}
                    self.events['topic_listener:'].append(event)
                elif value.strip() == 'variable_value_change':
                    event_type = EventType.VARIABLE_VALUE_CHANGE
                    event = {}
                    self.events['variable_value_change:'].append(event)

            if event_type is None:
                continue
            for field in fields_by_type[event_type]:
                if self.get_field(field, line) is not None:
                    event[field] = self.get_field(field, line)
    def parse_actions(self, content):

        self.actions['service_activation:'] = []
        self.actions['code:'] = []

        action_type = None
        action = {}


        break_words = []
        break_words.append('[Config]')
        break_words.append('[Events]')
        break_words.append('[Actions]')
        break_words.append('[Variables]')
        break_words.append('action:')
        break_words.append('service_handle_response_code:')
        break_words.append('service_activation_code:')

        fields_by_type = {ActionType.CODE: ['label:'], ActionType.SERVICE_ACTIVATION:['label:', 'imports:', 'service_path:', 'srv:']}
        code_fields_by_type = {ActionType.CODE: ['code:'],
                          ActionType.SERVICE_ACTIVATION: ['service_activation_code:', 'service_handle_response_code:']}

        i = 0
        while i < len(content):
            line = content[i]
            i+=1
            line = line.strip()
            value = self.get_field('action:',line)
            if value is not None:
                if value.strip() == 'service_activation':
                    action_type = ActionType.SERVICE_ACTIVATION
                    action = {}
                    self.actions['service_activation:'].append(action)
                if value.strip() == 'code':
                    action_type = ActionType.CODE
                    action = {}
                    self.actions['code:'].append(action)
            if action_type is None:
                continue
            for field in code_fields_by_type[action_type]:
                if self.get_field(field, line) is not None:
                    i + 1
                    code_field = []
                    while i < len(content) and all(not content[i].strip().startswith(item) for item in break_words):
                        code_field.append(content[i])
                        i+=1
                    action[field] = code_field
            for field in fields_by_type[action_type]:
                if self.get_field(field, line) is not None:
                    action[field] =  self.get_field(field, line)


    def generate_python_code(self, output_file):
        if os.path.exists(output_file):
            os.remove(output_file)
        with open(output_file, 'w') as file:
            file.write(self.generate_imports())
            a=self.generate_classes().replace('self.manager_node.external_variables"','self.manager_node.external_variables')
            file.write(a)
            file.write(self.generate_main())

    def generate_imports(self):
        imports = [
            "import rclpy",
            "from rclpy.node import Node",
            "from middleware_example1.Utils import Utils",
            "from rclpy.executors import MultiThreadedExecutor",
            "from middleware_example1.SkillManagerBase import SkillManagerBase, SkillStateEnum, SkillManagerType",
            "from middleware_example1.SharedData import SharedData, VariableType",
        ]
        # Add unique imports from actions
        imports_list = [d['imports:'] for value in self.actions.values() for d in value if 'imports:' in d]
        result = [item for s in imports_list for item in self.split_import_string(s)]
        imports.extend(result)

        # Add unique imports from events
        imports_list = [d['imports:'] for value in self.events.values() for d in value if 'imports:' in d]
        result = [item for s in imports_list for item in self.split_import_string(s)]
        imports.extend(result)

        return '\n'.join(imports) + '\n\n'

    def split_import_string(self,s):
        s = s.strip('[]')  # Remove square brackets
        return re.split(r',\s*', s)  # Split using regular expression
    def generate_classes(self):
        classes = [
            self.generate_shared_data_class('SetPenExternalVariables', VariableType.EXTERNAL_STATE),
            self.generate_shared_data_class('SetPenPersistent', VariableType.PERSISTENT),
            self.generate_shared_data_class('SetPenVolatile', VariableType.VOLATILE),
            self.generate_shared_data_class('SetPenParameters', VariableType.PARAMETER),
            self.generate_shared_data_class('SetPenTerminationModes', VariableType.TERMINATION_MODE),
            self.generate_manager_class(),
            self.generate_actions_class()
        ]
        return '\n\n'.join(classes)

    # event: variable_value_change
    # variable: [volatile, in_area]
    # actions: [set_pen_by_in_area]
    def get_events_for_variable_type(self, variable_type: VariableType):
        variables_actions = {}
        for event in self.events['variable_value_change:']:
            bits = event['variable:'].strip('[]').replace(' ', '').split(',')
            if VariableTypeToDesc[variable_type] ==  bits[0]+':':
                variables_actions[bits[1]] = event['actions:'].strip('[]').replace(' ', '').split(',')
        return variables_actions

    def generate_shared_data_class(self, class_name, var_type):
        variables_actions = self.get_events_for_variable_type(var_type)
        #var_collection = VariableTypeToCollectionName[var_type]
        var_cat = VariableTypeToDesc[var_type]
        class_code = f"""
class {class_name}(SharedData):
    def __init__(self, manager_node: SkillManagerBase, var_type: VariableType):
        super().__init__(manager_node, var_type)
    
    def init(self):
        parameters = self.manager_node.parameters
        persistent = self.manager_node.persistent
        termination_modes = self.manager_node.termination_modes
        volatiles = self.manager_node.volatiles
        external_variables = self.manager_node.external_variables"""
        for variable in self.variables[var_cat]:
            if 'init_code:' in variable:
                for line in variable['init_code:']:
                    class_code += '\n        ' + line
            var_type = 'int' if variable['type:'] == 'int' else ('bool' if variable['type:'] == 'bool' else 'str')
            class_code += f"""
    @property
    def {variable['name:']}(self):
        \"\"\"Getter method\"\"\"
        parameters = self.manager_node.parameters
        persistent = self.manager_node.persistent
        termination_modes = self.manager_node.termination_modes
        volatiles = self.manager_node.volatiles
        external_variables = self.manager_node.external_variables"""
            if 'default_code:' in variable:
                class_code += f"""
        value = self.r.get(self.prefix + '{variable['name:']}')
        if value is None:"""
                for l in variable['default_code:']:
                    class_code += '\n            ' + l
                class_code += f"""
            value = _default_value"""
                class_code +=f"""
        return {'int(value)' if var_type == 'int' or var_type == 'bool' else 'value'}
        """
            else:
                class_code += f""""
        {f"return int(self.r.get(self.prefix + '{variable['name:']}'))" if var_type == 'int' or var_type == 'bool' else f"return self.r.get(self.prefix + '{variable['name:']}')"}"""

            class_code +=f"""
    @{variable['name:']}.setter
    def {variable['name:']}(self, value):
        \"\"\"Setter method\"\"\"
        old_value = self.r.set(self.prefix + '{variable['name:']}', {'int(value)' if var_type == 'int' or var_type == 'bool' else 'value'}, get=True)
        old_value = {'int(old_value)' if var_type == 'int' or var_type == 'bool' else 'old_value'}
        if self.manager_node.dont_monitor:
            return
    """
            if variable['name:'] in variables_actions:
                class_code += '\n        if int(old_value) != value:'
                for action in variables_actions[variable['name:']]:
                    class_code += f"\n            self.manager_node.invoke_action('{action}')"
        return class_code

    def generate_manager_class(self):
        manager_class = f"""
class Manager_SetPen(SkillManagerBase):
    def __init__(self):
        super().__init__(skill_name='{self.skill_name}', manager_id='1', skill_manager_type= SkillManagerType.{'Background' if self.config['manager_type:'] == 'background' else 'Reactive'})
        Actions.initialize(self) 
        self.actions = Actions.actions
        self.parameters = SetPenParameters(self, var_type= VariableType.PARAMETERS)
        self.persistent = SetPenPersistent(self, var_type= VariableType.PERSISTENT)
        self.termination_modes = SetPenTerminationModes(self, var_type= VariableType.TERMINATION_MODE)
        self.volatiles = SetPenVolatile(self, var_type= VariableType.VOLATILE)
        self.external_variables = SetPenExternalVariables(self, var_type=VariableType.EXTERNAL_STATE)
        
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
                self.get_logger().error(f"invoke_action() cannot find action:'{{action}}'")
        except Exception as e:
            self.get_logger().error(f"invoke_action() during action:{{action}} exception: {{e}}")
            raise e

    def set_parameters(self, parameters, enforce=False):
        try:
            self.get_logger().info(f'start set_parameter()[{{parameters}}], enforce:{{enforce}}')
            """
        for par in self.variables['parameter:']:

            manager_class +=f"""
            if '{par['name:']}' in parameters:
                self.parameters.{par['name:']} = parameters['{par['name:']}']
            elif enforce:
                self.get_logger().error(f'Fatal error: Invoked skill:{{self.skill_name}} parameter:"{par['name:']}" was not set, trace:{{Utils.get_trace()}}')
                raise KeyError(f'Fatal error: Invoked skill:{{self.skill_name}} parameter: "{par['name:']}" was not set')
                """
            manager_class += """
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
        self.get_logger().info('start_monitoring()')
        if parameters is not None:
            self.set_parameters(parameters, enforce=False)
        self.dont_monitor = False
        if self.monitor_init: 
            return
        self.monitor_init = True
"""
        for topic_event in self.events['topic_listener:']:
            manager_class += f"""
        self.topic_subscriptions['{topic_event['topic:'].replace('/', '_')}'] = self.create_subscription(
            Pose,
            '{topic_event['topic:']}',
            self.{topic_event['actions:'].replace('[', '').replace(']','').strip()},
            10) 
        
    def {topic_event['actions:'].replace('[', '').replace(']','').strip()}(self, msg):
        if self.dont_monitor:
            return
        self.invoke_action('{topic_event['actions:'].replace('[', '').replace(']','').strip()}', [msg])

    def stop_monitoring(self):
        self.dont_monitor = True
        # for key in self.topic_subscriptions:
        #     self.topic_subscriptions[key].dispose()
        self.skill_state = SkillStateEnum.Waiting
"""
        return manager_class

    def generate_actions_class(self):
        actions_list =self.generate_actions_list()
        actions_dict = ",".join(["'" + action + "': Actions." + action for action in actions_list])
        action_methods = []
        for action in self.actions['service_activation:']:
            action_methods.append(self.generate_action_method(action, ActionType.SERVICE_ACTIVATION))
        for action in self.actions['code:']:
            action_methods.append(self.generate_action_method(action, ActionType.CODE))
        return f"""
class Actions():
    is_initialized = False
    manager_node = None
    actions = None

    @classmethod
    def initialize(cls, manager_node: SkillManagerBase):
        cls.manager_node = manager_node
        cls.actions = {{ { actions_dict} }}
        cls.is_initialized = True
 

    {''.join(action_methods)}
"""
    def generate_actions_list(self):
        actions_list = []
        for key, value in self.actions.items():
            if isinstance(value, list):
                for item in value:
                    if isinstance(item, dict) and 'label:' in item:
                        actions_list.append(item['label:'])
        return actions_list

    def generate_action_method(self, action, action_type):
        action_code = f"""
    @classmethod
    def {action['label:']}(cls, msg=None):
        try:
            parameters = cls.manager_node.parameters
            persistent = cls.manager_node.persistent
            termination_modes = cls.manager_node.termination_modes
            volatiles = cls.manager_node.volatiles
            external_variables = cls.manager_node.external_variables
"""
        if action_type == ActionType.SERVICE_ACTIVATION:
            action_code += f"""
            service_path = '{action['service_path:']}'
            client = cls.manager_node.create_client({action['srv:']}, service_path)
            while not client.wait_for_service(timeout_sec=1.0):
                cls.manager_node.get_logger().info('service not available, waiting again...')
            _request = {action['srv:']}.Request()"""
            if 'service_activation_code:' in action:
                for l in action['service_activation_code:']:
                    action_code += '\n            ' + l
            action_code += """
            cls.manager_node.get_logger().info(f'_request:{{_request}}')
            def handle_service_response(future):
                try:
                    cls.manager_node.get_logger().info(f'set_pen service response start handle')
                    _response = future.result()"""
            if 'service_activation_code:' in action:
                for l in action['service_handle_response_code:']:
                    action_code += '\n                    ' + l
            action_code += f"""
                    cls.manager_node.get_logger().info(f'Service call {{service_path}} Result: {{str(_response)}}') 
                except Exception as e:
                    cls.manager_node.get_logger().info(f'Service call {{service_path}} failed %r' % (e,))
             
            if cls.manager_node.skill_manager_type == SkillManagerType.Background:
                cls.manager_node.skill_state = SkillStateEnum.Monitoring
            if cls.manager_node.skill_manager_type == SkillManagerType.Reactive:
                cls.manager_node.stop_monitoring() 
            cls.future = client.call_async(_request)
            cls.future.add_done_callback(handle_service_response)
        except Exception as e:
            cls.manager_node.get_logger().info(f'Service call {{service_path}} failed %r' % (e,))"""



        elif action_type == ActionType.CODE:
            if 'code:' in action:
                for l in action['code:']:
                    action_code += '\n            ' + l
            action_code += f"""
        except Exception as e:
            cls.manager_node.get_logger().info(f"Action '{action['label:']}' failed %r" % (e,))
        """

        return action_code

    def generate_main(self):
        return """
def main(args=None):
    rclpy.init(args=args)
    skill_manager_node = Manager_SetPen()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(skill_manager_node)

    try:
        executor.spin()
    finally:
        skill_manager_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
"""

if __name__ == "__main__":
    parser = SkillParser('set_pen.am')
    parser.parse()
    parser.generate_python_code('output.py')

