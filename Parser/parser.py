import enum
import re
import os


class VariableType(enum.Enum):
    TERMINATION_MODE = 0,
    PARAMETER = 1,
    VOLATILE = 2,
    PERSISTENT = 3,
    EXTERNAL_STATE = 4,


VariableTypeToDesc = {VariableType.TERMINATION_MODE: 'termination_mode:', VariableType.PARAMETER: 'parameter:',
                      VariableType.VOLATILE: 'volatile:', VariableType.PERSISTENT: 'persistent:',
                      VariableType.EXTERNAL_STATE: 'external_state:', }
VariableTypeToClassDesc = {VariableType.TERMINATION_MODE: 'TerminationModes:', VariableType.PARAMETER: 'Parameters:',
                           VariableType.VOLATILE: 'Volatile:', VariableType.PERSISTENT: 'Persistent:',
                           VariableType.EXTERNAL_STATE: 'ExternalVariables:', }
VariableTypeToCollectionName = {VariableType.TERMINATION_MODE: 'termination_modes',
                                VariableType.PARAMETER: 'parameters', VariableType.VOLATILE: 'volatiles',
                                VariableType.PERSISTENT: 'persistent',
                                VariableType.EXTERNAL_STATE: 'external_variables', }


class EventType(enum.Enum):
    TOPIC_LISTENER = 0,
    VARIABLE_VALUE_CHANGE = 1,


class ActionType(enum.Enum):
    SERVICE_ACTIVATION = 0,
    CODE = 1,
    ACTION_ACTIVATION = 2,


class SkillParser:
    def __init__(self, base_file_name, content, package_name):
        self.package_name = package_name
        # base_name = os.path.basename(file_name)
        # base_name = os.path.splitext(base_name)[0]
        self.config = {}
        self.config['managet_type:'] = 'reactive'
        self.config['skill_name:'] = base_file_name # default
        self.config['manager_type:'] = 'reactive'# default
        self.content = content

        self.variables = {}
        self.variables['external_state:'] = []
        self.variables['termination_mode:'] = []
        self.variables['persistent:'] = []
        self.variables['volatile:'] = []
        self.variables['parameter:'] = []


        self.events = {}
        self.events['topic_listener:'] = []
        self.events['variable_value_change:'] = []

        self.actions = {}
        self.actions['service_activation:'] = []
        self.actions['code:'] = []
        self.actions['action_activation:'] = []
        self.parse()


    def parse(self):
        # with open(self.input_file, 'r') as file:
        #     content = file.read()

        content  =self.content
        sections = self.extract_sections(content)
        # Parse Config
        if 'Config' in sections:
            self.parse_config(sections['Config'])
        self.skill_name = self.config['skill_name:']
        self.skill_name_capitalize = self.skill_name.capitalize()
        self.manager_class_name = 'Manager_' + self.skill_name_capitalize
        # Parse variables
        if 'Variables' in sections:
            self.parse_variables(sections['Variables'])

        # Parse events
        if 'Events' in sections:
            self.parse_events(sections['Events'])

        # Parse actions
        if 'Actions' in sections:
            self.parse_actions(sections['Actions'])
        a=1
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
            if self.get_field('skill_name:', line) is not None:
                self.config['skill_name:'] = self.get_field('skill_name:', line.strip())

    def parse_variables(self, content):
        variable_type = None
        variable = {}

        break_words = [key for key in self.variables]
        break_words.append('[Config]')
        break_words.append('[Events]')
        break_words.append('[Actions]')
        break_words.append('[Variables]')

        fields_by_type = {VariableType.TERMINATION_MODE: ['type:', 'name:'],
                          VariableType.PERSISTENT: ['type:', 'name:'], VariableType.VOLATILE: ['type:', 'name:'],
                          VariableType.PARAMETER: ['type:', 'name:'], VariableType.EXTERNAL_STATE: ['type:', 'name:']}
        code_fields_by_type = {VariableType.TERMINATION_MODE: ['default_code:'],
                               VariableType.PERSISTENT: ['default_code:'], VariableType.VOLATILE: ['init_code:'],
                               VariableType.PARAMETER: [], VariableType.EXTERNAL_STATE: []}

        i = 0
        while i < len(content):
            line = content[i]
            i += 1
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
                        i += 1
                    variable[field] = code_field
            for field in fields_by_type[variable_type]:
                if self.get_field(field, line) is not None:
                    variable[field] = self.get_field(field, line)

    def parse_events(self, content):
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
            value = self.get_field('event:', line)
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
        break_words.append('send_goal_code:')
        break_words.append('goal_acceptance_code:')
        break_words.append('result_code:')
        break_words.append('feedback_code:')

        fields_by_type = {ActionType.CODE: ['label:'],
                          ActionType.SERVICE_ACTIVATION: ['label:', 'imports:', 'service_path:', 'srv:'],
                          ActionType.ACTION_ACTIVATION: ['label:', 'imports:', 'action_path:', 'action_type:']}
        code_fields_by_type = {ActionType.CODE: ['code:'],
                               ActionType.SERVICE_ACTIVATION: ['service_activation_code:',
                                                               'service_handle_response_code:'],
                               ActionType.ACTION_ACTIVATION: ['send_goal_code:','goal_acceptance_code:', 'result_code:', 'feedback_code:','cancel_action_code:']}

        action_types = {'service_activation': ActionType.SERVICE_ACTIVATION,'code':ActionType.CODE, 'action_activation': ActionType.ACTION_ACTIVATION}
        i = 0
        while i < len(content):
            line = content[i]
            i += 1
            line = line.strip()
            value = self.get_field('action:', line)
            if value is not None:
                if value.strip() in action_types:
                    action_type = action_types[value.strip()]
                    action = {}
                    self.actions[value.strip()+':'].append(action)
                # if value.strip() == 'service_activation':
                #     action_type = ActionType.SERVICE_ACTIVATION
                #     action = {}
                #     self.actions['service_activation:'].append(action)
                # if value.strip() == 'code':
                #     action_type = ActionType.CODE
                #     action = {}
                #     self.actions['code:'].append(action)
                # if value.strip() == 'action_activation':
                #     action_type = ActionType.ACTION_ACTIVATION
                #     action = {}
                #     self.actions['action_activation:'].append(action)
            if action_type is None:
                continue
            for field in code_fields_by_type[action_type]:
                if self.get_field(field, line) is not None:
                    i + 1
                    code_field = []
                    while i < len(content) and all(not content[i].strip().startswith(item) for item in break_words):
                        code_field.append(content[i])
                        i += 1
                    action[field] = code_field
            for field in fields_by_type[action_type]:
                if self.get_field(field, line) is not None:
                    action[field] = self.get_field(field, line)

    def generate_python_code(self):
        # if os.path.exists(output_file):
        #     os.remove(output_file)
        # with open(output_file, 'w') as file:
        #     file.write(self.generate_imports())
        #     a = self.generate_classes().replace('self.manager_node.external_variables"',
        #                                         'self.manager_node.external_variables')
        #     file.write(a)
        #     file.write()

        skill_manager_code = self.generate_imports()
        skill_manager_code += self.generate_classes().replace('self.manager_node.external_variables"',
                                                 'self.manager_node.external_variables')
        skill_manager_code += self.generate_main()
        return skill_manager_code
    def generate_imports(self):
        imports = [
            "import rclpy",
            "from rclpy.node import Node",
            f"from {self.package_name}.Utils import Utils",
            "from rclpy.executors import MultiThreadedExecutor",
            f"from {self.package_name}.SkillManagerBase import SkillManagerBase, SkillStateEnum, SkillManagerType",
            f"from {self.package_name}.SharedData import SharedData, VariableType",
            'from rclpy.action import ActionClient, CancelResponse'
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

    def split_import_string(self, s):
        s = s.strip('[]')  # Remove square brackets
        return re.split(r',\s*', s)  # Split using regular expression

    def generate_classes(self):
        classes = [
            self.generate_shared_data_class(self.skill_name_capitalize + 'ExternalVariables', VariableType.EXTERNAL_STATE),
            self.generate_shared_data_class(self.skill_name_capitalize + 'Persistent', VariableType.PERSISTENT),
            self.generate_shared_data_class(self.skill_name_capitalize + 'Volatile', VariableType.VOLATILE),
            self.generate_shared_data_class(self.skill_name_capitalize + 'Parameters', VariableType.PARAMETER),
            self.generate_shared_data_class(self.skill_name_capitalize + 'TerminationModes', VariableType.TERMINATION_MODE),
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
            if VariableTypeToDesc[variable_type] == bits[0] + ':':
                variables_actions[bits[1]] = event['actions:'].strip('[]').replace(' ', '').split(',')
        return variables_actions

    def generate_shared_data_class(self, class_name, var_type):

        variables_actions = self.get_events_for_variable_type(var_type)
        # var_collection = VariableTypeToCollectionName[var_type]
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
            #var_type = 'int' if variable['type:'] == 'int' else ('bool' if variable['type:'] == 'bool' else 'str')
            var_type = variable['type:']
            var_casting = 'int' if variable['type:'] == 'int' or var_type == 'bool' else 'float' if var_type == 'float' else None
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
                class_code += f"""
        return {f'{var_casting}(value)' if var_casting is not None else 'value'}
        """
                #return {'int(value)' if var_type == 'int' or var_type == 'bool' else 'value'}
            else:
                if var_casting is not None:
                    class_code += f""""
        return {var_casting}(self.r.get(self.prefix + '{variable['name:']}'))"""
                else:
                    class_code += f""""
        return self.r.get(self.prefix + '{variable['name:']}')"""

        #         class_code += f""""
        # {f"return int(self.r.get(self.prefix + '{variable['name:']}'))" if var_type == 'int' or var_type == 'bool' else f"return self.r.get(self.prefix + '{variable['name:']}')"}"""

            class_code += f"""
    @{variable['name:']}.setter
    def {variable['name:']}(self, value):
        \"\"\"Setter method\"\"\""""
            if var_casting is not None:
                class_code += f"""
        old_value = self.r.set(self.prefix + '{variable['name:']}', {var_casting}(value) , get=True)
        old_value = {var_casting}(old_value)"""
            else:
                class_code += f"""
        old_value = self.r.set(self.prefix + '{variable['name:']}', value , get=True)"""

            if len(variables_actions) > 0:
                class_code += """
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
class {self.manager_class_name}(SkillManagerBase):
    def __init__(self):
        super().__init__(skill_name='{self.skill_name}', manager_id='1', skill_manager_type= SkillManagerType.{'Background' if self.config['manager_type:'] == 'background' else 'Reactive'})
        Actions.initialize(self) 
        self.actions = Actions.actions
        self.parameters = {self.skill_name_capitalize}Parameters(self, var_type= VariableType.PARAMETERS)
        self.persistent = {self.skill_name_capitalize}Persistent(self, var_type= VariableType.PERSISTENT)
        self.termination_modes = {self.skill_name_capitalize}TerminationModes(self, var_type= VariableType.TERMINATION_MODE)
        self.volatiles = {self.skill_name_capitalize}Volatile(self, var_type= VariableType.VOLATILE)
        self.external_variables = {self.skill_name_capitalize}ExternalVariables(self, var_type=VariableType.EXTERNAL_STATE)

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
            manager_class += f"""
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
        super().start_monitoring() 
        if parameters is not None:
            self.set_parameters(parameters, enforce=False) 
        if self.monitor_init: 
            return
        self.monitor_init = True
"""
        for topic_event in self.events['topic_listener:']:
            manager_class += f"""
        self.topic_subscriptions['{topic_event['topic:'].replace('/', '_')}'] = self.create_subscription(
            Pose,
            '{topic_event['topic:']}',
            self.{topic_event['actions:'].replace('[', '').replace(']', '').strip()},
            10) 

    def {topic_event['actions:'].replace('[', '').replace(']', '').strip()}(self, msg):
        if self.dont_monitor:
            return
        self.invoke_action('{topic_event['actions:'].replace('[', '').replace(']', '').strip()}', [msg])
        """
        manager_class += """
    def stop_monitoring(self):
        super().stop_monitoring()
    
    def stop_execution(self):
        super().stop_execution()
"""
        return manager_class

    def generate_actions_class(self):
        actions_list = self.generate_actions_list()
        actions_dict = ",".join(["'" + action + "': Actions." + action for action in actions_list])
        action_methods = []
        more_labels = {}
        for action in self.actions['service_activation:']:
            another_label, code = self.generate_action_method(action, ActionType.SERVICE_ACTIVATION)
            more_labels.update(another_label)
            action_methods.append(code)
        for action in self.actions['action_activation:']:
            another_label, code = self.generate_action_method(action, ActionType.ACTION_ACTIVATION)
            more_labels.update(another_label)
            action_methods.append(code)

        for action in self.actions['code:']:
            another_label, code = self.generate_action_method(action, ActionType.CODE)
            more_labels.update(another_label)
            action_methods.append(code)
        for k in more_labels:
            actions_dict += f",'{k}': Actions.{more_labels[k]}"
        actions_class = f"""
class Actions():
    is_initialized = False
    manager_node = None
    actions = None

    @classmethod
    def initialize(cls, manager_node: SkillManagerBase):
        cls.manager_node = manager_node
        cls.actions = {{ {actions_dict} }}
        cls.is_initialized = True"""
        for action in self.actions['action_activation:']:
            prefix = action['action_path:'].replace('/', '_')
            actions_class += f"""
        cls.{prefix}_action_path = '{action['action_path:']}'
        cls.{prefix}_action_client = ActionClient(cls.manager_node, RotateAbsolute, cls.{prefix}_action_path)
        cls.{prefix}_goal_handle = None
"""
        actions_class += f"""

    {''.join(action_methods)}
"""
        return  actions_class

    def generate_actions_list(self):
        actions_list = []
        for key, value in self.actions.items():
            if isinstance(value, list):
                for item in value:
                    if isinstance(item, dict) and 'label:' in item:
                        actions_list.append(item['label:'])
        return actions_list

    def generate_action_method(self, action, action_type):
        another_action_label = {}
        action_code = ''
        action_func_name = action['label:']
        if action['label:'] == 'start_execution':
            if action_type == ActionType.SERVICE_ACTIVATION:
                action_func_name = action['service_path:'].replace('/', '_')
                another_action_label[action_func_name] = action_func_name
            if action_type == ActionType.ACTION_ACTIVATION:
                action_func_name = action['action_path:'].replace('/', '_')
                another_action_label[action_func_name] = action_func_name
                another_action_label['stop_execution']= action['action_path:'].replace('/', '_')+'_cancel_goal'
                another_action_label[action['action_path:'].replace('/', '_') + '_cancel_goal'] = action['action_path:'].replace('/', '_') + '_cancel_goal'
            if action_type == ActionType.CODE:
                action_func_name = action['label:']


        action_code += f"""
    @classmethod
    def {action_func_name}(cls, msg=None):
        try:
            {'cls.manager_node.skill_state = SkillStateEnum.Running' if action['label:'] == 'start_execution' else ''}
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
            if 'service_handle_response_code:' in action:
                for l in action['service_handle_response_code:']:
                    action_code += '\n                    ' + l
            action_code += f"""
                    cls.manager_node.get_logger().info(f'Service call {{service_path}} Result: {{str(_response)}}') 
                except Exception as e:
                    cls.manager_node.get_logger().info(f'Service call {{service_path}} failed %r' % (e,))
            
            cls.future = client.call_async(_request)
            cls.future.add_done_callback(handle_service_response)
        except Exception as e:
            cls.manager_node.get_logger().info(f'Service call {{service_path}} failed %r' % (e,))"""

        elif action_type == ActionType.CODE:
            if 'code:' in action:
                for l in action['code:']:
                    action_code += '\n            ' + l
            if action_func_name == 'start_execution':
                action_code += """
            cls.manager_node.stop_execution()"""
            action_code += f"""
        except Exception as e:
            cls.manager_node.get_logger().info(f"Action '{action['label:']}' failed %r" % (e,))
        """
        elif action_type == ActionType.ACTION_ACTIVATION:
            action_function_prefix = action['action_path:'].replace('/','_')
            action_code += f"""
            _goal_msg = {action['action_type:']}.Goal()
            """
            if 'send_goal_code:' in action:
                for l in action['send_goal_code:']:
                    action_code += '\n            ' + l
            action_code += f"""
            cls.{action_function_prefix}_action_client.wait_for_server()
                        
            def feedback_callback(_feedback_msg):
                try:"""
            if 'service_activation_code:' in action:
                for l in action['service_handle_response_code:']:
                    action_code += '\n                    ' + l
            else:
                action_code += '\n                    pass'
            action_code += f"""
                                
                except Exception as e:
                    cls.manager_node.get_logger().info(
                        f"feedback_callback() Action call '{action['action_path:']}' failed: {{e}}")

            def goal_response_callback(future):
                goal_handle = future.result()
                if not goal_handle.accepted:
                    cls.manager_node.get_logger().info('Goal rejected :(')
                    cls.manager_node.stop_execution()
                    return

                cls.manager_node.get_logger().info('Goal accepted :)')
                cls.{action_function_prefix}_goal_handle = goal_handle
                cls.manager_node._get_result_future = cls.{action_function_prefix}_goal_handle.get_result_async()
                cls.manager_node._get_result_future.add_done_callback(get_result_callback)

            def get_result_callback(future):
                cls.manager_node.get_logger().info('get_result_callback() start')
                _result = future.result().result"""
            if 'result_code:' in action:
                for l in action['result_code:']:
                    action_code += '\n                ' + l
            action_code += f"""
                cls.manager_node.get_logger().info(f'Action {action['action_path:']} Result: {{_result}}')
                cls.{action_function_prefix}_goal_handle = None"""
            if action['label:'] == 'start_execution':
                action_code += '\n                cls.manager_node.stop_execution()'
            action_code +=f"""
            _send_goal_future = cls.{action_function_prefix}_action_client.send_goal_async(
                _goal_msg,
                feedback_callback=feedback_callback
            )
            _send_goal_future.add_done_callback(goal_response_callback)
        except Exception as e:
            cls.manager_node.get_logger().info(f'start-execution() Action call {{cls.{action_function_prefix}_action_path}} failed: {{e}}')

    @classmethod
    def {action_function_prefix}_cancel_goal(cls):
        try:
            if cls.{action_function_prefix}_goal_handle is not None:
                cls.manager_node.get_logger().info('Cancelling goal')
                future = cls.{action_function_prefix}_goal_handle.cancel_goal_async()

                def cancel_done(future):
                    cancel_response = future.result()
                    if cancel_response == CancelResponse.ACCEPT:
                        cls.manager_node.get_logger().info('Goal successfully cancelled')
                        cls.stop_execution()
                    else:
                        cls.manager_node.get_logger().info('Failed to cancel goal')

                future.add_done_callback(cancel_done)
        except Exception as e:
            cls.manager_node.get_logger().info(f"Stopping Action '{{cls.{action_function_prefix}_action_path}}' failed {{e}}")
            """


        if action['label:'] == 'start_execution' and action_func_name != 'start_execution':
            action_code += f"""
    @classmethod
    def start_execution(cls, msg=None):
        Actions.{action_func_name}()"""
        if action_type == ActionType.SERVICE_ACTIVATION and action['label:'] == 'start_execution':
            action_code += """
        cls.manager_node.stop_execution() 
    """

        return another_action_label, action_code
    def generate_main(self):
        return f"""
def main(args=None):
    rclpy.init(args=args)
    skill_manager_node = {self.manager_class_name}()
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

class ParserWrapper:
    def __init__(self, file_name, content, package_name):
        self.package_name = package_name
        self.parser = SkillParser(file_name, content, package_name)
    def get_python_code(self):
        return self.parser.generate_python_code()


if __name__ == "__main__":
    test_set_pen = True

    file_name = None
    base=None
    if test_set_pen:
        file_name = '../Examples/Example1_monitoring/set_pen.am'
        base = 'set_pen'
    else:
        file_name = 'turn.am'
        base = 'turn'
    content = ''
    with open(file_name, 'r') as file:
        content = file.read()
    parser = SkillParser(base, content)
    python_code = parser.generate_python_code()

    output_file = 'output.py'
    if os.path.exists(output_file):
        os.remove(output_file)

    with open(output_file, 'w') as file:
        file.write(python_code)

