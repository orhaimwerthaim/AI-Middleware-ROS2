import re
import configparser

class Variable:
    def __init__(self, name, var_type, description):
        self.name = name
        self.var_type = var_type
        self.description = description

    def __repr__(self):
        return f"Variable(name={self.name}, type={self.var_type}, description={self.description})"

class Event:
    def __init__(self, name, trigger, actions):
        self.name = name
        self.trigger = trigger
        self.actions = actions

    def __repr__(self):
        return f"Event(name={self.name}, trigger={self.trigger}, actions={self.actions})"

class Trigger:
    def __init__(self, trigger_type, **kwargs):
        self.trigger_type = trigger_type
        self.kwargs = kwargs

    def __repr__(self):
        return f"Trigger(type={self.trigger_type}, kwargs={self.kwargs})"

class Action:
    def __init__(self, action_type, **kwargs):
        self.action_type = action_type
        self.kwargs = kwargs

    def __repr__(self):
        return f"Action(type={self.action_type}, kwargs={self.kwargs})"

class ConfigParser:
    def __init__(self, config_file):
        self.config_file = config_file
        self.variables = {}
        self.events = {}
        self.parse_config()

    def parse_config(self):
        config = configparser.ConfigParser(allow_no_value=True)
        config.read(self.config_file)

        if 'Variables' in config:
            for var_name in config['Variables']:
                var_info = self.parse_variable(config['Variables'][var_name])
                if var_info:
                    self.variables[var_name] = Variable(var_name, **var_info)

        if 'Events' in config:
            for event_name in config['Events']:
                event_info = self.parse_event(config['Events'][event_name])
                if event_info:
                    self.events[event_name] = Event(event_name, **event_info)

    def parse_variable(self, variable_str):
        var_type_match = re.search(r'type: (\w+)', variable_str)
        description_match = re.search(r'description: (.+)', variable_str)
        
        if var_type_match and description_match:
            return {
                'var_type': var_type_match.group(1),
                'description': description_match.group(1).strip()
            }
        else:
            print(f"Invalid variable definition: {variable_str}")
            return None

    def parse_event(self, event_str):
        trigger_match = re.search(r'trigger: (.+)', event_str)
        actions_match = re.findall(r'- type: (\w+)(.*)', event_str)
        
        if trigger_match:
            trigger_info = self.parse_trigger(trigger_match.group(1))
            actions_info = [self.parse_action(action[0], action[1]) for action in actions_match]
            
            if trigger_info and all(actions_info):
                return {
                    'trigger': Trigger(**trigger_info),
                    'actions': [Action(**action) for action in actions_info]
                }
            else:
                print(f"Invalid event definition: {event_str}")
                return None
        else:
            print(f"Invalid event trigger: {event_str}")
            return None

    def parse_trigger(self, trigger_str):
        trigger_parts = trigger_str.split()
        if len(trigger_parts) >= 2:
            trigger_type = trigger_parts[0]
            kwargs = {trigger_parts[i]: trigger_parts[i + 1] for i in range(1, len(trigger_parts), 2)}
            return {
                'trigger_type': trigger_type,
                'kwargs': kwargs
            }
        else:
            print(f"Invalid trigger definition: {trigger_str}")
            return None

    def parse_action(self, action_type, action_str):
        action_parts = action_str.split()
        kwargs = {action_parts[i]: action_parts[i + 1] for i in range(0, len(action_parts), 2)}
        return {
            'action_type': action_type,
            'kwargs': kwargs
        }

    def get_variables(self):
        return self.variables

    def get_events(self):
        return self.events

# Example usage
config_file = 'AM example.txt'  # Replace with your config file path
parser = ConfigParser(config_file)

print("Variables:")
for var_name, var in parser.get_variables().items():
    print(var)

print("\nEvents:")
for event_name, event in parser.get_events().items():
    print(event)

