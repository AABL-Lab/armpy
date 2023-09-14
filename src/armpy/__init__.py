
_registry = {}
def _register(name, loader):
    if name in _registry:
        raise ValueError(f"Cannot re-register robot {name}")
    _registry[name] = loader
def list_robots():
    return _registry.keys()


def initialize(robot_name, *args, **kwargs):
    return _registry[robot_name](*args, **kwargs)

try:
    from . import arm
except ImportError:
    pass
else:
    def _init_gen2_arm():
        args = {
            "planning_frame": 'base_link', 
            "eef_frame": 'j2s7s300_ee_link',
            "default_planner": "RRTConnectkConfigDefault"
        }
        return arm.Arm(**args)
    _register("gen2", _init_gen2_arm)

try:
    from . import kortex_arm
except ImportError:
    pass
else:
    def _init_gen3(robot_name="/my_gen3"):
        return kortex_arm.Arm(robot_name)
    def _init_gen3_lite(robot_name="/my_gen3_lite"):
        return kortex_arm.Arm(robot_name)
    _register("gen3", _init_gen3)
    _register("gen3_lite", _init_gen3_lite)


