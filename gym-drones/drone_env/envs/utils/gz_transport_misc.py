from gz.msgs11.boolean_pb2 import Boolean
from gz.msgs11.world_control_pb2 import Step
from gz.transport14 import Node

_node = Node()

# pause:
req = Boolean()
req.data = True
_node.request("/world/default/pause", req)

# unpause:
req.data = False
_node.request("/world/default/pause", req)

# reset:
# there’s also a `/world/default/reset` service you can call in exactly the same way
_node.request("/world/default/reset", Boolean(data=True))

# advance one iteration (if you want stepping):


step_req = Step()
step_req.iterations = 1
_node.request("/world/default/step", step_req)
