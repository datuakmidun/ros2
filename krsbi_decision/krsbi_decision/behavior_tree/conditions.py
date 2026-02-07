import py_trees

class IsBallVisible(py_trees.behaviour.Behaviour):
    def __init__(self, name="IsBallVisible"):
        super(IsBallVisible, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        ball = self.blackboard.get("world_state.ball")
        if ball and ball.is_visible:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class IsBallInKickRange(py_trees.behaviour.Behaviour):
    def __init__(self, threshold=0.3, name="IsBallKickable"):
        super(IsBallInKickRange, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.threshold = threshold

    def update(self):
        ball = self.blackboard.get("world_state.ball")
        robot_pose = self.blackboard.get("world_state.robot_pose")
        
        if not ball or not robot_pose:
            return py_trees.common.Status.FAILURE
            
        # Assuming ball position is relative to global frame
        # If robot hasn't localized, use relative ball pos directly?
        # krsbi_vision publishes local (relative) coordinates distance/angle usually.
        # But WorldModel is supposed to unify.
        # If WorldModel stores relative to robot (e.g. ball.x is forward dist), then dist = sqrt(x^2+y^2).
        
        dist = (ball.x**2 + ball.y**2)**0.5 # Simple relative distance assumption
        if dist < self.threshold:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class IsGameState(py_trees.behaviour.Behaviour):
    def __init__(self, state="PLAYING", name="IsStatePlay"):
        super(IsGameState, self).__init__(name)
        self.target_state = state
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        current = self.blackboard.get("world_state.game_phase")
        if current == self.target_state:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
