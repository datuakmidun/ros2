import py_trees
from .actions import GoToPosition, FollowBall, KickBall, RotateToGoal
from .conditions import IsBallVisible, IsBallInKickRange, IsGameState

def create_striker_root(node):
    """
    Striker Strategy:
    1. Check Game State (Must be PLAYING)
    2. Check Ball (Visible -> Approach -> Kick)
    3. Search Ball (Spin)
    """
    
    root = py_trees.composites.Selector("StrikerRole", memory=False)
    
    # 1. Game State Guard
    check_play = IsGameState("PLAYING", name="CheckPlay")
    play_seq = py_trees.composites.Sequence("PlayingSequence", memory=True)
    
    # Structure:
    # Root (Selector)
    #   - Not Playing? -> Stop (Implicit failure of CheckPlay means we fall through? No.)
    #   - Playing Sequence
    # Or:
    # Root (Sequence)
    #   - IsPlaying?
    #   - StrikerLogic (Selector)
    
    main_seq = py_trees.composites.Sequence("MainSequence", memory=False)
    main_seq.add_child(check_play)
    
    striker_logic = py_trees.composites.Selector("StrikerLogic", memory=False)
    main_seq.add_child(striker_logic)
    
    # Striker Logic:
    # Priority 1: Shoot if near goal? (Skip for now)
    # Priority 2: Kick if close to ball
    # Priority 3: Approach ball
    # Priority 4: Search ball
    
    # Kick Sequence
    kick_seq = py_trees.composites.Sequence("KickSequence", memory=True)
    kick_seq.add_children([
        IsBallVisible(),
        IsBallInKickRange(threshold=0.4),
        RotateToGoal(node=node), # Align
        KickBall(node=node)
    ])
    
    # Approach Sequence
    approach_seq = py_trees.composites.Sequence("ApproachSequence", memory=True)
    approach_seq.add_children([
        IsBallVisible(),
        FollowBall(node=node) # Using behavior_node command
    ])
    
    # Search (Spin)
    # Since FollowBall fails if ball invisible, we need fallback.
    # To implement Search, creates a Spin action.
    # For now, just Wait/Idle if no ball.
    # Or use behavior_node which spins if no ball?
    # behavior_node logic was: if no ball -> spin.
    # So `FollowBall` action might handle search if behavior_node is smart.
    # But `FollowBall` action says: if no ball visible -> FAILURE.
    # So we need Search Action.
    
    # Search Action (Spin)
    # Implement inline or add to actions.
    # For now, use FollowBall because behavior_node handles ball loss.
    # Wait, behavior_node handles ball loss if command is FOLLOW_BALL.
    # So ApproachSequence should succeed/run if FollowBall runs.
    # But `IsBallVisible` condition fails the sequence immediately if false.
    # So if Ball NOT visible, `ApproachSequence` fails.
    # Then `striker_logic` goes to next child.
    
    # Search Child
    # spin_action = RotateInPlace(node)
    # striker_logic.add_child(spin_action)
    
    striker_logic.add_children([kick_seq, approach_seq])
    
    return main_seq

def create_goalie_root(node):
    # Simplified Goalie
    root = py_trees.composites.Sequence("GoalieRole", memory=False)
    root.add_child(IsGameState("PLAYING"))
    
    goalie_logic = py_trees.composites.Selector("GoalieLogic")
    root.add_child(goalie_logic)
    
    # Logic:
    # 1. Clear ball if close
    # 2. Stay in goal box
    
    clear_seq = py_trees.composites.Sequence("ClearBall")
    clear_seq.add_children([
        IsBallVisible(),
        IsBallInKickRange(threshold=0.8), # Larger radius
        KickBall(node=node)
    ])
    
    defend_pos = GoToPosition(node=node, x=-4.5, y=0.0) # Own goal
    
    goalie_logic.add_children([clear_seq, defend_pos])
    
    return root
