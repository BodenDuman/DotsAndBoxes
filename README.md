# DotsAndBoxes

vision_node.py
  → Only watches the camera
  → Detects which line was drawn
  → Publishes /human_move
  → Knows nothing about whose turn it is

game_orchestrator.py  
  → Manages turns (human turn / robot turn)
  → Tells vision node when to watch and when to pause
  → Passes moves to game_node.py
  → Knows nothing about camera or robot arm

game_node.py
  → Runs Minimax to pick best move
  → Publishes /robot_move
  → Knows nothing about camera or arm

move_robot.py
  → Receives /robot_move
  → Controls the UR5e arm via MoveIt
  → Draws the line physically
  → Publishes /robot_done when finished