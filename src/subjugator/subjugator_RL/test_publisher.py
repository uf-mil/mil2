from movement_publisher import SubjugatorControl

control = SubjugatorControl()

action = []
action.append(10)
action.append(4)

control.force_action(action)
