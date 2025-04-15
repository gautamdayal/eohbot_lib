from inputs import get_gamepad
import calibration

class LogitechF710:
    def __init__(self):
        self.left_joystick = (calibration.joystick_l_hor_center, calibration.joystick_l_vert_center)
        self.right_joystick = (calibration.joystick_r_hor_center, calibration.joystick_r_vert_center)
    def __str__(self):
        return f"LeftJoystick: x = {self.left_joystick[0]}, y = {self.left_joystick[1]}\n\
                RightJoystick: x = {self.right_joystick[0]}, y = {self.right_joystick[1]}"
    def update_state(self, new_leftjoy_x, new_leftjoy_y, new_rightjoy_x, new_rightjoy_y):
        self.left_joystick = (new_leftjoy_x, new_leftjoy_y)
        self.right_joystick = (new_rightjoy_x, new_rightjoy_y)
    
if __name__ == "__main__":
    ctrl = LogitechF710()
    while 1:
        events = get_gamepad()
        for event in events:
            if event.code == "ABS_X":
                ctrl.left_joystick[0] = event.state 
            elif event.code == "ABS_Y":
                ctrl.left_joystick[1] = event.state 
            elif event.code == "ABS_RX":
                ctrl.right_joystick[0] = event.state
            elif event.code == "ABS_RY":
                ctrl.right_joystick[1] = event.state
        print(ctrl)

            # if event.ev_type == "Abs":
            #     print(event.code, event.state)




