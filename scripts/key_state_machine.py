import keyboard

def pass_func(**kwargs):
    pass

class keyStateMachine:
    def __init__(self, key_name="", 
                        key_modifier="",
                        on_press=pass_func, 
                        on_hold=pass_func, 
                        on_release=pass_func,
                        **kwargs):
        # Flags
        self.key_name = key_name
        self.key_modifier = key_modifier
        self.key_state = "released"

        # Function arguments
        self.kwargs = kwargs

        # States
        self.on_press = on_press
        self.on_hold = on_hold
        self.on_release = on_release

        keyboard.hook_key(key=self.key_name, callback=self.key_callback, suppress=False)

    def key_callback(self, event):
        self.event = event
        self.modifier_present = False

        # Check for modifier
        if self.key_modifier == "":
            self.modifier_present = True

        else:
            if self.key_modifier in self.event.modifiers:
                self.modifier_present = True

            else:
                self.modifier_present = False
            

        # Key is pressed initially
        if self.event.event_type == "down" and self.modifier_present and self.key_state == "released":
            self.key_state = "pressed"

            self.on_press(**self.kwargs)

        # Key is pressed constantly
        elif self.event.event_type == "down" and self.modifier_present and self.key_state == "pressed":
            self.key_state = "pressed"

            self.on_hold(**self.kwargs)

        # Key is released
        elif self.event.event_type == "up" and self.modifier_present and self.key_state == "pressed":
            self.key_state = "released"

            self.on_release(**self.kwargs)


if __name__ == "__main__":
    # rospy.init_node("keyboard_test", anonymous=True)

    def test1(**kwargs):
        print(kwargs['one'])

    def test2(**kwargs):
        print(kwargs['two'])

    def test3(**kwargs):
        print(3)

    keyStateMachine(key_name="w", key_modifier="", on_press=test1, on_release=test3)

    # Main loop
    while True:
        pass

    