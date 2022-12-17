# Dummy function
def pass_func(**kwargs):
    pass

class keyStateMachine:

    _registry = []

    def __init__(self, parent_name="",
                        key_name="", 
                        key_modifier="",
                        on_press=pass_func, 
                        on_hold=pass_func, 
                        on_release=pass_func,
                        **kwargs):
        
        # Flags
        self.parent_name = parent_name
        self.key_name = key_name
        self.key_modifier = key_modifier
        self.key_state = "released"

        # Function arguments
        self.kwargs = kwargs

        # States
        self.on_press = on_press
        self.on_hold = on_hold
        self.on_release = on_release

        self._registry.append(self)

    def key_callback(self, event, event_modifiers):
        self.event = event
        self.event_modifiers = event_modifiers
        self.modifier_present = False

        # Check for modifier
        if self.key_modifier == "":
            self.modifier_present = True

        else:
            if self.key_modifier in self.event_modifiers:
                self.modifier_present = True

            else:
                self.modifier_present = False

        # Key is pressed initially
        if self.event == "down" and self.modifier_present and self.key_state == "released":
            self.key_state = "pressed"
            try:
                self.on_press(**self.kwargs)
                # print("on_press")

            except Exception as e:
                print(e)
            

        # Key is pressed constantly
        elif self.event == "down" and self.modifier_present and self.key_state == "pressed":
            self.key_state = "pressed"
            try:
                self.on_hold(**self.kwargs)
                # print("on_hold")

            except Exception as e:
                print(e)

        # Key is released
        elif (self.event == "up" or not self.modifier_present) and self.key_state == "pressed":
            self.key_state = "released"
            try:
                self.on_release(**self.kwargs)
                # print("on_release")

            except Exception as e:
                print(e)

if __name__ == "__main__":
    import key_listener as kl

    kl.keyboard_init()

    # Main loop
    while True:
        pass
    

