from pynput import keyboard
import key_state_machine as ksm

# Special modifier keys which should be considered as regular alphanumeric keys
special_modifiers = ["space", "esc", "tab", "enter", "backspace", "f1", "f2", "f3", "f4", "f5", "f6", "f7", "f8", "f9", "f10", "f11", "f12"]


def on_press(key):
        global pressed_keys, pressed_modifiers
        
        # Alphanumeric keys
        try:
            key_name = str(key.char).lower()
            if key_name not in pressed_keys:
                pressed_keys.append(key_name)
        
        # Modifier keys
        except AttributeError:
            key_name = str(key).split(".")[1]

            # Check for special modifiers
            if key_name in special_modifiers and key_name not in pressed_keys:
                pressed_keys.append(key_name)

            elif key_name not in special_modifiers and key_name not in pressed_modifiers:
                pressed_modifiers.append(key_name)

        # Iterate over all keys of interest
        for keyObject in ksm.keyStateMachine._registry:
            if keyObject.key_name in pressed_keys:
                keyObject.key_callback("down", pressed_modifiers)

        # print(pressed_keys)
        # print(pressed_modifiers)


def on_release(key):
        global pressed_keys, pressed_modifiers

        # Alphanumeric keys
        try:
            key_name = str(key.char).lower()
            if key_name in pressed_keys:
                pressed_keys.remove(key_name)
        
        # Modifier keys
        except AttributeError:
            key_name = str(key).split(".")[1]

            # Check for special modifiers
            if key_name in special_modifiers and key_name in pressed_keys:
                pressed_keys.remove(key_name)

            elif key_name not in special_modifiers and key_name in pressed_modifiers:
                pressed_modifiers.remove(key_name)

        # Iterate over all keys of interest
        for keyObject in ksm.keyStateMachine._registry:
            if keyObject.key_name not in pressed_keys:
                keyObject.key_callback("up", pressed_modifiers)

        # print(pressed_keys)
        # print(pressed_modifiers)


def keyboard_init():
    global pressed_keys, pressed_modifiers
    
    pressed_keys = []
    pressed_modifiers = []  

    # Start keyboard listener thread
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()


if __name__ == "__main__":

    keyboard_init()
    
    # Main loop
    while True:
        pass