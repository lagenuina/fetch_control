from pynput import keyboard
import key_state_machine as ksm

# Special modifier keys which should be considered as regular alphanumeric keys
special_modifiers = ["space", "esc", "tab", "enter", "backspace", 
                    "f1", "f2", "f3", "f4", "f5", "f6", "f7", "f8", "f9", "f10", "f11", "f12",
                    "up", "down", "left", "right"] 

# A variable which helps to find a parent class instance name of a keyStateMachine class instance:
# E.g. key "w" is defined in two other classes "chestMapping" and "armMapping". "/chest" is a name 
# variable of an instance of a "chestMapping" class and "/right" is a name variable of an instance
# of a "armMapping" class. To call an on_press function which has a functionality of a "armMapping"
# class, but not "chestMapping" we need to change "target_name" variable to "/chest", so the
# corresponding method will be called. 
# - Keys from the "ignore_target_name" list ignore "target_name" variable and are supposed to have
# the same functionality accross all parent classes. Use to for keys which allow to switch "target_
# name" variable.

target_name = ""

ingore_target_name = ["up", "down", "left", "right"]

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
            if keyObject.parent_name == target_name or keyObject.key_name in ingore_target_name:
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
            if keyObject.parent_name == target_name or keyObject.key_name in ingore_target_name:
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