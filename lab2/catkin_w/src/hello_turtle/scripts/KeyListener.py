from pynput import keyboard

class KeyListener():
    lastKey = None

    def __init__(self):
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

    def on_press(self, key):
        self.lastKey = key
    def on_release(self, key):
        self.lastKey = None


