from kivy.app import App
from kivy.uix.boxlayout import BoxLayout


class MainView(BoxLayout):
    pass

class BezierApp(App):
    def build(self):
        app = MainView()
        return app


BezierApp().run()