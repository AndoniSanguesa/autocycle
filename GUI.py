from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput
from kivy.clock import Clock
from kivy.properties import NumericProperty
from bezier_visualization import Visualization as Graph

obstacle_data = []
resolution = 10


class SpaceAroundLayout(FloatLayout):
    """A Layout that evenly spreads children along its x-axis with space at both sides"""

    def calculate_children_width(self):
        """determines the total width of all children"""
        children_width = 0
        for child in self.children:
            children_width += child.width

        return int(children_width)

    def do_layout(self, *largs, **kwargs):
        """lays out the children such that they have space between and on either side of them"""
        self.set_height()
        dist = new_x = (self.width - self.calculate_children_width()) / (
            len(self.children) + 1
        )
        for child in self.children:
            child_y = (self.height - child.height) / 2
            child.pos = (new_x, self.y + child_y)
            new_x += dist + child.width

    def set_height(self):
        """
        Sets the height of the main widget
        """
        tallest_height = 0

        for child in self.children:
            if child.height > tallest_height:
                tallest_height = child.height

        self.height = tallest_height + 40


class CenteredTextInput(TextInput):

    def update_padding(self):
        """Updates padding so that text is centered on the cell"""
        self.validate_num()
        text_width = self._get_text_width(self.text, self.tab_width, self._label_cached)
        self.padding_x = (self.width - text_width) / 2
        self.update_resolution()

    def validate_num(self):
        try:
            int(self.text[-1])
        except ValueError:
            if self.text[-1] != ".":
                self.text = self.text[:-1]
        except IndexError:
            pass

    def update_resolution(self):
        global resolution
        if self.parent.children[1].text == "[b]Resolution: [/b]":
            try:
                resolution = int(self.text)
            except ValueError:
                pass


class ObstacleList(GridLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        Clock.schedule_once(self.add_children)

    def add_children(self, tick):
        global obstacle_data
        self.add_widget(BoxLayout(size_hint_x=None))
        col1 = NoScaleLabel(text="[b]Distance to\nObstacle[/b]", width="90dp")
        col2 = NoScaleLabel(text="[b]Obstacle Distance\nto Center[/b]", width="130dp")
        col3 = NoScaleLabel(text="[b]Obstacle\nlength[/b]", width="80dp")
        self.add_widget(col1)
        self.add_widget(col2)
        self.add_widget(col3)
        self.add_widget(BoxLayout())

        for ind in range(len(obstacle_data)):
            obstacle = obstacle_data[ind]
            self.add_widget(NoScaleLabel(text=f"[b]{ind + 1}[/b]"))
            self.add_widget(NoScaleLabelBordered(text=f"{obstacle[0]}", width=col1.width))
            self.add_widget(NoScaleLabelBordered(text=f"{obstacle[1]}", width=col2.width))
            self.add_widget(NoScaleLabelBordered(text=f"{obstacle[2]}", width=col3.width))
            delete_button = NoScaleButton(background_color=(1, 0, 0, 1), text="[b] x [/b]", font_size="15dp", rel_ind=ind)
            delete_button.fbind("on_press", delete_button.delete)
            self.add_widget(delete_button)
        self.add_widget(NoScaleLabel(text="[b]New:[/b]"))
        self.add_widget(CenteredTextInput(width="90dp"))
        self.add_widget(CenteredTextInput(width="130dp"))
        self.add_widget(CenteredTextInput(width="80dp"))
        new_line = NoScaleButton(text="[b] √ [/b]", font_size="15dp")
        new_line.fbind("on_press", new_line.add)
        self.add_widget(new_line)
        self.set_height()

    def set_height(self):
        self.height = self.children[1].height*(len(obstacle_data)+3)


class Visualization(BoxLayout):
    pass


class NoScaleLabel(Label):
    pass


class NoScaleLabelBordered(Label):
    pass


class NoScaleButton(Button):
    rel_ind = NumericProperty(0)

    def delete(self, tick1):
        obstacle_data.pop(self.rel_ind)
        scroll = self.parent.parent
        scroll.clear_widgets()
        new_list = ObstacleList()
        scroll.add_widget(new_list)

    def add(self, tick1):
        children = self.parent.children
        try:
            obstacle_data.append([float(children[3].text), float(children[2].text), float(children[1].text)])
        except ValueError:
            return
        scroll = self.parent.parent
        scroll.clear_widgets()
        new_list = ObstacleList()
        scroll.add_widget(new_list)

    def create_graph(self):
        lines = [resolution, obstacle_data]
        Graph(lines).create_environment()
        img = self.parent.parent.children[2]
        img.source = "plot.png"
        img.reload()

    def download_data(self):
        lines = [resolution, obstacle_data]
        graph = Graph(lines)
        graph.create_environment()
        file = open("output_data.txt", "w")
        file.write(str(graph.get_bez_vals()))


class Content(BoxLayout):
    pass


class Title(SpaceAroundLayout):
    pass


class MainOptions(BoxLayout):
    pass


class MainView(BoxLayout):
    pass


class BezierApp(App):
    def build(self):
        app = MainView()
        return app


BezierApp().run()