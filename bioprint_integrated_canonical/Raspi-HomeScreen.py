# main.py
from kivy.app import App
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.uix.gridlayout import GridLayout
from kivy.uix.textinput import TextInput
from kivy.config import Config
Config.set('graphics', 'width', '480')   # Set desired width (e.g., 320 px)
Config.set('graphics', 'height', '320')  # Set desired height (e.g., 240 px)

# Screens (Temperature, Speed, Concentration, etc.)
class HomeScreen(Screen):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        layout = BoxLayout(orientation='vertical', padding=10, spacing=10)

        layout.add_widget(Label(text="Temperature: 45 °C"))
        layout.add_widget(Label(text="Set Temp: 50 °C"))
        layout.add_widget(Label(text="Speed: 1 mm/min"))
        layout.add_widget(Label(text="Print Time: 60 sec"))
        layout.add_widget(Label(text="Conc: 0-100%"))
        layout.add_widget(Label(text="Final Conc: 100%"))
        layout.add_widget(Label(text="[color=0000FF]Printing...[/color]", markup=True))
        layout.add_widget(Button(text="Locked", background_color=(0.5, 0.5, 1, 1)))

        self.add_widget(layout)

class TemperatureScreen(Screen):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        layout = GridLayout(cols=3, spacing=10, padding=10)
        layout.add_widget(Button(text="-"))
        layout.add_widget(Label(text="____"))
        layout.add_widget(Button(text="+"))

        temps = ["20", "37", "40", "45", "50", "55", "60", "65"]
        for t in temps:
            layout.add_widget(Button(text=t))

        layout.add_widget(Button(text="Back", on_press=self.go_back))
        layout.add_widget(Label(text=""))
        layout.add_widget(Button(text="°C"))

        self.add_widget(layout)

    def go_back(self, instance):
        self.manager.current = 'home'

# Reusable screen class template
class SimpleScreen(Screen):
    def __init__(self, title, button_labels, **kwargs):
        super().__init__(**kwargs)
        layout = BoxLayout(orientation='vertical', spacing=10, padding=10)
        layout.add_widget(Label(text=title, font_size='20sp'))

        grid = GridLayout(cols=3, spacing=10)
        for label in button_labels:
            grid.add_widget(Button(text=label))
        layout.add_widget(grid)

        layout.add_widget(Button(text="Back", on_press=self.go_back))
        self.add_widget(layout)

    def go_back(self, instance):
        self.manager.current = 'home'

class SpeedScreen(SimpleScreen):
    def __init__(self, **kwargs):
        super().__init__("Set Speed (mm/min)", ["0.5", "1", "1.5", "2", "2.5", "3"], **kwargs)

class StartConcScreen(SimpleScreen):
    def __init__(self, **kwargs):
        super().__init__("Start Concentration (%)", ["0", "25", "50", "75", "100"], **kwargs)

class EndConcScreen(SimpleScreen):
    def __init__(self, **kwargs):
        super().__init__("End Concentration (%)", ["0", "25", "50", "75", "100"], **kwargs)

class VolumeS1Screen(SimpleScreen):
    def __init__(self, **kwargs):
        super().__init__("Volume S1 (uL)", ["10", "20", "30", "40", "50"], **kwargs)

class VolumeS2Screen(SimpleScreen):
    def __init__(self, **kwargs):
        super().__init__("Volume S2 (uL)", ["10", "20", "30", "40", "50"], **kwargs)

class PrintTimeScreen(SimpleScreen):
    def __init__(self, **kwargs):
        super().__init__("Print Time (sec)", ["30", "60", "90", "120"], **kwargs)

class MainApp(App):
    def build(self):
        sm = ScreenManager()
        sm.add_widget(HomeScreen(name='home'))
        sm.add_widget(TemperatureScreen(name='temperature'))
        sm.add_widget(SpeedScreen(name='speed'))
        sm.add_widget(StartConcScreen(name='start_conc'))
        sm.add_widget(EndConcScreen(name='end_conc'))
        sm.add_widget(VolumeS1Screen(name='volume_s1'))
        sm.add_widget(VolumeS2Screen(name='volume_s2'))
        sm.add_widget(PrintTimeScreen(name='print_time'))

        return sm

if __name__ == '__main__':
    MainApp().run()
