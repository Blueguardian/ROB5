import PySimpleGUI as GUI

class LaserGui:
    progress_column = [
        [GUI.Text("Current task: ", justification='center', size=(50, 10))],
        [GUI.Text(" ", justification='center', key='task')],
        [GUI.ProgressBar(
                  max_value=100,
                  orientation='h',
                  size=(20,20),
                  key='prog_main',
                  visible=False,
                  expand_x=True)],
        [GUI.ProgressBar(
                  max_value=100,
                  orientation='h',
                  size=(20,20),
                  key='prog_sub',
                  visible=False,
                  expand_x=True)],
    ]

    button_column = [[GUI.Text("Initialize program:")],
        [GUI.Button("Execute", size=(10, 10))],
                     [GUI.Text("Abort program:")],
        [GUI.Button("Abort", size=(10, 10))]]

    layout = [[GUI.Column(progress_column), GUI.VSeparator(), GUI.Column(button_column)]]


    def __init__(self):
        self.window = GUI.Window("Laser treatment system v0.1", self.layout, size=(560, 400))

    def getinput(self):
        event, values = self.window.read(timeout=10)
        return event, values

    def initprogressbar(self):
        self.window['prog_main'].update(visible=True)
        self.window['prog_sub'].update(visible=True)

    def updatetask(self, value):
        self.window['task'].update(f"{value}")

    def close(self):
        self.window.close()
