import PySimpleGUI as GUI

class LaserGui:
    GUI.theme('Dark Amber')
    layout = [[GUI.Text('System Status Indicators', size=(20, 1))],
              [GUI.Text('Movement'), GUI.Graph(canvas_size=(50, 50),
                                               graph_bottom_left=(-50, -50),
                                               graph_top_right=(50, 50),
                                               pad=(0, 0), key='_move_'), GUI.Text('Idle', key='_moveText_'),
               GUI.Button("Execute", size=(15, 2), button_color='lime green')],
              [GUI.Text('3D Scanner'), GUI.Graph(canvas_size=(50, 50),
                                                 graph_bottom_left=(-50, -50),
                                                 graph_top_right=(50, 50),
                                                 pad=(0, 0), key='_scan_'), GUI.Text('Idle', key='_scanText_'),
               GUI.Button("Abort", size=(15, 2), button_color='red2')],
              [GUI.Text('Laser Treatment'), GUI.Graph(canvas_size=(50, 50),
                                                      graph_bottom_left=(-50, -50),
                                                      graph_top_right=(50, 50),
                                                      pad=(0, 0), key='_treat_'), GUI.Text('Idle', key='_treatText_')],
              [GUI.Text('Rotating Object'), GUI.Graph(canvas_size=(50, 50),
                                                      graph_bottom_left=(-50, -50),
                                                      graph_top_right=(50, 50),
                                                      pad=(0, 0), key='_rotate_'),
               GUI.Text('Idle', key='_rotateText_')],
              ]


    def __init__(self):
        self.window = GUI.Window('Laser System Monitoring', self.layout, default_element_size=None, auto_size_text=False,
                           no_titlebar=False,
                           finalize=True, auto_size_buttons=False, keep_on_top=True, grab_anywhere=True)

    def getinput(self):
        event, values = self.window.read(timeout=5)
        return event, values

    def update_text(self, key, text):
        self.window[key].update(text)
    def SetLED(self, key, color):
        graph = self.window[key]
        graph.erase()
        graph.draw_circle((0, 0), 12, fill_color=color, line_color=color)

    def close(self):
        self.window.close()
