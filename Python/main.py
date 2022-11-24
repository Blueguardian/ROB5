from classes.tcp_serv import TCPServer
from classes.state import State
from classes.GUI import LaserGui
from classes.png2ply import Png2PlyConverter
from classes.xml_pars import XMLHandler
from classes.proc3d import Proc3D
import os
import time
import PySimpleGUI as Guihandle
import numpy as np

SOURCE_DIR = os.path.join(os.path.dirname(__file__), 'png')
TARGET_DIR = os.path.join(os.path.dirname(__file__), 'ply')
if not os.path.isdir(SOURCE_DIR):
    os.mkdir(SOURCE_DIR, 0o777)
if not os.path.isdir(TARGET_DIR):
    os.mkdir(TARGET_DIR, 0o777)
SAVE_RGB = True
SAVE_VERT = False

XML_handle = XMLHandler()
server = TCPServer()
GUI = LaserGui()
proc3d = Proc3D(TARGET_DIR)
state = State.STANDBY
quality_scan = 0
scan_bool = False

while True:
    events, values = GUI.getinput()
    identifier, message = server.recieveData()
    if identifier != "DATA" and message != "INVALID":
        print(f"Identifier: {identifier}, message: {message}")
    if events == 'Abort' or events == Guihandle.WIN_CLOSED:
        state = State.ABORT

    match state:
        case State.STANDBY:
            GUI.update_text('_moveText_', 'Idle')
            GUI.SetLED('_move_', 'red')
            GUI.SetLED('_scan_', 'red')
            GUI.SetLED('_treat_', 'red')
            GUI.SetLED('_rotate_', 'red')
            event, value = GUI.getinput()
            if events == 'Execute' and not (events == 'Abort' or events == Guihandle.WIN_CLOSED):
                GUI.update_text('_moveText_', 'Moving')
                events, values = GUI.getinput()
               # for file in os.listdir(SOURCE_DIR):
               #     if os.path.isfile(file):
               #         os.remove(file)
                for file in os.listdir(TARGET_DIR):
                    if os.path.isfile(os.path.join(TARGET_DIR, file)):
                        os.remove(os.path.join(TARGET_DIR, file))
                server.sendData("state", "start")
                state = State.PROG_SCANNING
                GUI.SetLED('_move_', 'green')
                events, values = GUI.getinput()
                quality_scan = 0
                scan_bool = False

        case State.PROG_SCANNING:
            GUI.update_text('_scanText_', 'Scanning Object')
            events, values = GUI.getinput()
            if identifier == 'SCAN' and message == 'DONE' and events != 'Abort':
                GUI.update_text('_moveText_', 'Idle')
                GUI.update_text('_scanText_', 'Converting scans')
                events, values = GUI.getinput()
                i = 0
                for file in os.listdir(SOURCE_DIR):
                    filename = os.fsdecode(file)
                    if filename.endswith(".png") or filename.endswith(".PNG"):
                        source_file = os.path.join(SOURCE_DIR, filename)
                        target_file = os.path.join(TARGET_DIR, f"pointcloud_{i}.ply")
                        converter = Png2PlyConverter(source_file, target_file)
                        converter.extract_data()
                        converter.write_ply(SAVE_RGB, SAVE_VERT)
                        i += 1
                        continue
                    else:
                        continue
                GUI.update_text('_scanText_', 'Conversion done')
                events, values = GUI.getinput()
                state = State.DONE_SCANNING
            elif identifier == 'QUALITY_SCAN' and message == 'DONE' and events != 'Abort':
                GUI.update_text('_scanText_', 'Converting scans')
                events, values = GUI.getinput()
                i = 0
                for file in os.listdir(SOURCE_DIR):
                    filename = os.fsdecode(file)
                    if filename.endswith(".png") or filename.endswith(".PNG"):
                        source_file = os.path.join(SOURCE_DIR, filename)
                        target_file = os.path.join(TARGET_DIR, f"pointcloud_{i}.ply")
                        converter = Png2PlyConverter(source_file, target_file)
                        converter.extract_data()
                        converter.write_ply(SAVE_RGB, SAVE_VERT)
                        continue
                    else:
                        continue
                GUI.update_text('_scanText_', 'Conversion done')
                events, values = GUI.getinput()
                state = State.DONE_SCANNING
                quality_scan += 1
            if events == 'Abort':
                state = State.ABORT

        case State.DONE_SCANNING:
            points_array = np.array([])
            if scan_bool is False and events != 'Abort':
                GUI.SetLED('_scan_', 'green')
                GUI.update_text('_scanText_', 'Scan done')
                events, values = GUI.getinput()
                scan_bool = True
                ispointcloud = proc3d.load_points()
                if ispointcloud is True and events != 'Abort':
                    proc3d.process_points(proc3d.object_type)
                    points_array = proc3d.output_points()
                    XML_handle.clearfile()
                    XML_handle.writepoints(points_array)
                    state = State.PROG_LASER
                else:
                    print("No cloud available for processing")
                    state = State.ABORT
            elif quality_scan > 0 and events != 'Abort':
                GUI.SetLED('_scan_', 'green')
                GUI.update_text('_scanText_', 'QUALITY SCAN DONE')
                events, values = GUI.getinput()
                ispointcloud = proc3d.load_points()
                if ispointcloud is True and events != 'Abort':
                    proc3d.process_points(proc3d.object_type)
                    points_array = proc3d.output_points()
                    XML_handle.clearfile()
                    XML_handle.writepoints(points_array)
                    # missing if statement = are we done? if yes then return to idle
                    # else:
                    state = State.PROG_LASER
                else:
                    print("No cloud available for processing")
                    state = State.ABORT
            if events == 'Abort':
                state = State.ABORT
        case State.PROG_LASER:
            server.sendData('state', 'Laser')
            identifier, message = server.recieveData()
            GUI.update_text('_treatText_', 'Ablating the Surface')
            events, values = GUI.getinput()
            if identifier == 'LASER' and message == 'DONE' and events != 'Abort':
                state = State.DONE_LASER
            if events == 'Abort':
                state = State.ABORT

        case State.DONE_LASER:
            GUI.update_text('_treatText_', 'Laser treatment done')
            GUI.SetLED('_treat_', 'green')
            events, values = GUI.getinput()
            state = State.REORIENTING
            if events == 'Abort:':
                state = State.ABORT

        case State.REORIENTING:
            GUI.SetLED('_rotate_', 'green')
            GUI.update_text('_rotateText_', 'Rotating the object')
            events, values = GUI.getinput()
            state = State.PROG_SCANNING
            if events == 'Abort':
                state = State.ABORT

        case State.ABORT:
            GUI.SetLED('_move_', 'red')
            GUI.update_text('_moveText_', 'Aborting...')
            GUI.SetLED('_scan_', 'red')
            GUI.update_text('_scanText_', 'Aborting...')
            GUI.SetLED('_treat_', 'red')
            GUI.update_text('_treatText_', 'Aborting...')
            GUI.SetLED('_rotate_', 'red')
            GUI.update_text('_rotateText_', 'Aborting...')
            server.sendData('STATE', 'ABORT')
            quality_scan = 0
            scan_bool = False
            GUI.update_text('_moveText_', '')
            GUI.update_text('_scanText_', '')
            GUI.update_text('_treatText_', '')
            GUI.update_text('_rotateText_', '')
            state = State.STANDBY





