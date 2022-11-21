from Python.classes.tcp_serv import TCPServer
from Python.classes.state import State
from Python.classes.GUI import LaserGui
from Python.classes.png2ply import Png2PlyConverter
from Python.classes.xml_pars import XMLHandler
from Python.classes.proc3d import Proc3D
import os
import time
import PySimpleGUI as Guihandle
import numpy as np

SOURCE_DIR = os.path.join(os.path.dirname(__file__), 'png')
TARGET_DIR = os.path.join(os.path.dirname(__file__), 'ply')
SAVE_RGB = True
SAVE_VERT = False

XML_handle = XMLHandler()
GUI = LaserGui()
server = TCPServer()
proc3d = Proc3D(TARGET_DIR)
state = State.STANDBY
quality_scan = 0
scan_bool = False

while True:
    events, values = GUI.getinput()
    GUI.initprogressbar()
    identifier, message = server.recieveData()
    if identifier != "DATA" and message != "INVALID":
        print(f"Identifier: {identifier}, message: {message}")
    if events == 'Abort' or events == Guihandle.WIN_CLOSED:
        state = State.ABORT

    match state:
        case State.STANDBY:
            events, values = GUI.getinput()
            GUI.updatetask("IDLE")
            # Delete all files in folder ply and png
            if events == 'Execute' and not (events == 'Abort' or events == Guihandle.WIN_CLOSED):
                server.sendData("state", "start")
                state = State.PROG_SCANNING
                quality_scan = 0

        case State.PROG_SCANNING:
            GUI.updatetask('SCANNING')
            events, values = GUI.getinput()
            if identifier == 'SCAN' and message == 'DONE' and events != 'Abort':
                GUI.updatetask('CONVERTING FILES')
                events, values = GUI.getinput()
                i = 0
                for file in os.listdir(SOURCE_DIR):
                    events, values = GUI.getinput()
                    filename = os.fsdecode(file)
                    if events != 'Abort' and (filename.endswith(".png") or filename.endswith(".PNG")):
                        source_file = os.path.join(SOURCE_DIR, filename)
                        target_file = os.path.join(TARGET_DIR, f"pointcloud_{i}.ply")
                        converter = Png2PlyConverter(source_file, target_file)
                        converter.extract_data()
                        converter.write_ply(SAVE_RGB, SAVE_VERT)
                        i = i + 1
                        continue
                    else:
                        continue
                GUI.updatetask('CONVERSION DONE')
                state = State.DONE_SCANNING
                scan_bool = True
            elif identifier == 'QUALITY_SCAN' and message == 'DONE' and events != 'Abort':
                GUI.updatetask('CONVERTING FILES')
                events, values = GUI.getinput()
                i = 0
                for file in os.listdir(SOURCE_DIR):
                    events, values = GUI.getinput()
                    filename = os.fsdecode(file)
                    if events != 'Abort' and (filename.endswith(".png") or filename.endswith(".PNG")):
                        source_file = os.path.join(SOURCE_DIR, filename)
                        target_file = os.path.join(TARGET_DIR, f"pointcloud_{i}.ply")
                        converter = Png2PlyConverter(source_file, target_file)
                        converter.extract_data()
                        converter.write_ply(SAVE_RGB, SAVE_VERT)
                        continue
                    else:
                        continue
                GUI.updatetask('CONVERSION DONE')
                events, values = GUI.getinput()
                state = State.DONE_SCANNING
                quality_scan = quality_scan + 1
            if events == 'Abort':
                state = State.ABORT

        case State.DONE_SCANNING:
            points_array = np.array([])
            if scan_bool == True:
                GUI.updatetask('SCAN DONE')
                events, values = GUI.getinput()
                scan_bool = False
            elif quality_scan > 0:
                GUI.updatetask('QUALITY SCAN DONE')
                events, values = GUI.getinput()
            ispointcloud = proc3d.load_points()
            if ispointcloud is True:
                proc3d.process_points()
                points_array = proc3d.output_points()
            else:
                print("No cloud available for processing")
                state = State.ABORT
            XML_handle.clearfile()
            XML_handle.writepoints(points_array, 'planar')
            state = State.PROG_LASER
            time.sleep(1)
            if events == 'Abort':
                state = State.ABORT
        case State.PROG_LASER:
            server.sendData('state', 'Laser')
            GUI.updatetask('LASER TREATMENT')
            events, values = GUI.getinput()
            identifier, message = server.recieveData()
            if identifier == 'LASER' and message == 'DONE':
                state = State.DONE_LASER
                time.sleep(1)
            events, values = GUI.getinput()
            if events == 'Abort':
                state = State.ABORT

        case State.DONE_LASER:
            GUI.updatetask('LASER TREATMENT DONE')
            events, values = GUI.getinput()
            state = State.REORIENTING
            time.sleep(1)
            if events == 'Abort:':
                state = State.ABORT

        case State.REORIENTING:
            GUI.updatetask('REORIENTING')
            events, values = GUI.getinput()
            state = State.PROG_SCANNING
            time.sleep(1)
            if events == 'Abort':
                state = State.ABORT

        case State.ABORT:
            GUI.updatetask("ABORTING...")
            server.sendData('STATE', 'ABORT')
            time.sleep(1)
            state = State.STANDBY





