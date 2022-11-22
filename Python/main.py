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
            if events == 'Execute' and not (events == 'Abort' or events == Guihandle.WIN_CLOSED):
               # for file in os.listdir(SOURCE_DIR):
               #     if os.path.isfile(file):
               #         os.remove(file)
                for file in os.listdir(TARGET_DIR):
                    if os.path.isfile(os.path.join(TARGET_DIR, file)):
                        os.remove(os.path.join(TARGET_DIR, file))
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
                    filename = os.fsdecode(file)
                    if filename.endswith(".png") or filename.endswith(".PNG"):
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
                GUI.updatetask('CONVERSION DONE')
                events, values = GUI.getinput()
                state = State.DONE_SCANNING
                quality_scan = quality_scan + 1
            if events == 'Abort':
                state = State.ABORT

        case State.DONE_SCANNING:
            points_array = np.array([])
            if scan_bool == True and events != 'Abort':
                GUI.updatetask('SCAN DONE')
                events, values = GUI.getinput()
                scan_bool = False
                ispointcloud = proc3d.load_points()
                if ispointcloud is True and events != 'Abort':
                    proc3d.process_points()
                    points_array = proc3d.output_points()
                    XML_handle.clearfile()
                    XML_handle.writepoints(points_array, 'planar')
                    state = State.PROG_LASER
                else:
                    print("No cloud available for processing")
                    state = State.ABORT
            elif quality_scan > 0 and events != 'Abort':
                GUI.updatetask('QUALITY SCAN DONE')
                events, values = GUI.getinput()
                ispointcloud = proc3d.load_points()
                if ispointcloud is True and events != 'Abort':
                    proc3d.process_points()
                    points_array = proc3d.output_points()
                    XML_handle.clearfile()
                    XML_handle.writepoints(points_array, 'planar')
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
            GUI.updatetask('LASER TREATMENT')
            events, values = GUI.getinput()
            identifier, message = server.recieveData()
            if identifier == 'LASER' and message == 'DONE':
                state = State.DONE_LASER
            events, values = GUI.getinput()
            if events == 'Abort':
                state = State.ABORT

        case State.DONE_LASER:
            GUI.updatetask('LASER TREATMENT DONE')
            events, values = GUI.getinput()
            state = State.REORIENTING
            if events == 'Abort:':
                state = State.ABORT

        case State.REORIENTING:
            GUI.updatetask('REORIENTING')
            events, values = GUI.getinput()
            state = State.PROG_SCANNING
            if events == 'Abort':
                state = State.ABORT

        case State.ABORT:
            GUI.updatetask("ABORTING...")
            server.sendData('STATE', 'ABORT')
            state = State.STANDBY





