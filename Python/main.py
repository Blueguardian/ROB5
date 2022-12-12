from classes.tcp_serv import TCPServer
from classes.state import State
from classes.GUI import LaserGui
from classes.png2ply import Png2PlyConverter
from classes.xml_pars import XMLHandler
from classes.proc3d import Proc3D
from classes.kinematics import Kinematics
from time import sleep
import os
import PySimpleGUI as Guihandle
import numpy as np

SOURCE_DIR = os.path.join(os.path.dirname(__file__), 'png')
TARGET_DIR = os.path.join(os.path.dirname(__file__), 'ply')
MESH_DIR = os.path.join(os.path.dirname(__file__), 'meshes', 'obj1_mesh.ply')
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
kinematics = Kinematics()
state = State.STANDBY
quality_scan = 0
scan_bool = False
send_once = True

while True:
    events, values = GUI.getinput()
    identifier, message = server.recieveData()
    if(identifier != 'DATA' and message != 'INVALID'):
        print(f"Identifier: {identifier}, message: {message}")
    if events == 'Abort' or events == Guihandle.WIN_CLOSED:
        state = State.ABORT

    match state:
        case State.STANDBY:
            GUI.update_text('_moveText_', 'Idle')
            GUI.update_text('_scanText_', 'Idle')
            GUI.update_text('_treatText_', 'Idle')
            GUI.update_text('_rotateText_', 'Idle')
            GUI.SetLED('_move_', 'red')
            GUI.SetLED('_scan_', 'red')
            GUI.SetLED('_treat_', 'red')
            GUI.SetLED('_rotate_', 'red')
            #events, value = GUI.getinput()
            if events == 'Execute' and not (events == 'Abort' or events == Guihandle.WIN_CLOSED):
                GUI.update_text('_moveText_', 'Moving')
                events, values = GUI.getinput()
                for file in os.listdir(SOURCE_DIR):
                    if os.path.isfile(os.path.join(SOURCE_DIR, file)):
                       os.remove(os.path.join(SOURCE_DIR, file))
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
                sleep(1)
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
                sleep(1)
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
                    proc_bool = proc3d.process_points(MESH_DIR)
                    if proc_bool is True:
                        points_array = proc3d.output_points()
                        input_transformations = []
                        for pt in points_array:
                            input_transformations.append(kinematics.populate_transform(pt))

                        # Get object points relative to the galvo head
                        transformed_pts = np.asarray(kinematics.get_pts_ref_to_galvo(input_transformations,
                                                                          kinematics.T_moverg_tile))
                        XML_handle.clearfile()
                        XML_handle.writepoints(transformed_pts)
                        state = State.PROG_LASER
                    else:
                        print("Object does not need cleaning")
                        state = State.STANDBY
                else:
                    print("No cloud available for processing")
                    state = State.ABORT
            elif quality_scan > 0 and events != 'Abort':
                GUI.SetLED('_scan_', 'green')
                GUI.update_text('_scanText_', 'QUALITY SCAN DONE')
                events, values = GUI.getinput()
                ispointcloud = proc3d.load_points()
                if ispointcloud is True and events != 'Abort':
                    proc_bool = proc3d.process_points(proc3d.object_type)
                    if proc_bool is True:
                        points_array = proc3d.output_points()
                        input_transformations = []
                        for pt in points_array:
                            input_transformations.append(kinematics.populate_transform(pt))

                        # Get object points relative to the galvo head
                        transformed_pts = np.asarray(kinematics.get_pts_ref_to_galvo(input_transformations,
                                                                          kinematics.T_moverg_tile))
                        XML_handle.clearfile()
                        XML_handle.writepoints(transformed_pts)
                        state = State.PROG_LASER
                    else:
                        GUI.update_text('_moveText_', 'Done')
                        GUI.update_text('_scanText_', 'Done')
                        GUI.update_text('_treatText_', 'Done')
                        GUI.update_text('_rotateText_', 'Done')
                        state = State.DONE_LASER
                else:
                    print("No cloud available for processing")
                    state = State.ABORT
            if events == 'Abort':
                state = State.ABORT

        case State.PROG_LASER:
            if send_once is True:
                server.sendData('state', 'Laser')
                send_once = False
            GUI.update_text('_treatText_', 'Ablating the Surface')
            events, values = GUI.getinput()
            if identifier == 'LASER' and message == 'DONE' and events != 'Abort':
                state = State.DONE_LASER
                send_once = True
            if events == 'Abort':
                state = State.ABORT

        case State.DONE_LASER:
            GUI.update_text('_treatText_', 'Laser treatment done')
            GUI.SetLED('_treat_', 'green')
            events, values = GUI.getinput()
            server.sendData('SCAN', 'ACCEPTED')
            quality_scan = 0
            scan_bool = False
            state = State.DONE_PROCESS
            if events == 'Abort:':
                state = State.ABORT

        case State.DONE_PROCESS:
            if identifier == 'PROCESS' and message == 'DONE' and events != 'Abort':
                GUI.update_text('_rotateText_', 'Process done')
                GUI.SetLED('_rotate_', 'green')
                events, values = GUI.getinput()
                sleep(2)
                state = State.STANDBY
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





