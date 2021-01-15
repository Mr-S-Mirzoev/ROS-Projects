import PySimpleGUI as sg
import subprocess  
import time
import os
import yaml

"""
Logger part
"""
try:
    hd = os.path.expanduser('~')
    os.mkdir(os.path.join(hd, '.package_creator'))
except FileExistsError:
    pass
except:
    print("Failed to create logger dir")

file_logger = None
try:
    file_log = "ros_pc_{}.log".format(str(time.time()).replace('.','_'))
    hd = os.path.expanduser('~')
    log_dir = os.path.join(hd, '.package_creator')
    file_path = os.path.join(log_dir, file_log)
    file_logger = open(file_path, "w")
except OSError:
    print("Could not open log file:", file_path)
    file_logger = None

def log(s):
    if file_logger:
        file_logger.write(s + "\n")

"""
Configs
"""
hd = os.path.expanduser('~')
cfg_dir = os.path.join(hd, '.package_creator')
file_path = os.path.join(cfg_dir, "defaults.yaml")

cfg = dict()
keys = [
    'maintainer_name',
    'maintainer_email',
    'license'
]
for key in keys:
    cfg[key] = ""

with open(file_path) as file_cfg:
    cfg_list = yaml.load(file_cfg, Loader=yaml.FullLoader)
    for key in keys:
        if cfg_list[key]:
            cfg[key] = cfg_list[key]

print("Using defaults: {} from {}".format(cfg, file_path))

"""
Request handling
"""

def form_a_request(values):
    request = ["ros2", "pkg", "create"]
    flags = [
        "--description",
        "--maintainer-name",
        "--maintainer-email",
        "--license",
        "--build-type",
        "--dependencies",
        "--node-name"
    ]

    errors = ""

    if not values[0]:
        errors += "No package name specified; "

    if not values[5]:
        errors += "No build type specified; "

    k = len(flags)
    
    request.append(values[0])

    for i in range(k):
        if values[i + 1] != '':
            request.append(flags[i])
            if i != 4:
                request.append(values[i + 1])
            else:
                request += values[i + 1].split()

    return request, errors
    
def execute_ros2_create(request):
    
    print(request)

    process = subprocess.Popen(request,
                     stdout=subprocess.PIPE, 
                     stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()
    stdout, stderr

    output = stdout.decode("utf-8") + "\n" + stderr.decode("utf-8")

    print(output)

    ret = 0
    errors = ""

    abort_loc = output.find("Aborted!", 0) 
    if abort_loc != -1:
        ret = -1 
        errors += output[abort_loc:].replace('\n', ' ') + "; " 
        
    abort_loc = output.find("ros2: error: ", 0)
    if abort_loc != -1:
        ret = -1 
        errors += output[abort_loc:].replace('\n', ' ') + "; " 

    return (errors, output, ret)

"""
GUI Part
"""

def execution_in_window(request):
    layout = [
        [sg.Output(size=(60,10))],
        [sg.Button('Go'), sg.Button('Exit')]  
    ]

    window = sg.Window('Command Line Output', layout)
    
    errors = ""
    ret_code = -1

    try:
        while True:             # Event Loop
            event, values = window.read()
            if event == sg.WIN_CLOSED or event == 'Exit':
                break
            elif event == 'Go':
                print("Executing shell equivalent. It's output: ")
                errors, output, ret_code = execute_ros2_create(request)
                print(output)
                print('Your operation completed')
    except Exception as e:
        print("Got an exception!")
        errors += "Unknown exception: {}; ".format(e)

    print(errors)
    window.close()
    return errors, ret_code

"""
Main
"""


# Green & tan color scheme      
sg.ChangeLookAndFeel('GreenTan')      

sg.SetOptions(text_justification='right')      

layout = [  
            [sg.Text('ROS2 Create Package', size=(30, 1), justification='center', font=("Helvetica", 25))],
            [sg.Text('Package Name', size=(15, 1)),      
            sg.In(default_text='', size=(25, 1))],
            [sg.Text('Description', size=(15, 1)),      
            sg.Multiline(size=(45, 2))],     
            [sg.Text('Maintainer Name', size=(15, 1)), sg.In(default_text=cfg["maintainer_name"], size=(15, 1)), sg.Text('Maintainer Email', size=(15, 1)),      
            sg.In(default_text=cfg["maintainer_email"], size=(15, 1))],      
            [sg.Text('License', size=(15, 1)), sg.In(default_text=cfg["license"], size=(15, 1)), sg.Text('Build type', size=(15, 1)),      
            sg.Drop(values=('ament_cmake', 'ament_python'), auto_size_text=True)],      
            [sg.Text('Dependencies', size=(15, 1)), sg.In(size=(15, 1)), sg.Text('Node Name', size=(15, 1)),      
            sg.In(default_text='', size=(15, 1))],      
            [sg.Text('_'  * 100, size=(65, 1))],      
            [sg.Submit(), sg.Cancel()]
        ]      

window = sg.Window('ROS2 Create Package', layout, font=("Helvetica", 12))      

event, values = window.read()
if event == "Cancel" or event == sg.WIN_CLOSED:
    print("Canceled")
    exit(0)

log("[DEBUG]: Got values = {}".format(values))
request, errors = form_a_request(values)
log("[DEBUG]: Formed a request = {}".format(request))
log("[DEBUG]: During procceeding got errors = {}".format(errors))

if errors == "":
    errors, result = execution_in_window(request) # shell run
else:
    result = -1
log("[DEBUG]: Result of execution in a window = {}".format(result))

if result == 0:
    result_str = 'Package successfully created'
else:
    result_str = 'Package not created'

if result != 0:
    sg.theme('Dark Red')

    string_1 = list()
    string_1.append(sg.Text(result_str))

    string_2 = list()
    string_2.append(sg.Text("Errors: {}".format(errors)))

    string_3 = list()
    string_3.append(sg.Button('OK'))

    layout = [
        string_1,
        string_2,
        string_3
    ]
else:
    sg.theme('Dark Green')

    string_1 = list()
    string_1.append(sg.Text(result_str))

    string_2 = list()
    string_2.append(sg.Button('OK'))

    layout = [
        string_1,
        string_2
    ]

event, values = sg.Window('Operation result', layout).read(close=True)

if file_logger:
    file_logger.close()