import zipfile 
import xml.etree.ElementTree as ET
import json

MYNAME = "amlx_to_buxxe.py"

def create_empty_config():
    # create empty buxxe config
    # note: fmus will be stored in targetpath/FMUs/fmuname.fmu
    config = {\
            "ros_in"            : [], # [inputtopic,[fmuname, [fmuinputs]]]
            "ros_out"           : {}, # {outtopic : list of [fmuname, fmuoutput]}
            "ros_connections"   : [], # contains list of [[fmuname, fmuoutput], [fmuname, fmuinput]]
            "ros_fmus"          : []  # contains list of [fmuname,[fmuinputs],[fmuoutputs]]
        }
    return config

def add_fmu(config,fmuname,inputnames,outputnames):
    config["ros_fmus"].append([fmuname,inputnames,outputnames])

def add_connection(config, fmu1_name, out_name, fmu2_name, in_name):
    config["ros_connections"].append([[str(fmu1_name),str(out_name)],[str(fmu2_name),str(in_name)]])

# ROS topic is created for outputs
def add_rosout(config,fmu_name,fmu_output_names):
    config["ros_out"]["/"+fmu_name+"_output"]=[fmu_name,fmu_output_names]

# ROS topic is created for inputs
def add_rosin(config, fmu_name,fmu_input_names):
    config["ros_in"].append(["/"+fmu_name+"_input",[fmu_name,fmu_input_names]])

def aml_find_by_name(root, name):
    return root.find(".//{http://www.dke.de/CAEX}InternalElement[@Name='"+name+"']")
def aml_find_by_tag(root, tag):
    return root.find(".//{http://www.dke.de/CAEX}"+tag)

# ToDo: Determination of data types for inputs and outputs
def amlx_to_buxxe(sourceamlx, targetpath):
    # check if source is zip file
    if not zipfile.is_zipfile(sourceamlx):
        print MYNAME + " ERROR: sourcefile is no zipfile. Exiting."
        return
    
    config = create_empty_config()
    
    # open amlx
    with zipfile.ZipFile(sourceamlx) as amlx_file:
        for filename in amlx_file.namelist():
            # handle .fmu files
            if filename.endswith(".fmu"):
                # extract fmus to config folder
                amlx_file.extract(filename, targetpath)
            # handle .aml file
            if filename.endswith(".aml"):
                # read .aml
                with amlx_file.open(filename) as aml_file:
                    root = ET.parse(aml_file).getroot()
                    # find fmu containing element
                    fmu_container = aml_find_by_name(root, "SimulationEnviroment") # find solution for identifying the FMU container list

                    # create empty id -> fmu, io name dict
                    id_to_io = {}
                    # create empty (id, id) connection list
                    id_connections = []

                    # read fmu data
                    for fmu_elem in fmu_container:
                        if fmu_elem.attrib["Name"].endswith("FMU"):

                            # fmu name
                            fmu_name = fmu_elem.attrib["Name"][:-4]
                            print fmu_name
                       
                            # create empty io name lists
                            inputnames=[]
                            outputnames=[]
                            
                            # inputs
                            print "\tInputs"
                            input_container = aml_find_by_name(fmu_elem, "Inputs")
                            for input_elem in input_container:
                                inputnames.append(input_elem.attrib["Name"])
                                print "\t\t", input_elem.attrib["Name"]

                                # external interface
                                ei_elem = aml_find_by_tag(input_elem,"ExternalInterface")
                                if ei_elem is not None:
                                    print "\t\t\t", ei_elem.attrib["ID"]
                                    id_to_io[ei_elem.attrib["ID"]] = (fmu_name, input_elem.attrib["Name"])

                                # internal link 
                                il_elem = aml_find_by_tag(input_elem,"InternalLink")
                                if il_elem is not None:
                                    print "\t\t\t", il_elem.attrib["RefPartnerSideA"], "->", il_elem.attrib["RefPartnerSideB"]
                                    # required order: output, input
                                    id_connections.append((il_elem.attrib["RefPartnerSideB"], il_elem.attrib["RefPartnerSideA"]))
                            

                            # outputs
                            print "\tOutputs"
                            output_container = aml_find_by_name(fmu_elem, "Outputs")
                            for output_elem in output_container:
                                outputnames.append(output_elem.attrib["Name"])
                                print "\t\t", output_elem.attrib["Name"]

                                # external interface
                                ei_elem  = aml_find_by_tag(output_elem,"ExternalInterface")
                                if ei_elem is not None:
                                    print "\t\t\t", ei_elem.attrib["ID"]
                                    id_to_io[ei_elem.attrib["ID"]] = (fmu_name, output_elem.attrib["Name"])
                            
                                # internal link 
                                il_elem = aml_find_by_tag(output_elem,"InternalLink")
                                if il_elem is not None:
                                    print "\t\t\t", il_elem.attrib["RefPartnerSideA"], "->", il_elem.attrib["RefPartnerSideB"]
                                    # required order: output, input
                                    id_connections.append((il_elem.attrib["RefPartnerSideA"], il_elem.attrib["RefPartnerSideB"]))

                            # add fmu to config
                            add_fmu(config, fmu_name, inputnames, outputnames)
                        
                    # parse connections and add to config
                    for id_connection in id_connections:
                        fmu1_name = id_to_io[id_connection[0]][0]
                        io1_name = id_to_io[id_connection[0]][1]
                        fmu2_name = id_to_io[id_connection[1]][0]
                        io2_name = id_to_io[id_connection[1]][1]
                        
                        add_connection(config, fmu1_name, io1_name, fmu2_name, io2_name)



    # create connections (somewhat hacky)
    # fmu data format: [fmuname,[fmuinputs],[fmuoutputs]]
    # connections data format: [[fmuname, fmuoutput], [fmuname, fmuinput]]  TODO make sure that this is stored correctly!

    # check all pairs of fmus
    for curfmu in config["ros_fmus"]:

        # create empty lists for not yet connected inputs, outputs
        inputnames=[]
        outputnames=[]

        for targetfmu in config["ros_fmus"]:

            # only check different targets
            if curfmu[0] != targetfmu[0]:
                
                # handle curfmu inputs
                for curfmu_input in curfmu[1]:
                    connected = False
                    # check if curfmu_input is connected to any targetfmu output
                    for targetfmu_output in targetfmu[2]:
                        if [[targetfmu[0],targetfmu_output],[curfmu[0],curfmu_input]] in config["ros_connections"]:
                            connected = True 
                    if not connected: 
                        # store input name
                        inputnames.append(str(curfmu_input))

                # handle curfmu outputs
                for curfmu_output in curfmu[2]:
                    connected = False
                    # check if curfmu_output is connected to any targetfmu input
                    for targetfmu_input in targetfmu[1]:
                        if [[curfmu[0],curfmu_output],[targetfmu[0],targetfmu_input]] in config["ros_connections"]:
                            connected = True 
                    if not connected: 
                        # store output name
                        outputnames.append(str(curfmu_output))
        
        # add rosin, rosout if nonempty
        if inputnames:
            add_rosin(config, curfmu[0], inputnames)
        if outputnames:
            add_rosout(config, curfmu[0], outputnames)

    # change config data to strings
    for key in ["ros_in","ros_out","ros_connections"]:
        config[key] = repr(config[key])
    for fmu in config["ros_fmus"]:
        for c,fmu_input in enumerate(fmu[1]):
            fmu[1][c] = str(fmu_input)
        for c,fmu_output in enumerate(fmu[2]):
            fmu[2][c] = str(fmu_output)

    # create json config
    json_config = json.dumps(config)
        
    # save config file to config folder
    with open(targetpath+"config.txt", "w") as configfile:
        configfile.write(json_config)
    return json_config




# ToDo: Remove hard-coded paths, add main method
amlx_to_buxxe("/home/jwerth/Documents/whb/catkin_ws/src/spear_git/amlx_containers/arm/Turn_Table_ARM.amlx","tmp/")