import os
import yaml
import time


with open("parameter_ranges.yaml", 'r') as stream:
    param_dict = yaml.safe_load(stream)

current_luas = [""]

for key in param_dict:
    val = param_dict[key]
    
    new_luas = []

    for lua in current_luas:
        if isinstance(val, list):
            for param in val:
                if isinstance(param, bool):
                    new_luas.append(lua + "\t" + key + " = " + str(param).lower() + ",\n")
                elif isinstance(param, str):
                    new_luas.append(lua + "\t" + key + " = " + '"' + str(param) + '"' + ",\n")
                else:
                    new_luas.append(lua + "\t" + key + " = " + str(param) + ",\n")

        else:
            if isinstance(val, bool):
                new_luas.append(lua + "\t" + key + " = " + str(val).lower() + ",\n")
            elif isinstance(val, str):
                new_luas.append(lua + "\t" + key + " = " + '"' + str(val) + '"' + ",\n")
            else:
                new_luas.append(lua + "\t" + key + " = " + str(val) + ",\n")

    current_luas = new_luas


open("overview.txt", "w")

for i in range(len(current_luas)):
    luafile = open("test" + str(i+1).zfill(3) + ".lua", "w")
    luafile.write("options = {\n")
    luafile.write(current_luas[i])
    luafile.write("}\nreturn options \n")
    luafile.close()

    start = time.time()
    os.system('$HOME/hector/devel/bin/cartographer_pointcloud_converter -config_file "test' + str(i+1).zfill(3) + '.lua"')
    end = time.time()

    overview = open("overview.txt", "a")
    overview.write("Run No. " + str(i+1).zfill(3) + ":\n")
    overview.write("Calculation time: " + str(end-start) + "\n")
    overview.close()









