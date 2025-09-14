import shutil
Import("env", "projenv")
#from shutil import copyfile

HTTP_PATH = r"\\file-server\all\ota"

def upload_to_http(source, target, env):
    binsrc = str(source[0])
    hostname = env.GetProjectOption("custom_hostname") 
    if (len(hostname)>0):
        bindst = hostname + "-firmware.bin"
    else:
        bindst = env.GetProjectOption("custom_firmware_name")
    print(F"Copying\n  from: {binsrc} \n  to  : {HTTP_PATH}\\{bindst}")
    shutil.copy(binsrc, HTTP_PATH + "\\" + bindst)

#upload_to_http()
env.Replace(UPLOADCMD=upload_to_http)