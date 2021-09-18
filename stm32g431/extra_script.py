Import("env", "projenv")
import ftplib

from SCons.Script import COMMAND_LINE_TARGETS

from extra_script_config import *
# Define these variables in extra_script_config.py:
# FTP_IP_ADDRESS = 'YOUR_IP_ADDRESS'
# FTP_USER = 'YOUR_USERNAME'
# FTP_PASSWORD = 'YOUR_PASSWORD'



# if "idedata" in COMMAND_LINE_TARGETS:
#     env.Exit(0)

def firm_callback(source, target, env):
    try:
        session = ftplib.FTP(FTP_IP_ADDRESS, FTP_USER, FTP_PASSWORD)
        file = open('./.pio/build/nucleo_g431kb/firmware.bin','rb')
        session.storbinary('STOR firmware.bin', file)
        file.close()
        session.quit()
        print("File moved to OrangePi Zero")
    except Exception:
        print("/!\\/!\\/!\\ WARNING: File not moved to OrangePi Zero /!\\/!\\/!\\")

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", firm_callback)
