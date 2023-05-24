import os, sys, time, serial, serial.tools.list_ports
import argparse
from datetime import datetime

ERROR_CODE_UPDATE_SUCCESSUL     = 0
ERROR_CODE_COM_PORT_FAILED      = 1
ERROR_CODE_BINARY_FAILED        = 2
ERROR_CODE_UPDATE_FAILED        = 3
ERROR_CODE_UPDATE_TIMEOUT       = 4

NATIVE_PORT_VID                 = 0x0419
NATIVE_PORT_PID                 = 0x225

LENOVO_VID_SPEAKERPRO_DSP       = 0x17EF
LENOVO_PID_SPEAKERPRO_DSP       = 0xA055

PORT_BAUDRATE                   = 3000000
PORT_TIMEOUT                    = 15
DSP_TIMEOUT                     = 30
DSP_DETECT_TIMEOUT              = 10

MAJOR_ID                        = 0
MINOR_ID                        = 10

ATS3607_UPDATE_OPEN             = 0x55
ATS3607_UPDATE_TRANSFER         = 0x56
ATS3607_UPDATE_CLOSE            = 0x57
ATS3607_PAYLOAD_SUFFIX          = 0x59

ATS3607_LOG_FILENAME            = "ats3607"

def main():
    print("{} version: {}.{}".format(os.path.split(sys.argv[0])[1], MAJOR_ID, MINOR_ID))

    parser = argparse.ArgumentParser(fromfile_prefix_chars='@')
    parser.add_argument('-vid', '--vid', default = 0, required = False, type = int, help = 'VID')
    parser.add_argument('-pid', '--pid', default = 0, required = False, type = int, help = 'PID.')
    parser.add_argument('-com', '--com', default = 0, type = int, help = 'COM port to open. Default: auto-scan the device')
    parser.add_argument('-b', '--baudrate', default = PORT_BAUDRATE, type = int, help = 'Baudrate. Default: {}'.format(PORT_BAUDRATE))
    parser.add_argument('-pto', '--port_timeout', default = PORT_TIMEOUT, type = int, help = 'Timeout in seconds. Default: {}'.format(PORT_TIMEOUT))
    parser.add_argument('-uto', '--update_timeout', default = DSP_TIMEOUT, type = int, help = 'Timeout in seconds. Default: {}'.format(DSP_TIMEOUT))
    parser.add_argument('-bin', '--bin', type = str, required = True, help = 'Binary file to update.')
    args = parser.parse_args()

    try:
        with open(args.bin, 'rb'):
            pass
    except IOError:
        print("DSP update no binary found")
        sys.exit(ERROR_CODE_BINARY_FAILED)

    ret = ERROR_CODE_UPDATE_SUCCESSUL
    if args.com == 0 :
        com_port_scan = True
        chosePortName = None
        ret = ERROR_CODE_COM_PORT_FAILED
    else:
        com_port_scan = False
        chosePortName = "COM{}".format(args.com)
        ret = ERROR_CODE_UPDATE_SUCCESSUL

    last = datetime.now()
    while com_port_scan:
        now = datetime.now()
        portList = serial.tools.list_ports.comports()
        if len(portList) > 0:
            for port in portList:
                if args.vid == 0 or args.vid == 0:
                    if (port.vid == NATIVE_PORT_VID and port.pid == NATIVE_PORT_PID) or \
                        (port.vid == LENOVO_VID_SPEAKERPRO_DSP and port.pid == LENOVO_PID_SPEAKERPRO_DSP):
                        chosePortName = list(port)[0]
                        com_port_scan = False
                        ret = ERROR_CODE_UPDATE_SUCCESSUL
                        break
                else:
                    if port.vid == args.vid and port.pid == args.pid:
                        chosePortName = list(port)[0]
                        com_port_scan = False
                        ret = ERROR_CODE_UPDATE_SUCCESSUL
                        break
        if (now - last).total_seconds() > args.port_timeout:
            break

    new_line = None
    cmd = None
    begin = datetime.now()
    last = datetime.now()
    if ret == ERROR_CODE_UPDATE_SUCCESSUL:
        fileHandle = open(args.bin, 'rb')
        try:
            chosePort = serial.Serial(chosePortName, args.baudrate, timeout = 1)
            chosePort.baudrate = args.baudrate
            chosePort.bytesize = 8
            chosePort.parity = serial.PARITY_NONE
            chosePort.stopbits = 1
            chosePort.timeout = 0.5
            chosePort.writeTimeout = 0.5

            fileHandle.seek(0, os.SEEK_END)
            binary_len = fileHandle.tell()
            print("DSP update initial. Port: {}. Binary-size: {}".format(chosePortName, binary_len))

            timeout_ms = DSP_DETECT_TIMEOUT
            f = open(os.getcwd() + "\\" + ATS3607_LOG_FILENAME + '.log','w')
            state = 0
            dsp_data = bytearray()
            while True:
                if new_line != None:
                    new_line = True
                cmd = None
                now = datetime.now()
                queue_len = chosePort.inWaiting()
                if queue_len != 0:
                    dsp_data = dsp_data + bytearray(chosePort.read_all())
                    if len(dsp_data) == 15:
                        f.write('Rx: {} \n'.format(''.join("{:02X} ".format(x) for x in dsp_data)))
                        cmd = dsp_data[0]
                        res = None
                        if cmd == ATS3607_UPDATE_OPEN:
                            new_line = True
                            timeout_ms =  args.update_timeout
                            res = [ATS3607_UPDATE_OPEN, 0x0F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, ATS3607_PAYLOAD_SUFFIX]
                            print("DSP update opened")
                        elif cmd == ATS3607_UPDATE_TRANSFER:
                            offset = dsp_data[2] | (dsp_data[3] << 8) | (dsp_data[4] << 16) | (dsp_data[5] << 24)
                            size = dsp_data[6] | (dsp_data[7] << 8) | (dsp_data[8] << 16) | (dsp_data[9] << 24)
                            state = int((offset + size + 512)*100/binary_len)
                            fileHandle.seek(offset, 0)
                            res = fileHandle.read(size)
                            new_line = False
                            print("DSP update state:", state, end = "\r")
                        elif cmd == ATS3607_UPDATE_CLOSE:
                            print("{}DSP update closed. Take {} seconds".format('\n' if new_line else '', (now - begin).total_seconds()))
                            break

                        if res != None:
                            last = now
                            chosePort.write(res)
                            f.write('Tx: {} \n'.format(''.join("{:02X} ".format(x) for x in res)))

                        dsp_data = bytearray()

                if (now - last).total_seconds() > timeout_ms:
                    print("{}DSP update timeout".format('\n' if new_line else ''))
                    ret = ERROR_CODE_UPDATE_TIMEOUT
                    break
            f.close()
        except:
            print("{}DSP update failed".format('\n' if new_line else ''))
            ret = ERROR_CODE_UPDATE_FAILED
        fileHandle.close()
    else:
        print("{}DSP update open failed".format('\n' if new_line else ''))
        ret = ERROR_CODE_COM_PORT_FAILED

    filename = os.path.splitext(os.path.split(sys.argv[0])[1])[0]
    f = open(os.getcwd() + "\\" + ATS3607_LOG_FILENAME + '_status.log', "w")
    f.write("{} status: {}".format(filename, ret))
    f.close()

    sys.exit(ret)

if __name__ == '__main__':
    main()
