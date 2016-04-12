import math, sys, argparse, serial, syslog, time
# Add reference to 'lib' folder
sys.path.append('../lib')
import SerialArm

# Parse any command line arguments
parser = argparse.ArgumentParser(description='Process command line arguments')
parser.add_argument("-t", "--testmode", help="Run without serial device attached", action='store_true')
parser.add_argument("-w", "--wireless", help="Run over UDP", action='store_true')
args = parser.parse_args()

fishing_pole = SerialArm.Tool(0,0,0)

def main():
    if args.testmode:
        arm = SerialArm.SerialArm(15.5,14,fishing_pole,z_offset=10.5)
    elif args.wireless:
        arm = SerialArm.SerialArm(15.5,14,fishing_pole,z_offset=10.5,wireless=True)    
    else:
        arm = SerialArm.SerialArm(15.5,14,fishing_pole,'/dev/cu.usbserial-A4012H3E',z_offset=10.5)
    while True: 
        print "Enter XYZ Coordinates (ex. 10,10,10): "
        coords = raw_input()
        coords = coords.split(',')
        coords = map(int, coords)
        arm.setPosition(coords)
        print arm.getPosition()

if __name__ == "__main__":
    main()
