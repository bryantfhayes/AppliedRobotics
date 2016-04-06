import sys, argparse, serial, syslog, time
# Add reference to 'lib' folder
sys.path.append('../lib')
import SerialArm

# Parse any command line arguments
parser = argparse.ArgumentParser(description='Process command line arguments')
parser.add_argument("-t", "--testmode", help="Run without serial device attached", action='store_true')
args = parser.parse_args()

def main():
    if args.testmode:
        arm = SerialArm.SerialArm(14, 15.5)
    else:
        arm = SerialArm.SerialArm(14,15.5,'/dev/cu.usbmodem1411')
    while True:
        arm.setPosition(0,10,15)
        time.sleep(2.5)
        arm.setPosition(13,20,-5)
        time.sleep(2.5)
        arm.setPosition(-13,20,-5)
        time.sleep(2.5)

if __name__ == "__main__":
    main()
