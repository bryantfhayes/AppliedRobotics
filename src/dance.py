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
        arm.setPosition(-10,10,0)
        time.sleep(1)
        arm.setPosition(-10,25,0)
        time.sleep(1)
        arm.setPosition(0,25,0)
        time.sleep(1)
        arm.setPosition(10,25,0)
        time.sleep(1)
        arm.setPosition(10,10,0)
        time.sleep(1)
        arm.setPosition(0,5,0)
        time.sleep(1)
        arm.setPosition(-10,10,0)
        time.sleep(1)

if __name__ == "__main__":
    main()
