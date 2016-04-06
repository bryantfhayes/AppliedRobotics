import sys, argparse, serial, syslog, time
# Add reference to 'lib' folder
sys.path.append('../lib')
import SerialArm

# Parse any command line arguments
parser = argparse.ArgumentParser(description='Process command line arguments')
parser.add_argument("-t", "--testmode", help="Run without serial device attached", action='store_true')
args = parser.parse_args()

fishing_pole = SerialArm.Tool(1,19,17)

def main():
    if args.testmode:
        arm = SerialArm.SerialArm(15.5, 14, fishing_pole, z_offset=10.5)
    else:
        arm = SerialArm.SerialArm(15.5,14, fishing_pole,'/dev/cu.usbmodem1411', z_offset=10.5)
    arm.setPosition(0,33,0)
    time.sleep(5)
    while True: 
        arm.setPosition(0,33,-6)
        time.sleep(1)
        raw_input("Press Enter to fish...")
        arm.setPosition(0,33,-10)
        time.sleep(0.5)
        arm.setPosition(0,33,0)
        time.sleep(2)
        arm.setPosition(-10,33,0)
        time.sleep(2)
        arm.setPosition(-10,33,-12)
        time.sleep(2)
        arm.setPosition(-10,33,0)
        time.sleep(2)

if __name__ == "__main__":
    main()
