import sys, argparse, serial, syslog, time
# Add reference to 'lib' folder
sys.path.append('../lib')
import SerialArm

# Parse any command line arguments
parser = argparse.ArgumentParser(description='Process command line arguments')
parser.add_argument("-w", "--wireless", help="Run over UDP packets", action='store_true')
args = parser.parse_args()

fishing_pole = SerialArm.Tool(0,0,0)

steps = 100

def main():
    if args.wireless:
      arm = SerialArm.SerialArm(15.5,14,fishing_pole,z_offset=10.5,wireless=True)
    else:
        arm = SerialArm.SerialArm(14,15.5,'/dev/cu.usbmodem1411')
    for x in xrange(0,steps,1):
        arm.setPosition(10-(x/float(steps/10)),20,3)
        time.sleep(1)
        print "X = {}".format(10-(x/float(steps/10)))
if __name__ == "__main__":
    main()
