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
		arm = SerialArm.SerialArm(14,15.5,'/dev/cu/usbmodem1411')

	while True:	
		print "Enter XYZ Coordinates (ex. 10,10,10): "
		coords = raw_input()
		coords = coords.split(',')
		coords = map(int, coords)
		arm.setPosition(coords)
		print arm.getPosition()

if __name__ == "__main__":
	main()