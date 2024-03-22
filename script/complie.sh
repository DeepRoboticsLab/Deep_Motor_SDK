SCRIPT_DIR=$(dirname $(readlink -f "$0"))
echo "current folder: $SCRIPT_DIR"

cd $SCRIPT_DIR/../example

gcc -o single_motor single_motor.c -g -lpthread

gcc -o multi_motor multi_motor.c -g -lpthread

